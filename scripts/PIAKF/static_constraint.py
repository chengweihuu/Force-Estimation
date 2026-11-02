"""Paired fixed-budget dynamic-only versus static-augmented learning probe."""
import argparse
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import traceback

for _name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_name] = "1"

import numpy as np
import torch

from checks import environment
from learning import load_split, torque_metrics, validate_learning_config
from motion_evaluation import load_record, plotting, replay_one, save_figure, window_mask
from simulation import error_statistics, evaluate as load_metrics, external_wrench, write_json
from static_gravity import evaluate as static_reference, statistics as gravity_statistics, training_bounds
from structured import load_network, save_model


ARMS = ("baseline", "static_augmented")
LABELS = {"baseline": "Dynamic loss only", "static_augmented": "+ static supervision"}
COLORS = {"baseline": "#d1495b", "static_augmented": "#0077b6"}
COMPONENTS = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")
UNITS = ("N", "N", "N", "Nm", "Nm", "Nm")


def configuration(root, config):
    design = json.loads((root / config["learning_config"]).read_text())
    validate_learning_config(design)
    training = config["training"]
    for key, old_key in (("seed", "seed"),):
        if training[key] != design[old_key]:
            raise ValueError("Original seed must be retained")
    for key, old_key in (("updates_per_arm", "updates"), ("dynamic_batch_size", "batch_size"),
                         ("learning_rate", "learning_rate"), ("optimizer", "optimizer"),
                         ("log_every", "log_every"), ("checkpoint_rule", "checkpoint_rule")):
        if training[key] != design["training"][old_key]:
            raise ValueError(f"Training setting differs from original: {key}")
    if [(a["id"], a["static_loss_weight"]) for a in training["arms"]] != [("baseline", 0.), ("static_augmented", 1.)]:
        raise ValueError("Use the two declared static-loss coefficients")
    initial = torch.load(root / config["initial_checkpoint"], map_location="cpu", weights_only=True)
    statistics = json.loads((root / config["statistics"]).read_text())
    if initial["updates"] != 0 or initial["network_config"] != design["network"] or initial["statistics"] != statistics:
        raise ValueError("Original initialization, statistics or architecture mismatch")
    network = load_network(root / config["initial_checkpoint"])
    bounds = training_bounds(root, {"training_config": config["learning_config"],
                                   "training_data_dir": config["training_data_dir"]}, network)
    data_dir = root / config["training_data_dir"]
    train_data = load_split(data_dir, design, "train")
    development = load_split(data_dir, design, "development")
    # Draw exactly as the original runner did, then reuse this immutable sequence.
    generator = torch.Generator().manual_seed(training["seed"])
    schedule = torch.stack([torch.randint(len(train_data["q"]), (training["dynamic_batch_size"],), generator=generator)
                            for _ in range(training["updates_per_arm"])])
    return design, statistics, bounds, train_data, development, schedule


def static_poses(config, center):
    poses = [{"id": "center", "q": center.copy()}]
    for joint in range(6):
        for offset in config["static_training"]["axis_offsets_rad"]:
            q = center.copy()
            q[joint] += offset
            poses.append({"id": f"J{joint+1}_{offset:+g}", "q": q})
    if len(poses) != config["static_training"]["pose_count"] or len({tuple(p["q"]) for p in poses}) != len(poses):
        raise ValueError("Static pose count or uniqueness mismatch")
    for name in ("velocity_rad_s", "acceleration_rad_s2", "external_wrench"):
        np.testing.assert_array_equal(config["static_training"][name], np.zeros(6))
    return poses


def static_labels(root, config, network, poses):
    reference_config = {"model": config["model"],
                        "comparison_rtol": config["checks"]["static_balance_rtol"],
                        "comparison_atol_Nm": config["checks"]["static_balance_atol_Nm"]}
    reference, _, checks = static_reference(root, reference_config, network, poses)
    return torch.from_numpy(np.stack([p["q"] for p in poses])), torch.from_numpy(reference), checks


def losses(model, data, indices, static_q, static_tau, weight):
    result = model.dynamics(data["q"][indices], data["v"][indices], data["a"][indices])
    dynamic = ((result["tau"] - data["tau"][indices]) / model.torque_scale).square().mean()
    # Keep the baseline backward computation identical to the original runner.
    if weight:
        _, gravity = model.mass_gravity(static_q)
        static = ((gravity - static_tau) / model.torque_scale).square().mean()
        total = dynamic + weight * static
    else:
        total = dynamic
    if not torch.isfinite(total):
        raise FloatingPointError("Nonfinite training loss")
    return total


def update(model, optimizer, loss):
    optimizer.zero_grad()
    loss.backward()
    if not all(p.grad is not None and torch.isfinite(p.grad).all() for p in model.parameters()):
        raise FloatingPointError("Nonfinite or missing training gradient")
    optimizer.step()
    if not all(torch.isfinite(p).all() for p in model.parameters()):
        raise FloatingPointError("Nonfinite trained parameter")


def measure(model, train_data, development, static_q, static_tau, chunk, update_number):
    with torch.no_grad():
        mass, gravity = model.mass_gravity(static_q)
        if not torch.isfinite(mass).all() or torch.linalg.eigvalsh(mass).min() <= 0:
            raise ValueError("Invalid learned static inertia")
        static_error = gravity - static_tau
        static_mse = float((static_error / model.torque_scale).square().mean())
    return {"update": update_number, "train": torque_metrics(model, train_data, chunk),
            "development": torque_metrics(model, development, chunk),
            "static_normalized_mse": static_mse,
            "static_training_rmse_Nm": static_error.square().mean(0).sqrt().tolist()}


def precheck(root, config, output):
    design, _, bounds, data, _, schedule = configuration(root, config)
    base = load_network(root / config["initial_checkpoint"])
    poses = static_poses(config, base.q_center.numpy())
    selected = [poses[0], poses[5], poses[7]]  # center, J2 -0.3 and +0.075 rad.
    if len(selected) != config["checks"]["precheck_static_pose_count"]:
        raise ValueError("Precheck static pose count mismatch")
    q, tau, checks = static_labels(root, config, base, selected)
    initial_weights = None
    for arm in config["training"]["arms"]:
        model = load_network(root / config["initial_checkpoint"])
        if initial_weights is None:
            initial_weights = {k: v.clone() for k, v in model.state_dict().items()}
        else:
            for name, value in model.state_dict().items():
                torch.testing.assert_close(value, initial_weights[name], rtol=0, atol=0)
        optimizer = torch.optim.Adam(model.parameters(), lr=config["training"]["learning_rate"])
        update(model, optimizer, losses(model, data, schedule[0], q, tau, arm["static_loss_weight"]))
        static_labels(root, config, model, selected)
    return {"passed": True, "static_pose_count": len(selected), "discarded_training_updates": 2,
            "main_training_updates": 0, "checks": checks, "training_bounds": bounds,
            "dynamic_train_samples": len(data["q"]), "architecture": design["network"]}


def train(root, config, output):
    design, statistics, bounds, data, development, schedule = configuration(root, config)
    initial = load_network(root / config["initial_checkpoint"])
    poses = static_poses(config, initial.q_center.numpy())
    static_q, static_tau, checks = static_labels(root, config, initial, poses)
    np.savez_compressed(output / "static_training.npz", position_rad=static_q.numpy(),
                        equilibrium_drive_torque_Nm=static_tau.numpy(), pose_id=[p["id"] for p in poses])
    np.savez_compressed(output / "shared_dynamic_batches.npz", indices=schedule.numpy())
    write_json(output / "training_bounds.json", bounds)
    histories, regression = {}, {}
    for arm in config["training"]["arms"]:
        name, weight = arm["id"], arm["static_loss_weight"]
        destination = output / name
        destination.mkdir(exist_ok=False)
        model = load_network(root / config["initial_checkpoint"])
        for key, value in model.state_dict().items():
            torch.testing.assert_close(value, initial.state_dict()[key], rtol=0, atol=0)
        optimizer = torch.optim.Adam(model.parameters(), lr=config["training"]["learning_rate"])
        chunk = design["training"]["evaluation_chunk"]
        history = [measure(model, data, development, static_q, static_tau, chunk, 0)]
        for index, indices in enumerate(schedule, start=1):
            update(model, optimizer, losses(model, data, indices, static_q, static_tau, weight))
            if index % config["training"]["log_every"] == 0 or index == len(schedule):
                row = measure(model, data, development, static_q, static_tau, chunk, index)
                history.append(row)
                write_json(destination / "training-history.json", history)
                print(json.dumps({"arm": name, "update": index, "dynamic_train_mse": row["train"]["normalized_mse"],
                                  "development_mse": row["development"]["normalized_mse"],
                                  "static_mse": row["static_normalized_mse"]}), flush=True)
        save_model(destination / "model.pt", model, statistics, len(schedule))
        torch.save(optimizer.state_dict(), destination / "optimizer.pt")
        histories[name] = history
        if name == "baseline":
            old = load_network(root / config["previous_baseline_checkpoint"])
            for key, value in model.state_dict().items():
                torch.testing.assert_close(value, old.state_dict()[key],
                                           rtol=config["checks"]["baseline_regression_rtol"],
                                           atol=config["checks"]["baseline_regression_atol"], msg=f"Baseline regression: {key}")
                regression[key] = float((value.to(torch.float64) - old.state_dict()[key].to(torch.float64)).abs().max())
            write_json(output / "baseline-regression.json", regression)
            print(json.dumps({"baseline_regression_passed": True, "max_parameter_difference": max(regression.values())}), flush=True)
    figure_paths = plot_training(output, histories)
    return {"passed": True, "main_training_updates": 2 * len(schedule), "arm_count": len(histories),
            "static_pose_count": len(poses), "paired_dynamic_batch_schedule": True,
            "baseline_regression_max_abs_difference": max(regression.values()), "static_checks": checks,
            "final": {name: history[-1] for name, history in histories.items()}, "figures": figure_paths}


def plot_training(output, histories):
    plt = plotting()
    figure, axes = plt.subplots(1, 3, figsize=(14, 4))
    for name, history in histories.items():
        updates = [row["update"] for row in history]
        for index, split in enumerate(("train", "development")):
            axes[index].semilogy(updates, [row[split]["normalized_mse"] for row in history],
                                 color=COLORS[name], label=LABELS[name])
            axes[index].set_title(f"Dynamic torque: {split}")
        axes[2].semilogy(updates, [row["static_normalized_mse"] for row in history], color=COLORS[name], label=LABELS[name])
    axes[2].set_title("Static torque: 25 training poses")
    axes[0].legend(fontsize=8)
    for axis in axes:
        axis.set(xlabel="Parameter updates", ylabel="Normalized MSE")
        axis.grid(alpha=.25)
    return save_figure(figure, output, "paired_learning_curves")


def evaluate_static(root, config, networks, training_dir, output):
    with np.load(training_dir / "static_training.npz", allow_pickle=False) as f:
        train_q = f["position_rad"].copy()
    with np.load(root / config["static_evaluation"]["source"], allow_pickle=False) as f:
        q, reference, identifiers = f["position_rad"].copy(), f["gravity_reference_Nm"].copy(), f["pose_id"].copy()
    overlap = np.any(np.all(q[:, None, :] == train_q[None, :, :], axis=2), axis=1)
    np.testing.assert_array_equal(overlap, identifiers == "center")
    if int((~overlap).sum()) != config["static_evaluation"]["expected_primary_pose_count"]:
        raise ValueError("Static primary evaluation count or overlap differs")
    poses = [{"id": str(name), "q": position} for name, position in zip(identifiers, q)]
    masks = {"primary_48": ~overlap, "center_training_overlap": overlap,
             "near": np.array([str(name).startswith("near_") for name in identifiers]),
             "extended": np.array([str(name).startswith("extended_") for name in identifiers])}
    reference_config = {"model": config["model"], "comparison_rtol": config["checks"]["static_balance_rtol"],
                        "comparison_atol_Nm": config["checks"]["static_balance_atol_Nm"]}
    summaries = {}
    for name, model in networks.items():
        recomputed, prediction, checks = static_reference(root, reference_config, model, poses)
        np.testing.assert_allclose(recomputed, reference, rtol=reference_config["comparison_rtol"],
                                   atol=reference_config["comparison_atol_Nm"])
        summaries[name] = {key: gravity_statistics(prediction - reference, mask) for key, mask in masks.items()}
        summaries[name]["checks"] = checks
        np.savez_compressed(output / f"static_{name}.npz", position_rad=q, pose_id=identifiers,
                            gravity_reference_Nm=reference, gravity_prediction_Nm=prediction,
                            gravity_error_Nm=prediction - reference, training_overlap=overlap)
    plt = plotting()
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    x = np.arange(6)
    for ax, group in zip(axes, ("near", "extended")):
        for index, name in enumerate(ARMS):
            ax.bar(x + (index - .5) * .35, summaries[name][group]["rmse_Nm"], .35,
                   color=COLORS[name], label=LABELS[name])
        ax.set(title=f"{group}: 24 non-training configurations", ylabel="Gravity torque RMSE [Nm]",
               xticks=x, xticklabels=[f"J{i+1}" for i in x])
        ax.grid(axis="y", alpha=.25)
    axes[0].legend(fontsize=8)
    return summaries, save_figure(fig, output, "static_gravity_comparison")


def plot_motion(output, records, curves, summary, motion_config):
    plt = plotting()
    paths = []
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    for row, case in enumerate(records):
        for column, start in enumerate((0, 3)):
            ax = axes[row, column]
            for index, name in enumerate(ARMS):
                ax.bar(np.arange(3) + (index - .5) * .35, summary[case][name]["core_motion"]["rmse"][start:start+3],
                       .35, color=COLORS[name], label=LABELS[name])
            ax.set(title=case + " / core [1,5) s", xticks=np.arange(3), xticklabels=COMPONENTS[start:start+3],
                   ylabel=f"RMSE [{UNITS[start]}]")
            ax.grid(axis="y", alpha=.25)
    axes[0, 0].legend(fontsize=8)
    paths += save_figure(fig, output, "motion_core_comparison")
    for case, record in records.items():
        mask = window_mask(record["time_s"], motion_config["windows"]["core_motion"])
        fig, axes = plt.subplots(3, 2, figsize=(11, 8), sharex=True)
        for component, ax in enumerate(axes.flat):
            ax.axhline(0, color="black", linestyle=":", label="No external wrench")
            for name in ARMS:
                ax.plot(record["time_s"][mask], curves[case][name][mask, component],
                        color=COLORS[name], label=LABELS[name], linewidth=1)
            ax.set_ylabel(f"{COMPONENTS[component]} [{UNITS[component]}]")
            ax.grid(alpha=.25)
        axes[0, 0].legend(fontsize=8)
        for ax in axes[-1]:
            ax.set_xlabel("Original time [s]")
        fig.suptitle(case + ": continuous native KF, core motion")
        paths += save_figure(fig, output, case + "_core_wrench")
    return paths


def plot_loads(output, records, references, curves):
    plt = plotting()
    fig, axes = plt.subplots(3, 3, figsize=(15, 10), sharex=True)
    styles = ("-", "--", ":")
    for row, (case, record) in enumerate(records.items()):
        t = record["time_s"]
        axes[row, 0].plot(t, references[case][:, 0], color="black", linewidth=1, label="Applied reference")
        for name in ARMS:
            axes[row, 0].plot(t, curves[case][name][:, 0], color=COLORS[name], label=LABELS[name], linewidth=.9)
            for component in (1, 2):
                axes[row, 1].plot(t, curves[case][name][:, component], color=COLORS[name],
                                  linestyle=styles[component-1], label=f"{name}/{COMPONENTS[component]}", linewidth=.8)
            for component in (3, 4, 5):
                axes[row, 2].plot(t, curves[case][name][:, component], color=COLORS[name],
                                  linestyle=styles[component-3], label=f"{name}/{COMPONENTS[component]}", linewidth=.8)
        for column, title in enumerate(("Fx [N]", "Fy/Fz [N]", "Mx/My/Mz [Nm]")):
            axes[row, column].set_title(case + " / " + title)
            axes[row, column].grid(alpha=.25)
    for ax in axes[0]:
        ax.legend(fontsize=6)
    for ax in axes[-1]:
        ax.set_xlabel("Original time [s]")
    return save_figure(fig, output, "load_response_comparison")


def evaluate(root, config, output, training_dir):
    networks = {name: load_network(training_dir / name / "model.pt") for name in ARMS}
    static, figure_paths = evaluate_static(root, config, networks, training_dir, output)
    motion_config = json.loads((root / config["motion_evaluation"]["config"]).read_text())
    if [r["id"] for r in motion_config["records"]] != config["motion_evaluation"]["record_ids"]:
        raise ValueError("Motion record schedule changed")
    motion, records, curves = {}, {}, {}
    for spec in motion_config["records"]:
        case = spec["id"]
        record, _ = load_record(root, motion_config, spec)
        records[case], motion[case], curves[case] = record, {}, {}
        destination = output / case
        destination.mkdir(exist_ok=False)
        for name in ARMS:
            arrays, diagnostics = replay_one(root, motion_config, record, networks, name)
            truth = np.zeros_like(arrays["wrench"])
            motion[case][name] = {window: error_statistics(arrays["wrench"], truth, window_mask(arrays["time_s"], rule))
                                  for window, rule in motion_config["windows"].items()}
            motion[case][name]["diagnostics"] = diagnostics
            curves[case][name] = arrays["wrench"]
            np.savez_compressed(destination / (name + ".npz"), **arrays)
        print(json.dumps({"motion_evaluated": case, "Fx_core_RMSE_N":
                          {name: motion[case][name]["core_motion"]["rmse"][0] for name in ARMS}}), flush=True)
    figure_paths += plot_motion(output, records, curves, motion, motion_config)
    design = json.loads((root / config["learning_config"]).read_text())
    replay_config = design["replay_config"]
    loads, records, curves, references = {}, {}, {}, {}
    for case in config["load_regression"]["scenarios"]:
        source = root / config["load_regression"]["source_dir"] / case
        with np.load(source / "measurements.npz", allow_pickle=False) as f:
            record = {key: f[key].copy() for key in f.files}
        if set(record) != {"time_s", "position_rad", "velocity_rad_s", "drive_torque_Nm"}:
            raise ValueError("Undeclared loading measurement inputs")
        with np.load(source / "reference.npz", allow_pickle=False) as f:
            reference = f["wrench"].copy()
            np.testing.assert_array_equal(f["time_s"], record["time_s"])
        expected = round(replay_config["duration_s"] / replay_config["observation_period_s"]) + 1
        np.testing.assert_allclose(record["time_s"], np.arange(expected) * replay_config["observation_period_s"], rtol=0, atol=1e-12)
        np.testing.assert_allclose(reference, np.array([external_wrench(case, t, replay_config) for t in record["time_s"]]), rtol=0, atol=0)
        if any(not np.isfinite(x).all() for x in record.values()):
            raise ValueError("Nonfinite loading record")
        records[case], references[case], loads[case], curves[case] = record, reference, {}, {}
        destination = output / case
        destination.mkdir(exist_ok=False)
        for name in ARMS:
            arrays, diagnostics = replay_one(root, replay_config, record, networks, name)
            loads[case][name] = load_metrics(arrays["time_s"], arrays["wrench"], reference, case, replay_config)
            loads[case][name]["diagnostics"] = diagnostics
            curves[case][name] = arrays["wrench"]
            np.savez_compressed(destination / (name + ".npz"), **arrays)
        print(json.dumps({"load_regression_evaluated": case}), flush=True)
    figure_paths += plot_loads(output, records, references, curves)
    write_report(output, static, motion, loads, figure_paths)
    return {"passed": True, "training_updates": 0, "arm_count": 2, "primary_static_pose_count": 48,
            "motion_trajectory_count": len(motion), "load_scenario_count": len(loads),
            "static": static, "motion": motion, "loads": loads, "figures": figure_paths,
            "scope": "Single-seed paired EXPLORE on previously analyzed evaluation records; no claim-level acceptance"}


def write_report(output, static, motion, loads, figure_paths):
    with (output / "all_metrics.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["domain", "case", "arm", "window", "samples_or_poses", "component", "unit", "rmse", "bias", "absolute_error_peak"])
        for arm, groups in static.items():
            for group, value in groups.items():
                if "rmse_Nm" not in value:
                    continue
                for index in range(6):
                    writer.writerow(["gravity", "static", arm, group, value["poses"], f"J{index+1}", "Nm",
                                     value["rmse_Nm"][index], value["bias_Nm"][index], value["absolute_error_peak_Nm"][index]])
        for domain, cases in (("motion", motion), ("load", loads)):
            for case, arms in cases.items():
                for arm, groups in arms.items():
                    for group, value in groups.items():
                        if "rmse" not in value:
                            continue
                        for index in range(6):
                            writer.writerow([domain, case, arm, group, value["samples"], COMPONENTS[index], UNITS[index],
                                             value["rmse"][index], value["bias"][index], value["absolute_error_peak"][index]])
    lines = ["# Paired static-supervision exploration", "",
             "Same original initialization, dynamic data, normalization, minibatch schedule, Adam settings and 1000 updates per arm.",
             "Only treatment: add equally weighted synthetic equilibrium labels at 25 declared static configurations, with static-loss coefficient 1.0.",
             "Labels come from MuJoCo algebraic equilibrium; this is additional gravity supervision, not measured controller holds or dynamic-only training.",
             "Static primary evaluation uses 48 non-training configurations. One overlapping center is reported separately. All evaluation sources have previously been analyzed; none is untouched holdout.",
             "Native KF R=Mhat Sigma_v Mhat^T, Q and covariance rules are unchanged; actual R can differ across arms through M.",
             "Continuous KF from t=0; no core reset, timestamp alignment, sample exclusion, tuning or best-checkpoint selection.",
             "This single-seed EXPLORE comparison does not assign an overall scientific pass or claim-level evidence.", "",
             "## Primary static gravity RMSE [Nm], 48 non-training configurations", "",
             "| Arm | J1 | J2 | J3 | J4 | J5 | J6 |", "|---|---:|---:|---:|---:|---:|---:|"]
    for name in ARMS:
        lines.append("| " + name + " | " + " | ".join(f"{x:.6g}" for x in static[name]["primary_48"]["rmse_Nm"]) + " |")
    lines += ["", "## Core moving no-contact RMSE", "", "| Trajectory | Arm | Fx [N] | Fy [N] | Fz [N] | Mx [Nm] | My [Nm] | Mz [Nm] |",
              "|---|---|---:|---:|---:|---:|---:|---:|"]
    for case, arms in motion.items():
        for name in ARMS:
            lines.append(f"| {case} | {name} | " + " | ".join(f"{x:.6g}" for x in arms[name]["core_motion"]["rmse"]) + " |")
    lines += ["", "## Loading regression", "", "| Case | Arm | Loaded-after-transient Fx RMSE [N] | Bias [N] |", "|---|---|---:|---:|"]
    for case, arms in loads.items():
        for name in ARMS:
            values = arms[name]["loaded_after_transient"]
            lines.append(f"| {case} | {name} | {values['rmse'][0]:.6g} | {values['bias'][0]:.6g} |")
    lines += ["", "Full step/sine response, cross-axis and window statistics remain in metrics.json and all_metrics.csv.",
              "Loading logs are existing fixed-reference probes, not independent moving-contact validation.", "", "## Figures", "",
              "![Paired training](../train/paired_learning_curves.png)", ""]
    for path in figure_paths:
        if path.endswith(".png"):
            lines += [f"![{path}]({path})", ""]
    (output / "report.md").write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stage", choices=("check", "train", "evaluate"), required=True)
    parser.add_argument("--prerequisite-dir", type=Path)
    parser.add_argument("--training-dir", type=Path)
    args = parser.parse_args()
    if not Path("/.dockerenv").is_file():
        parser.error("Use the existing approved Docker service")
    config = json.loads(args.config.read_text())
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output / "config.snapshot.json").exists() or (output / "metrics.json").exists():
        raise FileExistsError("Do not overwrite or silently repeat a stage")
    try:
        torch.set_num_threads(1)
        runtime = environment()
        root = Path(__file__).resolve().parents[2]
        if args.stage != "check":
            if args.prerequisite_dir is None:
                raise ValueError("Explicit prerequisite directory required")
            previous = json.loads((args.prerequisite_dir / "metrics.json").read_text())
            previous_config = json.loads((args.prerequisite_dir / "config.snapshot.json").read_text())
            if previous.get("passed") is not True or previous_config != config:
                raise ValueError("Prerequisite did not pass or configuration changed")
        write_json(output / "config.snapshot.json", config)
        if args.stage == "check":
            result = precheck(root, config, output)
        elif args.stage == "train":
            result = train(root, config, output)
        else:
            if args.training_dir is None or args.training_dir.resolve() != args.prerequisite_dir.resolve():
                raise ValueError("Evaluation requires the explicitly passed training stage")
            result = evaluate(root, config, output, args.training_dir)
        result.update({"environment": runtime, "completed_at_utc": datetime.now(timezone.utc).isoformat()})
        write_json(output / "metrics.json", result)
        print(json.dumps({"stage": args.stage, "passed": True, "output_dir": str(output)}))
        return 0
    except Exception as error:
        write_json(output / "failure.json", {"stage": args.stage, "type": type(error).__name__, "message": str(error)})
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
