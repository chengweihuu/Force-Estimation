"""Native-KF no-contact moving-trace evaluation with predetermined windows."""
import argparse
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
from time import perf_counter
import traceback

for _name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_name] = "1"

import numpy as np
import torch

from checks import environment
from dynamics import KnownDynamics
from observer import Measurement, MomentumKF
from simulation import error_statistics, write_json
from structured import LearnedDynamics, load_network


NAMES = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
UNITS = ["N"] * 3 + ["Nm"] * 3
LABELS = {"known": "Known model", "initialized": "Before training", "trained": "Frozen trained model"}
COLORS = {"known": "#0077b6", "initialized": "#888888", "trained": "#d1495b"}


def window_mask(t, specification):
    right = t <= specification["end_s"] if specification["right_closed"] else t < specification["end_s"]
    return (t >= specification["start_s"]) & right


def load_record(root, config, specification):
    manifest = json.loads((root / config["manifest"]).read_text())
    np.testing.assert_array_equal(manifest["external_wrench"], np.zeros(6))
    matches = [x for x in manifest["trajectories"] if x["id"] == specification["id"]]
    if len(matches) != 1 or matches[0]["split"] != specification["split"] or matches[0]["split"] == "train":
        raise ValueError("Moving record is not the declared trajectory outside gradient training")
    if Path(specification["path"]).stem != specification["id"]:
        raise ValueError("Trajectory filename/identity mismatch")
    with np.load(root / specification["path"], allow_pickle=False) as f:
        if set(f.files) != {"time_s", "position_rad", "velocity_rad_s", "drive_torque_Nm"}:
            raise ValueError("Measurement packet must contain exactly four declared fields")
        record = {key: f[key].copy() for key in f.files}
    expected = round(config["duration_s"] / config["sample_period_s"]) + 1
    t = record["time_s"]
    if t.shape != (expected,) or matches[0]["samples"] != expected:
        raise ValueError("Record length differs from fixed schedule")
    np.testing.assert_allclose(t, np.arange(expected) * config["sample_period_s"], rtol=0, atol=1e-12)
    for name in ("position_rad", "velocity_rad_s", "drive_torque_Nm"):
        if record[name].shape != (expected, 6) or not np.isfinite(record[name]).all():
            raise ValueError(f"Invalid recorded {name}")
    masks = {name: window_mask(t, value) for name, value in config["windows"].items()}
    if any(not mask.any() for mask in masks.values()):
        raise ValueError("Empty predeclared scoring interval")
    partition = sum(mask.astype(int) for name, mask in masks.items() if name != "full")
    np.testing.assert_array_equal(partition, np.ones(expected))
    core = masks["core_motion"]
    q, v = record["position_rad"][core], record["velocity_rad_s"][core]
    peak_speed = np.max(np.abs(v), axis=0)
    if not np.all(peak_speed > 0):
        raise ValueError("A declared moving joint has no recorded motion in the fixed core interval")
    summary = {"id": specification["id"], "split": specification["split"], "samples": expected,
               "window_samples": {name: int(mask.sum()) for name, mask in masks.items()},
               "core_q_span_rad": np.ptp(q, axis=0).tolist(), "core_max_abs_velocity_rad_s": peak_speed.tolist(),
               "core_velocity_rms_rad_s": np.sqrt(np.mean(v**2, axis=0)).tolist(),
               "reference_wrench": manifest["external_wrench"],
               "source_status": "Excluded from gradient updates; previously used for development or dynamics diagnostics"}
    return record, summary


def replay_one(root, config, record, networks, name, prefix=None):
    nominal = (KnownDynamics(root / config["model"], config["derivative_epsilon_s"])
               if name == "known" else LearnedDynamics(networks[name], root / config["model"]))
    observer = MomentumKF(nominal, config["observer"])
    count = len(record["time_s"]) if prefix is None else prefix
    estimate, innovation, diagonal = [], [], []
    minimum_covariance, minimum_jacobian = float("inf"), float("inf")
    started = perf_counter()
    for index in range(count):
        result = observer.update(Measurement(float(record["time_s"][index]), record["position_rad"][index],
                                             record["velocity_rad_s"][index], record["drive_torque_Nm"][index]))
        estimate.append(result["wrench"])
        innovation.append(result["innovation"])
        diagonal.append(np.diag(result["covariance"]))
        minimum_covariance = min(minimum_covariance, result["covariance_min_eigenvalue"])
        minimum_jacobian = min(minimum_jacobian, float(result["jacobian_singular_values"][-1]))
    return {"time_s": record["time_s"][:count].copy(), "wrench": np.asarray(estimate),
            "innovation": np.asarray(innovation), "covariance_diagonal": np.asarray(diagonal)}, {
                "processed_frames": count, "wall_time_s": perf_counter() - started,
                "covariance_min_eigenvalue": minimum_covariance, "jacobian_min_singular_value": minimum_jacobian,
                "initializations": 1, "resets_at_core_start": 0,
                "R_rule": "Native R=Mhat*Sigma_v*Mhat.T; actual R varies across models"}


def run(root, config, output, check_only):
    torch.set_num_threads(1)
    runtime = environment()
    networks = {"initialized": load_network(root / config["initialized_checkpoint"]),
                "trained": load_network(root / config["trained_checkpoint"])}
    summaries, motion, figures, recorded = {}, {}, [], {}
    for specification in config["records"]:
        name = specification["id"]
        record, motion[name] = load_record(root, config, specification)
        recorded[name] = record
        summaries[name], curves = {}, {}
        if not check_only:
            case_output = output / name
            case_output.mkdir(exist_ok=False)
        for model_name in config["models"]:
            arrays, diagnostics = replay_one(root, config, record, networks, model_name,
                                             config["prefix_check_frames"] if check_only else None)
            if check_only:
                summaries[name][model_name] = diagnostics
            else:
                # Reference is the source collection's explicitly zero external wrench.
                reference = np.tile(np.asarray(motion[name]["reference_wrench"]), (len(record["time_s"]), 1))
                metrics = {window: error_statistics(arrays["wrench"], reference, window_mask(arrays["time_s"], rule))
                           for window, rule in config["windows"].items()}
                metrics["diagnostics"] = diagnostics
                summaries[name][model_name] = metrics
                curves[model_name] = arrays["wrench"]
                np.savez_compressed(case_output / (model_name + ".npz"), **arrays)
                write_json(case_output / (model_name + "-metrics.json"), metrics)
            print(json.dumps({"trajectory": name, "model": model_name,
                              "mode": "check" if check_only else "full"}), flush=True)
        if not check_only:
            figures += plot_curves(output, name, record, curves, config)
    if not check_only:
        figures += plot_motion(output, recorded, config)
        figures += plot_bars(output, summaries, config)
        write_tables(output, summaries, motion, config)
    return {"passed": True, "environment": runtime, "trajectory_count": len(summaries),
            "model_count": len(config["models"]), "training_updates": 0,
            "motion": motion, "summaries": summaries, "figures": figures,
            "scope": "Frozen-model moving no-contact regression; native R; no training, physical stepping or holdout claim"}


def plotting():
    import matplotlib
    matplotlib.use("Agg")
    matplotlib.rcParams.update({"font.size": 10, "svg.fonttype": "none"})
    import matplotlib.pyplot as plt
    return plt


def save_figure(figure, output, name):
    figure.tight_layout()
    result = []
    for extension in ("png", "svg"):
        filename = name + "." + extension
        figure.savefig(output / filename, dpi=180)
        result.append(filename)
    plotting().close(figure)
    return result


def plot_curves(output, name, record, curves, config):
    plt = plotting()
    t = record["time_s"]
    files = []
    for window in ("full", "core_motion"):
        mask = window_mask(t, config["windows"][window])
        figure, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
        for index, axis in enumerate(axes.flat):
            axis.axhline(0, color="black", linestyle="--", linewidth=1., label="No external wrench")
            for model in config["models"]:
                axis.plot(t[mask], curves[model][mask, index], linewidth=.9, color=COLORS[model], label=LABELS[model])
            axis.set_ylabel(f"{NAMES[index]} [{UNITS[index]}]")
            axis.grid(alpha=.25)
        axes[0, 0].legend(fontsize=7)
        for axis in axes[-1]:
            axis.set_xlabel("Original simulation timestamp [s]")
        figure.suptitle(f"{name}: no-contact motion / {window}; continuous KF from t=0, native R", fontsize=12)
        files += save_figure(figure, output, f"{name}_{window}_wrench")
    return files


def plot_motion(output, records, config):
    plt = plotting()
    figure, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    core = config["windows"]["core_motion"]
    for row, (name, record) in enumerate(records.items()):
        t = record["time_s"]
        for joint in range(6):
            axes[row, 0].plot(t, record["position_rad"][:, joint] - record["position_rad"][0, joint], label=f"J{joint+1}")
            axes[row, 1].plot(t, record["velocity_rad_s"][:, joint], label=f"J{joint+1}")
        axes[row, 0].set_ylabel(f"{name}: q-q(0) [rad]")
        axes[row, 1].set_ylabel("Recorded velocity [rad/s]")
        for axis in axes[row]:
            axis.axvspan(core["start_s"], core["end_s"], color="#dddddd", alpha=.3)
            axis.grid(alpha=.25)
    axes[0, 0].legend(ncol=3, fontsize=8)
    for axis in axes[-1]:
        axis.set_xlabel("Original simulation timestamp [s]")
    figure.suptitle("Actual recorded no-contact motion; shaded core scoring window [1,5) s")
    return save_figure(figure, output, "recorded_motion")


def plot_bars(output, summaries, config):
    plt = plotting()
    files = []
    for statistic, title in (("rmse", "RMSE"), ("absolute_error_peak", "Absolute peak")):
        figure, axes = plt.subplots(2, 2, figsize=(12, 7))
        for row, (name, variants) in enumerate(summaries.items()):
            for column, start in enumerate((0, 3)):
                axis = axes[row, column]
                x, width = np.arange(3), .24
                for group, model in enumerate(config["models"]):
                    values = variants[model]["core_motion"][statistic][start:start+3]
                    axis.bar(x + (group-1)*width, values, width, color=COLORS[model], label=LABELS[model])
                axis.set_xticks(x, NAMES[start:start+3])
                axis.set_ylabel(f"{title} [{'N' if column==0 else 'Nm'}]")
                axis.set_title(name + " / core motion [1,5) s")
                axis.grid(axis="y", alpha=.25)
        axes[0, 0].legend(fontsize=8)
        figure.suptitle("Moving no-contact false wrench: " + title + "; no state reset or timestamp shift")
        files += save_figure(figure, output, "core_motion_" + statistic + "_bars")
    return files


def write_tables(output, summaries, motion, config):
    with (output / "all_windows.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["trajectory", "model", "window", "samples", "component", "unit", "rmse", "bias", "absolute_error_peak"])
        for case, variants in summaries.items():
            for model, metrics in variants.items():
                for window in config["windows"]:
                    m = metrics[window]
                    for index, component in enumerate(NAMES):
                        writer.writerow([case, model, window, m["samples"], component, UNITS[index],
                                         m["rmse"][index], m["bias"][index], m["absolute_error_peak"][index]])
    write_json(output / "motion-summary.json", motion)
    lines = ["# Frozen model: moving no-contact false-wrench evaluation", "",
             "Two existing local Fourier trajectories, excluded from gradient updates but previously used for development or dynamics diagnostics. This is EXPLORE regression, not untouched holdout.",
             "Known, initialized and trained models use the same native MomentumKF parameters and covariance rule; actual R depends on each inertia matrix.",
             "Every KF starts at t=0 and runs continuously to 6 s. Scoring the core does not reset or realign the filter.",
             "Core [1,5) s includes all frames, including accelerations and reversals, without velocity-threshold selection.", "",
             "## Core motion RMSE", "",
             "| Trajectory | Model | Fx [N] | Fy [N] | Fz [N] | Mx [Nm] | My [Nm] | Mz [Nm] |", "|---|---|---:|---:|---:|---:|---:|---:|"]
    for case, variants in summaries.items():
        for model, metrics in variants.items():
            lines.append(f"| {case} | {LABELS[model]} | " + " | ".join(f"{x:.6g}" for x in metrics["core_motion"]["rmse"]) + " |")
    lines += ["", "## Figures", "", "![Actual recorded motion](recorded_motion.png)", "",
              "![Core RMSE](core_motion_rmse_bars.png)", "", "![Core peaks](core_motion_absolute_error_peak_bars.png)", ""]
    for case in summaries:
        lines += [f"![{case} core motion]({case}_core_motion_wrench.png)", ""]
    lines += ["Each figure also has an editable SVG. Full six-second traces preserve startup/shutdown behavior.",
              "All five predetermined windows and all directions have RMSE, bias and peaks in all_windows.csv and metrics.json.",
              "No retraining, physical stepping, force feedback or teacher component replacement was performed.",
              "Small local motions and two existing records do not establish generalization across the robot workspace or noisy real feedback."]
    (output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("check", "replay"), required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--prerequisite-dir", type=Path)
    args = parser.parse_args()
    if not Path("/.dockerenv").is_file():
        parser.error("Execute inside the approved Docker service")
    config = json.loads(args.config.read_text())
    if [x["id"] for x in config["records"]] != ["trajectory_04", "trajectory_05"] or config["models"] != ["known", "initialized", "trained"]:
        raise ValueError("Preserve the declared two records and three models")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output / "config.snapshot.json").exists() or (output / "metrics.json").exists():
        raise FileExistsError("Do not overwrite or silently repeat a motion evaluation")
    try:
        if args.stage == "replay":
            if args.prerequisite_dir is None:
                raise ValueError("Require the explicit precheck directory")
            prerequisite = json.loads((args.prerequisite_dir / "metrics.json").read_text())
            if prerequisite.get("passed") is not True or json.loads((args.prerequisite_dir / "config.snapshot.json").read_text()) != config:
                raise ValueError("Precheck did not pass or config changed")
        write_json(output / "config.snapshot.json", config)
        metrics = run(Path(__file__).resolve().parents[2], config, output, args.stage == "check")
        metrics["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
        write_json(output / "metrics.json", metrics)
        print(json.dumps({"stage": args.stage, "passed": True, "output_dir": str(output)}))
        return 0
    except Exception as error:
        write_json(output / "failure.json", {"stage": args.stage, "type": type(error).__name__,
                                            "message": str(error), "traceback": traceback.format_exc(), "automatic_retry": False})
        print(str(error), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
