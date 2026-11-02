"""Frozen paired-model component swaps and exact shared-KF error accounting."""
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
from diagnosis import dynamic_arrays, run_kf
from dynamics import KnownDynamics
from motion_evaluation import NAMES, UNITS, load_record, plotting, save_figure, window_mask
from simulation import error_statistics, write_json
from structured import load_network


MODELS = ("baseline", "static_augmented")
COLORS = {"baseline": "#d1495b", "static_augmented": "#0077b6", "known": "#555555",
          "swap_inertia": "#8e6cbb", "swap_gravity": "#159895", "swap_friction": "#ee9b00"}


def select_terms(sources, arrays):
    inertia, gravity, friction = (arrays[source] for source in sources)
    return {"M": inertia["M"], "Mdot": inertia["Mdot"], "c": inertia["c"],
            "G": gravity["G"], "friction": friction["friction"], "J": arrays["known"]["J"]}


def compare(actual, expected, config, name):
    np.testing.assert_allclose(actual, expected, rtol=config["comparison_rtol"],
                               atol=config["comparison_atol"], err_msg=name)
    return float(np.max(np.abs(actual - expected)))


def signals_and_checks(config, curves):
    index = {tuple(spec["sources"]): curves[spec["id"]] for spec in config["variants"]}
    known = index[("known",) * 3]
    signals, max_difference = {}, 0.
    for model in MODELS:
        parts = [known]
        for component in range(3):
            sources = ["known"] * 3
            sources[component] = model
            parts.append(index[tuple(sources)] - known)
        signals[model] = np.stack(parts, axis=1)  # time, source, wrench component
        max_difference = max(max_difference, compare(signals[model].sum(axis=1), index[(model,) * 3],
                                                      config, model + " additive error signals"))
    delta = signals["static_augmented"][:, 1:] - signals["baseline"][:, 1:]
    baseline = index[("baseline",) * 3]
    for spec in config["variants"]:
        if spec["family"] != "hybrid":
            continue
        reconstructed = baseline.copy()
        for component, source in enumerate(spec["sources"]):
            if source == "static_augmented":
                reconstructed += delta[:, component]
        max_difference = max(max_difference, compare(reconstructed, curves[spec["id"]], config,
                                                      spec["id"] + " hybrid reconstruction"))
    return signals, max_difference


def mse_accounting(signals, curves, mask, config):
    result = {}
    for model in MODELS:
        selected = signals[model][mask]
        matrix = np.einsum("tic,tjc->cij", selected, selected) / len(selected)
        mse = np.mean(curves[model][mask] ** 2, axis=0)
        compare(matrix.sum(axis=(1, 2)), mse, config, model + " MSE cross-term accounting")
        diagonal = np.diagonal(matrix, axis1=1, axis2=2)
        self_sum = diagonal.sum(axis=1)
        result[model] = {"signal_names": config["signal_names"], "samples": int(mask.sum()),
                         "per_axis_units": [unit + "^2" for unit in UNITS],
                         "correlation_matrix": matrix.tolist(), "component_rms": np.sqrt(diagonal).tolist(),
                         "self_terms_sum": self_sum.tolist(), "cross_terms_sum": (mse - self_sum).tolist(),
                         "total_mse": mse.tolist()}
    return result


def raw_diagnostics(root, config, arrays, networks, mask):
    known = arrays["known"]
    result = {"known_friction_coefficients_Nms_rad": KnownDynamics(
        root / config["model"], config["derivative_epsilon_s"]).model.dof_damping.tolist()}
    denominator = np.linalg.norm(known["M"][mask], axis=(1, 2))
    for model in MODELS:
        values = arrays[model]
        result[model] = {
            "mass_relative_Frobenius_mean": float(np.mean(np.linalg.norm(values["M"][mask] - known["M"][mask], axis=(1, 2)) / denominator)),
            "mass_dot_Frobenius_error_mean": float(np.mean(np.linalg.norm(values["Mdot"][mask] - known["Mdot"][mask], axis=(1, 2)))),
            "friction_coefficients_Nms_rad": torch.nn.functional.softplus(networks[model].friction_raw).detach().tolist()}
        for key in ("c", "G", "friction"):
            error = values[key][mask] - known[key][mask]
            result[model][key + "_rmse_Nm"] = np.sqrt(np.mean(error ** 2, axis=0)).tolist()
    return result


def run(root, config, output, check_only):
    torch.set_num_threads(1)
    if len(config["variants"]) != 15 or len({tuple(v["sources"]) for v in config["variants"]}) != 15:
        raise ValueError("Require 15 unique declared component combinations")
    if len(config["records"]) != 1:
        raise ValueError("Use only the single declared diagnostic record")
    record, metadata = load_record(root, config, config["records"][0])
    if check_only:
        record = {key: value[:config["prefix_check_frames"]] for key, value in record.items()}
    networks = {name: load_network(root / path) for name, path in config["checkpoints"].items()}
    arrays = {}
    dynamic_config = {"model": config["model"], "replay_config": {"derivative_epsilon_s": config["derivative_epsilon_s"]}}
    for model in MODELS:
        known, arrays[model] = dynamic_arrays(root, dynamic_config, networks[model], record)
        if "known" in arrays:
            for key in known:
                np.testing.assert_array_equal(known[key], arrays["known"][key])
        else:
            arrays["known"] = known
    regression, native = {}, {}
    for model in MODELS:
        terms = select_terms([model] * 3, arrays)
        values, _, _, _ = run_kf(record, terms, config["observer"], terms["M"])
        with np.load(root / config["native_reference"][model], allow_pickle=False) as previous:
            count = len(record["time_s"])
            np.testing.assert_array_equal(record["time_s"], previous["time_s"][:count])
            regression[model] = compare(values, previous["wrench"][:count], config, model + " native R regression")
        native[model] = values
    curves, summaries, common_covariance = {}, {}, None
    max_covariance_difference = 0.
    for spec in config["variants"]:
        terms = select_terms(spec["sources"], arrays)
        values, innovation, covariance, difference = run_kf(record, terms, config["observer"],
                                                         arrays["known"]["M"], common_covariance)
        if common_covariance is None:
            common_covariance = covariance
        max_covariance_difference = max(max_covariance_difference, difference)
        name = spec["id"]
        curves[name] = values
        if not check_only:
            summaries[name] = {window: error_statistics(values, np.zeros_like(values), window_mask(record["time_s"], rule))
                               for window, rule in config["windows"].items()}
            np.savez_compressed(output / (name + ".npz"), time_s=record["time_s"], wrench=values,
                                innovation=innovation, covariance_diagonal=np.diagonal(covariance, axis1=1, axis2=2))
    signals, reconstruction_difference = signals_and_checks(config, curves)
    accounting, raw, bridge, figure_paths = {}, {}, {}, []
    if not check_only:
        core = window_mask(record["time_s"], config["windows"]["core_motion"])
        accounting = mse_accounting(signals, curves, core, config)
        raw = raw_diagnostics(root, config, arrays, networks, core)
        for model in MODELS:
            bridge[model] = {
                "native_core": error_statistics(native[model], np.zeros_like(native[model]), core),
                "shared_core": summaries[model]["core_motion"],
                "native_vs_shared_wrench_max_abs": float(np.max(np.abs(native[model] - curves[model])))}
            np.savez_compressed(output / (model + "_signals.npz"), time_s=record["time_s"],
                                signals=signals[model], signal_names=config["signal_names"], native_wrench=native[model])
        figure_paths = plots(output, record["time_s"], config, curves, summaries, accounting)
        write_report(output, config, summaries, accounting, raw, bridge, figure_paths)
    return {"passed": True, "training_updates": 0, "trajectory_count": 1, "variant_count": len(curves),
            "processed_frames": len(record["time_s"]), "native_R_regression": regression,
            "shared_covariance_max_abs_difference": max_covariance_difference,
            "signal_reconstruction_max_abs_difference": reconstruction_difference,
            "motion": metadata, "summaries": summaries, "mse_accounting": accounting,
            "raw_core_dynamics": raw, "native_vs_shared": bridge, "figures": figure_paths,
            "scope": "Frozen paired-model diagnostic with common known covariance; non-deployable, previously analyzed record"}


def plots(output, t, config, curves, summaries, accounting):
    plt = plotting()
    paths = []
    labels = {v["id"]: v["label"] for v in config["variants"]}
    fig, ax = plt.subplots(figsize=(12, 4.5))
    ids = config["hybrid_ids"]
    values = [summaries[name]["core_motion"]["rmse"][0] for name in ids]
    ax.bar(np.arange(len(ids)), values, color=[COLORS.get(name, "#9ca3af") for name in ids])
    ax.set(xticks=np.arange(len(ids)), xticklabels=[labels[name] for name in ids],
           ylabel="Fx RMSE [N]", title="Trajectory 05: replace baseline components with augmented components; common R")
    for index, value in enumerate(values):
        ax.text(index, value, f"{value:.3f}", ha="center", va="bottom", fontsize=9)
    ax.tick_params(axis="x", labelrotation=15)
    ax.grid(axis="y", alpha=.25)
    paths += save_figure(fig, output, "core_Fx_hybrid_comparison")

    fig, axes = plt.subplots(3, 2, figsize=(11, 9))
    for axis, ax in enumerate(axes.flat):
        for offset, model in enumerate(MODELS):
            ax.bar(np.arange(3) + (offset - .5) * .35, accounting[model]["component_rms"][axis][1:], .35,
                   color=COLORS[model], label=model)
        ax.set(title=NAMES[axis], xticks=np.arange(3), xticklabels=["Inertia", "Gravity", "Friction"],
               ylabel=f"Isolated error-signal RMS [{UNITS[axis]}]")
        ax.grid(axis="y", alpha=.25)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Shared-KF error transfer relative to known dynamics; signals can cancel")
    paths += save_figure(fig, output, "core_component_error_signals")

    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    limit = max(np.max(np.abs(accounting[model]["correlation_matrix"][0])) for model in MODELS)
    limit = max(float(limit), np.finfo(float).eps)
    for model, ax in zip(MODELS, axes):
        matrix = np.asarray(accounting[model]["correlation_matrix"])[0]
        im = ax.imshow(matrix, cmap="RdBu_r", vmin=-limit, vmax=limit)
        ax.set(xticks=np.arange(4), yticks=np.arange(4), xticklabels=["Known", "Inertia", "Gravity", "Friction"],
               yticklabels=["Known", "Inertia", "Gravity", "Friction"],
               title=f"{model}\nFx MSE={accounting[model]['total_mse'][0]:.4f} N²")
        for row in range(4):
            for column in range(4):
                ax.text(column, row, f"{matrix[row,column]:+.3g}", ha="center", va="center", fontsize=8,
                        color="white" if abs(matrix[row,column]) > .6 * limit else "black")
        fig.colorbar(im, ax=ax, shrink=.7, label="mean(signal_i × signal_j) [N²]")
    fig.suptitle("Fx error accounting: sum every cell for MSE; off-diagonal pairs occur twice")
    paths += save_figure(fig, output, "core_Fx_mse_cross_terms")

    for window in ("full", "core_motion"):
        mask = window_mask(t, config["windows"][window])
        names = ["baseline", "static_augmented", "known"]
        if window == "core_motion":
            names = ["baseline", "static_augmented", "swap_inertia", "swap_gravity", "swap_friction", "known"]
        fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
        for component, ax in enumerate(axes.flat):
            ax.axhline(0, color="black", linestyle=":", linewidth=.7)
            for name in names:
                ax.plot(t[mask], curves[name][mask, component], color=COLORS[name], label=labels[name], linewidth=.9)
            ax.set_ylabel(f"{NAMES[component]} [{UNITS[component]}]")
            ax.grid(alpha=.25)
        axes[0, 0].legend(ncol=2, fontsize=7)
        for ax in axes[-1]:
            ax.set_xlabel("Original simulation time [s]")
        fig.suptitle("Trajectory 05 / " + window + ": frozen components, common covariance")
        paths += save_figure(fig, output, window + "_wrench_components")
    return paths


def write_report(output, config, summaries, accounting, raw, bridge, figure_paths):
    specs = {spec["id"]: spec for spec in config["variants"]}
    with (output / "all_variants.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["variant", "inertia_source", "gravity_source", "friction_source", "covariance", "window", "samples",
                         "component", "unit", "rmse", "bias", "absolute_error_peak"])
        for name, windows in summaries.items():
            for window, values in windows.items():
                for axis in range(6):
                    writer.writerow([name, *specs[name]["sources"], "common_known_R", window, values["samples"],
                                     NAMES[axis], UNITS[axis], values["rmse"][axis], values["bias"][axis], values["absolute_error_peak"][axis]])
    with (output / "component_mse.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["model", "component", "unit", "signal_i", "signal_j", "mean_product"])
        for model, values in accounting.items():
            for axis in range(6):
                for i, first in enumerate(config["signal_names"]):
                    for j, second in enumerate(config["signal_names"]):
                        writer.writerow([model, NAMES[axis], UNITS[axis] + "^2", first, second,
                                         values["correlation_matrix"][axis][i][j]])
    with (output / "component_rms.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["model", "component", "unit", "signal", "rms"])
        for model, values in accounting.items():
            for axis in range(6):
                for i, signal in enumerate(config["signal_names"]):
                    writer.writerow([model, NAMES[axis], UNITS[axis], signal, values["component_rms"][axis][i]])
    lines = ["# Frozen paired-model diagnosis on trajectory 05", "",
             "EXPLORE; no retraining, tuning, integration, extra trajectories or reconstructed acceleration.",
             "All 15 combinations share known-model J, R(t), P0 and Q. Native-R endpoints are separately regressed to the preceding paired evaluation.",
             "Inertia means the consistent bundle (M,Mdot,c). Synthetic hybrids and oracle covariance are diagnostic only, not deployable alternatives.",
             "Previously analyzed trajectory; fixed core [1,5) s includes all 4000 frames. KF runs continuously from t=0 to 6 s.", "",
             "## Native versus common covariance endpoints", "", "| Model | Native Fx RMSE [N] | Common Fx RMSE [N] |",
             "|---|---:|---:|"]
    for model, value in bridge.items():
        lines.append(f"| {model} | {value['native_core']['rmse'][0]:.6g} | {value['shared_core']['rmse'][0]:.6g} |")
    lines += ["", "## Baseline-to-augmented hybrids, common covariance", "",
              "| Variant | M/Mdot/c | G | Friction | Fx RMSE [N] | Fx bias [N] |", "|---|---|---|---|---:|---:|"]
    for name in config["hybrid_ids"]:
        v = summaries[name]["core_motion"]
        lines.append(f"| {name} | " + " | ".join(specs[name]["sources"]) + f" | {v['rmse'][0]:.6g} | {v['bias'][0]:.6g} |")
    lines += ["", "## Fx component error signals and cancellation", "",
              "Each isolated signal is (one model component, all other components known) minus (all known output). These are output-error transfer signals, not true joint torque residuals.",
              "Known residual plus the three isolated signals reconstructs each shared-KF endpoint. Signal changes reconstruct all eight hybrids; these identities are numerically checked over the entire record.",
              "RMSE is not additive. For four signals z_i, C_ij=mean(z_i*z_j), MSE=sum(C). Diagonal cells are self terms; all off-diagonal cells are signed cross terms.", "",
              "| Model | Inertia RMS [N] | Gravity RMS [N] | Friction RMS [N] | Self sum [N²] | Cross sum [N²] | Total MSE [N²] |",
              "|---|---:|---:|---:|---:|---:|---:|"]
    for model, value in accounting.items():
        rms = value["component_rms"][0]
        lines.append(f"| {model} | {rms[1]:.6g} | {rms[2]:.6g} | {rms[3]:.6g} | {value['self_terms_sum'][0]:.6g} | {value['cross_terms_sum'][0]:.6g} | {value['total_mse'][0]:.6g} |")
    lines += ["", "## Raw core dynamics diagnostics", "", "```json", json.dumps(raw, indent=2), "```", "",
              "These comparisons are diagnostics, not simulator component labels used for further training. Shared hidden weights and native covariance propagation limit any universal causal interpretation.",
              "All six directions and original five windows, including deterioration, remain in metrics.json and all_variants.csv.", "", "## Figures", ""]
    for filename in figure_paths:
        if filename.endswith(".png"):
            lines += [f"![{filename}]({filename})", ""]
    (output / "report.md").write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stage", choices=("check", "diagnose"), required=True)
    parser.add_argument("--prerequisite-dir", type=Path)
    args = parser.parse_args()
    if not Path("/.dockerenv").is_file():
        parser.error("Use the existing approved Docker service")
    config = json.loads(args.config.read_text())
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output / "config.snapshot.json").exists() or (output / "metrics.json").exists():
        raise FileExistsError("Do not overwrite or silently repeat this diagnostic")
    try:
        runtime = environment()
        if args.stage == "diagnose":
            if args.prerequisite_dir is None:
                raise ValueError("Explicit prerequisite directory required")
            previous = json.loads((args.prerequisite_dir / "metrics.json").read_text())
            if previous.get("passed") is not True or json.loads((args.prerequisite_dir / "config.snapshot.json").read_text()) != config:
                raise ValueError("Prerequisite did not pass or configuration changed")
        write_json(output / "config.snapshot.json", config)
        result = run(Path(__file__).resolve().parents[2], config, output, args.stage == "check")
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
