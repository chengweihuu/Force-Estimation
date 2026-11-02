"""Frozen-component teacher diagnosis on existing no-contact moving records."""
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
from diagnosis import COLORS, COMPONENTS, LABELS, UNITS, choose_components, dynamic_arrays, plotting, run_kf, save_figure
from motion_evaluation import load_record, window_mask
from simulation import error_statistics, write_json
from structured import load_network


def native_regression(root, config, case, measurement, truth, learned):
    differences = {}
    for name, mask, filename in (("known", 7, "known.npz"), ("learned", 0, "trained.npz")):
        terms = choose_components(truth, learned, mask)
        estimate, _, _, _ = run_kf(measurement, terms, config["observer"], terms["M"])
        with np.load(root / config["native_replay"] / case / filename, allow_pickle=False) as previous:
            count = len(measurement["time_s"])
            np.testing.assert_array_equal(measurement["time_s"], previous["time_s"][:count])
            np.testing.assert_allclose(estimate, previous["wrench"][:count], rtol=config["comparison_rtol"],
                                       atol=config["comparison_atol"], err_msg=f"Native-R regression: {case}/{name}")
            differences[name] = float(np.max(np.abs(estimate - previous["wrench"][:count])))
    return differences


def run(root, config, output, check_only):
    torch.set_num_threads(1)
    runtime = environment()
    network = load_network(root / config["trained_checkpoint"])
    dynamic_config = {"model": config["model"], "replay_config": {"derivative_epsilon_s": config["derivative_epsilon_s"]}}
    summaries, motion, regression, figures = {}, {}, {}, []
    max_covariance_difference = 0.
    for specification in config["records"]:
        case = specification["id"]
        measurement, motion[case] = load_record(root, config, specification)
        if check_only:
            measurement = {key: value[:config["prefix_check_frames"]] for key, value in measurement.items()}
        truth, learned = dynamic_arrays(root, dynamic_config, network, measurement)
        for mask, source in ((0, learned), (7, truth)):
            selected = choose_components(truth, learned, mask)
            for key in ("M", "Mdot", "c", "G", "friction"):
                np.testing.assert_array_equal(selected[key], source[key])
        regression[case] = native_regression(root, config, case, measurement, truth, learned)
        common_covariance = None
        summaries[case], curves = {}, {}
        if not check_only:
            case_output = output / case
            case_output.mkdir(exist_ok=False)
        for specification in config["variants"]:
            name, mask = specification["id"], specification["mask"]
            terms = choose_components(truth, learned, mask)
            estimate, innovation, covariance, difference = run_kf(
                measurement, terms, config["observer"], truth["M"], common_covariance)
            if common_covariance is None:
                common_covariance = covariance
            max_covariance_difference = max(max_covariance_difference, difference)
            if not check_only:
                reference = np.tile(np.asarray(motion[case]["reference_wrench"]), (len(estimate), 1))
                metrics = {window: error_statistics(estimate, reference, window_mask(measurement["time_s"], rule))
                           for window, rule in config["windows"].items()}
                summaries[case][name] = metrics
                curves[name] = estimate
                np.savez_compressed(case_output / (name + ".npz"), time_s=measurement["time_s"],
                                    wrench=estimate, innovation=innovation,
                                    covariance_diagonal=np.diagonal(covariance, axis1=1, axis2=2))
        if not check_only:
            figures += plot_curves(output, case, measurement["time_s"], curves, config)
        print(json.dumps({"trajectory": case, "mode": "check" if check_only else "full", "variants": 8}), flush=True)
    if not check_only:
        figures += plot_bars(output, summaries, config)
        write_report(output, summaries, motion, regression, config)
    return {"passed": True, "environment": runtime, "trajectory_count": len(summaries),
            "variant_count": len(config["variants"]), "training_updates": 0,
            "shared_covariance_max_abs_difference": max_covariance_difference,
            "native_R_regression": regression, "motion": motion, "summaries": summaries, "figures": figures,
            "scope": "Frozen moving no-contact teacher diagnosis; common oracle R, no training or physical stepping"}


def plot_curves(output, case, t, curves, config):
    plt = plotting()
    files = []
    for window in ("full", "core_motion"):
        mask = window_mask(t, config["windows"][window])
        figure, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
        for index, axis in enumerate(axes.flat):
            axis.axhline(0, color="black", linestyle="--", linewidth=1., label="No external wrench")
            for name in config["main_variants"]:
                axis.plot(t[mask], curves[name][mask, index], color=COLORS[name], linewidth=.9, label=LABELS[name])
            axis.set_ylabel(f"{COMPONENTS[index]} [{UNITS[index]}]")
            axis.grid(alpha=.25)
        axes[0, 0].legend(fontsize=7)
        for axis in axes[-1]:
            axis.set_xlabel("Original simulation timestamp [s]")
        figure.suptitle(f"{case} / {window}: frozen component substitutions, common covariance", fontsize=12)
        files += save_figure(figure, output, f"{case}_{window}_components")
    return files


def plot_bars(output, summaries, config):
    plt = plotting()
    files = []
    for statistic, title in (("rmse", "RMSE"), ("absolute_error_peak", "Absolute peak")):
        figure, axes = plt.subplots(2, 2, figsize=(12, 8))
        for row, (case, variants) in enumerate(summaries.items()):
            for column, start in enumerate((0, 3)):
                axis = axes[row, column]
                x, width = np.arange(3), .15
                for group, name in enumerate(config["main_variants"]):
                    values = variants[name]["core_motion"][statistic][start:start + 3]
                    axis.bar(x + (group - 2) * width, values, width, color=COLORS[name], label=LABELS[name])
                axis.set_xticks(x, COMPONENTS[start:start + 3])
                axis.set_ylabel(f"{title} [{'N' if column == 0 else 'Nm'}]")
                axis.set_title(case + " / core motion [1,5) s")
                axis.grid(axis="y", alpha=.25)
        axes[0, 0].legend(fontsize=7)
        figure.suptitle(f"Moving no-contact component diagnosis: {title}, shared covariance", fontsize=12)
        files += save_figure(figure, output, "core_motion_" + statistic + "_components")
    return files


def write_report(output, summaries, motion, regression, config):
    with (output / "all_variants.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["trajectory", "variant", "covariance", "window", "samples", "component", "unit", "rmse", "bias", "absolute_error_peak"])
        for case, variants in summaries.items():
            for name, metrics in variants.items():
                for window, values in metrics.items():
                    for index, component in enumerate(COMPONENTS):
                        writer.writerow([case, name, "shared_oracle_R", window, values["samples"], component, UNITS[index],
                                         values["rmse"][index], values["bias"][index], values["absolute_error_peak"][index]])
    write_json(output / "motion-summary.json", motion)
    lines = ["# Moving-trace frozen component diagnosis", "",
             "EXPLORE offline teacher diagnosis. Same frozen weights, measurements and fixed [1,5) s core as the preceding moving-trace evaluation.",
             "All variants run continuously from t=0, with identical J, Q, known-model R(t) and initial covariance. No reset, timestamp shift or velocity filtering.",
             "M, Mdot and c are switched as one consistent inertia bundle; gravity and friction are independent switches. All eight combinations are retained.",
             "Common-covariance oracle substitutions are diagnostic interventions, not a deployable performance ranking. Single replacements can interact or cancel, so effects are not additive.",
             "No independent acceleration record is available or reconstructed for this analysis; the KF never receives acceleration or force truth.", "",
             "## Main core-motion comparisons", "",
             "| Trajectory | Variant | Fx RMSE [N] | Fy RMSE [N] | Fz RMSE [N] | Fx peak [N] |", "|---|---|---:|---:|---:|---:|"]
    for case, variants in summaries.items():
        for name in config["main_variants"]:
            m = variants[name]["core_motion"]
            lines.append(f"| {case} | {LABELS[name]} | {m['rmse'][0]:.6g} | {m['rmse'][1]:.6g} | {m['rmse'][2]:.6g} | {m['absolute_error_peak'][0]:.6g} |")
    lines += ["", "## Figures", "", "![Core RMSE](core_motion_rmse_components.png)", "",
              "![Core peaks](core_motion_absolute_error_peak_components.png)", ""]
    for case in summaries:
        lines += [f"![{case} core-motion curves]({case}_core_motion_components.png)", ""]
    lines += ["Every figure has PNG and SVG versions. Full six-second traces are retained as well.",
              "CSV and metrics.json contain all eight combinations, both records, all six directions and all five fixed windows.",
              "The two local trajectories were already analyzed; they are not untouched holdout or independent statistical replicates.", "",
              "## Native-R endpoint regression maximum differences", "", "```json", json.dumps(regression, indent=2), "```", "",
              "Native-R endpoint regressions reproduce prior outputs; do not silently mix common-R diagnostics with prior native-R metrics."]
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
    if [x["mask"] for x in config["variants"]] != list(range(8)):
        raise ValueError("Use all eight predetermined component combinations")
    if [x["id"] for x in config["records"]] != ["trajectory_04", "trajectory_05"]:
        raise ValueError("Use exactly the two declared existing moving records")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output / "config.snapshot.json").exists() or (output / "metrics.json").exists():
        raise FileExistsError("Do not overwrite or silently repeat the diagnostic")
    try:
        if args.stage == "replay":
            if args.prerequisite_dir is None:
                raise ValueError("Require the explicit precheck directory")
            previous = json.loads((args.prerequisite_dir / "metrics.json").read_text())
            if previous.get("passed") is not True or json.loads((args.prerequisite_dir / "config.snapshot.json").read_text()) != config:
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
