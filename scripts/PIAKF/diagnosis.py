"""Frozen-component oracle diagnostics. This is not a deployable estimator."""
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
from dynamics import KnownDynamics
from observer import correct, covariance_check
from simulation import error_statistics, evaluate, write_json
from structured import load_network


def load_record(root, config, scenario, thin=False):
    source = root / config["source_run"] / scenario
    with np.load(source / "measurements.npz", allow_pickle=False) as f:
        measurement = {key: f[key].copy() for key in f.files}
    if set(measurement) != {"time_s", "position_rad", "velocity_rad_s", "drive_torque_Nm"}:
        raise ValueError("Measurement contains undeclared inputs")
    with np.load(source / "reference.npz", allow_pickle=False) as f:
        reference = {key: f[key].copy() for key in f.files}
    np.testing.assert_array_equal(measurement["time_s"], reference["time_s"])
    t = measurement["time_s"]
    if not np.isfinite(t).all() or not np.all(np.diff(t) > 0):
        raise ValueError("Invalid time axis")
    if thin:
        indices = np.linspace(0, len(t) - 1, config["check_samples_per_scenario"], dtype=int)
        measurement = {key: value[indices] for key, value in measurement.items()}
        reference = {key: value[indices] for key, value in reference.items()}
    return measurement, reference


def dynamic_arrays(root, config, network, measurement):
    q, v = measurement["position_rad"], measurement["velocity_rad_s"]
    known = KnownDynamics(root / config["model"], config["replay_config"]["derivative_epsilon_s"])
    truth = {key: [] for key in ("M", "Mdot", "c", "G", "friction", "J")}
    for position, velocity in zip(q, v):
        terms = known.evaluate(position, velocity)
        gravity = known.evaluate(position, np.zeros(6)).bias
        for key, value in (("M", terms.mass), ("Mdot", terms.mass_dot),
                           ("c", terms.bias - gravity), ("G", gravity),
                           ("friction", terms.friction), ("J", terms.jacobian)):
            truth[key].append(value)
    truth = {key: np.asarray(value) for key, value in truth.items()}
    prediction = {key: [] for key in ("M", "Mdot", "c", "G", "friction")}
    with torch.no_grad():
        for lo in range(0, len(q), 512):
            result = network.dynamics(torch.from_numpy(q[lo:lo + 512]), torch.from_numpy(v[lo:lo + 512]))
            for key in prediction:
                prediction[key].append(result[key].numpy())
    prediction = {key: np.concatenate(value) for key, value in prediction.items()}
    for quantities in (truth, prediction):
        if not all(np.isfinite(value).all() for value in quantities.values()):
            raise FloatingPointError("Nonfinite dynamics")
        if np.linalg.eigvalsh(quantities["M"]).min() <= 0:
            raise ValueError("Nonpositive inertia")
    return truth, prediction


def choose_components(truth, learned, mask):
    inertia = truth if mask & 1 else learned
    gravity = truth if mask & 2 else learned
    friction = truth if mask & 4 else learned
    return {"M": inertia["M"], "Mdot": inertia["Mdot"], "c": inertia["c"],
            "G": gravity["G"], "friction": friction["friction"], "J": truth["J"]}


def torque_error_decomposition(measurement, reference, truth, learned, config):
    """True a and applied torque are diagnostic references, NOT KF arguments."""
    a, v = reference["acceleration_rad_s2"], measurement["velocity_rad_s"]
    parts = {"inertia_acceleration": np.einsum("bij,bj->bi", learned["M"] - truth["M"], a),
             "velocity_term": learned["c"] - truth["c"],
             "gravity": learned["G"] - truth["G"],
             "friction": learned["friction"] - truth["friction"]}
    total = sum(parts.values())
    predicted_torque = (np.einsum("bij,bj->bi", learned["M"], a)
                        + learned["c"] + learned["G"] + learned["friction"])
    independent = (predicted_torque - measurement["drive_torque_Nm"]
                   - reference["joint_external_torque_Nm"])
    np.testing.assert_allclose(total, independent, rtol=config["comparison_rtol"],
                               atol=config["comparison_atol"], err_msg="Torque-error decomposition")
    mapped = np.einsum("bji,bj->bi", truth["J"], reference["wrench"])
    np.testing.assert_allclose(mapped, reference["joint_external_torque_Nm"],
                               rtol=config["comparison_rtol"], atol=config["comparison_atol"])
    return {**parts, "total": total}, float(np.max(np.abs(total - independent)))


def run_kf(measurement, terms, observer_config, covariance_mass, reference_covariances=None):
    """The KF sees the measurement packet and selected nominal dynamics only."""
    Qp, Qw = np.diag(observer_config["Qp_continuous_diag"]), np.diag(observer_config["Qw_continuous_diag"])
    Sv = np.diag(np.square(observer_config["assumed_velocity_std_rad_s"]))
    t, v, tau = (measurement[key] for key in ("time_s", "velocity_rad_s", "drive_torque_Nm"))
    estimates, innovations, covariances = [], [], []
    previous = None
    max_covariance_difference = 0.
    for index, time_s in enumerate(t):
        M, Md, J = terms["M"][index], terms["Mdot"][index], terms["J"][index]
        y = M @ v[index]
        R = covariance_mass[index] @ Sv @ covariance_mass[index].T
        if previous is None:
            x = np.r_[y, np.zeros(6)]
            P = np.zeros((12, 12))
            P[:6, :6], P[6:, 6:] = R, np.diag(observer_config["initial_wrench_variance"])
            innovation = np.zeros(6)
        else:
            t0, u0, J0 = previous
            x, P, innovation = correct(x, P, y, u0, J0, float(time_s - t0), Qp, Qw, R)
        covariance_check(P, observer_config["covariance_psd_atol"])
        if not np.isfinite(x).all():
            raise FloatingPointError("Nonfinite diagnostic KF state")
        if reference_covariances is not None:
            difference = float(np.max(np.abs(P - reference_covariances[index])))
            max_covariance_difference = max(max_covariance_difference, difference)
        u = Md @ v[index] + tau[index] - (terms["c"][index] + terms["G"][index]) - terms["friction"][index]
        previous = (float(time_s), u, J)
        estimates.append(x[6:].copy())
        innovations.append(innovation.copy())
        covariances.append(P.copy())
    if max_covariance_difference > observer_config["covariance_psd_atol"]:
        raise ValueError("Diagnostic variants do not share the declared covariance recursion")
    return np.asarray(estimates), np.asarray(innovations), np.asarray(covariances), max_covariance_difference


def score(t, estimate, reference, scenario, config):
    metrics = evaluate(t, estimate, reference, scenario, config["replay_config"])
    for name, (start, end) in config["windows"].items():
        metrics[name] = error_statistics(estimate, reference, (t >= start) & (t < end))
    return metrics


def run_diagnosis(root, config, output, check_only):
    torch.set_num_threads(1)
    runtime = environment()
    network = load_network(root / config["checkpoint"])
    summaries, regression, figures = {}, {}, []
    errors_max, cov_difference = 0., 0.
    for scenario in config["replay_config"]["scenarios"]:
        measurement, reference = load_record(root, config, scenario, thin=check_only)
        truth, learned = dynamic_arrays(root, config, network, measurement)
        parts, error = torque_error_decomposition(measurement, reference, truth, learned, config)
        errors_max = max(errors_max, error)
        scenario_dir = output / scenario
        scenario_dir.mkdir(exist_ok=False)
        if not check_only:
            np.savez_compressed(scenario_dir / "torque-error-components.npz", time_s=measurement["time_s"], **parts)
        endpoint = choose_components(truth, learned, 0)
        oracle = choose_components(truth, learned, 7)
        for key in ("M", "Mdot", "c", "G", "friction"):
            np.testing.assert_array_equal(endpoint[key], learned[key])
            np.testing.assert_array_equal(oracle[key], truth[key])
        common_covariances = None
        summaries[scenario], curves = {}, {}
        for specification in config["variants"]:
            name, mask = specification["id"], specification["mask"]
            terms = choose_components(truth, learned, mask)
            estimate, innovation, covariances, difference = run_kf(
                measurement, terms, config["replay_config"]["observer"], truth["M"], common_covariances)
            if common_covariances is None:
                common_covariances = covariances
            cov_difference = max(cov_difference, difference)
            if not check_only:
                curves[name] = estimate
                summaries[scenario][name] = score(measurement["time_s"], estimate, reference["wrench"], scenario, config)
                np.savez_compressed(scenario_dir / (name + ".npz"), time_s=measurement["time_s"],
                                    wrench=estimate, innovation=innovation, covariance_diagonal=np.diagonal(covariances, axis1=1, axis2=2))
        if not check_only:
            regression[scenario] = {}
            for name, terms, old_name in (("known", oracle, "known"), ("learned", endpoint, "trained")):
                estimate, _, _, _ = run_kf(measurement, terms, config["replay_config"]["observer"], terms["M"])
                with np.load(root / config["native_replay"] / scenario / (old_name + ".npz"), allow_pickle=False) as old:
                    np.testing.assert_array_equal(measurement["time_s"], old["time_s"])
                    np.testing.assert_allclose(estimate, old["wrench"], rtol=config["comparison_rtol"],
                                               atol=config["comparison_atol"], err_msg=f"Native-R regression: {scenario}/{name}")
                    regression[scenario][name] = float(np.max(np.abs(estimate - old["wrench"])))
            figures += plot_scenario(output, scenario, measurement, reference, curves, parts, config)
            if scenario == "no_force":
                figures += plot_summary(output, summaries[scenario], config)
                figures += plot_inertia(output, truth, learned)
        print(json.dumps({"diagnosed": scenario, "mode": "check" if check_only else "full"}), flush=True)
    if not check_only:
        write_tables_and_report(output, summaries, regression, config)
    return {"passed": True, "environment": runtime, "scenario_count": len(summaries),
            "variant_count": len(config["variants"]), "torque_decomposition_max_abs_error_Nm": errors_max,
            "shared_covariance_max_abs_difference": cov_difference,
            "native_R_regression": regression, "summaries": summaries, "figures": figures,
            "training_updates": 0,
            "scope": "Frozen-model offline oracle diagnosis; EXPLORE only; no training or physical stepping"}


LABELS = {"learned": "Learned", "oracle_inertia": "+ known inertia bundle",
          "oracle_gravity": "+ known gravity", "oracle_friction": "+ known friction", "known": "Fully known"}
COLORS = {"learned": "#d1495b", "oracle_inertia": "#0077b6", "oracle_gravity": "#2a9d8f",
          "oracle_friction": "#e9a23b", "known": "#555555"}
COMPONENTS = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
UNITS = ["N"] * 3 + ["Nm"] * 3


def plotting():
    import matplotlib
    matplotlib.use("Agg")
    matplotlib.rcParams.update({"font.size": 10, "svg.fonttype": "none"})
    import matplotlib.pyplot as plt
    return plt


def save_figure(figure, output, name):
    figure.tight_layout()
    paths = []
    for extension in ("png", "svg"):
        filename = f"{name}.{extension}"
        figure.savefig(output / filename, dpi=180)
        paths.append(filename)
    plotting().close(figure)
    return paths


def plot_scenario(output, scenario, measurement, reference, curves, parts, config):
    plt = plotting()
    t = measurement["time_s"]
    figures = []
    for window in ("full", "startup"):
        mask = np.ones(len(t), dtype=bool) if window == "full" else t < config["windows"]["startup"][1]
        figure, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
        for index, axis in enumerate(axes.flat):
            axis.plot(t[mask], reference["wrench"][mask, index], color="black", linewidth=1.3, label="Applied reference")
            for name in config["main_variants"]:
                axis.plot(t[mask], curves[name][mask, index], linewidth=1., color=COLORS[name], label=LABELS[name])
            axis.set_ylabel(f"{COMPONENTS[index]} [{UNITS[index]}]")
            axis.grid(alpha=.25)
        axes[0, 0].legend(fontsize=7)
        for axis in axes[-1]:
            axis.set_xlabel("Original simulation time [s]")
        figure.suptitle(f"{scenario} / {window}: frozen-model oracle substitutions, shared covariance", fontsize=12)
        figures += save_figure(figure, output, f"{scenario}_{window}_wrench")
    mask = t < config["windows"]["startup"][1]
    figure, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for index, axis in enumerate(axes.flat):
        for key, label in (("inertia_acceleration", "Delta M * actual a"), ("velocity_term", "Delta c"),
                           ("gravity", "Delta G"), ("friction", "Delta friction")):
            axis.plot(t[mask], parts[key][mask, index], linewidth=1., label=label)
        axis.plot(t[mask], parts["total"][mask, index], color="black", linewidth=1.2, linestyle="--", label="Signed sum")
        axis.set_ylabel(f"Joint {index + 1} error [Nm]")
        axis.grid(alpha=.25)
    axes[0, 0].legend(fontsize=7)
    for axis in axes[-1]:
        axis.set_xlabel("Original simulation time [s]")
    figure.suptitle(f"{scenario}: offline startup torque-error decomposition; true acceleration is diagnostic only", fontsize=11)
    figures += save_figure(figure, output, f"{scenario}_startup_torque_errors")
    return figures


def plot_summary(output, metrics, config):
    plt = plotting()
    figures = []
    methods = config["main_variants"]
    for statistic, label in (("absolute_error_peak", "Absolute error peak"), ("rmse", "RMSE")):
        figure, axes = plt.subplots(2, 2, figsize=(12, 8))
        for row, window in enumerate(("startup", "steady")):
            for column, start in enumerate((0, 3)):
                axis = axes[row, column]
                x = np.arange(3)
                width = .15
                for group, name in enumerate(methods):
                    values = metrics[name][window][statistic][start:start + 3]
                    axis.bar(x + (group - (len(methods) - 1) / 2) * width, values, width,
                             label=LABELS[name], color=COLORS[name])
                axis.set_xticks(x, COMPONENTS[start:start + 3])
                axis.set_ylabel(f"{label} [{'N' if column == 0 else 'Nm'}]")
                axis.set_title(f"{window}: {config['windows'][window]} s (right endpoint excluded)")
                axis.grid(axis="y", alpha=.25)
        axes[0, 0].legend(fontsize=7)
        figure.suptitle(f"No external force: {label.lower()} under fixed covariance", fontsize=13)
        figures += save_figure(figure, output, f"no_force_{statistic}_bars")
    return figures


def plot_inertia(output, truth, learned):
    plt = plotting()
    known = np.diagonal(truth["M"], axis1=1, axis2=2).mean(0)
    predicted = np.diagonal(learned["M"], axis1=1, axis2=2).mean(0)
    ratio = predicted / known
    figure, axes = plt.subplots(1, 2, figsize=(12, 4))
    x = np.arange(6)
    axes[0].bar(x - .18, known, .36, color="#555555", label="Known")
    axes[0].bar(x + .18, predicted, .36, color=COLORS["learned"], label="Learned")
    axes[0].set_yscale("log")
    axes[0].set_ylabel("Mean diagonal inertia [kg m^2], log scale")
    axes[0].legend()
    axes[1].bar(x, ratio, color=COLORS["learned"])
    axes[1].axhline(1, color="black", linestyle="--", linewidth=1)
    axes[1].set_yscale("log")
    axes[1].set_ylabel("Mean learned / mean known, log scale")
    for index, value in enumerate(ratio):
        axes[1].annotate(f"{value:.3g}", (index, value), xytext=(0, 3), textcoords="offset points", ha="center", fontsize=8)
    for axis in axes:
        axis.set_xticks(x, [f"J{i + 1}" for i in x])
        axis.grid(axis="y", alpha=.25)
    figure.suptitle("Inertia diagonals over the no-force 4 s trace; whole-matrix error can hide wrist errors", fontsize=11)
    write_json(output / "inertia-diagonal-summary.json", {"known_mean_kgm2": known.tolist(),
                                                           "learned_mean_kgm2": predicted.tolist(),
                                                           "ratio_of_means": ratio.tolist()})
    return save_figure(figure, output, "inertia_diagonals")


def write_tables_and_report(output, summaries, regression, config):
    with (output / "all_variants.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["scenario", "variant", "covariance", "window", "component", "unit", "rmse", "bias", "absolute_error_peak"])
        for scenario, variants in summaries.items():
            for name, metrics in variants.items():
                for window in ("full", "startup", "steady", "loaded", "loaded_after_transient"):
                    values = metrics[window]
                    for index, component in enumerate(COMPONENTS):
                        writer.writerow([scenario, name, "shared_oracle_R", window, component, UNITS[index],
                                         values["rmse"][index], values["bias"][index], values["absolute_error_peak"][index]])
    lines = ["# Frozen dynamics component diagnosis", "",
             "EXPLORE only. Weights and measurements are unchanged. This is an offline oracle diagnostic, not a deployable estimator or a formal claim.",
             "Inertia M, its derivative Mdot and c are swapped together. Gravity and viscous friction are independent factors.",
             "All eight fixed combinations use the same known-model R(t) and initial covariance. Native-R endpoint regressions are checked separately.",
             "True acceleration is used only for signed offline torque-error decomposition; it is not a KF input.", "",
             "## No-force main comparisons", "",
             "| Variant | Full Fx RMSE [N] | Startup Fx peak [N] | Steady Fx RMSE [N] | Startup Fz peak [N] |",
             "|---|---:|---:|---:|---:|"]
    for name in config["main_variants"]:
        m = summaries["no_force"][name]
        lines.append(f"| {LABELS[name]} | {m['full']['rmse'][0]:.6g} | {m['startup']['absolute_error_peak'][0]:.6g} | {m['steady']['rmse'][0]:.6g} | {m['startup']['absolute_error_peak'][2]:.6g} |")
    lines += ["", "Startup is [0,0.25) s; steady is [1,3) s. Both intervals are fixed before computation.",
              "Single substitutions are not additive contributions; components can cancel and interact through momentum and the KF.", "",
              "## Figures", "",
              "![No-force startup](no_force_startup_wrench.png)", "",
              "![No-force peak comparison](no_force_absolute_error_peak_bars.png)", "",
              "![No-force RMSE comparison](no_force_rmse_bars.png)", "",
              "![Startup torque errors](no_force_startup_torque_errors.png)", "",
              "![Inertia diagonals](inertia_diagonals.png)", "",
              "Constant and sine full/startup plots and signed joint-error plots are also retained. Each figure has PNG and SVG versions.",
              "All eight variants, all three scenarios, all six directions and all fixed windows are available in all_variants.csv and metrics.json.", "",
              "## Native-R endpoint regression maximum differences", "", "```json", json.dumps(regression, indent=2), "```", "",
              "The common-R diagnostic numbers must not be silently combined with prior native-R performance rankings."]
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
    if [(x["mask"], x["id"]) for x in config["variants"]] != list(enumerate([
            "learned", "oracle_inertia", "oracle_gravity", "oracle_inertia_gravity", "oracle_friction",
            "oracle_inertia_friction", "oracle_gravity_friction", "known"])):
        raise ValueError("Preserve the declared eight component combinations")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output / "config.snapshot.json").exists() or (output / "metrics.json").exists():
        raise FileExistsError("Do not overwrite or silently repeat this diagnostic")
    try:
        if args.stage == "replay":
            if args.prerequisite_dir is None:
                raise ValueError("Require the explicit preceding check directory")
            previous = json.loads((args.prerequisite_dir / "metrics.json").read_text())
            if previous.get("passed") is not True:
                raise ValueError("Diagnostic precheck did not pass")
            if json.loads((args.prerequisite_dir / "config.snapshot.json").read_text()) != config:
                raise ValueError("Diagnostic config changed after the checks")
        write_json(output / "config.snapshot.json", config)
        metrics = run_diagnosis(Path(__file__).resolve().parents[2], config, output, args.stage == "check")
        metrics["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
        write_json(output / "metrics.json", metrics)
        print(json.dumps({"stage": args.stage, "passed": True, "output_dir": str(output)}))
        return 0
    except Exception as error:
        write_json(output / "failure.json", {"stage": args.stage, "type": type(error).__name__,
                                            "message": str(error), "traceback": traceback.format_exc(),
                                            "automatic_retry": False})
        print(str(error), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
