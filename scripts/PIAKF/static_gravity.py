"""Frozen gravity diagnostics at declared static configurations; no integration."""
import argparse
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import traceback

for _name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_name] = "1"

import mujoco
import numpy as np
import torch

from checks import environment
from dynamics import load_model, mass_matrix
from simulation import write_json
from structured import load_network


def training_bounds(root, config, network):
    """Only training positions establish coverage; no new acceleration labels."""
    design = json.loads((root / config["training_config"]).read_text())
    data_dir = root / config["training_data_dir"]
    manifest = json.loads((data_dir / "manifest.json").read_text())
    pieces = []
    ids = []
    for specification in design["collection"]["trajectories"]:
        if specification["split"] != "train":
            continue
        identity = specification["id"]
        with np.load(data_dir / (identity + ".npz"), allow_pickle=False) as record:
            if set(record.files) != {"time_s", "position_rad", "velocity_rad_s", "drive_torque_Nm"}:
                raise ValueError("Training measurement schema differs")
            t, q = record["time_s"], record["position_rad"]
            count = round(design["collection"]["duration_s"] / design["observation_period_s"]) + 1
            if len(t) != count or q.shape != (count, 6) or not np.isfinite(q).all():
                raise ValueError("Training shape or positions differ")
            np.testing.assert_allclose(np.diff(t), design["observation_period_s"], rtol=0, atol=1e-12)
            trim = design["training"]["trim_s"]
            pieces.append(q[(t >= trim) & (t <= t[-1] - trim)].copy())
        ids.append(identity)
    if ids != [f"trajectory_{i:02d}" for i in range(4)]:
        raise ValueError("Expected the four original training trajectories")
    # The original collection manifest must retain these identities and roles.
    declared = {item["id"]: item["split"] for item in manifest["trajectories"]}
    if any(declared.get(identity) != "train" for identity in ids):
        raise ValueError("Manifest training identity mismatch")
    np.testing.assert_array_equal(manifest["external_wrench"], np.zeros(6))
    q = np.concatenate(pieces)
    center, scale = network.q_center.numpy(), network.q_scale.numpy()
    np.testing.assert_allclose(q.mean(axis=0), center, rtol=1e-12, atol=1e-12)
    expected_scale = q.std(axis=0, ddof=0).clip(min=design["training"]["scale_floor"])
    np.testing.assert_allclose(expected_scale, scale, rtol=1e-12, atol=1e-12)
    return {"samples": len(q), "trajectory_ids": ids, "trim_s": trim,
            "center_rad": center.tolist(), "scale_rad": scale.tolist(),
            "min_rad": q.min(axis=0).tolist(), "max_rad": q.max(axis=0).tolist()}


def pose_schedule(config, center):
    result = [{"id": "center", "group": "center", "joint": -1, "offset_rad": 0., "q": center.copy()}]
    for group, key in (("near", "near_offsets_rad"), ("extended", "extended_offsets_rad")):
        for joint in range(6):
            for offset in config[key]:
                q = center.copy()
                q[joint] += offset
                sign = "p" if offset > 0 else "m"
                result.append({"id": f"{group}_J{joint + 1}_{sign}{abs(offset):g}", "group": group,
                               "joint": joint, "offset_rad": offset, "q": q})
    if len(result) != 49 or len({tuple(x["q"]) for x in result}) != 49:
        raise ValueError("Expected 49 distinct declared poses")
    return result


def evaluate(root, config, network, poses):
    model = load_model(root / config["model"])
    reference, prediction = [], []
    checks = {"inverse_vs_bias_max_abs_Nm": 0., "equilibrium_residual_max_abs_Nm": 0.,
              "equilibrium_acceleration_max_abs_rad_s2": 0., "learned_mass_min_eigenvalue": float("inf")}
    rtol, atol = config["comparison_rtol"], config["comparison_atol_Nm"]
    for pose in poses:
        q = pose["q"]
        if np.any(model.jnt_limited & ((q < model.jnt_range[:, 0]) | (q > model.jnt_range[:, 1]))):
            raise ValueError(f"Pose outside joint limits: {pose['id']}")
        data = mujoco.MjData(model)
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        if data.ncon or np.any(data.qfrc_constraint):
            raise ValueError(f"Static pose has contacts or constraints: {pose['id']}")
        gravity = data.qfrc_bias.copy()
        data.qacc[:] = 0
        mujoco.mj_inverse(model, data)
        np.testing.assert_allclose(data.qfrc_inverse, gravity, rtol=rtol, atol=atol)
        checks["inverse_vs_bias_max_abs_Nm"] = max(checks["inverse_vs_bias_max_abs_Nm"],
                                                   float(np.max(np.abs(data.qfrc_inverse - gravity))))
        if np.any(model.actuator_ctrllimited & ((gravity < model.actuator_ctrlrange[:, 0]) |
                                               (gravity > model.actuator_ctrlrange[:, 1]))):
            raise ValueError(f"Equilibrium drive exceeds actuator range: {pose['id']}")
        data.ctrl[:] = gravity
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.qfrc_actuator, gravity, rtol=rtol, atol=atol)
        np.testing.assert_allclose(data.qfrc_passive, 0, rtol=0, atol=atol)
        np.testing.assert_allclose(data.qfrc_constraint, 0, rtol=0, atol=atol)
        residual = mass_matrix(model, data) @ data.qacc
        np.testing.assert_allclose(residual, 0, rtol=0, atol=atol)
        checks["equilibrium_residual_max_abs_Nm"] = max(checks["equilibrium_residual_max_abs_Nm"],
                                                       float(np.max(np.abs(residual))))
        checks["equilibrium_acceleration_max_abs_rad_s2"] = max(checks["equilibrium_acceleration_max_abs_rad_s2"],
                                                               float(np.max(np.abs(data.qacc))))
        reference.append(gravity)
    q = torch.from_numpy(np.stack([x["q"] for x in poses]))
    with torch.no_grad():
        result = network.dynamics(q, torch.zeros_like(q), torch.zeros_like(q))
    if not all(torch.isfinite(value).all() for value in result.values()):
        raise FloatingPointError("Nonfinite network output")
    for key in ("c", "Mdot", "gradient_T", "friction"):
        np.testing.assert_array_equal(result[key].numpy(), 0)
    np.testing.assert_array_equal(result["tau"].numpy(), result["G"].numpy())
    checks["learned_mass_min_eigenvalue"] = float(torch.linalg.eigvalsh(result["M"]).min())
    if checks["learned_mass_min_eigenvalue"] <= 0:
        raise ValueError("Nonpositive learned inertia")
    prediction = result["G"].numpy()
    reference = np.asarray(reference)
    if not all(np.isfinite(x).all() for x in (reference, prediction)):
        raise FloatingPointError("Nonfinite static reference or prediction")
    return reference, prediction, checks


def statistics(error, mask):
    selected = error[mask]
    if not len(selected):
        return {"poses": 0}
    return {"poses": len(selected), "rmse_Nm": np.sqrt(np.mean(selected ** 2, axis=0)).tolist(),
            "bias_Nm": selected.mean(axis=0).tolist(),
            "absolute_error_peak_Nm": np.abs(selected).max(axis=0).tolist()}


def figures(output, poses, error, summary):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    paths = []

    def save(fig, name):
        fig.tight_layout()
        for suffix in ("png", "svg"):
            path = output / f"{name}.{suffix}"
            fig.savefig(path, dpi=160)
            paths.append(path.name)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4))
    x = np.arange(6)
    for index, group in enumerate(("center", "near", "extended")):
        ax.bar(x + (index - 1) * .25, summary[group]["rmse_Nm"], .25,
               label=f"{group}: {summary[group]['poses']} poses")
    ax.set(xticks=x, xticklabels=[f"J{i+1}" for i in x], ylabel="Gravity torque RMSE [Nm]",
           title="Frozen gravity head: center / near / extended axis poses")
    ax.grid(axis="y", alpha=.25)
    ax.legend()
    save(fig, "gravity_rmse_by_group")

    fig, ax = plt.subplots(figsize=(9, 10))
    limit = max(float(np.max(np.abs(error))), np.finfo(float).eps)
    im = ax.imshow(error, aspect="auto", cmap="RdBu_r", vmin=-limit, vmax=limit)
    ax.set(xticks=np.arange(6), xticklabels=[f"J{i+1}" for i in range(6)],
           yticks=np.arange(len(poses)), yticklabels=[p["id"] for p in poses],
           title="Learned - MuJoCo gravity torque [Nm]")
    ax.tick_params(axis="y", labelsize=6)
    ax.axhline(.5, color="black", linewidth=.7)
    ax.axhline(24.5, color="black", linewidth=.7)
    fig.colorbar(im, ax=ax, label="Signed error [Nm]")
    save(fig, "gravity_error_heatmap")

    fig, axes = plt.subplots(3, 2, figsize=(11, 9), sharex=True)
    for joint, ax in enumerate(axes.flat):
        indices = [i for i, p in enumerate(poses) if p["joint"] in (-1, joint)]
        indices.sort(key=lambda i: poses[i]["offset_rad"])
        delta = [poses[i]["offset_rad"] for i in indices]
        for component in range(6):
            ax.plot(delta, error[indices, component], marker=".", label=f"G{component+1}")
        ax.axvspan(-.1, .1, color="grey", alpha=.1, label="Near schedule")
        ax.axhline(0, color="black", linewidth=.6)
        ax.set(title=f"Perturb input J{joint+1}", ylabel="Gravity torque error [Nm]")
        ax.grid(alpha=.25)
    axes[0, 0].legend(ncol=4, fontsize=7)
    for ax in axes[-1]:
        ax.set_xlabel("Offset from training center [rad]")
    fig.suptitle("Single-axis static gravity error; fixed weights", y=1.01)
    save(fig, "gravity_axis_sweeps")
    return paths


def run(root, config, output, check_only):
    torch.set_num_threads(1)
    runtime = environment()
    network = load_network(root / config["trained_checkpoint"])
    bounds = training_bounds(root, config, network)
    poses = pose_schedule(config, np.asarray(bounds["center_rad"]))
    if check_only:
        poses = [next(p for p in poses if p["id"] == name) for name in config["precheck_pose_ids"]]
    reference, prediction, checks = evaluate(root, config, network, poses)
    error = prediction - reference
    q = np.stack([p["q"] for p in poses])
    inside = np.all((q >= np.asarray(bounds["min_rad"])) & (q <= np.asarray(bounds["max_rad"])), axis=1)
    summary = {group: statistics(error, np.array([p["group"] == group for p in poses]))
               for group in ("center", "near", "extended")}
    summary.update({"all": statistics(error, np.ones(len(poses), dtype=bool)),
                    "inside_training_box": statistics(error, inside),
                    "outside_training_box": statistics(error, ~inside)})
    figure_paths = []
    if not check_only:
        np.savez_compressed(output / "poses.npz", position_rad=q, velocity_rad_s=np.zeros_like(q),
                            acceleration_rad_s2=np.zeros_like(q), gravity_reference_Nm=reference,
                            gravity_prediction_Nm=prediction, gravity_error_Nm=error,
                            inside_training_box=inside, pose_id=np.array([p["id"] for p in poses]))
        with (output / "all_poses.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["pose", "group", "perturbed_joint", "offset_rad", "inside_training_box",
                             *[f"q{i+1}_rad" for i in range(6)], "gravity_component", "reference_Nm", "prediction_Nm", "error_Nm"])
            for i, p in enumerate(poses):
                for j in range(6):
                    writer.writerow([p["id"], p["group"], p["joint"] + 1, p["offset_rad"], bool(inside[i]),
                                     *q[i].tolist(), f"J{j+1}", reference[i, j], prediction[i, j], error[i, j]])
        figure_paths = figures(output, poses, error, summary)
        write_json(output / "training_bounds.json", bounds)
        lines = ["# Frozen-network static gravity diagnosis", "",
                 "EXPLORE; no training updates, time integration, KF, or real robot commands.",
                 "MuJoCo algebraic gravity is the simulation reference. Inverse dynamics and an equilibrium forward calculation check it independently of the learned network.",
                 "At every declared configuration v=a=0: learned inverse torque equals its gravity head, with zero velocity terms and viscous friction.",
                 "Center is the frozen training q mean. Near offsets are +/-0.05, +/-0.1 rad; extended offsets +/-0.2, +/-0.4 rad, one input joint at a time.",
                 "These are synthetic static configurations, not newly collected controlled motion records or untouched trajectory holdout.",
                 "Inside the training coordinate box does not imply coverage of joint combinations or independent identifiability.",
                 "Gravity torque error [Nm] is not an external wrench [N/Nm]; no Jacobian conversion or KF is performed.", "",
                 "## Per-joint gravity RMSE [Nm]", "", "| Group | Poses | J1 | J2 | J3 | J4 | J5 | J6 |",
                 "|---|---:|---:|---:|---:|---:|---:|---:|"]
        for group, value in summary.items():
            values = " | ".join(f"{x:.6g}" for x in value.get("rmse_Nm", [float("nan")] * 6))
            lines.append(f"| {group} | {value['poses']} | {values} |")
        lines += ["", "## Numerical checks", "", "```json", json.dumps(checks, indent=2), "```", "",
                  "## Figures", ""]
        for name in figure_paths:
            if name.endswith(".png"):
                lines += [f"![{name}]({name})", ""]
        lines += ["Full signed errors and pose coordinates are retained in all_poses.csv and poses.npz.",
                  "This probe cannot establish that added static supervision, extra data, another activation, or longer training will improve moving external-force estimates."]
        (output / "report.md").write_text("\n".join(lines) + "\n")
    return {"passed": True, "environment": runtime, "training_updates": 0, "pose_count": len(poses),
            "checks": checks, "summary": summary, "training_bounds": bounds, "figures": figure_paths,
            "inside_training_box_count": int(inside.sum()), "scope": "Frozen algebraic static gravity diagnosis only"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stage", choices=("check", "diagnose"), required=True)
    parser.add_argument("--prerequisite-dir", type=Path)
    args = parser.parse_args()
    if not Path("/.dockerenv").is_file():
        parser.error("Run inside the existing approved Docker service")
    config = json.loads(args.config.read_text())
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output / "config.snapshot.json").exists() or (output / "metrics.json").exists():
        raise FileExistsError("Do not overwrite or silently repeat a diagnostic")
    try:
        if args.stage == "diagnose":
            if args.prerequisite_dir is None:
                raise ValueError("Explicit prerequisite directory required")
            previous = json.loads((args.prerequisite_dir / "metrics.json").read_text())
            if previous.get("passed") is not True or json.loads((args.prerequisite_dir / "config.snapshot.json").read_text()) != config:
                raise ValueError("Precheck did not pass or config changed")
        write_json(output / "config.snapshot.json", config)
        metrics = run(Path(__file__).resolve().parents[2], config, output, args.stage == "check")
        metrics["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
        write_json(output / "metrics.json", metrics)
        print(json.dumps({"stage": args.stage, "passed": True, "poses": metrics["pose_count"], "output_dir": str(output)}))
        return 0
    except Exception as error:
        write_json(output / "failure.json", {"stage": args.stage, "type": type(error).__name__, "message": str(error)})
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
