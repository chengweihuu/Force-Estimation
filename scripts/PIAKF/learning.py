"""Bounded collection, point-loss training and frozen-model replay for PIAKF."""
import json
from pathlib import Path
from time import perf_counter

import mujoco
import numpy as np
import torch

from checks import environment
from dynamics import KnownDynamics, load_model
from observer import Measurement, MomentumKF
from simulation import evaluate, guard, moving_bodies, write_json
from structured import LearnedDynamics, StructuredDynamics, load_network, save_model


def validate_learning_config(config):
    trajectories = config["collection"]["trajectories"]
    if ([x["split"] for x in trajectories] != ["train"] * 4 + ["development", "diagnostic"]
            or len({x["id"] for x in trajectories}) != 6):
        raise ValueError("Use the declared six whole-trajectory split")
    if config["seed"] != 0 or config["training"]["updates"] != 1000:
        raise ValueError("This bounded probe uses one seed and exactly 1000 updates")
    if config["network"]["widths"] != [128, 128, 128]:
        raise ValueError("Preserve the declared network widths")


def trajectory(t, q0, specification, config):
    c = config["collection"]
    duration, ramp = c["duration_s"], c["ramp_s"]
    if t < ramp:
        envelope = .5 * (1 - np.cos(np.pi * t / ramp))
        derivative = .5 * np.pi / ramp * np.sin(np.pi * t / ramp)
    elif t > duration - ramp:
        s = (duration - t) / ramp
        envelope = .5 * (1 - np.cos(np.pi * s))
        derivative = -.5 * np.pi / ramp * np.sin(np.pi * s)
    else:
        envelope, derivative = 1., 0.
    harmonics = np.arange(1, c["harmonics"] + 1)
    weights = harmonics.astype(float)**(-c["harmonic_power"])
    weights /= weights.sum()
    frequencies = 2 * np.pi * c["fundamental_Hz"] * harmonics
    angles = frequencies[None, :] * t + np.array(specification["phase_rad"])
    amplitude = np.array(c["amplitude_rad"])
    s = amplitude * (np.sin(angles) @ weights)
    ds = amplitude * (np.cos(angles) @ (weights * frequencies))
    return q0 + np.array(specification["offset_rad"]) + envelope * s, derivative * s + envelope * ds


def collect(root, config, output):
    runtime = environment()
    design = json.loads((root / config["control_design"]).read_text())
    q0 = np.array(design["q_hold_rad"])
    kp, kd = np.array(design["kp_Nm_per_rad"]), np.array(design["kd_Nms_per_rad"])
    base = config["replay_config"]
    summaries = []
    total_steps = 0
    for specification in config["collection"]["trajectories"]:
        model = load_model(root / config["model"])
        data = mujoco.MjData(model)
        data.qpos[:] = q0 + np.array(specification["offset_rad"])
        moving = moving_bodies(model)
        dt = float(model.opt.timestep)
        if not np.isclose(dt, config["physics_dt_s"], rtol=0, atol=1e-14):
            raise ValueError("Model timestep changed")
        strides = [round(config[key] / dt) for key in ("control_period_s", "observation_period_s")]
        if any(not np.isclose(strides[i] * dt, config[key], rtol=0, atol=1e-12)
               for i, key in enumerate(("control_period_s", "observation_period_s"))):
            raise ValueError("Sampling periods must be physical-step multiples")
        steps = round(config["collection"]["duration_s"] / dt)
        packet = {key: [] for key in ("time_s", "position_rad", "velocity_rad_s", "drive_torque_Nm")}
        command = np.zeros(6)
        started = perf_counter()
        failure = None
        try:
            for step in range(steps + 1):
                t = step * dt
                if not np.isclose(data.time, t, rtol=0, atol=1e-9):
                    raise ValueError("Physical clock differs from schedule")
                if step % strides[0] == 0:
                    qr, vr = trajectory(t, q0, specification, config)
                    command = kp * (qr - data.qpos) + kd * (vr - data.qvel)
                data.ctrl[:] = command
                data.qfrc_applied[:] = 0
                data.xfrc_applied[:] = 0
                mujoco.mj_forward(model, data)
                guard(model, data, command, moving, base["checks"])
                if step % strides[1] == 0:
                    packet["time_s"].append(t)
                    packet["position_rad"].append(data.qpos.copy())
                    packet["velocity_rad_s"].append(data.qvel.copy())
                    packet["drive_torque_Nm"].append(data.qfrc_actuator.copy())
                if step < steps:
                    mujoco.mj_step(model, data)
                    total_steps += 1
        except Exception as error:
            failure = {"trajectory": specification["id"], "type": type(error).__name__,
                       "message": str(error), "time_s": float(data.time),
                       "q_rad": data.qpos.tolist(), "v_rad_s": data.qvel.tolist()}
        finally:
            np.savez_compressed(output / (specification["id"] + ".npz"), **packet)
        if failure is not None:
            write_json(output / "collection-failure.json", failure)
            raise RuntimeError(f"Collection stopped: {failure['message']}")
        q, v = np.asarray(packet["position_rad"]), np.asarray(packet["velocity_rad_s"])
        summaries.append({"id": specification["id"], "split": specification["split"],
                          "samples": len(q), "q_min_rad": q.min(0).tolist(), "q_max_rad": q.max(0).tolist(),
                          "max_abs_velocity_rad_s": np.max(np.abs(v), 0).tolist(),
                          "wall_time_s": perf_counter() - started})
        print(json.dumps({"collected": specification["id"], "split": specification["split"], "samples": len(q)}), flush=True)
    write_json(output / "manifest.json", {"trajectories": summaries, "external_wrench": [0.] * 6,
                                          "packet_fields": list(packet), "config": config["collection"]})
    runtime["physics_steps"] = total_steps
    return {"passed": True, "trajectory_count": len(summaries), "environment": runtime,
            "summaries": summaries}


def load_split(data_dir, config, split):
    pieces = {key: [] for key in ("q", "v", "a", "tau")}
    for specification in config["collection"]["trajectories"]:
        if specification["split"] != split:
            continue
        with np.load(data_dir / (specification["id"] + ".npz"), allow_pickle=False) as record:
            if set(record.files) != {"time_s", "position_rad", "velocity_rad_s", "drive_torque_Nm"}:
                raise ValueError("Undeclared training information in measurement packet")
            t, q, v, tau = (record[key].copy() for key in
                            ("time_s", "position_rad", "velocity_rad_s", "drive_torque_Nm"))
        expected = round(config["collection"]["duration_s"] / config["observation_period_s"]) + 1
        if len(t) != expected or any(x.shape != (expected, 6) for x in (q, v, tau)):
            raise ValueError("Training packet shape differs from schedule")
        if not np.allclose(np.diff(t), config["observation_period_s"], rtol=0, atol=1e-12):
            raise ValueError("Training clock mismatch")
        acceleration = np.gradient(v, t, axis=0, edge_order=2)
        trim = config["training"]["trim_s"]
        mask = (t >= trim) & (t <= t[-1] - trim)
        for key, value in (("q", q), ("v", v), ("a", acceleration), ("tau", tau)):
            if not np.isfinite(value).all():
                raise FloatingPointError("Nonfinite training signal")
            pieces[key].append(value[mask])
    return {key: torch.from_numpy(np.concatenate(values)) for key, values in pieces.items()}


def torque_metrics(model, dataset, chunk):
    residuals = []
    with torch.no_grad():
        for lo in range(0, len(dataset["q"]), chunk):
            sl = slice(lo, lo + chunk)
            prediction = model.dynamics(dataset["q"][sl], dataset["v"][sl], dataset["a"][sl])["tau"]
            residuals.append(prediction - dataset["tau"][sl])
    residual = torch.cat(residuals)
    if not torch.isfinite(residual).all():
        raise FloatingPointError("Nonfinite torque evaluation")
    return {"samples": len(residual), "rmse_Nm": residual.square().mean(0).sqrt().tolist(),
            "bias_Nm": residual.mean(0).tolist(),
            "normalized_mse": float((residual / model.torque_scale).square().mean())}


def train(config, data_dir, output):
    torch.set_num_threads(1)
    torch.manual_seed(config["seed"])
    train_data = load_split(data_dir, config, "train")
    development = load_split(data_dir, config, "development")
    floor = config["training"]["scale_floor"]
    statistics = {
        "q_center": train_data["q"].mean(0).tolist(),
        "q_scale": train_data["q"].std(0, correction=0).clamp_min(floor).tolist(),
        "gravity_center": train_data["tau"].mean(0).tolist(),
        "gravity_scale": train_data["tau"].std(0, correction=0).clamp_min(floor).tolist(),
        "torque_scale": train_data["tau"].std(0, correction=0).clamp_min(floor).tolist(),
        "source": "Only the four training trajectories; gravity_center is a torque-label mean, not ground-truth gravity",
    }
    model = StructuredDynamics(config["network"], statistics)
    optimizer = torch.optim.Adam(model.parameters(), lr=config["training"]["learning_rate"])
    chunk = config["training"]["evaluation_chunk"]
    initial = {"train": torque_metrics(model, train_data, chunk),
               "development": torque_metrics(model, development, chunk)}
    save_model(output / "initial.pt", model, statistics, 0)
    history = [{"update": 0, **initial}]
    generator = torch.Generator().manual_seed(config["seed"])
    started = perf_counter()
    for update in range(1, config["training"]["updates"] + 1):
        indices = torch.randint(len(train_data["q"]), (config["training"]["batch_size"],), generator=generator)
        prediction = model.dynamics(train_data["q"][indices], train_data["v"][indices], train_data["a"][indices])["tau"]
        loss = ((prediction - train_data["tau"][indices]) / model.torque_scale).square().mean()
        if not torch.isfinite(loss):
            raise FloatingPointError("Nonfinite training loss")
        optimizer.zero_grad()
        loss.backward()
        if not all(p.grad is not None and torch.isfinite(p.grad).all() for p in model.parameters()):
            raise FloatingPointError("Nonfinite training gradient")
        optimizer.step()
        if not all(torch.isfinite(p).all() for p in model.parameters()):
            raise FloatingPointError("Nonfinite trained parameter")
        if update % config["training"]["log_every"] == 0 or update == config["training"]["updates"]:
            row = {"update": update, "train": torque_metrics(model, train_data, chunk),
                   "development": torque_metrics(model, development, chunk)}
            history.append(row)
            write_json(output / "training-history.json", history)
            print(json.dumps({"update": update, "train_mse": row["train"]["normalized_mse"],
                              "development_mse": row["development"]["normalized_mse"]}), flush=True)
    save_model(output / "model.pt", model, statistics, update)
    torch.save(optimizer.state_dict(), output / "optimizer.pt")
    write_json(output / "statistics.json", statistics)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axis = plt.subplots(figsize=(7, 4))
    for split in ("train", "development"):
        axis.semilogy([x["update"] for x in history], [x[split]["normalized_mse"] for x in history], label=split)
    axis.set(xlabel="Parameter updates", ylabel="Normalized torque MSE", title="Fixed-budget training; final checkpoint")
    axis.legend()
    axis.grid(alpha=.3)
    figure.tight_layout()
    figure.savefig(output / "training.png", dpi=150)
    plt.close(figure)
    return {"passed": True, "updates": update, "initial": initial, "final": history[-1],
            "friction_coefficients_Nms_rad": torch.nn.functional.softplus(model.friction_raw).detach().tolist(),
            "checkpoint_rule": config["training"]["checkpoint_rule"], "wall_time_s": perf_counter() - started,
            "data_split": {"train": 4, "development": 1, "diagnostic": 1}}


def diagnose_dynamics(root, config, model, data_dir):
    diagnostic = load_split(data_dir, config, "diagnostic")
    indices = np.linspace(0, len(diagnostic["q"]) - 1, config["verification"]["diagnostic_samples"], dtype=int)
    nominal = KnownDynamics(root / config["model"], config["replay_config"]["derivative_epsilon_s"])
    mass_errors, mass_norms, gravity_errors, velocity_errors, derivative_errors = [], [], [], [], []
    with torch.no_grad():
        for index in indices:
            q, v = diagnostic["q"][index], diagnostic["v"][index]
            prediction = model.dynamics(q[None], v[None])
            truth = nominal.evaluate(q.numpy(), v.numpy())
            G = nominal.evaluate(q.numpy(), np.zeros(6)).bias
            mass_errors.append(np.linalg.norm(prediction["M"][0].numpy() - truth.mass))
            mass_norms.append(np.linalg.norm(truth.mass))
            gravity_errors.append(prediction["G"][0].numpy() - G)
            velocity_errors.append(prediction["c"][0].numpy() - (truth.bias - G))
            derivative_errors.append(np.linalg.norm(prediction["Mdot"][0].numpy() - truth.mass_dot))
    return {"scope": "Held-out local trajectory; simulator dynamics used ONLY after freezing for diagnostics",
            "sample_count": len(indices), "mass_relative_Frobenius_mean": float(np.mean(np.array(mass_errors) / np.array(mass_norms))),
            "gravity_rmse_Nm": np.sqrt(np.mean(np.square(gravity_errors), axis=0)).tolist(),
            "velocity_term_rmse_Nm": np.sqrt(np.mean(np.square(velocity_errors), axis=0)).tolist(),
            "mass_dot_Frobenius_error_mean": float(np.mean(derivative_errors)),
            "torque": torque_metrics(model, diagnostic, config["training"]["evaluation_chunk"])}


def replay(root, config, training_dir, data_dir, output):
    networks = {"initial": load_network(training_dir / "initial.pt"),
                "trained": load_network(training_dir / "model.pt")}
    diagnostics = diagnose_dynamics(root, config, networks["trained"], data_dir)
    write_json(output / "dynamics-diagnostic.json", diagnostics)
    summaries = {}
    started = perf_counter()
    for scenario in config["replay_config"]["scenarios"]:
        source = root / config["replay_run"] / scenario
        with np.load(source / "measurements.npz", allow_pickle=False) as f:
            packet = {name: f[name].copy() for name in f.files}
        with np.load(source / "reference.npz", allow_pickle=False) as f:
            t_reference, truth = f["time_s"].copy(), f["wrench"].copy()
        np.testing.assert_array_equal(packet["time_s"], t_reference)
        scenario_dir = output / scenario
        scenario_dir.mkdir(exist_ok=False)
        summaries[scenario] = {}
        curves = {}
        for name in ("known", "initial", "trained"):
            nominal = (KnownDynamics(root / config["model"], config["replay_config"]["derivative_epsilon_s"])
                       if name == "known" else LearnedDynamics(networks[name], root / config["model"]))
            observer = MomentumKF(nominal, config["replay_config"]["observer"])
            estimates, innovations, covariances = [], [], []
            for index, time_s in enumerate(packet["time_s"]):
                result = observer.update(Measurement(float(time_s), packet["position_rad"][index],
                                                     packet["velocity_rad_s"][index], packet["drive_torque_Nm"][index]))
                estimates.append(result["wrench"])
                innovations.append(result["innovation"])
                covariances.append(result["covariance"])
            values = np.asarray(estimates)
            curves[name] = values
            summaries[scenario][name] = evaluate(packet["time_s"], values, truth, scenario, config["replay_config"])
            np.savez_compressed(scenario_dir / (name + ".npz"), time_s=packet["time_s"], wrench=values,
                                innovation=innovations, covariance=covariances)
            write_json(scenario_dir / (name + "-metrics.json"), summaries[scenario][name])
        _plot_replay(scenario_dir, packet["time_s"], truth, curves, scenario)
        print(json.dumps({"replayed": scenario, "trained_Fx_full_rmse_N": summaries[scenario]["trained"]["full"]["rmse"][0]}), flush=True)
    _write_report(output, summaries, diagnostics)
    return {"passed": True, "scenario_count": len(summaries), "summaries": summaries,
            "dynamics_diagnostic": diagnostics, "wall_time_s": perf_counter() - started,
            "scope": "PIKF structured-dynamics probe only; no backlash compensation or WMA",
            "R_disclosure": "Same assumed velocity noise; R=Mhat*Sigma_v*Mhat.T varies with each model"}


def _plot_replay(output, t, truth, curves, scenario):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for index, axis in enumerate(axes.flat):
        axis.plot(t, truth[:, index], color="black", linewidth=1.2, label="Reference")
        for name, values in curves.items():
            axis.plot(t, values[:, index], linewidth=.8, label=name)
        axis.set_ylabel(["Fx [N]", "Fy [N]", "Fz [N]", "Mx [Nm]", "My [Nm]", "Mz [Nm]"][index])
        axis.grid(alpha=.3)
    axes[0, 0].legend(fontsize=8)
    for axis in axes[-1]:
        axis.set_xlabel("Original simulation timestamp [s]")
    figure.suptitle(f"{scenario}: identical measurements, known / initialized / trained dynamics")
    figure.tight_layout()
    figure.savefig(output / "comparison.png", dpi=150)
    plt.close(figure)


def _write_report(output, summaries, diagnostics):
    lines = ["# Structured learned dynamics and momentum KF", "",
             "EXPLORE local reconstruction: whole-trajectory 4/1/1 split; fixed last checkpoint; no simulator dynamic-component training labels.",
             "Original three previously analyzed probe logs are reused for regression, not untouched holdout.",
             "Same KF settings and covariance rule; actual R depends on each inertia model. No time alignment of estimates.", "",
             "| Scenario | Model | Full Fx RMSE [N] | Loaded Fx RMSE [N] |", "|---|---|---:|---:|"]
    for scenario, models in summaries.items():
        for name, metrics in models.items():
            lines.append(f"| {scenario} | {name} | {metrics['full']['rmse'][0]:.6g} | {metrics['loaded']['rmse'][0]:.6g} |")
    lines += ["", "Held-out local trajectory diagnostics after freezing:",
              f"- Mean relative inertia Frobenius error: {diagnostics['mass_relative_Frobenius_mean']:.6g}",
              f"- Torque RMSE by joint [Nm]: {diagnostics['torque']['rmse_Nm']}",
              f"- Gravity RMSE by joint [Nm]: {diagnostics['gravity_rmse_Nm']}",
              "", "Each scenario has complete six-direction metrics, NPZ estimates and an unshifted comparison plot.",
              "Torque fit, derivative structure and force-estimation accuracy are separate outcomes. This probe does not establish complete PIAKF reproduction or convergence."]
    (output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
