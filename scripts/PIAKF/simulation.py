"""Three short known-model force probes; truth never enters the KF packet."""
import json
from pathlib import Path
from time import perf_counter

import mujoco
import numpy as np

from dynamics import KnownDynamics, apply_wrench, jacobian, load_model, mass_matrix
from observer import Measurement, MomentumKF


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)
                          + "\n", encoding="utf-8")


def moving_bodies(model):
    mask = np.zeros(model.nbody, dtype=bool)
    for body in range(1, model.nbody):
        mask[body] = mask[model.body_parentid[body]] or model.body_dofnum[body] > 0
    return mask


def guard(model, data, command, moving, tolerances):
    if not all(np.isfinite(x).all() for x in (data.qpos, data.qvel, data.qacc, command)):
        raise FloatingPointError("Nonfinite plant state or command")
    if np.any(data.warning.number):
        raise RuntimeError(f"MuJoCo warning counts: {data.warning.number.tolist()}")
    limited = model.jnt_limited.astype(bool)
    if not np.all((data.qpos[limited] > model.jnt_range[limited, 0])
                  & (data.qpos[limited] < model.jnt_range[limited, 1])):
        raise RuntimeError("Joint position limit reached")
    for enabled, bounds in ((model.actuator_ctrllimited, model.actuator_ctrlrange),
                            (model.actuator_forcelimited, model.actuator_forcerange)):
        active = enabled.astype(bool)
        if np.any(command[active] < bounds[active, 0]) or np.any(command[active] > bounds[active, 1]):
            raise RuntimeError("Actuator saturation requested; command is NOT clipped")
    if np.max(np.abs(data.qfrc_actuator - command)) > tolerances["actual_torque_atol_Nm"]:
        raise RuntimeError("Actual actuator torque differs from the declared ideal motor command")
    if np.max(np.abs(data.qfrc_constraint)) > tolerances["constraint_torque_atol_Nm"]:
        raise RuntimeError("Unexpected constraint torque")
    for index in range(data.ncon):
        contact = data.contact[index]
        if contact.geom1 < 0 or contact.geom2 < 0:
            raise RuntimeError("Unexpected non-rigid contact")
        if (moving[model.geom_bodyid[contact.geom1]] or moving[model.geom_bodyid[contact.geom2]]):
            if contact.dist <= 0 or contact.efc_address >= 0:
                raise RuntimeError("Unexpected contact involving a moving link")


def external_wrench(scenario, time_s, config):
    result = np.zeros(6)
    if config["load_start_s"] <= time_s < config["load_end_s"]:
        if scenario == "constant":
            result[0] = config["force_amplitude_N"]
        elif scenario == "sine":
            result[0] = config["force_amplitude_N"] * np.sin(
                2 * np.pi * config["force_frequency_Hz"] * (time_s - config["load_start_s"]))
        elif scenario != "no_force":
            raise ValueError(f"Unknown scenario: {scenario}")
    elif scenario not in ("no_force", "constant", "sine"):
        raise ValueError(f"Unknown scenario: {scenario}")
    return result


def error_statistics(estimate, truth, mask):
    error = estimate[mask] - truth[mask]
    if not len(error):
        raise ValueError("Empty evaluation window")
    return {
        "samples": int(len(error)), "rmse": np.sqrt(np.mean(error**2, axis=0)).tolist(),
        "bias": np.mean(error, axis=0).tolist(),
        "absolute_error_peak": np.max(np.abs(error), axis=0).tolist(),
        "units": ["N", "N", "N", "Nm", "Nm", "Nm"],
    }


def evaluate(t, estimate, truth, scenario, config):
    start, end = config["load_start_s"], config["load_end_s"]
    exclusion = config["evaluation"]["transient_exclusion_s"]
    masks = {
        "full": np.ones(len(t), dtype=bool), "pre_load": t < start,
        "loaded": (t >= start) & (t < end), "post_load": t >= end,
        "loaded_after_transient": (t >= start + exclusion) & (t < end),
    }
    result = {name: error_statistics(estimate, truth, mask) for name, mask in masks.items()}
    if scenario == "constant":
        mask = masks["loaded"]
        levels = config["evaluation"]["crossing_levels"]
        delays = []
        for level in levels:
            indices = np.flatnonzero(mask & (estimate[:, 0] >= level * config["force_amplitude_N"]))
            delays.append(float(t[indices[0]] - start) if len(indices) else None)
        result["Fx_step_response"] = {
            "first_10pct_crossing_delay_s": delays[0], "first_90pct_crossing_delay_s": delays[1],
            "rise_10_to_90_s": delays[1] - delays[0] if all(d is not None for d in delays) else None,
            "overshoot_fraction": float(np.max(estimate[mask, 0]) / config["force_amplitude_N"] - 1),
            "definition": "First crossings on original timestamps, relative to applied 20 N step; may be negative overshoot if undershooting",
        }
    if scenario == "sine":
        mask = masks["loaded_after_transient"]
        omega = 2 * np.pi * config["force_frequency_Hz"]
        phase = omega * (t[mask] - start)
        basis = np.column_stack((np.sin(phase), np.cos(phase), np.ones(len(phase))))
        coefficients = np.linalg.lstsq(basis, estimate[mask, 0], rcond=None)[0]
        phase_rad = float(np.arctan2(coefficients[1], coefficients[0]))
        result["Fx_sine_response"] = {
            "amplitude_N": float(np.hypot(coefficients[0], coefficients[1])),
            "amplitude_ratio": float(np.hypot(coefficients[0], coefficients[1]) / config["force_amplitude_N"]),
            "phase_relative_to_reference_rad": phase_rad,
            "equivalent_lag_s": -phase_rad / omega,
            "fitted_offset_N": float(coefficients[2]),
            "definition": "Least-squares sine/cosine/DC fit at declared frequency; diagnostic only, no alignment of RMSE",
        }
    return result


def plot_result(output_dir, t, estimate, truth, scenario):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for index, axis in enumerate(axes.flat):
        axis.plot(t, truth[:, index], color="black", linewidth=1.4, label="Applied reference")
        axis.plot(t, estimate[:, index], linewidth=0.9, label="Known-model momentum KF")
        axis.set_ylabel(["Fx [N]", "Fy [N]", "Fz [N]", "Mx [Nm]", "My [Nm]", "Mz [Nm]"][index])
        axis.grid(alpha=0.3)
    axes[0, 0].legend(fontsize=8)
    for axis in axes[-1]:
        axis.set_xlabel("Simulation time [s]")
    figure.suptitle(f"{scenario}: world wrench at tool0; unshifted timestamps")
    figure.tight_layout()
    figure.savefig(Path(output_dir) / "wrench.png", dpi=150)
    plt.close(figure)


def run_scenario(root, config, scenario, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)
    model_path = root / config["model"]
    plant = load_model(model_path)
    nominal = KnownDynamics(model_path, config["derivative_epsilon_s"])
    observer = MomentumKF(nominal, config["observer"])
    design = json.loads((root / config["control_design"]).read_text())
    q_reference = np.array(design["q_hold_rad"])
    kp, kd = np.array(design["kp_Nm_per_rad"]), np.array(design["kd_Nms_per_rad"])
    data = mujoco.MjData(plant)
    data.qpos[:] = q_reference
    moving = moving_bodies(plant)
    dt = float(plant.opt.timestep)
    if not np.isclose(dt, config["physics_dt_s"], rtol=0, atol=1e-14):
        raise ValueError("Model timestep differs from configuration")
    strides = []
    for period in (config["control_period_s"], config["observation_period_s"], config["duration_s"]):
        stride = round(period / dt)
        if stride <= 0 or not np.isclose(stride * dt, period, rtol=0, atol=1e-12):
            raise ValueError("Periods must be integer multiples of the unchanged physical timestep")
        strides.append(stride)
    control_stride, observation_stride, steps = strides
    if np.any(config["measured_velocity_noise_std_rad_s"]):
        raise ValueError("The approved prerequisite is noise-free; do not silently add noise")
    times, positions, velocities, torques = [], [], [], []
    references, external_torques, accelerations = [], [], []
    estimates, innovations, momenta, inputs, covariances = [], [], [], [], []
    balance_max, mapping_max, min_cov, min_jacobian = 0., 0., float("inf"), float("inf")
    warning_max = 0
    command = np.zeros(6)
    started = perf_counter()
    failed = None
    try:
        for step in range(steps + 1):
            t = step * dt
            if not np.isclose(data.time, t, rtol=0, atol=1e-9):
                raise ValueError("Physical clock disagrees with the declared schedule")
            # Refresh geometry at q(t); mj_step leaves derived fields at its prior evaluation.
            mujoco.mj_forward(plant, data)
            if step % control_stride == 0:
                command = kp * (q_reference - data.qpos) - kd * data.qvel
            data.ctrl[:] = command
            reference = external_wrench(scenario, t, config)
            external_torque = apply_wrench(plant, data, reference)
            mujoco.mj_forward(plant, data)
            guard(plant, data, command, moving, config["checks"])
            warning_max = max(warning_max, int(np.max(data.warning.number)))
            if step % observation_stride == 0:
                packet = Measurement(t, data.qpos.copy(), data.qvel.copy(), data.qfrc_actuator.copy())
                result = observer.update(packet)
                # Diagnostics are evaluated separately from the estimator packet.
                plant_mass = mass_matrix(plant, data)
                balance = (plant_mass @ data.qacc + data.qfrc_bias - data.qfrc_actuator
                           - data.qfrc_passive - data.qfrc_applied - data.qfrc_constraint)
                balance_max = max(balance_max, float(np.max(np.abs(balance))))
                mapping_max = max(mapping_max, float(np.max(np.abs(
                    external_torque - jacobian(plant, data).T @ reference))))
                if balance_max > config["checks"]["dynamics_balance_atol_Nm"]:
                    raise RuntimeError(f"Dynamics balance failed: {balance_max} Nm")
                if mapping_max > config["checks"]["algebra_atol"]:
                    raise RuntimeError(f"Wrench mapping failed: {mapping_max} Nm")
                min_cov = min(min_cov, result["covariance_min_eigenvalue"])
                min_jacobian = min(min_jacobian, float(result["jacobian_singular_values"][-1]))
                times.append(t)
                positions.append(packet.position_rad)
                velocities.append(packet.velocity_rad_s)
                torques.append(packet.drive_torque_Nm)
                references.append(reference)
                external_torques.append(external_torque)
                accelerations.append(data.qacc.copy())
                estimates.append(result["wrench"])
                innovations.append(result["innovation"])
                momenta.append(result["momentum"])
                inputs.append(result["input"])
                covariances.append(result["covariance"])
            if step < steps:
                mujoco.mj_step(plant, data)
    except Exception as error:
        failed = {"type": type(error).__name__, "message": str(error), "time_s": float(data.time),
                  "q_rad": data.qpos.tolist(), "v_rad_s": data.qvel.tolist(),
                  "command_Nm": command.tolist(), "warning_counts": data.warning.number.tolist()}
    finally:
        elapsed = perf_counter() - started
        np.savez_compressed(output_dir / "measurements.npz", time_s=times,
                            position_rad=positions, velocity_rad_s=velocities, drive_torque_Nm=torques)
        np.savez_compressed(output_dir / "reference.npz", time_s=times, wrench=references,
                            joint_external_torque_Nm=external_torques, acceleration_rad_s2=accelerations)
        np.savez_compressed(output_dir / "estimates.npz", time_s=times, wrench=estimates,
                            innovation=innovations, momentum=momenta, nominal_input=inputs, covariance=covariances)
    if failed is not None:
        write_json(output_dir / "failure.json", failed)
        raise RuntimeError(f"{scenario} stopped; partial logs preserved: {failed['message']}")
    t, estimate, truth = np.asarray(times), np.asarray(estimates), np.asarray(references)
    metrics = evaluate(t, estimate, truth, scenario, config)
    metrics["diagnostics"] = {
        "scenario": scenario, "samples": len(t), "simulation_duration_s": float(t[-1]),
        "wall_time_s": elapsed, "simulated_seconds_per_wall_second": float(t[-1] / elapsed),
        "dynamics_balance_max_abs_Nm": balance_max, "wrench_mapping_max_abs_Nm": mapping_max,
        "covariance_min_eigenvalue": min_cov, "jacobian_min_singular_value": min_jacobian,
        "warning_count_max": warning_max, "safety_passed": True,
        "jacobian_note": "Mixed translational/rotational units; diagnostic, not a dimensionless condition gate",
    }
    write_json(output_dir / "metrics.json", metrics)
    plot_result(output_dir, t, estimate, truth, scenario)
    return metrics
