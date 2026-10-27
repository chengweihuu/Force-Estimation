"""Original guarded controller/physics loop with direct high-rate logging added.

The simulate body is copied from the accepted timestep probe; the only runtime
extension is record_state(k,d) before each integration, including terminal state.
Training records contain no simulator dynamics quantities.
"""
import argparse
import time
from pathlib import Path
import mujoco
import numpy as np
from interval_loss import read_json, write_json, require, load_module, PACKET_FIELDS


def schedule(h, control_dt, feedback_dt, duration):
    ratios = np.array([control_dt, feedback_dt, duration])/h
    require(np.allclose(ratios, np.round(ratios), rtol=0, atol=1e-9), "Invalid schedule")
    return tuple(int(x) for x in np.round(ratios))


def simulate(m, pilot, plan, a, b, lower, upper, record_state):
    h = float(m.opt.timestep)
    control_stride, stride, count = schedule(h, plan["control_dt_s"], plan["feedback_dt_s"], plan["duration_s"])
    moving = pilot.moving_body_mask(m)
    geometry = pilot.ground_clearance_geometry(m, moving)
    limited = m.jnt_limited.astype(bool)
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    d.qpos[:] = 0
    d.qvel[:] = 0
    stats = {"completed": False, "physics_samples_checked": 0, "clipped_physics_samples": 0,
             "max_moving_contacts": 0, "max_constraint_Nm": 0., "max_external_Nm": 0.,
             "max_balance_residual_Nm": 0., "max_motor_command_error_Nm": 0.,
             "max_abs_tracking_error_rad": 0., "max_abs_core_tracking_error_rad": 0.,
             "phase_samples_checked": {name: 0 for name in pilot.PHASE_NAMES},
             "phase_max_tracking_error_rad": {name: 0. for name in pilot.PHASE_NAMES},
             "joint_limit_violation_samples": 0, "minimum_ground_clearance_m": None,
             "finite": True, "warnings_seen": False, "stop_reason": None}
    x, r, z, steps, matrices, bias_rows, compensation = [], [], [], [], [], [], []
    continuous, actual = np.empty((count, 6)), np.empty((count, 6))
    inertia_torque, mass_acc, dense = np.zeros(6), np.zeros(6), np.empty((6, 6))
    updates, held_checks, integrated = 0, 0, 0
    held = np.zeros(6)
    start = time.perf_counter()
    try:
        for k in range(count+1):
            d.qfrc_applied[:] = 0
            d.xfrc_applied[:] = 0
            mujoco.mj_forward(m, d)
            qref, vref, aref, phase = pilot.reference(k*h, a, b)
            if k % control_stride == 0:
                # The same control-index clock is used at every physical step size.
                cq, cv, ca, _ = pilot.reference((k//control_stride)*plan["control_dt_s"], a, b)
                command_acc = ca + pilot.WN**2*(cq-d.qpos) + 2*pilot.WN*(cv-d.qvel)
                mujoco.mj_mulM(m, d, inertia_torque, command_acc)
                damping = pilot.viscous_compensation(m, d.qvel, True)
                raw = inertia_torque + d.qfrc_bias + damping
                d.ctrl[:] = np.clip(raw, lower, upper)
                held = d.ctrl.copy()
                clipped = bool(np.any(np.abs(raw-held) > 1e-9))
                updates += 1
            else:
                require(np.array_equal(d.ctrl, held), "Control changed between control ticks")
                held_checks += 1
            mujoco.mj_forward(m, d)
            contacts = pilot.moving_contacts(m, d, moving)
            clearance = pilot.mesh_ground_clearance(d, geometry)
            violations = limited & ((d.qpos < m.jnt_range[:, 0]) | (d.qpos > m.jnt_range[:, 1]))
            mujoco.mj_mulM(m, d, mass_acc, d.qacc)
            balance = mass_acc+d.qfrc_bias-d.qfrc_passive-d.qfrc_actuator-d.qfrc_applied-d.qfrc_constraint
            err = float(np.max(np.abs(d.qpos-qref)))
            row = np.r_[d.time, d.qpos, d.qvel, d.qfrc_actuator]
            ref = np.r_[d.time, qref, vref, aref, d.ctrl, phase]
            diag = np.r_[d.time, d.qacc, d.qfrc_passive, d.qfrc_applied, d.qfrc_constraint,
                         clipped, d.ncon, len(contacts), np.max(np.abs(balance)),
                         np.sum(d.warning.number), clearance, np.count_nonzero(violations)]
            finite = all(np.isfinite(v).all() for v in (row, ref, diag, raw))
            stats["finite"] &= bool(finite)
            stats["warnings_seen"] |= bool(np.any(d.warning.number))
            stats["physics_samples_checked"] += 1
            stats["clipped_physics_samples"] += int(clipped)
            stats["max_moving_contacts"] = max(stats["max_moving_contacts"], len(contacts))
            stats["joint_limit_violation_samples"] += int(np.any(violations))
            pilot.update_tracking_stats(stats, phase, err)
            if np.isfinite(clearance):
                old = stats["minimum_ground_clearance_m"]
                stats["minimum_ground_clearance_m"] = clearance if old is None else min(old, clearance)
            if finite:
                for key, value in (("max_constraint_Nm", np.max(np.abs(d.qfrc_constraint))),
                                   ("max_external_Nm", np.max(np.abs(d.qfrc_applied))),
                                   ("max_balance_residual_Nm", np.max(np.abs(balance))),
                                   ("max_motor_command_error_Nm", np.max(np.abs(d.qfrc_actuator-d.ctrl))),
                                   ("max_abs_tracking_error_rad", err)):
                    stats[key] = max(stats[key], float(value))
                if round(pilot.CORE_START/h) <= k < round(pilot.CORE_END/h):
                    stats["max_abs_core_tracking_error_rad"] = max(stats["max_abs_core_tracking_error_rad"], err)
            failures = [(not finite, "nonfinite"), (stats["warnings_seen"], "MuJoCo warning"),
                        (bool(contacts), "moving-link contact"), (bool(np.any(violations)), "joint limit"),
                        (clearance < pilot.GROUND_CLEARANCE, "ground clearance"),
                        (np.max(np.abs(d.qfrc_constraint)) >= 1e-8, "constraint torque"),
                        (clipped, "torque clipping"), (err > .35, "tracking emergency"),
                        (not np.isclose(d.time, k*h, rtol=0, atol=1e-9), "clock mismatch"),
                        (np.max(np.abs(d.qfrc_passive+m.dof_damping*d.qvel)) >= 1e-10, "non-viscous passive force")]
            reason = next((message for failed, message in failures if failed), None)
            if k % stride == 0 or reason:
                steps.append(k); x.append(row); r.append(ref); z.append(diag)
                mujoco.mj_fullM(m, dense, d.qM)
                matrices.append(dense.copy()); bias_rows.append(d.qfrc_bias.copy())
                compensation.append(np.r_[d.time, damping])
            record_state(k, d)
            if reason:
                stats["stop_reason"] = f"tick {k}: {reason}"
                break
            if k == count:
                stats["completed"] = True
                stats["stop_reason"] = "completed"
                break
            continuous[k] = d.qacc
            before = d.qvel.copy()
            mujoco.mj_step(m, d)
            actual[k] = (d.qvel-before)/h
            integrated += 1
            if k and k % round(5/h) == 0:
                print(f"h={h*1e6:g} us: {k*h:.1f}/17 s", flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        stats["stop_reason"] = f"{type(exc).__name__}: {exc}"
    return {"x": np.asarray(x), "r": np.asarray(r), "z": np.asarray(z), "steps": np.asarray(steps),
            "M": np.asarray(matrices), "bias": np.asarray(bias_rows), "compensation": np.asarray(compensation),
            "continuous": continuous[:integrated], "actual": actual[:integrated], "runtime": stats,
            "simulation_wall_s": time.perf_counter()-start, "control_updates_including_terminal": updates,
            "held_control_checks": held_checks, "physics_integrations": integrated}


def collect(args, plan, out):
    started = time.perf_counter()
    pilot = load_module(args.collector, "original_collector")
    source = Path(args.original_source)
    base = read_json(source/"config.json")
    model = mujoco.MjModel.from_binary_path(str(source/"model_snapshot.mjb"))
    lower, upper = pilot.validate_model(model)
    require(np.array_equal(model.dof_damping, plan["known_integrator_B"]), "Integrator prior mismatch")
    model.opt.timestep = h = plan["physics_dt_s"]
    a, b = pilot.make_coefficients(args.seed, base["amplitude_scale"])
    split = "train" if args.seed in plan["train_seeds"] else "validation" if args.seed == plan["validation_seed"] else "test"
    require(args.seed in plan["train_seeds"]+[plan["validation_seed"], plan["test_seed"]], "Undeclared seed")
    preserved = {key: base[key] for key in ("base", "amplitude_scale", "fundamental_frequency_Hz", "angular_frequencies_rad_s",
                  "coefficient_method", "q0_rad", "q_bound_rad", "duration_s", "core_interval_s", "controller",
                  "damping_compensation", "damping", "frictionloss", "armature")}
    config = dict(preserved, seed=args.seed, sine_coefficients_rad=a.tolist(), cosine_coefficients_rad=b.tolist(),
                  physics_dt_s=h, control_dt_s=plan["control_dt_s"], feedback_dt_s=plan["feedback_dt_s"],
                  split=split, training_signal_rate_Hz=1/h, known_integrator_B=plan["known_integrator_B"],
                  truth_policy="Training packet contains direct state/torque records only; diagnostics separate")
    write_json(out/"config.json", config)
    mujoco.mj_saveModel(model, str(out/"model_snapshot.mjb"), None)
    preflight = pilot.preflight(model, pilot.moving_body_mask(model), a, b, lower, upper)
    write_json(out/"preflight.json", preflight)
    require(preflight["passed"], "Reference safety check failed")
    cs, fs, count = schedule(h, plan["control_dt_s"], plan["feedback_dt_s"], plan["duration_s"])
    ticks = np.arange(count+1)
    values = {"t": np.empty(count+1), "q": np.empty((count+1,6)), "v": np.empty((count+1,6)),
              "tau_actuator": np.empty((count+1,6)), "ctrl": np.empty((count+1,6)),
              "control_update": ticks % cs == 0, "control_index": ticks//cs,
              "feedback_sample": ticks % fs == 0, "physics_dt_s": np.array(h)}
    written = 0
    def record_state(k, data):
        nonlocal written
        require(k == written, "Logging tick skipped")
        values["t"][k] = data.time
        values["q"][k], values["v"][k] = data.qpos, data.qvel
        values["tau_actuator"][k], values["ctrl"][k] = data.qfrc_actuator, data.ctrl
        written += 1
    sim = simulate(model, pilot, plan, a, b, lower, upper, record_state)
    for key in list(values):
        if key != "physics_dt_s": values[key] = values[key][:written]
    require(set(values) == PACKET_FIELDS, "Training packet schema differs")
    np.savez_compressed(out/"training_packet.npz", **values)
    np.savez_compressed(out/"diagnostics_1kHz.npz", **{key:sim[key] for key in ("r","z","M","bias","steps")})
    pilot.save_csv(out/"feedback_1kHz.csv", pilot.MEASUREMENT_HEADER, sim["x"])
    summary, _ = pilot.assess(sim["x"],sim["r"],sim["z"],sim["steps"],sim["runtime"],h)
    write_json(out/"collection_checks.json",summary)
    require(summary["passed"], "Original collection gate failed")
    require(written == count+1 and sim["physics_integrations"] == count, "Incomplete physics logging")
    measured = np.column_stack((values["t"][::fs],values["q"][::fs],values["v"][::fs],values["tau_actuator"][::fs]))
    require(np.array_equal(measured,sim["x"]), "High-rate log does not match feedback")
    held = np.repeat(values["ctrl"][:-1:cs],cs,axis=0)
    require(np.array_equal(held,values["ctrl"][:-1]), "Control not held within update interval")
    require(np.max(np.abs(values["tau_actuator"]-values["ctrl"])) < 1e-10, "Actual motor/control mismatch")
    replay = None
    if args.baseline:
        previous = np.loadtxt(Path(args.baseline)/"joint_feedback.csv",delimiter=",",skiprows=1)
        replay = float(np.max(np.abs(sim["x"]-previous)))
        require(replay <= 1e-10, "Seed18 does not reproduce existing finest trajectory")
    return {"passed":True,"seed":args.seed,"split":split,"physics_steps":count,"state_rows":written,
            "control_updates":int(np.count_nonzero(values["control_update"])),"feedback_rows":len(sim["x"]),
            "original_finest_feedback_max_abs":replay,"recording":"direct actual measurements; no reconstructed targets",
            "wall_seconds":time.perf_counter()-started,"simulation_wall_seconds":sim["simulation_wall_s"],
            "collection_checks":summary["checks"],"coverage":summary["coverage"]}


def main():
    p=argparse.ArgumentParser()
    for name in ("plan","collector","original-source","output-dir"):
        p.add_argument("--"+name,required=True)
    p.add_argument("--seed",required=True,type=int)
    p.add_argument("--baseline")
    args=p.parse_args(); out=Path(args.output_dir); out.mkdir(parents=True,exist_ok=True)
    try:
        result=collect(args,read_json(args.plan),out)
    except (Exception,KeyboardInterrupt) as exc:
        write_json(out/"metrics.json",{"passed":False,"error":f"{type(exc).__name__}: {exc}"})
        raise
    write_json(out/"metrics.json",result)
    print(f"Completed direct high-rate collection seed {args.seed}",flush=True)


if __name__ == "__main__":
    main()
