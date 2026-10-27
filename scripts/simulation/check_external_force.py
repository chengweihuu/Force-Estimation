#!/usr/bin/env python3
"""UR5 step 1.4: a small world-frame force pulse at tool0, with matched control.

Run in the existing MuJoCo 3.3.7 container. No GUI, ROS node, or estimator.
Keep the validated absolute base Z=0.50 m and computed-torque controller.
Hold six zero joint angles; compare independent baseline/force trials.
The controller NEVER reads external-force truth. Measurements are only t/q/v/tau.
External forces are cleared every step, injected once, and logged separately.
This is an interface check, not a paper benchmark or force-estimation result.
"""
import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
WN = 12.0
DURATION = 6.0
PULSE_START = 2.0
GROUND_CLEARANCE = 0.005

def snapshot(m, d, qref, raw):
    """Copy diagnostic information; do not change or advance the simulation."""
    contacts = []
    for i in range(min(d.ncon, 20)):
        c = d.contact[i]
        wrench = np.zeros(6)
        mujoco.mj_contactForce(m, d, i, wrench)
        contacts.append({
            "geom1": m.geom(int(c.geom1)).name or f"geom#{c.geom1}",
            "body1": m.body(int(m.geom_bodyid[c.geom1])).name,
            "geom2": m.geom(int(c.geom2)).name or f"geom#{c.geom2}",
            "body2": m.body(int(m.geom_bodyid[c.geom2])).name,
            "distance_m": float(c.dist), "active": int(c.efc_address) >= 0,
            "force_norm_N": float(np.linalg.norm(wrench[:3])),
        })
    return {
        "t_s": float(d.time), "q_rad": d.qpos.tolist(),
        "qref_rad": qref.tolist(), "error_rad": (d.qpos - qref).tolist(),
        "velocity_rad_s": d.qvel.tolist(), "acceleration_rad_s2": d.qacc.tolist(),
        "raw_command_Nm": raw.tolist(), "ctrl_Nm": d.ctrl.tolist(),
        "actual_torque_Nm": d.qfrc_actuator.tolist(),
        "bias_Nm": d.qfrc_bias.tolist(), "passive_Nm": d.qfrc_passive.tolist(),
        "constraint_Nm": d.qfrc_constraint.tolist(), "contacts_total": int(d.ncon),
        "contacts_first_20": contacts, "warning_counts": d.warning.number.tolist(),
    }

def moving_body_mask(m):
    moving = np.zeros(m.nbody, dtype=bool)
    for body in range(1, m.nbody):
        moving[body] = bool(m.body_dofnum[body]) or moving[m.body_parentid[body]]
    return moving

def set_robot_base_height(m, height):
    """Move the common world-child subtree containing the six robot joints.

    No body name is assumed. Translation is along WORLD Z because this body's
    parent is world. Relative link transforms, gravity and ground are untouched.
    Recompile-derived constants are refreshed, then the caller initializes state.
    """
    if not np.isfinite(height):
        raise ValueError("基座高度必须为有限数值")
    roots = set()
    for name in JOINTS:
        body = int(m.jnt_bodyid[m.joint(name).id])
        while m.body_parentid[body] != 0:
            body = int(m.body_parentid[body])
        roots.add(body)
    if len(roots) != 1:
        raise RuntimeError("六个关节不属于同一个机器人顶层 body，无法整体设置基座高度")
    base = roots.pop()
    if int(m.body_mocapid[base]) >= 0:
        raise RuntimeError("机器人根 body 是 mocap body，不符合本脚本的固定基座假设")
    ground = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    if ground < 0:
        raise RuntimeError("找不到 ground 几何体")
    ancestor = int(m.geom_bodyid[ground])
    while ancestor != 0:
        if ancestor == base:
            raise RuntimeError("ground 在机器人根 body 内，不能与机器人一起移动")
        ancestor = int(m.body_parentid[ancestor])
    probe = mujoco.MjData(m)
    mujoco.mj_forward(m, probe)
    ground_before = probe.geom_xpos[ground].copy()
    before = m.body_pos[base].copy()
    m.body_pos[base, 2] = float(height)
    mujoco.mj_setConst(m, probe)
    mujoco.mj_resetData(m, probe)
    mujoco.mj_forward(m, probe)
    assert np.allclose(probe.geom_xpos[ground], ground_before), "地面位置意外变化"
    after = m.body_pos[base].copy()
    return {"base_body_id": int(base), "base_body_name": m.body(base).name,
            "base_position_before_m": before.tolist(),
            "base_position_after_m": after.tolist(),
            "world_z_target_m": float(height), "actual_shift_z_m": float(after[2] - before[2]),
            "ground_world_position_m": ground_before.tolist(),
            "height_semantics": "absolute world Z; replace XML base Z in memory only"}

def moving_contacts(m, d, moving):
    """Enabled contacts involving at least one moving rigid link.

    Fixed base / ground support contacts are excluded from this check only;
    nothing in the MuJoCo collision model or solver is disabled.
    """
    found = []
    for i in range(d.ncon):
        c = d.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        if g1 < 0 or g2 < 0:
            continue  # this script is validated for rigid UR5 models
        if not (moving[m.geom_bodyid[g1]] or moving[m.geom_bodyid[g2]]):
            continue
        if c.dist <= 0 or c.efc_address >= 0:
            found.append({"geom1": m.geom(g1).name or f"geom#{g1}",
                          "geom2": m.geom(g2).name or f"geom#{g2}",
                          "distance_m": float(c.dist)})
    return found


def save_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


def save_csv(path, header, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def validate_model(m):
    assert m.nq == m.nv == m.njnt == m.nu == 6, "需要六轴 UR5"
    assert [m.joint(i).name for i in range(6)] == JOINTS, "关节顺序不符"
    assert np.all(m.jnt_type == int(mujoco.mjtJoint.mjJNT_HINGE))
    assert np.array_equal(m.jnt_qposadr, np.arange(6))
    assert np.array_equal(m.jnt_dofadr, np.arange(6))
    assert np.all(m.actuator_trntype == int(mujoco.mjtTrn.mjTRN_JOINT))
    assert np.array_equal(m.actuator_trnid[:, 0], np.arange(6))
    assert np.all(m.actuator_dyntype == 0) and np.all(m.actuator_gaintype == 0)
    assert np.all(m.actuator_biastype == 0)
    assert np.allclose(m.actuator_gainprm[:, 0], 1)
    assert np.allclose(m.actuator_gear, np.tile([1., 0., 0., 0., 0., 0.], (6, 1)))
    assert np.all(m.actuator_ctrllimited) and np.all(m.actuator_forcelimited)
    assert np.isclose(m.opt.timestep, 0.0005, rtol=0, atol=1e-12), "步长应为 0.0005 s"
    # The pulse is a zero-order hold over each physics interval. For RK4 the
    # mapping would need refreshing at its internal stages; do not silently use it.
    assert m.opt.integrator != int(mujoco.mjtIntegrator.mjINT_RK4), "本脚本不使用 RK4"
    assert m.nflex == 0, "本脚本针对刚体 UR5"
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tool0") >= 0
    lower = np.maximum(m.actuator_ctrlrange[:, 0], m.actuator_forcerange[:, 0])
    upper = np.minimum(m.actuator_ctrlrange[:, 1], m.actuator_forcerange[:, 1])
    assert np.all(lower < upper)
    return lower, upper


def preflight(m, moving):
    d = mujoco.MjData(m)
    d.qpos[:] = 0
    mujoco.mj_forward(m, d)
    for flag in (mujoco.mjtDisableBit.mjDSBL_CONTACT, mujoco.mjtDisableBit.mjDSBL_CONSTRAINT):
        assert not (int(m.opt.disableflags) & int(flag)), "接触和约束必须保持启用"
    limited = m.jnt_limited.astype(bool)
    assert np.all(m.jnt_range[limited, 0] <= 0) and np.all(m.jnt_range[limited, 1] >= 0)
    contacts = moving_contacts(m, d, moving)
    assert not contacts, f"零位存在运动连杆接触：{contacts}"
    ground = m.geom("ground").id
    geoms = [g for g in range(m.ngeom) if moving[m.geom_bodyid[g]]
             and (m.geom_contype[g] or m.geom_conaffinity[g])]
    assert geoms, "未找到运动连杆碰撞几何体"
    clearance = min(float(mujoco.mj_geomDistance(m, d, ground, g, .05, None)) for g in geoms)
    assert clearance >= GROUND_CLEARANCE, f"零位离地间隙不足 5 mm：{clearance} m"
    assert moving[m.site_bodyid[m.site("tool0").id]], "tool0 应属于运动连杆"
    return {"passed": True, "q0_rad": [0.] * 6,
            "minimum_ground_distance_capped_at_0.05_m": clearance}


def run_trial(m, folder, force_world, start, end, lower, upper, moving):
    """State/torque/force rows all precede integration of the same interval."""
    folder.mkdir()
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    d.qpos[:] = 0
    d.qvel[:] = 0
    site = m.site("tool0").id
    body = int(m.site_bodyid[site])
    dt = float(m.opt.timestep)
    steps = round(DURATION / dt)
    measurements, truth, diagnostics = [], [], []
    qref = np.zeros(6)
    inertia_torque, jacp, jacr = np.zeros(6), np.zeros((3, 6)), np.zeros((3, 6))
    events, onset = {}, None
    stop_reason, completed = None, False
    try:
        for k in range(steps + 1):
            # Applied-force arrays persist in MuJoCo: clear them on EVERY step.
            d.qfrc_applied[:] = 0
            d.xfrc_applied[:] = 0
            mujoco.mj_forward(m, d)
            # Same controller as the successful 50 cm motion check, now qref=0.
            # No force truth, qfrc_applied or qacc enters this controller.
            a_cmd = WN**2 * (qref - d.qpos) - 2 * WN * d.qvel
            mujoco.mj_mulM(m, d, inertia_torque, a_cmd)
            raw = inertia_torque + d.qfrc_bias
            d.ctrl[:] = np.clip(raw, lower, upper)
            clipped = bool(np.any(np.abs(raw - d.ctrl) > 1e-9))
            force = force_world if start <= k < end else np.zeros(3)
            point = d.site_xpos[site].copy()  # current tool0, WORLD coordinates
            # Pure force at tool0, with zero additional couple at that point.
            # mj_applyFT accounts for the moment arm automatically.
            mujoco.mj_applyFT(m, d, force, np.zeros(3), point, body, d.qfrc_applied)
            mujoco.mj_forward(m, d)  # refresh actual torque and acceleration
            mujoco.mj_jacSite(m, d, jacp, jacr, site)
            expected_tau = jacp.T @ force
            mapping_error = float(np.max(np.abs(d.qfrc_applied - expected_tau)))
            if k == start and np.any(force):
                # Controlled same-state intervention: same q/v/ctrl, force on/off.
                # This validates input mechanics only, not an estimator.
                applied = d.qfrc_applied.copy()
                acc_with = d.qacc.copy()
                d.qfrc_applied[:] = 0
                mujoco.mj_forward(m, d)
                acc_without = d.qacc.copy()
                d.qfrc_applied[:] = applied
                mujoco.mj_forward(m, d)
                delta_acc = acc_with - acc_without
                mass_delta_acc = np.zeros(6)
                mujoco.mj_mulM(m, d, mass_delta_acc, delta_acc)
                onset = {"t_s": float(d.time), "force_world_N": force.tolist(),
                         "tool0_world_m": point.tolist(),
                         "tau_ext_Nm": applied.tolist(),
                         "delta_acc_rad_s2": delta_acc.tolist(),
                         "M_delta_acc_Nm": mass_delta_acc.tolist(),
                         "max_abs_balance_error_Nm": float(np.max(np.abs(mass_delta_acc - applied))),
                         "force_direction_acceleration_response": float(force @ (jacp @ delta_acc))}
            robot_contacts = moving_contacts(m, d, moving)
            measurement = np.r_[d.time, d.qpos.copy(), d.qvel.copy(), d.qfrc_actuator.copy()]
            truth_row = np.r_[d.time, force, np.zeros(3), point, d.qfrc_applied.copy()]
            diagnostic = np.r_[d.time, d.ctrl.copy(), raw, d.qacc.copy(),
                               float(clipped), d.ncon, len(robot_contacts),
                               np.max(np.abs(d.qfrc_constraint)), mapping_error]
            measurements.append(measurement)
            truth.append(truth_row)
            diagnostics.append(diagnostic)
            if k in (0, start, end, steps):
                events[f"step_{k}"] = snapshot(m, d, qref, raw)
            if not all(np.isfinite(x).all() for x in (measurement, truth_row, diagnostic)):
                stop_reason = f"t={d.time:.6f}: 出现非有限数值"
            elif np.any(d.warning.number):
                stop_reason = f"t={d.time:.6f}: MuJoCo warnings={d.warning.number.tolist()}"
            elif robot_contacts:
                stop_reason = f"t={d.time:.6f}: 存在运动连杆接触：{robot_contacts}"
            elif np.max(np.abs(d.qpos)) > .35:
                stop_reason = f"t={d.time:.6f}: 姿态误差超过 0.35 rad"
            elif not np.isclose(d.time, k * dt, rtol=0, atol=1e-9):
                stop_reason = f"t={d.time:.6f}: 仿真时钟异常"
            if stop_reason:
                events["failure"] = snapshot(m, d, qref, raw)
                break
            if k == steps:
                completed = True
                break
            mujoco.mj_step(m, d)
    except (Exception, KeyboardInterrupt) as exc:
        stop_reason = f"{type(exc).__name__}: {exc}"

    a = np.asarray(measurements, dtype=float).reshape(-1, 19)
    f = np.asarray(truth, dtype=float).reshape(-1, 16)
    g = np.asarray(diagnostics, dtype=float).reshape(-1, 24)
    save_csv(folder / "joint_feedback.csv", ["t_s"]
             + [f"q{i}_rad" for i in range(1, 7)]
             + [f"v{i}_rad_s" for i in range(1, 7)]
             + [f"tau{i}_Nm" for i in range(1, 7)], a)
    save_csv(folder / "external_force_truth.csv", ["t_s"]
             + [f"F{axis}_world_N" for axis in "xyz"]
             + [f"M{axis}_at_tool0_world_Nm" for axis in "xyz"]
             + [f"tool0_{axis}_world_m" for axis in "xyz"]
             + [f"tau_ext{i}_Nm" for i in range(1, 7)], f)
    save_csv(folder / "diagnostics.csv", ["t_s"]
             + [f"ctrl{i}_Nm" for i in range(1, 7)]
             + [f"raw{i}_Nm" for i in range(1, 7)]
             + [f"qacc{i}_rad_s2" for i in range(1, 7)]
             + ["clipped", "ncon", "moving_contacts", "max_constraint_Nm", "mapping_error_Nm"], g)
    save_json(folder / "events.json", events)
    save_json(folder / "onset_dynamics_check.json", onset)
    report = {"completed": completed, "stop_reason": stop_reason or "完成 6 秒",
              "rows": len(a), "final_time_s": float(a[-1, 0]) if len(a) else None,
              "max_abs_q_rad": float(np.max(np.abs(a[:, 1:7]))) if len(a) else None,
              "clipped_samples": int(np.count_nonzero(g[:, 19])) if len(g) else None}
    save_json(folder / "trial_summary.json", report)
    print(folder.name, report, flush=True)
    return {"measurement": a, "truth": f, "diagnostic": g, "report": report, "onset": onset}


def summarize(baseline, loaded, dt, force, start, end):
    checks = {}
    for name, trial in (("baseline", baseline), ("force", loaded)):
        a, f, g = trial["measurement"], trial["truth"], trial["diagnostic"]
        nonempty = len(a) > 1
        checks[name + "_completed_6_seconds"] = trial["report"]["completed"]
        checks[name + "_finite_samples"] = nonempty and all(np.isfinite(x).all() for x in (a, f, g))
        checks[name + "_synchronized_uniform_time"] = (nonempty and np.array_equal(a[:, 0], f[:, 0])
            and np.array_equal(a[:, 0], g[:, 0]) and np.allclose(np.diff(a[:, 0]), dt, rtol=0, atol=1e-10))
        checks[name + "_no_contact_or_constraint"] = nonempty and np.all(g[:, 21] == 0) and np.max(g[:, 22]) < 1e-8
        checks[name + "_no_command_clipping"] = nonempty and np.all(g[:, 19] == 0)
        checks[name + "_actual_motor_torque_matches_command"] = nonempty and np.allclose(a[:, 13:19], g[:, 1:7], atol=1e-10, rtol=0)
        checks[name + "_force_mapping_J_transpose_F"] = nonempty and np.max(g[:, 23]) < 1e-10
        desired = np.zeros((len(f), 3))
        if name == "force":
            desired[start:min(end, len(f))] = force
        checks[name + "_exact_force_schedule"] = nonempty and np.array_equal(f[:, 1:4], desired)
    a, b = baseline["measurement"], loaded["measurement"]
    f = loaded["truth"]
    checks["baseline_holds_zero_below_0.01_rad"] = len(a) > 0 and np.max(np.abs(a[:, 1:7])) < .01
    count = min(len(a), len(b))
    stats = {}
    checks["matched_before_pulse"] = count >= start and np.allclose(a[:start], b[:start], rtol=0, atol=1e-12)
    onset = loaded["onset"]
    checks["same_state_dynamics_M_delta_acc_equals_tau_ext"] = onset is not None and onset["max_abs_balance_error_Nm"] < 1e-8
    checks["same_state_force_direction_response"] = onset is not None and onset["force_direction_acceleration_response"] > 1e-10
    checks["force_removed_after_pulse"] = len(f) > end and np.all(f[end:, 1:7] == 0) and np.all(f[end:, 10:16] == 0)
    checks["nonzero_joint_response"] = False
    checks["recovered_after_removal"] = False
    if count > end and np.isfinite(a[:count]).all() and np.isfinite(b[:count]).all():
        delta = b[:count, 1:] - a[:count, 1:]
        peak_q = float(np.max(np.abs(delta[start:, :6])))
        peak_v = float(np.max(np.abs(delta[start:, 6:12])))
        tail = b[:count, 0] >= DURATION - .5
        tail_q = float(np.max(np.abs(delta[tail, :6]))) if np.any(tail) else None
        tail_v = float(np.max(np.abs(delta[tail, 6:12]))) if np.any(tail) else None
        stats = {"peak_joint_difference_rad": peak_q, "peak_velocity_difference_rad_s": peak_v,
                 "tail_0_5s_max_joint_difference_rad": tail_q,
                 "tail_0_5s_max_velocity_difference_rad_s": tail_v,
                 "peak_difference_per_joint_rad": np.max(np.abs(delta[:, :6]), axis=0).tolist(),
                 "delivered_impulse_world_Ns": (np.sum(f[:-1, 1:4], axis=0) * dt).tolist()}
        checks["nonzero_joint_response"] = peak_q > 1e-8 and peak_v > 1e-7
        checks["recovered_after_removal"] = (tail_q is not None and tail_v is not None
            and tail_q < max(1e-7, .05 * peak_q) and tail_v < max(1e-6, .05 * peak_v))
    checks = {key: bool(value) for key, value in checks.items()}
    return {"passed": all(checks.values()), "checks": checks, "statistics": stats,
            "threshold_note": "Engineering interface checks, not paper accuracy criteria."}


def plot_comparison(baseline, loaded, path, dt, start, end):
    a, b, f = baseline["measurement"], loaded["measurement"], loaded["truth"]
    n = min(len(a), len(b))
    if n < 2:
        return
    t, delta = b[:n, 0], b[:n, 1:] - a[:n, 1:]
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    for i, axis in enumerate("xyz"):
        axes[0].step(f[:, 0], f[:, i + 1], where="post", label=f"F{axis}")
    axes[0].set_ylabel("World force [N]")
    axes[0].legend(ncol=3)
    for j in range(6):
        axes[1].plot(t, np.rad2deg(delta[:, j]), label=f"J{j+1}")
        axes[2].plot(t, delta[:, j + 6])
        axes[3].plot(t, delta[:, j + 12])
    axes[1].set_ylabel("q(force) - q(base) [deg]")
    axes[1].legend(ncol=6)
    axes[2].set_ylabel("v difference [rad/s]")
    axes[3].set_ylabel("Motor torque difference [Nm]")
    axes[3].set_xlabel("Simulation time [s]")
    for ax in axes:
        ax.axvspan(start * dt, end * dt, color="orange", alpha=.15)
        ax.grid(alpha=.25)
    fig.suptitle("UR5 step 1.4: tool0 force pulse vs independent no-force trial")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    for ax in axes:
        ax.set_xlim(start * dt - .1, min(DURATION, end * dt + .8))
    fig.savefig(path.with_name("force_response_zoom.png"), dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=Path(__file__).resolve().parents[1] / "models" / "ur5" / "scene.xml")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[2] / "runs" / "manual_checks")
    parser.add_argument("--base-z", type=float, default=.5, help="基座绝对世界 Z，m")
    parser.add_argument("--force-world", type=float, nargs=3, default=[0., 0., 1.], metavar=("FX", "FY", "FZ"))
    parser.add_argument("--pulse-width", type=float, default=.02, help="脉冲持续秒数，必须为步长整数倍")
    args = parser.parse_args()
    assert mujoco.__version__ == "3.3.7", f"需要 MuJoCo 3.3.7，当前 {mujoco.__version__}"
    force = np.asarray(args.force_world, dtype=float)
    assert np.isfinite(force).all() and np.linalg.norm(force) > 0, "外力必须有限且非零"
    assert np.isfinite(args.pulse_width) and 0 < args.pulse_width <= 1., "脉冲宽度应在 (0, 1] s"
    model_path = args.model.resolve(strict=True)
    m = mujoco.MjModel.from_xml_path(str(model_path))
    lower, upper = validate_model(m)
    base_info = set_robot_base_height(m, args.base_z)
    moving = moving_body_mask(m)
    dt = float(m.opt.timestep)
    width_steps = round(args.pulse_width / dt)
    assert width_steps >= 1 and abs(width_steps * dt - args.pulse_width) < 1e-12, "脉冲宽度不是步长整数倍"
    start, end = round(PULSE_START / dt), round(PULSE_START / dt) + width_steps
    folder = args.output.resolve() / ("step14_force_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:6])
    folder.mkdir(parents=True)
    config = {"model": str(model_path), "mujoco_version": mujoco.__version__,
              "numpy_version": np.__version__, "dt_s": dt, "duration_s": DURATION,
              "base": base_info, "qref_rad": [0.] * 6, "controller_wn_rad_s": WN,
              "force_world_N": force.tolist(), "application_site": "tool0",
              "additional_couple_at_tool0_world_Nm": [0.] * 3,
              "pulse_interval_s": [start * dt, end * dt], "interval_convention": "[start, end)",
              "pulse_steps": width_steps, "force_hold": "constant generalized force per physics interval; remapped each step",
              "expected_impulse_world_Ns": (force * width_steps * dt).tolist(),
              "measurement_columns": "t_s, q1..6_rad, v1..6_rad_s, tau1..6_Nm; actual actuator torque only",
              "truth_note": "Scripted simulator input, not a noisy sensor reading or estimator output."}
    save_json(folder / "config.json", config)
    print("模块 1.4：tool0 外力脉冲与无外力对照；基座 Z=", args.base_z, "m")
    print("六轴目标角度全为 0；控制器与已通过的版本相同。")
    print(f"世界坐标外力 {force.tolist()} N，区间 [{start*dt:.4f}, {end*dt:.4f}) s，共 {width_steps} 步")
    print("每组 6 秒；输出目录：", folder, flush=True)
    try:
        report = preflight(m, moving)
    except Exception as exc:
        save_json(folder / "preflight.json", {"passed": False, "reason": str(exc)})
        print("[CHECK] 几何预检失败：", exc)
        return 2
    save_json(folder / "preflight.json", report)
    baseline = run_trial(m, folder / "baseline", np.zeros(3), start, end, lower, upper, moving)
    if not baseline["report"]["completed"]:
        print("[CHECK] 无外力对照失败，数据已保存；暂不运行施力组。")
        return 2
    loaded = run_trial(m, folder / "with_force", force, start, end, lower, upper, moving)
    summary = summarize(baseline, loaded, dt, force, start, end)
    save_json(folder / "summary.json", summary)
    plot_comparison(baseline, loaded, folder / "force_response.png", dt, start, end)
    for name, passed in summary["checks"].items():
        print("[PASS]" if passed else "[CHECK]", name)
    print("响应统计：", json.dumps(summary["statistics"], ensure_ascii=False))
    print("曲线：", folder / "force_response.png")
    print("局部放大：", folder / "force_response_zoom.png")
    print("统计：", folder / "summary.json")
    print("[PASS] 1.4 外力注入、同步记录、对照响应和撤力恢复检查通过。" if summary["passed"]
          else "[CHECK] 检查未全部通过；已保留数据，请先排查。")
    return 0 if summary["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
