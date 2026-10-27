#!/usr/bin/env python3
"""UR5 step 2.1: ONE no-external-force Fourier pilot trajectory.

MuJoCo 3.3.7, fixed base world Z=0.50 m, six torque motors, zero start.
Physics/control: 2 kHz. Raw feedback: 1 kHz, sampled before integration.
PIAKF paper Sec. 4.2 motivates five harmonics, f0=0.1 Hz and a 10 s core.
These are NEW conservative UR5 coefficients, NOT the authors' optimized ones.
This is a collection/trajectory pilot, not a complete training dataset, not
proof of persistent excitation, and not a paper force-estimation result.
No dependencies on scripts from earlier steps. No GUI or ROS required.
An explicit --output-dir (new run directory) or --output (parent) is required.
Use --compensate-damping to add known viscous damping to the motor command.
"""
import argparse
import csv
import hashlib
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
DURATION = 17.0
CORE_START, CORE_END = 3.0, 13.0
RAMP_DURATION = 2.0
SAMPLE_DT = .001
F0 = .1  # Hz; angular frequency is 2*pi*F0, not 0.1 rad/s
AMPLITUDE_BOUNDS = np.array([.10, .08, .08, .04, .04, .02])  # rad
GROUND_CLEARANCE = .005
PHASE_NAMES = ("initial_hold", "smooth_entry", "excitation", "smooth_exit", "recovery")
# Initial hold < 0.01 rad; every subsequent phase < 0.02 rad.
TRACKING_LIMITS = (.01, .02, .02, .02, .02)
MIN_SPAN_RATIO = .5
MEASUREMENT_HEADER = (["t_s"] + [f"q{i}_rad" for i in range(1, 7)]
                      + [f"v{i}_rad_s" for i in range(1, 7)]
                      + [f"tau{i}_Nm" for i in range(1, 7)])

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


def ground_clearance_geometry(m, moving):
    """Cache compiled collision meshes; their vertices use the geom frame.

    Mesh-plane distance is the minimum vertex projection onto the plane normal.
    This avoids treating mj_geomDistance's cutoff as an exact clearance.
    Fail explicitly if a different model needs a different geometric check.
    """
    ground = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    if ground < 0 or m.geom_type[ground] != mujoco.mjtGeom.mjGEOM_PLANE:
        raise ValueError("离地检查需要名为 ground 的 plane")
    meshes = []
    for g in range(m.ngeom):
        if not moving[m.geom_bodyid[g]] or not (m.geom_contype[g] or m.geom_conaffinity[g]):
            continue
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            raise ValueError(f"离地检查仅支持运动碰撞 mesh：geom#{g}")
        mesh = int(m.geom_dataid[g])
        start, count = int(m.mesh_vertadr[mesh]), int(m.mesh_vertnum[mesh])
        vertices = m.mesh_vert[start:start+count].astype(float)
        if not count or not np.isfinite(vertices).all():
            raise ValueError(f"碰撞 mesh 顶点无效：geom#{g}")
        meshes.append((g, vertices))
    if not meshes:
        raise ValueError("没有可检查的运动连杆碰撞 mesh")
    return ground, meshes


def mesh_ground_clearance(d, geometry):
    """Signed distance in metres, using current forward-computed transforms."""
    ground, meshes = geometry
    normal = d.geom_xmat[ground].reshape(3, 3)[:, 2]
    origin = d.geom_xpos[ground]
    distances = []
    for geom, vertices in meshes:
        rotation = d.geom_xmat[geom].reshape(3, 3)
        distances.append(np.min(vertices @ (rotation.T @ normal))
                         + np.dot(d.geom_xpos[geom] - origin, normal))
    # np.min propagates a NaN from ANY mesh into the finite-value guard.
    return float(np.min(distances))


def update_tracking_stats(stats, phase, error):
    """Called at physics rate, including frames omitted from 1 kHz feedback."""
    name = PHASE_NAMES[phase]
    stats["phase_samples_checked"][name] += 1
    if np.isfinite(error):
        stats["phase_max_tracking_error_rad"][name] = max(
            stats["phase_max_tracking_error_rad"][name], float(error))


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
    # Controls are held over each physics step; RK4 callbacks are not used.
    assert m.opt.integrator != int(mujoco.mjtIntegrator.mjINT_RK4), "本脚本不使用 RK4"
    assert m.nflex == 0, "本脚本针对刚体 UR5"
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tool0") >= 0
    lower = np.maximum(m.actuator_ctrlrange[:, 0], m.actuator_forcerange[:, 0])
    upper = np.minimum(m.actuator_ctrlrange[:, 1], m.actuator_forcerange[:, 1])
    assert np.all(lower < upper)
    return lower, upper


def make_coefficients(seed, scale):
    rng = np.random.default_rng(seed)
    orders = np.arange(1., 6.)
    coefficients = rng.normal(size=(6, 2, 5)) / orders**2
    # Sum of absolute sine/cosine amplitudes gives a conservative position bound.
    norm = np.sum(np.abs(coefficients), axis=(1, 2))
    coefficients *= (AMPLITUDE_BOUNDS * scale / norm)[:, None, None]
    return coefficients[:, 0, :], coefficients[:, 1, :]


def smoothstep(u):
    """Quintic envelope and its first two derivatives w.r.t. u."""
    return (10*u**3 - 15*u**4 + 6*u**5,
            30*u**2 - 60*u**3 + 30*u**4,
            60*u - 180*u**2 + 120*u**3)


def reference(t, sin_coeff, cos_coeff):
    """C2 ramp * five-harmonic position signal; exact analytic derivatives."""
    if t < 1.0 or t >= 15.0:
        return np.zeros(6), np.zeros(6), np.zeros(6), 0 if t < 1 else 4
    if t < CORE_START:
        e, de, dde = smoothstep((t - 1.0) / RAMP_DURATION)
        de, dde, phase = de / RAMP_DURATION, dde / RAMP_DURATION**2, 1
    elif t < CORE_END:
        e, de, dde, phase = 1., 0., 0., 2
    else:
        e, de, dde = smoothstep((t - CORE_END) / RAMP_DURATION)
        e, de, dde, phase = 1-e, -de/RAMP_DURATION, -dde/RAMP_DURATION**2, 3
    w = 2*np.pi*F0*np.arange(1., 6.)
    s, c = np.sin(w*(t-CORE_START)), np.cos(w*(t-CORE_START))
    h = sin_coeff @ s + cos_coeff @ c
    dh = sin_coeff @ (w*c) - cos_coeff @ (w*s)
    ddh = -sin_coeff @ (w*w*s) - cos_coeff @ (w*w*c)
    return e*h, de*h+e*dh, dde*h+2*de*dh+e*ddh, phase


def preflight(m, moving, a, b, lower, upper):
    """Sample the reference every 10 ms. Runtime checks cover actual states."""
    for flag in (mujoco.mjtDisableBit.mjDSBL_CONTACT, mujoco.mjtDisableBit.mjDSBL_CONSTRAINT):
        if int(m.opt.disableflags) & int(flag):
            return {"passed": False, "reason": "接触和约束计算必须保持开启"}
    d = mujoco.MjData(m)
    try:
        geometry = ground_clearance_geometry(m, moving)
    except ValueError as exc:
        return {"passed": False, "reason": str(exc)}
    ground, meshes = geometry
    limited = m.jnt_limited.astype(bool)
    minimum, torque_peak = .05, np.zeros(6)
    mesh_minimum = None
    q_peak, v_peak, acc_peak = np.zeros(6), np.zeros(6), np.zeros(6)
    report = {"passed": False, "sample_dt_s": .01, "samples_checked": 0,
              "note": "Sampled reference check only; actual motion is checked every physics step."}
    for i, t in enumerate(np.linspace(0, DURATION, round(DURATION/.01)+1)):
        q, v, acc, _ = reference(float(t), a, b)
        report.update(samples_checked=i+1, checked_until_s=float(t))
        if np.any(q[limited] < m.jnt_range[limited, 0]) or np.any(q[limited] > m.jnt_range[limited, 1]):
            report.update(reason="参考姿态超过关节限位", q_rad=q.tolist())
            return report
        d.qpos[:], d.qvel[:] = q, v
        mujoco.mj_forward(m, d)
        contacts = moving_contacts(m, d, moving)
        if contacts:
            report.update(reason="参考轨迹存在运动连杆接触", contacts=contacts)
            return report
        clearance = mesh_ground_clearance(d, geometry)
        if not np.isfinite(clearance) or clearance < GROUND_CLEARANCE:
            report.update(reason="参考轨迹网格离地间隙无效或不足 5 mm", distance_m=clearance)
            return report
        mesh_minimum = clearance if mesh_minimum is None else min(mesh_minimum, clearance)
        for geom, _ in meshes:
            distance = float(mujoco.mj_geomDistance(m, d, ground, geom, .05, None))
            minimum = min(minimum, distance)
            if not np.isfinite(distance) or distance < GROUND_CLEARANCE:
                report.update(reason="参考轨迹离地间隙不足 5 mm", geom=m.geom(geom).name, distance_m=distance)
                return report
        inertia_torque = np.zeros(6)
        mujoco.mj_mulM(m, d, inertia_torque, acc)
        # Feasibility diagnostic only. The runtime controller's optional
        # compensation is limited to known viscous damping, not all passive forces.
        required = inertia_torque + d.qfrc_bias - d.qfrc_passive
        if np.any(required < lower) or np.any(required > upper):
            report.update(reason="理想跟踪所需力矩超过驱动器限幅", required_torque_Nm=required.tolist())
            return report
        torque_peak = np.maximum(torque_peak, np.abs(required))
        q_peak, v_peak, acc_peak = (np.maximum(q_peak, np.abs(q)),
                                  np.maximum(v_peak, np.abs(v)), np.maximum(acc_peak, np.abs(acc)))
    report.update(passed=True, minimum_ground_distance_capped_at_0_05_m=minimum,
                  minimum_mesh_ground_clearance_m=mesh_minimum,
                  ideal_torque_abs_peak_Nm=torque_peak.tolist(), q_abs_peak_rad=q_peak.tolist(),
                  v_abs_peak_rad_s=v_peak.tolist(), acc_abs_peak_rad_s2=acc_peak.tolist())
    return report


def viscous_compensation(m, velocity, enabled=False):
    """Known viscous feedforward (+b*v); plant damping remains active.

    This only changes the controller. The recorded motor torque still includes
    the torque used to overcome damping; qfrc_passive is not a control input.
    """
    return m.dof_damping * velocity if enabled else np.zeros_like(velocity)


def run_simulation(m, folder, a, b, lower, upper, moving, compensate_damping=False):
    geometry = ground_clearance_geometry(m, moving)
    limited = m.jnt_limited.astype(bool)
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    d.qpos[:] = 0
    d.qvel[:] = 0
    dt = float(m.opt.timestep)
    stride, count = round(SAMPLE_DT / dt), round(DURATION / dt)
    sample_steps, measurements, references, diagnostics = [], [], [], []
    compensation_rows = []
    inertia_torque, mass_acc = np.zeros(6), np.zeros(6)
    events = {}
    stats = {"completed": False, "physics_samples_checked": 0, "clipped_physics_samples": 0,
             "max_moving_contacts": 0, "max_constraint_Nm": 0., "max_external_Nm": 0.,
             "max_balance_residual_Nm": 0., "max_motor_command_error_Nm": 0.,
             "max_abs_tracking_error_rad": 0., "max_abs_core_tracking_error_rad": 0.,
             "phase_samples_checked": {name: 0 for name in PHASE_NAMES},
             "phase_max_tracking_error_rad": {name: 0. for name in PHASE_NAMES},
             "joint_limit_violation_samples": 0, "minimum_ground_clearance_m": None,
             "finite": True, "warnings_seen": False, "stop_reason": None}
    try:
        for k in range(count + 1):
            d.qfrc_applied[:] = 0
            d.xfrc_applied[:] = 0
            mujoco.mj_forward(m, d)
            qref, vref, aref, phase = reference(k*dt, a, b)
            accel_cmd = aref + WN**2*(qref-d.qpos) + 2*WN*(vref-d.qvel)
            mujoco.mj_mulM(m, d, inertia_torque, accel_cmd)
            damping_torque = viscous_compensation(m, d.qvel, compensate_damping)
            raw = inertia_torque + d.qfrc_bias + damping_torque
            d.ctrl[:] = np.clip(raw, lower, upper)
            clipped = bool(np.any(np.abs(raw-d.ctrl) > 1e-9))
            # Refresh derived quantities at current t/q/v after updating control.
            mujoco.mj_forward(m, d)
            contacts = moving_contacts(m, d, moving)
            clearance = mesh_ground_clearance(d, geometry)
            joint_violations = limited & ((d.qpos < m.jnt_range[:, 0]) | (d.qpos > m.jnt_range[:, 1]))
            mujoco.mj_mulM(m, d, mass_acc, d.qacc)
            balance = mass_acc + d.qfrc_bias - d.qfrc_passive - d.qfrc_actuator - d.qfrc_applied - d.qfrc_constraint
            err = float(np.max(np.abs(d.qpos-qref)))
            row = np.r_[d.time, d.qpos.copy(), d.qvel.copy(), d.qfrc_actuator.copy()]
            ref = np.r_[d.time, qref, vref, aref, d.ctrl.copy(), phase]
            diag = np.r_[d.time, d.qacc.copy(), d.qfrc_passive.copy(), d.qfrc_applied.copy(),
                         d.qfrc_constraint.copy(), clipped, d.ncon, len(contacts),
                         np.max(np.abs(balance)), np.sum(d.warning.number),
                         clearance, np.count_nonzero(joint_violations)]
            finite = all(np.isfinite(x).all() for x in (row, ref, diag, raw))
            stats["finite"] &= bool(finite)
            stats["warnings_seen"] |= bool(np.any(d.warning.number))
            stats["physics_samples_checked"] += 1
            stats["clipped_physics_samples"] += int(clipped)
            stats["max_moving_contacts"] = max(stats["max_moving_contacts"], len(contacts))
            stats["joint_limit_violation_samples"] += int(np.any(joint_violations))
            update_tracking_stats(stats, phase, err)
            if np.isfinite(clearance):
                old_min = stats["minimum_ground_clearance_m"]
                stats["minimum_ground_clearance_m"] = clearance if old_min is None else min(old_min, clearance)
            if finite:
                for key, value in (("max_constraint_Nm", np.max(np.abs(d.qfrc_constraint))),
                                   ("max_external_Nm", np.max(np.abs(d.qfrc_applied))),
                                   ("max_balance_residual_Nm", np.max(np.abs(balance))),
                                   ("max_motor_command_error_Nm", np.max(np.abs(d.qfrc_actuator-d.ctrl))),
                                   ("max_abs_tracking_error_rad", err)):
                    stats[key] = max(stats[key], float(value))
                if round(CORE_START/dt) <= k < round(CORE_END/dt):
                    stats["max_abs_core_tracking_error_rad"] = max(stats["max_abs_core_tracking_error_rad"], err)
            if not finite:
                stats["stop_reason"] = f"t={d.time:.6f}: 非有限数值"
            elif stats["warnings_seen"]:
                stats["stop_reason"] = f"t={d.time:.6f}: MuJoCo warnings={d.warning.number.tolist()}"
            elif contacts:
                stats["stop_reason"] = f"t={d.time:.6f}: 运动连杆接触：{contacts}"
            elif np.any(joint_violations):
                names = [JOINTS[j] for j in np.flatnonzero(joint_violations)]
                stats["stop_reason"] = f"t={d.time:.6f}: 实际关节越界：{names}"
            elif clearance < GROUND_CLEARANCE:
                stats["stop_reason"] = f"t={d.time:.6f}: 实际离地间隙 {clearance:.6g} m 不足 5 mm"
            elif np.max(np.abs(d.qfrc_constraint)) >= 1e-8:
                stats["stop_reason"] = f"t={d.time:.6f}: 约束力矩达到 1e-8 Nm"
            elif clipped:
                stats["stop_reason"] = f"t={d.time:.6f}: 控制力矩触及限幅"
            elif err > .35:
                stats["stop_reason"] = f"t={d.time:.6f}: 跟踪误差超过 0.35 rad"
            elif not np.isclose(d.time, k*dt, rtol=0, atol=1e-9):
                stats["stop_reason"] = f"t={d.time:.6f}: 仿真时钟异常"
            # A failed off-grid sample is retained. It will correctly fail the
            # uniform-time gate and cannot become an accepted pilot segment.
            if k % stride == 0 or stats["stop_reason"]:
                sample_steps.append(k)
                measurements.append(row)
                references.append(ref)
                diagnostics.append(diag)
                compensation_rows.append(np.r_[d.time, damping_torque])
            if k in (0, round(CORE_START/dt), round(CORE_END/dt), count) or stats["stop_reason"]:
                events[f"step_{k}"] = snapshot(m, d, qref, raw)
                events[f"step_{k}"].update(ground_clearance_m=clearance,
                                           damping_compensation_Nm=damping_torque.tolist(),
                                           joint_limit_violations=[JOINTS[j] for j in np.flatnonzero(joint_violations)])
            if stats["stop_reason"]:
                break
            if k == count:
                stats["completed"] = True
                stats["stop_reason"] = "完成 17 秒"
                break
            if k and k % round(5/dt) == 0:
                print(f"仿真进度：{d.time:.1f} / {DURATION:.1f} s", flush=True)
            mujoco.mj_step(m, d)
    except (Exception, KeyboardInterrupt) as exc:
        stats["stop_reason"] = f"{type(exc).__name__}: {exc}"
    x = np.asarray(measurements, dtype=float).reshape(-1, 19)
    r = np.asarray(references, dtype=float).reshape(-1, 26)
    z = np.asarray(diagnostics, dtype=float).reshape(-1, 32)
    save_csv(folder / "joint_feedback.csv", MEASUREMENT_HEADER, x)
    save_csv(folder / "controller_reference.csv", ["t_s"]
             + [f"qref{i}_rad" for i in range(1, 7)]
             + [f"vref{i}_rad_s" for i in range(1, 7)]
             + [f"aref{i}_rad_s2" for i in range(1, 7)]
             + [f"ctrl{i}_Nm" for i in range(1, 7)] + ["phase_id"], r)
    save_csv(folder / "simulator_diagnostics.csv", ["t_s"]
             + [f"qacc_truth{i}_rad_s2" for i in range(1, 7)]
             + [f"passive{i}_Nm" for i in range(1, 7)]
             + [f"tau_ext{i}_Nm" for i in range(1, 7)]
             + [f"constraint{i}_Nm" for i in range(1, 7)]
             + ["clipped", "ncon", "moving_contacts", "balance_residual_Nm", "warning_count",
                "ground_clearance_m", "joint_limit_violations"], z)
    save_json(folder / "events.json", events)
    save_csv(folder / "damping_compensation.csv", ["t_s"]
             + [f"damping_compensation{i}_Nm" for i in range(1, 7)], compensation_rows)
    save_json(folder / "runtime.json", stats)
    return x, r, z, np.asarray(sample_steps), stats


def assess(x, r, z, steps, stats, dt):
    core = (steps >= round(CORE_START/dt)) & (steps < round(CORE_END/dt))
    enough = len(x) > 1
    checks = {
        "completed_17_seconds": stats["completed"],
        "finite_samples": enough and stats["finite"],
        "no_mujoco_warnings": not stats["warnings_seen"],
        "synchronized_1kHz_feedback": enough and np.allclose(np.diff(x[:, 0]), SAMPLE_DT, rtol=0, atol=1e-10)
            and np.array_equal(x[:, 0], r[:, 0]) and np.array_equal(x[:, 0], z[:, 0]),
        "exactly_10000_excitation_samples": int(np.count_nonzero(core)) == 10000,
        "no_moving_link_contact": stats["max_moving_contacts"] == 0,
        "actual_joints_within_limits": stats["joint_limit_violation_samples"] == 0,
        "actual_ground_clearance_at_least_5mm": stats["minimum_ground_clearance_m"] is not None
            and stats["minimum_ground_clearance_m"] >= GROUND_CLEARANCE,
        "no_constraint_torque": stats["max_constraint_Nm"] < 1e-8,
        "no_external_force": stats["max_external_Nm"] == 0,
        "no_command_clipping": stats["clipped_physics_samples"] == 0,
        "actual_motor_torque_matches_command": stats["max_motor_command_error_Nm"] < 1e-10,
        "simulator_dynamics_balance": stats["max_balance_residual_Nm"] < 1e-7,
        "core_tracking_error_below_0_02_rad": np.any(core) and stats["max_abs_core_tracking_error_rad"] < .02,
        "all_joints_move_and_reverse": False,
        "all_joints_cover_reference_span": False,
    }
    for name, limit in zip(PHASE_NAMES, TRACKING_LIMITS):
        checks[f"{name}_tracking_error"] = (stats["phase_samples_checked"][name] > 0
            and stats["phase_max_tracking_error_rad"][name] < limit)
    coverage = []
    if np.any(core) and np.isfinite(x[core]).all() and np.isfinite(r[core]).all():
        for j in range(6):
            q, v, tau = x[core, 1+j], x[core, 7+j], x[core, 13+j]
            err = q - r[core, 1+j]
            actual_span, reference_span = float(np.ptp(q)), float(np.ptp(r[core, 1+j]))
            span_ratio = actual_span / reference_span if reference_span > 0 else None
            signs = np.sign(v[np.abs(v) > .001])
            reversals = int(np.count_nonzero(signs[1:] != signs[:-1]))
            coverage.append({"joint": JOINTS[j], "q_min_rad": float(q.min()), "q_max_rad": float(q.max()),
                             "q_span_rad": actual_span, "reference_q_span_rad": reference_span,
                             "actual_reference_span_ratio": span_ratio, "v_min_rad_s": float(v.min()),
                             "v_max_rad_s": float(v.max()), "reversals_with_0_001_rad_s_deadband": reversals,
                             "tracking_max_abs_rad": float(np.max(np.abs(err))),
                             "tracking_rmse_rad": float(np.sqrt(np.mean(err**2))),
                             "torque_abs_peak_Nm": float(np.max(np.abs(tau)))})
        checks["all_joints_move_and_reverse"] = all(
            c["q_span_rad"] > .001 and c["v_min_rad_s"] < -.001 and c["v_max_rad_s"] > .001
            and c["reversals_with_0_001_rad_s_deadband"] >= 1 for c in coverage)
        checks["all_joints_cover_reference_span"] = all(
            c["actual_reference_span_ratio"] is not None
            and c["actual_reference_span_ratio"] >= MIN_SPAN_RATIO for c in coverage)
    checks = {k: bool(v) for k, v in checks.items()}
    return {"passed": all(checks.values()), "checks": checks, "coverage": coverage,
            "runtime": stats, "feedback_rows": len(x), "excitation_rows": int(np.count_nonzero(core)),
            "scope": "One conservative pilot; coverage checks do not establish parameter identifiability."}, core


def plot_results(x, r, folder):
    if len(x) < 2:
        return
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    for j, ax in enumerate(axes.flat):
        ax.plot(x[:, 0], np.rad2deg(r[:, 1+j]), "--", label="Reference", lw=1.3)
        ax.plot(x[:, 0], np.rad2deg(x[:, 1+j]), label="Actual", lw=1)
        ax.axvspan(CORE_START, CORE_END, color="green", alpha=.06)
        ax.set_title(f"J{j+1}: {JOINTS[j]}")
        ax.set_ylabel("Angle [deg]")
        ax.grid(alpha=.25)
    axes[0, 0].legend()
    for ax in axes[-1]:
        ax.set_xlabel("Simulation time [s]")
    fig.suptitle("UR5 pilot: six-joint tracking; shaded interval = 10 s excitation")
    fig.tight_layout()
    fig.savefig(folder / "trajectory_tracking.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for j in range(6):
        axes[0].plot(x[:, 0], x[:, 1+j]-r[:, 1+j], label=f"J{j+1}")
        axes[1].plot(x[:, 0], x[:, 7+j])
        axes[2].plot(x[:, 0], x[:, 13+j])
    for ax, ylabel in zip(axes, ("Tracking error [rad]", "Actual velocity [rad/s]", "Actual motor torque [Nm]")):
        ax.set_ylabel(ylabel)
        ax.axvspan(CORE_START, CORE_END, color="green", alpha=.06)
        ax.grid(alpha=.25)
    axes[0].legend(ncol=6)
    axes[-1].set_xlabel("Simulation time [s]")
    fig.tight_layout()
    fig.savefig(folder / "joint_signals.png", dpi=150)
    plt.close(fig)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=Path(__file__).resolve().parents[1] / "models" / "ur5" / "scene.xml")
    output = parser.add_mutually_exclusive_group(required=True)
    output.add_argument("--output-dir", type=Path, help="本次运行的完整新目录，必须尚不存在")
    output.add_argument("--output", type=Path, help="兼容旧参数：在指定父目录下创建新的 UTC Run 目录")
    parser.add_argument("--base-z", type=float, default=.5)
    parser.add_argument("--seed", type=int, default=20261018)
    parser.add_argument("--amplitude-scale", type=float, default=1., help="试采幅度缩放，0 < scale <= 1")
    parser.add_argument("--compensate-damping", action="store_true",
                        help="在控制命令中加入已知黏性阻尼 b*v；保留 plant 阻尼和实际力矩测量")
    return parser.parse_args(argv)


def create_output_directory(args):
    if args.output_dir is not None:
        folder = args.output_dir.resolve()
    else:
        run_id = "R-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-") + uuid4().hex[:6]
        folder = args.output.resolve() / run_id
    folder.mkdir(parents=True, exist_ok=False)
    return folder


def main():
    args = parse_args()
    assert mujoco.__version__ == "3.3.7", f"需要 MuJoCo 3.3.7，当前 {mujoco.__version__}"
    assert np.isfinite(args.amplitude_scale) and 0 < args.amplitude_scale <= 1
    model_path = args.model.resolve(strict=True)
    m = mujoco.MjModel.from_xml_path(str(model_path))
    lower, upper = validate_model(m)
    base_info = set_robot_base_height(m, args.base_z)
    moving = moving_body_mask(m)
    a, b = make_coefficients(args.seed, args.amplitude_scale)
    folder = create_output_directory(args)
    binary = folder / "model_snapshot.mjb"
    mujoco.mj_saveModel(m, str(binary), None)
    config = {"purpose": "one trajectory collection pilot, not a final training dataset", "model_path": str(model_path),
              "model_snapshot_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
              "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "mujoco": mujoco.__version__, "numpy": np.__version__, "base": base_info,
              "seed": args.seed, "amplitude_scale": args.amplitude_scale, "sine_coefficients_rad": a.tolist(),
              "cosine_coefficients_rad": b.tolist(), "fundamental_frequency_Hz": F0,
              "angular_frequencies_rad_s": (2*np.pi*F0*np.arange(1, 6)).tolist(),
              "coefficient_method": "seeded coefficients scaled by L1 position bound; no excitation optimization",
              "q0_rad": [0.] * 6, "q_bound_rad": (AMPLITUDE_BOUNDS*args.amplitude_scale).tolist(),
              "duration_s": DURATION, "core_interval_s": [CORE_START, CORE_END],
              "core_interval_convention": "[3, 13); retain global sample time, phase time = t - 3",
              "phase_ids": {str(i): name for i, name in enumerate(PHASE_NAMES)},
              "acceptance": {"phase_tracking_limits_rad": dict(zip(PHASE_NAMES, TRACKING_LIMITS)),
                             "tracking_limit_comparison": "strictly below, using every physics state",
                             "minimum_actual_reference_span_ratio": MIN_SPAN_RATIO,
                             "minimum_ground_clearance_m": GROUND_CLEARANCE,
                             "ground_clearance_method": "compiled collision-mesh vertices projected onto ground plane",
                             "guard_dt_s": float(m.opt.timestep)},
              "physics_control_dt_s": float(m.opt.timestep), "measurement_dt_s": SAMPLE_DT,
              "sampling_method": "every second physics step, before integration; no filter/no noise/no delay",
              "controller": ("M*(aref + 144*(qref-q) + 24*(vref-v)) + bias"
                             + (" + b*v; known viscous compensation" if args.compensate_damping
                                else "; no passive compensation")),
              "damping_compensation": {"enabled": args.compensate_damping,
                                       "coefficients_Nm_s_rad": m.dof_damping.tolist(),
                                       "source": "known model dof_damping; current measured qvel",
                                       "measurement": "actual motor torque includes compensation; not subtracted"},
              "damping": m.dof_damping.tolist(), "frictionloss": m.dof_frictionloss.tolist(), "armature": m.dof_armature.tolist(),
              "truth_boundary": "qacc_truth in simulator_diagnostics is debug only; it is not measured acceleration",
              "paper_alignment": "Sec 4.2: 5 harmonics, 0.1 Hz fundamental, 10 s core, 1 kHz logging",
              "deviations": ["UR5 instead of SN7B", "MuJoCo instead of original platform",
                             "one small unoptimized pilot instead of twelve optimized trajectories",
                             "2 kHz physics/control instead of paper 1 kHz control",
                             "ideal actual motor torque proxy instead of current conversion",
                             "raw unfiltered data; preprocessing and train/validation/test split not yet performed"]}
    save_json(folder / "config.json", config)
    print("模块 2.1：UR5 无外力六轴 Fourier 试采；仅一条轨迹")
    print("基座绝对世界 Z：", args.base_z, "m；关节起始角度：六轴零位")
    print("物理/控制：2000 Hz；反馈记录：1000 Hz；原控制增益不变")
    print("已知黏性阻尼补偿：", "开启（+b*v）" if args.compensate_damping else "关闭（原基线）")
    print("0–1 s 保持，1–3 s 平滑进入，3–13 s 激励，13–15 s 平滑退出，15–17 s 恢复")
    print("基频：0.1 Hz，五个谐波；理论位置幅度上界 [rad]：", config["q_bound_rad"])
    print("输出目录：", folder, flush=True)
    report = preflight(m, moving, a, b, lower, upper)
    save_json(folder / "preflight.json", report)
    if not report["passed"]:
        print("[CHECK] 参考轨迹预检失败，仿真尚未推进：", json.dumps(report, ensure_ascii=False))
        return 2
    print(f"[PASS] {report['samples_checked']} 个参考姿态通过碰撞、离地间隙和理想力矩检查。", flush=True)
    x, r, z, steps, stats = run_simulation(m, folder, a, b, lower, upper, moving, args.compensate_damping)
    summary, core = assess(x, r, z, steps, stats, float(m.opt.timestep))
    segment = "excitation_measurements.csv" if summary["passed"] else "unaccepted_excitation_measurements.csv"
    save_csv(folder / segment, MEASUREMENT_HEADER, x[core])
    save_json(folder / "summary.json", summary)
    if summary["coverage"]:
        header = list(summary["coverage"][0])
        save_csv(folder / "coverage.csv", header, [[c[h] for h in header] for c in summary["coverage"]])
    plot_results(x, r, folder)
    print("停止原因：", stats["stop_reason"])
    print(f"记录 {len(x)} 行；10 秒激励段 {summary['excitation_rows']} 行")
    print("激励段最大跟踪误差 [rad]：", stats["max_abs_core_tracking_error_rad"])
    print("全物理步分段最大跟踪误差 [rad]：", stats["phase_max_tracking_error_rad"])
    print("实际最小离地间隙 [m]：", stats["minimum_ground_clearance_m"])
    print("逐轴统计：关节；实际/参考跨度 rad；跨度比例；速度范围 rad/s；换向次数；最大跟踪误差 rad")
    for c in summary["coverage"]:
        ratio = c["actual_reference_span_ratio"]
        ratio_text = f"{ratio:.1%}" if ratio is not None else "无效（参考跨度为零）"
        print(f"  {c['joint']}: {c['q_span_rad']:.6f}/{c['reference_q_span_rad']:.6f}; {ratio_text}; "
              f"[{c['v_min_rad_s']:.6f}, {c['v_max_rad_s']:.6f}]; "
              f"{c['reversals_with_0_001_rad_s_deadband']}; {c['tracking_max_abs_rad']:.6f}")
    for name, passed in summary["checks"].items():
        print("[PASS]" if passed else "[CHECK]", name)
    print("轨迹图：", folder / "trajectory_tracking.png")
    print("信号图：", folder / "joint_signals.png")
    print("激励段反馈：", folder / segment)
    print("统计：", folder / "summary.json")
    print("[PASS] 2.1 单条无外力轨迹试采通过；尚不代表完整建模数据集。" if summary["passed"]
          else "[CHECK] 未全部通过，已有数据已保留；请先排查。")
    return 0 if summary["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
