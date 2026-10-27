#!/usr/bin/env python3
"""UR5 motion-interface check for MuJoCo 3.3.7, six unit-gear torque motors.

Run inside the user's existing container. No ROS node or GUI is required.
This checks simulator control and feedback, not a PIAKF estimator or paper result.
The controller reads simulator inertia and bias; these are not measurements.
CSV measurements for a future estimator are ONLY t_s, q*_rad, v*_rad_s, tau*_Nm.
Other columns are controller/debug information.
50 cm variant: set the robot's top-level body origin to world Z=0.50 m,
and initialize all six joint angles to zero by default. The height is absolute,
so an earlier manual XML height edit is not added a second time. XML is not saved.
Sample reference configurations for ground clearance and collisions; monitor
moving-link contacts during motion. This is a fixed-base robot simulation.
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
HOLD = 2.0
DURATION = 12.0
FREQUENCY = 0.2  # Hz: two complete cycles during the last ten seconds
EXCURSION = 0.1  # rad: q2 moves between its initial value and initial + 0.1
WN = 12.0       # rad/s: virtual error dynamics, with critical damping
DEFAULT_Q0_DEG = [0., 0., 0., 0., 0., 0.]
DEFAULT_BASE_Z = 0.50  # world-coordinate height, metres; NOT an extra displacement
GROUND_CLEARANCE = 0.005  # m: sampled reference configurations need 5 mm clearance


def reference(t, q0):
    qref, vref, aref = q0.copy(), np.zeros(6), np.zeros(6)
    if t >= HOLD:
        w = 2 * np.pi * FREQUENCY
        phase = w * (t - HOLD)
        qref[1] += 0.5 * EXCURSION * (1 - np.cos(phase))
        vref[1] = 0.5 * EXCURSION * w * np.sin(phase)
        aref[1] = 0.5 * EXCURSION * w * w * np.cos(phase)
    return qref, vref, aref


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


def preflight(m, q0, moving):
    """Sample geometric configurations only; this does not certify continuous motion."""
    probe = mujoco.MjData(m)
    ground = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    report = {"passed": False, "samples_requested": 101, "samples_checked": 0,
              "q0_deg": np.rad2deg(q0).tolist(), "excursion_rad": EXCURSION,
              "required_ground_clearance_m": GROUND_CLEARANCE,
              "minimum_ground_distance_capped_at_0.05_m": None,
              "note": "Sampled reference geometry check; runtime contacts are also monitored."}
    disabled = int(m.opt.disableflags)
    if disabled & int(mujoco.mjtDisableBit.mjDSBL_CONTACT):
        report["reason"] = "模型关闭了接触计算，无法验证本步骤的碰撞条件"
        return report
    if disabled & int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT):
        report["reason"] = "模型关闭了约束计算，无法验证本步骤的碰撞条件"
        return report
    if ground < 0:
        report["reason"] = "找不到已验证模型中的 ground 几何体"
        return report
    geoms = [g for g in range(m.ngeom) if moving[m.geom_bodyid[g]]
             and (m.geom_contype[g] or m.geom_conaffinity[g])]
    if not geoms:
        report["reason"] = "没有可检查的运动连杆碰撞几何体"
        return report
    limited = m.jnt_limited.astype(bool)
    minimum = 0.05
    closest = None
    for index, offset in enumerate(np.linspace(0., EXCURSION, 101)):
        q = q0.copy()
        q[1] += offset
        report["samples_checked"] = index + 1
        if np.any(q[limited] < m.jnt_range[limited, 0]) or np.any(q[limited] > m.jnt_range[limited, 1]):
            report.update(reason="目标轨迹超过关节限位", offset_rad=float(offset), q_rad=q.tolist())
            return report
        probe.qpos[:] = q
        probe.qvel[:] = 0
        mujoco.mj_forward(m, probe)
        contacts = moving_contacts(m, probe, moving)
        if contacts:
            report.update(reason="目标姿态存在涉及运动连杆的接触", offset_rad=float(offset),
                          q_rad=q.tolist(), contacts=contacts)
            return report
        for g in geoms:
            # Explicit distance check includes all moving collision geoms, so a
            # collision mask/exclusion cannot hide ground penetration here.
            distance = float(mujoco.mj_geomDistance(m, probe, ground, g, 0.05, None))
            if distance < minimum:
                minimum = distance
                closest = {"geom": m.geom(g).name or f"geom#{g}", "offset_rad": float(offset)}
            report["minimum_ground_distance_capped_at_0.05_m"] = minimum
            if distance < GROUND_CLEARANCE:
                report.update(reason="目标姿态的连杆离地间隙不足 5 mm", offset_rad=float(offset),
                              geom=m.geom(g).name, distance_m=distance, q_rad=q.tolist())
                return report
    report.update(passed=True, reason="101 个采样姿态通过几何检查", closest=closest)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path,
                        default=Path(__file__).resolve().parents[1] / "models" / "ur5" / "scene.xml")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parents[2] / "runs" / "manual_checks")
    parser.add_argument("--q0-deg", type=float, nargs=6, default=DEFAULT_Q0_DEG,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                        help="六轴候选初始角度，单位度；先检查再运行")
    parser.add_argument("--base-z", type=float, default=DEFAULT_BASE_Z,
                        help="机器人根 body 的世界 Z 坐标，单位 m；默认 0.50，不与 XML 高度叠加")
    args = parser.parse_args()
    if mujoco.__version__ != "3.3.7":
        raise RuntimeError(f"本脚本对应 MuJoCo 3.3.7，当前为 {mujoco.__version__}")
    model_path = args.model.resolve(strict=True)
    m = mujoco.MjModel.from_xml_path(str(model_path))
    assert m.nq == m.nv == m.njnt == m.nu == 6, "需要六个单自由度关节和驱动器"
    assert [m.joint(i).name for i in range(6)] == JOINTS, "关节顺序与已验证模型不同"
    assert np.all(m.jnt_type == int(mujoco.mjtJoint.mjJNT_HINGE))
    assert np.array_equal(m.jnt_qposadr, np.arange(6))
    assert np.array_equal(m.jnt_dofadr, np.arange(6))
    assert np.all(m.actuator_trntype == int(mujoco.mjtTrn.mjTRN_JOINT))
    assert np.array_equal(m.actuator_trnid[:, 0], np.arange(6)), "驱动器映射发生变化"
    assert np.all(m.actuator_dyntype == 0) and np.all(m.actuator_gaintype == 0)
    assert np.all(m.actuator_biastype == 0)
    assert np.allclose(m.actuator_gainprm[:, 0], 1)
    assert np.allclose(m.actuator_gear, np.tile([1., 0., 0., 0., 0., 0.], (6, 1)))
    assert np.all(m.actuator_ctrllimited) and np.all(m.actuator_forcelimited)
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tool0") >= 0
    # For these verified unit-gear motors, both limits have units N*m.
    lower = np.maximum(m.actuator_ctrlrange[:, 0], m.actuator_forcerange[:, 0])
    upper = np.minimum(m.actuator_ctrlrange[:, 1], m.actuator_forcerange[:, 1])
    assert np.all(lower < upper)
    dt = float(m.opt.timestep)
    assert np.isclose(dt, 0.0005, rtol=0, atol=1e-12), f"模型步长变为 {dt}"

    base_info = set_robot_base_height(m, args.base_z)
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    # The entire robot has been relocated before initializing the joint state.
    d.qpos[:] = np.deg2rad(args.q0_deg)
    assert np.isfinite(d.qpos).all(), "初始角度必须有限"
    pose_source = "explicit --q0-deg (default: six zeros)"
    d.time = 0.0
    d.qvel[:] = 0.0
    d.ctrl[:] = 0.0
    d.qfrc_applied[:] = 0.0
    d.xfrc_applied[:] = 0.0
    mujoco.mj_forward(m, d)
    q0 = d.qpos.copy()

    run = args.output.resolve() / ("step12_50cm_" + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%SZ_") + uuid4().hex[:6])
    run.mkdir(parents=True, exist_ok=False)
    print("脚本版本：4（基座世界 Z=0.50 m；默认六轴零位；高度可用 --base-z 调整）")
    print("模型：", model_path)
    print("机器人根 body：", base_info["base_body_name"])
    print("加载时基座位置 [m]：", base_info["base_position_before_m"])
    print("本次基座位置 [m]：", base_info["base_position_after_m"])
    print("本次实际向上平移 [m]：", base_info["actual_shift_z_m"])
    print("地面世界位置 [m]：", base_info["ground_world_position_m"])
    print("起始姿态来源：", pose_source)
    print("初始角度 [rad]：", np.round(q0, 6))
    print("初始角度 [deg]：", np.round(np.rad2deg(q0), 3))
    print(f"控制与记录周期：{dt} s；仿真时长：{DURATION} s")
    print("前 2 秒保持；随后第 2 关节在初始角度到初始 + 0.1 rad 之间往复两次。")
    print("输出目录：", run, flush=True)
    (run / "base_placement.json").write_text(json.dumps(base_info, indent=2), encoding="utf-8")
    moving = moving_body_mask(m)
    preflight_report = preflight(m, q0, moving)
    (run / "preflight.json").write_text(json.dumps(preflight_report, indent=2, ensure_ascii=False), encoding="utf-8")
    if not preflight_report["passed"]:
        print("[CHECK] 几何预检未通过，仿真尚未推进：")
        print(json.dumps(preflight_report, indent=2, ensure_ascii=False))
        print("预检记录：", run / "preflight.json")
        raise SystemExit(2)
    print("[PASS] 101 个采样姿态：无涉及运动连杆的启用接触，离地间隙至少 5 mm。")

    # Every row is a pre-integration sample at the state named by t_s.
    header = (["t_s"] + [f"q{i}_rad" for i in range(1, 7)]
              + [f"v{i}_rad_s" for i in range(1, 7)]
              + [f"tau{i}_Nm" for i in range(1, 7)]
              + [f"qref{i}_rad" for i in range(1, 7)]
              + [f"ctrl{i}_Nm" for i in range(1, 7)]
              + ["command_clipped", "ncon", "max_abs_constraint_Nm"]
              + [f"raw_command{i}_Nm" for i in range(1, 7)]
              + [f"passive{i}_Nm" for i in range(1, 7)]
              + [f"constraint{i}_Nm" for i in range(1, 7)]
              + [f"acceleration{i}_rad_s2" for i in range(1, 7)]
              + ["moving_contact_count"])
    count = round(DURATION / dt)
    log = np.empty((count + 1, len(header)))
    inertia_torque = np.zeros(6)
    events = {}
    stop_reason = None
    failure_state = None
    for k in range(count + 1):
        # Refresh M(q), bias(q,v), contacts, etc. for the CURRENT q and v.
        mujoco.mj_forward(m, d)
        qref, vref, aref = reference(float(d.time), q0)
        accel_cmd = aref + WN**2 * (qref - d.qpos) + 2 * WN * (vref - d.qvel)
        mujoco.mj_mulM(m, d, inertia_torque, accel_cmd)
        raw = inertia_torque + d.qfrc_bias
        d.ctrl[:] = np.clip(raw, lower, upper)
        clipped = bool(np.any(np.abs(raw - d.ctrl) > 1e-9))
        # Recompute actual actuator torques after setting this time's command.
        # mj_forward does NOT advance time; never mix post-step q/v with old forces.
        mujoco.mj_forward(m, d)
        robot_contacts = moving_contacts(m, d, moving)
        log[k] = np.concatenate((
            [float(d.time)], d.qpos.copy(), d.qvel.copy(),
            d.qfrc_actuator.copy(), qref, d.ctrl.copy(),
            [float(clipped), float(d.ncon), float(np.max(np.abs(d.qfrc_constraint)))],
            raw.copy(), d.qfrc_passive.copy(), d.qfrc_constraint.copy(), d.qacc.copy(),
            [len(robot_contacts)]))
        candidates = {
            "initial": k == 0,
            "first_clipping": clipped,
            "first_contact": d.ncon > 0,
            "first_moving_link_contact": bool(robot_contacts),
            "first_constraint_above_0.1_Nm": np.max(np.abs(d.qfrc_constraint)) > 0.1,
            "first_error_above_0.01_rad": np.max(np.abs(d.qpos - qref)) > 0.01,
        }
        for name, occurred in candidates.items():
            if occurred and name not in events:
                events[name] = snapshot(m, d, qref, raw)
        if not np.isfinite(log[k]).all() or not np.isfinite(d.qacc).all():
            stop_reason = f"t={d.time:.6f}: 出现非有限数值，未通过"
        elif np.any(d.warning.number):
            stop_reason = f"t={d.time:.6f}: MuJoCo warning counts={d.warning.number}"
        elif robot_contacts:
            stop_reason = f"t={d.time:.6f}: 出现涉及运动连杆的接触：{robot_contacts[0]}"
        elif np.max(np.abs(qref - d.qpos)) > 0.35:
            stop_reason = f"t={d.time:.6f}: 跟踪误差超过 0.35 rad，停止检查"
        if stop_reason:
            failure_state = snapshot(m, d, qref, raw)
            break
        if k < count:
            mujoco.mj_step(m, d)
            if abs(float(d.time) - (k + 1) * dt) > 1e-8:
                stop_reason = "仿真时间异常，可能发生数值失稳后的自动重置"
                # This state is post-step; the CSV ends at the last pre-step sample.
                failure_state = {"post_step_time_s": float(d.time),
                                 "q_rad": d.qpos.tolist(), "velocity_rad_s": d.qvel.tolist()}
                break

    log = log[:k + 1]
    t, q, velocity, tau = log[:, 0], log[:, 1:7], log[:, 7:13], log[:, 13:19]
    qref, ctrl = log[:, 19:25], log[:, 25:31]
    error = q - qref
    hold_mask = t < HOLD - dt / 2
    move_mask = ~hold_mask
    hold_max = float(np.max(np.abs(error[hold_mask]))) if hold_mask.any() else None
    move_max = float(np.max(np.abs(error[move_mask]))) if move_mask.any() else None
    saturated = int(np.count_nonzero(log[:, 31]))
    torque_difference = float(np.max(np.abs(tau - ctrl)))
    time_ok = bool(np.allclose(np.diff(t), dt, rtol=0, atol=1e-10))
    checks = {
        "completed_12_seconds": stop_reason is None and len(t) == count + 1,
        "finite_samples": bool(np.isfinite(log).all()),
        "no_moving_link_contact": bool(np.all(log[:, -1] == 0)),
        "uniform_time": time_ok and len(t) > 1,
        "hold_max_error_below_0.01_rad": hold_max is not None and hold_max < 0.01,
        "motion_max_error_below_0.02_rad": move_max is not None and move_max < 0.02,
        "no_command_clipping": saturated == 0,
        "unit_gear_motor_torque_matches_command": torque_difference < 1e-8,
    }
    # These thresholds are engineering checks for this step, not paper metrics.
    result = {
        "purpose": "UR5 simulator motion and feedback interface check; no PIAKF",
        "script_revision": 4, "variant": "base_world_z_50cm_default", "stop_reason": stop_reason,
        "base_placement": base_info,
        "geometric_preflight": preflight_report,
        "last_recorded_time_s": float(t[-1]),
        "model": str(model_path), "mujoco": mujoco.__version__,
        "numpy": np.__version__, "initial_pose_source": pose_source,
        "initial_q_rad": q0.tolist(), "dt_s": dt, "duration_s": DURATION,
        "frequency_Hz": FREQUENCY, "excursion_rad": EXCURSION, "wn_rad_s": WN,
        "integrator_enum": int(m.opt.integrator), "samples": len(t),
        "hold_max_error_rad": hold_max, "motion_max_error_rad": move_max,
        "clipped_samples": saturated, "max_tau_ctrl_difference_Nm": torque_difference,
        "max_abs_tau_Nm": np.max(np.abs(tau), axis=0).tolist(),
        "max_contacts": int(np.max(log[:, 32])),
        "max_abs_constraint_Nm": float(np.max(log[:, 33])),
        "checks": checks, "passed": all(checks.values()),
    }
    diagnostic = {
        "joint_order": JOINTS, "events": events, "failure_state": failure_state,
        "last_recorded_state": snapshot(m, d, qref[-1], log[-1, 34:40])
            if stop_reason != "仿真时间异常，可能发生数值失稳后的自动重置" else None,
        "damping_Nm_s_rad": m.dof_damping.tolist(),
        "frictionloss_Nm": m.dof_frictionloss.tolist(),
        "armature_kg_m2": m.dof_armature.tolist(),
        "joint_stiffness": m.jnt_stiffness.tolist(),
        "joint_ranges_rad": m.jnt_range.tolist(),
        "joint_limited": m.jnt_limited.tolist(),
        "ctrl_limits_Nm": np.column_stack((lower, upper)).tolist(),
        "joint_actuator_force_limited": m.jnt_actfrclimited.tolist(),
        "joint_actuator_force_range_Nm": m.jnt_actfrcrange.tolist(),
        "integrator_enum": int(m.opt.integrator),
        "disableflags": int(m.opt.disableflags), "enableflags": int(m.opt.enableflags),
    }
    with (run / "joint_feedback.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(log)
    (run / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    (run / "diagnostic.json").write_text(json.dumps(diagnostic, indent=2), encoding="utf-8")
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True, layout="constrained")
    axes[0].plot(t, qref[:, 1] - q0[1], "k--", lw=2, label="J2 target")
    axes[0].plot(t, q[:, 1] - q0[1], lw=1, label="J2 measured")
    axes[0].set_ylabel("J2 displacement [rad]")
    axes[0].legend()
    for j in range(6):
        axes[1].plot(t, np.rad2deg(error[:, j]), label=f"J{j + 1}")
        axes[2].plot(t, tau[:, j], label=f"J{j + 1}")
    axes[1].set_ylabel("Tracking error [deg]")
    axes[2].set_ylabel("Actuator torque [N m]")
    axes[2].set_xlabel("Simulation time [s]")
    axes[1].legend(ncol=3)
    axes[2].legend(ncol=3)
    for ax in axes:
        ax.axvline(HOLD, color="gray", ls=":", lw=1)
        ax.grid(alpha=0.3)
    fig.savefig(run / "motion_check.png", dpi=150)
    plt.close(fig)

    print(f"记录：{len(t)} 行，t={t[0]:.4f} 至 {t[-1]:.4f} s")
    print(f"保持阶段最大误差（六轴）：{hold_max} rad")
    print(f"运动阶段最大误差（六轴）：{move_max} rad（None 表示尚未进入运动阶段）")
    print(f"控制限幅样本：{saturated}/{len(t)}")
    print("各轴实际力矩绝对值峰值 [N m]：", np.round(np.max(np.abs(tau), axis=0), 4))
    print(f"最大接触数量：{result['max_contacts']}；最大约束力矩："
          f"{result['max_abs_constraint_Nm']:.6f} N m")
    print("\n=== 诊断摘要 ===")
    print("停止原因：", stop_reason or "完成 12 秒")
    print("各轴全过程最大绝对误差 [rad]：", np.round(np.max(np.abs(error), axis=0), 6))
    print("最后一条记录：error=q-qref，力矩单位 N m")
    print("joint                    qref        q      error       v      raw     ctrl      tau   passive constraint")
    for j, name in enumerate(JOINTS):
        values = (qref[-1, j], q[-1, j], error[-1, j], velocity[-1, j],
                  log[-1, 34 + j], ctrl[-1, j], tau[-1, j], log[-1, 40 + j], log[-1, 46 + j])
        print(f"{name:23s}" + " ".join(f"{v:8.4f}" for v in values))
    print("各轴限幅样本数：", np.count_nonzero(np.abs(log[:, 34:40] - ctrl) > 1e-9, axis=0))
    print("damping：", m.dof_damping, "frictionloss：", m.dof_frictionloss)
    for name, event in events.items():
        print(f"事件 {name}: t={event['t_s']:.6f} s")
    # Report named contact pairs at the first contact event and at the stop.
    for label, state in (("首次接触", events.get("first_contact")),
                         ("停止时", failure_state or diagnostic["last_recorded_state"])):
        if state and state.get("contacts_total", 0):
            print(f"{label}：共 {state['contacts_total']} 个接触，最多列出 20 个")
            for contact in state["contacts_first_20"]:
                print(" ", contact)
    print("=== 诊断摘要结束 ===\n")
    for name, ok in checks.items():
        print(f"[{'PASS' if ok else 'CHECK'}] {name}")
    print("曲线：", run / "motion_check.png")
    print("反馈：", run / "joint_feedback.csv")
    print("统计：", run / "summary.json")
    print("诊断：", run / "diagnostic.json")
    if all(checks.values()):
        print("[PASS] 本步骤的姿态保持、小幅运动和同步记录检查通过。")
    else:
        print("[CHECK] 数据已保存；请根据统计和曲线排查，不应直接进入施加外力步骤。")
        raise SystemExit(2)


if __name__ == "__main__":
    main()