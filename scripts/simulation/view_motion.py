#!/usr/bin/env python3
"""Live UR5 viewer. Run scripts/simulation/view_motion.py in the HOST desktop terminal.

The host uses only Python's standard library. It starts the existing piakf Docker
service with an X11 socket and a private Xauthority cookie file. The container
uses MuJoCo 3.3.7, the validated 50 cm base placement and the same torque controller.
No xhost access-control changes, privileged container or GPU passthrough required.
Controls: Space = play/pause; R = reset/replay; close the window = exit.
"""
import argparse
import ctypes
import os
from pathlib import Path
import queue
import re
import shutil
import subprocess
import sys
import tempfile
import time

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
HOLD, DURATION, FREQUENCY, EXCURSION, WN = 2.0, 12.0, 0.2, 0.1, 12.0
GROUND_CLEARANCE = 0.005

def reference(t, q0):
    qref, vref, aref = q0.copy(), np.zeros(6), np.zeros(6)
    if t >= HOLD:
        w = 2 * np.pi * FREQUENCY
        phase = w * (t - HOLD)
        qref[1] += 0.5 * EXCURSION * (1 - np.cos(phase))
        vref[1] = 0.5 * EXCURSION * w * np.sin(phase)
        aref[1] = 0.5 * EXCURSION * w * w * np.cos(phase)
    return qref, vref, aref

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


def launch_from_host(args):
    project = args.project.resolve() if args.project else Path(__file__).resolve().parents[2]
    compose = project / "docker/compose.yaml"
    if not compose.is_file():
        raise RuntimeError("未找到 docker/compose.yaml；请用 --project 指定项目根目录。")
    model = args.model if args.model.is_absolute() else project / args.model
    try:
        model = Path("/workspace") / model.relative_to(project)
    except ValueError:
        if not model.is_relative_to("/workspace"):
            raise RuntimeError("模型需位于项目挂载目录内，或使用容器内的 /workspace 路径。")
    for executable in ("docker", "xauth"):
        if not shutil.which(executable):
            hint = "宿主机执行 sudo apt-get install xauth" if executable == "xauth" else "检查 Docker 安装"
            raise RuntimeError(f"宿主机缺少 {executable}；{hint}")
    display = os.environ.get("DISPLAY", "")
    match = re.fullmatch(r"(?:unix)?:(\d+)(?:\.\d+)?", display)
    if not match:
        raise RuntimeError("请从 Ubuntu 本机桌面终端启动；当前 DISPLAY 不是本地 X11/XWayland 显示地址。")
    if not Path(f"/tmp/.X11-unix/X{match.group(1)}").exists():
        raise RuntimeError(f"未找到 DISPLAY={display} 对应的本机 X11 socket。")
    auth = subprocess.run(["xauth", "nlist", display], capture_output=True, text=True, check=True)
    entries = [line.strip() for line in auth.stdout.splitlines() if line.strip()]
    if not entries:
        raise RuntimeError("没有找到当前 DISPLAY 的 Xauthority 记录。请在已登录桌面的普通用户终端运行，不要使用 sudo。")
    # FamilyWild permits the same cookie to work with the container hostname.
    # Cookie bytes are never printed and no X server access policy is changed.
    with tempfile.TemporaryDirectory(prefix="piakf-viewer-") as directory:
        auth_file = Path(directory) / "Xauthority"
        auth_file.touch(mode=0o600)
        payload = "\n".join("ffff" + line[4:] for line in entries) + "\n"
        subprocess.run(["xauth", "-f", str(auth_file), "nmerge", "-"],
                       input=payload, text=True, capture_output=True, check=True)
        if not auth_file.stat().st_size:
            raise RuntimeError("生成显示授权文件失败。")
        command = ["docker", "compose", "-f", str(compose), "run", "--rm", "--no-deps",
                   "--user", f"{os.getuid()}:{os.getgid()}",
                   "-e", f"DISPLAY={display}", "-e", "XAUTHORITY=/tmp/piakf.Xauthority",
                   "-e", "MUJOCO_GL=glfw", "-e", "PYOPENGL_PLATFORM=glx",
                   "-e", "LIBGL_ALWAYS_SOFTWARE=1",
                   "-v", "/tmp/.X11-unix:/tmp/.X11-unix:ro",
                   "-v", f"{auth_file}:/tmp/piakf.Xauthority:ro",
                   "-v", f"{Path(__file__).resolve()}:/tmp/ur5_view_motion.py:ro",
                   "piakf", "/opt/piakf-venv/bin/python", "/tmp/ur5_view_motion.py",
                   "--in-container", "--model", str(model), "--base-z", str(args.base_z),
                   "--q0-deg", *map(str, args.q0_deg)]
        if args.check:
            command.append("--check")
        print("正在启动 MuJoCo 窗口。使用 CPU 软件渲染；关闭窗口后容器自动退出。", flush=True)
        completed = subprocess.run(command, cwd=project)
        return completed.returncode


def prepare_simulation(args):
    m = mujoco.MjModel.from_xml_path(str(args.model.resolve(strict=True)))
    assert m.nq == m.nv == m.njnt == m.nu == 6, "需要已验证的六轴模型"
    assert [m.joint(i).name for i in range(6)] == JOINTS
    assert np.all(m.jnt_type == int(mujoco.mjtJoint.mjJNT_HINGE))
    assert np.array_equal(m.jnt_qposadr, np.arange(6))
    assert np.array_equal(m.jnt_dofadr, np.arange(6))
    assert np.all(m.actuator_trntype == int(mujoco.mjtTrn.mjTRN_JOINT))
    assert np.array_equal(m.actuator_trnid[:, 0], np.arange(6))
    assert np.all(m.actuator_dyntype == 0) and np.all(m.actuator_gaintype == 0)
    assert np.all(m.actuator_biastype == 0) and np.allclose(m.actuator_gainprm[:, 0], 1)
    assert np.allclose(m.actuator_gear, np.tile([1., 0., 0., 0., 0., 0.], (6, 1)))
    assert np.all(m.actuator_ctrllimited) and np.all(m.actuator_forcelimited)
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tool0") >= 0
    assert np.isclose(m.opt.timestep, 0.0005, rtol=0, atol=1e-12)
    base_info = set_robot_base_height(m, args.base_z)
    print("模型：", args.model, flush=True)
    print("基座：", base_info["base_body_name"], base_info["base_position_after_m"], "m")
    q0 = np.deg2rad(args.q0_deg)
    assert np.isfinite(q0).all()
    moving = moving_body_mask(m)
    report = preflight(m, q0, moving)
    if not report["passed"]:
        raise RuntimeError(f"几何预检失败：{report}")
    print("[PASS] 101 个姿态点通过碰撞与离地间隙检查。", flush=True)
    lower = np.maximum(m.actuator_ctrlrange[:, 0], m.actuator_forcerange[:, 0])
    upper = np.minimum(m.actuator_ctrlrange[:, 1], m.actuator_forcerange[:, 1])
    assert np.all(lower < upper)
    d = mujoco.MjData(m)
    reset_state(m, d, q0, lower, upper)
    return m, d, q0, lower, upper, moving


def reset_state(m, d, q0, lower, upper):
    mujoco.mj_resetData(m, d)
    d.qpos[:] = q0
    mujoco.mj_forward(m, d)
    d.ctrl[:] = np.clip(d.qfrc_bias, lower, upper)
    mujoco.mj_forward(m, d)


def control_step(m, d, q0, lower, upper, moving, inertia_torque, step_index):
    # Viewing mode reuses the no-external-force baseline controller.
    d.qfrc_applied[:] = 0
    d.xfrc_applied[:] = 0
    mujoco.mj_forward(m, d)
    qref, vref, aref = reference(float(d.time), q0)
    accel = aref + WN**2 * (qref - d.qpos) + 2 * WN * (vref - d.qvel)
    mujoco.mj_mulM(m, d, inertia_torque, accel)
    raw = inertia_torque + d.qfrc_bias
    d.ctrl[:] = np.clip(raw, lower, upper)
    mujoco.mj_forward(m, d)
    error = float(np.max(np.abs(qref - d.qpos)))
    if not np.isfinite(np.concatenate((d.qpos, d.qvel, d.qacc))).all():
        raise RuntimeError("出现非有限状态，已停止推进。")
    if np.any(d.warning.number):
        raise RuntimeError(f"MuJoCo 警告：{d.warning.number}")
    contacts = moving_contacts(m, d, moving)
    if contacts:
        raise RuntimeError(f"发生运动连杆接触：{contacts[0]}")
    if error > .35:
        raise RuntimeError(f"跟踪误差超过 0.35 rad：{error}")
    clipped = bool(np.any(np.abs(raw - d.ctrl) > 1e-9))
    mujoco.mj_step(m, d)
    if abs(float(d.time) - (step_index + 1) * float(m.opt.timestep)) > 1e-8:
        raise RuntimeError("仿真时间异常，可能发生自动重置。")
    return error, clipped


def run_container(args):
    global mujoco, np
    os.environ["MUJOCO_GL"] = "glfw"
    os.environ["PYOPENGL_PLATFORM"] = "glx"
    if not args.check and not os.environ.get("DISPLAY"):
        raise RuntimeError("当前容器没有图形显示连接。请在宿主机桌面终端运行此文件，它会自动启动图形容器。")
    import mujoco
    import numpy as np
    if mujoco.__version__ != "3.3.7":
        raise RuntimeError(f"需要现有 MuJoCo 3.3.7 环境，当前为 {mujoco.__version__}")
    m, d, q0, lower, upper, moving = prepare_simulation(args)
    torque = np.zeros(6)
    count = round(DURATION / float(m.opt.timestep))
    max_error, clipped_samples = 0., 0
    if args.check:
        for step in range(count):
            error, clipped = control_step(m, d, q0, lower, upper, moving, torque, step)
            max_error = max(max_error, error)
            clipped_samples += int(clipped)
        print(f"控制回路检查：t={d.time:.3f} s，最大误差={max_error:.6f} rad，限幅={clipped_samples}")
        return 0 if max_error < .02 and clipped_samples == 0 else 2
    try:
        ctypes.CDLL("libGLX_mesa.so.0")
    except OSError as exc:
        raise RuntimeError("容器缺少 Mesa GLX 库。请在 Dockerfile 的 apt 安装列表加入 "
                           "libglx-mesa0 和 libgl1-mesa-dri，然后重新构建镜像。") from exc
    import mujoco.viewer
    commands = queue.SimpleQueue()

    def key_callback(keycode):
        if keycode == 32:
            commands.put("pause")
        elif keycode in (82, 114):
            commands.put("restart")

    print("窗口初始为暂停。空格：播放/暂停；R：从头播放；关闭窗口：退出。", flush=True)
    print("第 2 关节最大偏移仅 5.73 度，可用鼠标滚轮放大观察。", flush=True)
    with mujoco.viewer.launch_passive(m, d, key_callback=key_callback,
                                      show_left_ui=False, show_right_ui=False) as viewer:
        with viewer.lock():
            points = d.xpos[moving]
            low, high = points.min(axis=0), points.max(axis=0)
            viewer.cam.lookat[:] = (low + high) / 2
            viewer.cam.distance = max(1.4, 2.2 * float(np.linalg.norm(high - low)))
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
        viewer.sync()
        paused, failed, step = True, False, 0
        frame_steps = max(1, round((1 / 60) / float(m.opt.timestep)))
        while viewer.is_running():
            started = time.perf_counter()
            while not commands.empty():
                command = commands.get()
                if command == "restart":
                    with viewer.lock():
                        reset_state(m, d, q0, lower, upper)
                    step, max_error, clipped_samples = 0, 0., 0
                    paused, failed = False, False
                    print("从 t=0 重新播放。", flush=True)
                elif command == "pause":
                    if step == count or failed:
                        print("按 R 从头播放。", flush=True)
                    else:
                        paused = not paused
            if not paused:
                try:
                    with viewer.lock():
                        for _ in range(min(frame_steps, count - step)):
                            error, clipped = control_step(m, d, q0, lower, upper, moving, torque, step)
                            max_error = max(max_error, error)
                            clipped_samples += int(clipped)
                            step += 1
                        mujoco.mj_forward(m, d)
                    if step == count:
                        paused = True
                        print(f"播放完成：12 s；最大误差={max_error:.6f} rad；限幅={clipped_samples}。"
                              "窗口保留，按 R 可重新播放。", flush=True)
                except RuntimeError as exc:
                    paused, failed = True, True
                    print(f"[CHECK] {exc} 按 R 重置或关闭窗口。", flush=True)
            viewer.sync()
            delay = frame_steps * float(m.opt.timestep) - (time.perf_counter() - started)
            if delay > 0:
                time.sleep(delay)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, help="宿主机项目根目录，默认从 scripts/simulation/ 定位")
    parser.add_argument("--model", type=Path, default=Path(__file__).resolve().parents[1] / "models" / "ur5" / "scene.xml")
    parser.add_argument("--base-z", type=float, default=.5)
    parser.add_argument("--q0-deg", type=float, nargs=6, default=[0.] * 6)
    parser.add_argument("--check", action="store_true", help="只检查 12 秒控制回路，不打开窗口")
    parser.add_argument("--in-container", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        if args.in_container or Path("/.dockerenv").exists():
            return run_container(args)
        return launch_from_host(args)
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        # CalledProcessError does not include captured cookie data in its str().
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

