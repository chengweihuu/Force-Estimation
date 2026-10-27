"""Bounded current/torque interface comparison. All run identity is CLI/config data."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import signal
import time
import traceback

import mujoco
import numpy as np

from current_interface import CurrentObserver, EquivalentCurrentSensor, load_current_packet
from checks import close, preflight, require


def write(path, value):
    def convert(x):
        if isinstance(x, np.ndarray):
            return x.tolist()
        if isinstance(x, np.generic):
            return x.item()
        raise TypeError(type(x).__name__)
    with Path(path).open("x") as f:
        json.dump(value, f, default=convert, ensure_ascii=False, indent=2, allow_nan=False)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def save_series(path, series):
    with Path(path).open("xb") as f:
        np.savez_compressed(f, **{key: np.asarray(value) for key, value in series.items()})


def sample(series, **values):
    for key, value in values.items():
        series.setdefault(key, []).append(np.array(value, copy=True))


def simulate(cfg, case, output, oracle, helpers, ledger):
    m = mujoco.MjModel.from_binary_path(cfg["model"])
    d = mujoco.MjData(m)
    controller_model = oracle.Model(cfg["model"], cfg["derivative_epsilon_s"])
    sensor = EquivalentCurrentSensor(cfg["current"]["D_true_Nm_per_Aeq"])
    d.qpos[:] = cfg["initial_q_rad"]
    site = m.site("tool0").id
    body = int(m.site_bodyid[site])
    mask = helpers.moving_mask(m)
    kp, kd = np.asarray(cfg["controller"]["kp_Nm_rad"]), np.asarray(cfg["controller"]["kd_Nm_s_rad"])
    dt = cfg["physics_dt_s"]
    control_stride = round(cfg["control_dt_s"] / dt)
    observer_stride = round(cfg["observer_dt_s"] / dt)
    steps = round(case["duration_s"] / dt)
    feedback, diagnostic = {}, {}
    control_times = []
    tracking = np.zeros(6)
    torque_peak = np.zeros(6)
    balance_peak = np.zeros(6)
    mapping_peak = 0.0
    time_error = 0.0
    scaled_min_sv = float("inf")
    command = np.zeros(6)
    try:
        for n in range(steps + 1):
            t = n * dt
            d.qfrc_applied[:] = 0
            d.xfrc_applied[:] = 0
            mujoco.mj_forward(m, d)
            qr, vr, ar = helpers.trajectory(t, cfg, True)
            control_updated = n % control_stride == 0
            if control_updated:
                M, h, _, _ = controller_model.evaluate(d.qpos.copy(), d.qvel.copy())
                command = M @ ar + h + controller_model.B * d.qvel + kp*(qr-d.qpos) + kd*(vr-d.qvel)
                d.ctrl[:] = command
                control_times.append(t)
            wrench = np.zeros(6)
            if case["force"] == "step" and t >= cfg["force"]["start_s"]:
                wrench[cfg["force"]["axis"]] = cfg["force"]["amplitude_N"]
            mujoco.mj_applyFT(m, d, wrench[:3], wrench[3:], d.site_xpos[site], body, d.qfrc_applied)
            mujoco.mj_forward(m, d)
            helpers.valid(m, d, mask, command)
            M = np.zeros((6, 6))
            mujoco.mj_fullM(m, M, d.qM)
            balance = M @ d.qacc + d.qfrc_bias - d.qfrc_actuator - d.qfrc_passive - d.qfrc_applied
            require(np.allclose(balance, 0, rtol=0, atol=cfg["comparison_atol"]), "Physical balance failed")
            balance_peak = np.maximum(balance_peak, np.abs(balance))
            tracking = np.maximum(tracking, np.abs(d.qpos-qr))
            torque_peak = np.maximum(torque_peak, np.abs(d.qfrc_actuator))
            time_error = max(time_error, abs(float(d.time)-t))
            if n % observer_stride == 0:
                jp, jr = np.zeros((3, 6)), np.zeros((3, 6))
                mujoco.mj_jacSite(m, d, jp, jr, site)
                J = np.vstack([jp, jr])
                mapping_peak = max(mapping_peak, close(J.T @ wrench, d.qfrc_applied, "force/J map", cfg))
                scaled = J.copy()
                scaled[:3] /= cfg["evaluation"]["jacobian_characteristic_length_m"]
                sv = np.linalg.svd(scaled, compute_uv=False)
                require(sv[-1] > np.finfo(float).eps * 6 * sv[0], "Jacobian rank loss")
                scaled_min_sv = min(scaled_min_sv, float(sv[-1]))
                sample(feedback, t=t, q=d.qpos, v=d.qvel, current=sensor.measure(d.qfrc_actuator))
                sample(diagnostic, t=t, tau_act=d.qfrc_actuator, wrench=wrench,
                       tau_ext=d.qfrc_applied, qacc=d.qacc, J=J, q_ref=qr,
                       command=command, control_updated=control_updated, physical_time=d.time)
            if n < steps:
                require(ledger["physics_steps"] < cfg["limits"]["physics_steps"], "Physical step budget exhausted")
                mujoco.mj_step(m, d)
                ledger["physics_steps"] += 1
            if n and n % 16000 == 0:
                print(json.dumps(dict(event="simulation", case=case["name"], t_s=t,
                                      cumulative_physics_steps=ledger["physics_steps"])), flush=True)
    finally:
        save_series(output / "current_measurements.npz", feedback)
        save_series(output / "diagnostics.npz", diagnostic)
        save_series(output / "control_clock.npz", {"t": control_times})
    packet = load_current_packet(output / "current_measurements.npz")
    expected_t = np.arange(steps // observer_stride + 1) * cfg["observer_dt_s"]
    close(packet["t"], expected_t, "125 Hz sample times", cfg)
    close(np.diff(control_times), np.full(len(control_times)-1, cfg["control_dt_s"]), "1 kHz control times", cfg)
    require(time_error <= cfg["comparison_atol"], "Simulated and reported timestamps differ")
    require(all(diagnostic["control_updated"]), "Feedback was not sampled after control refresh")
    result = dict(passed=True, physics_steps=steps, feedback_samples=len(packet["t"]),
                  control_updates=len(control_times), max_timestamp_roundoff_s=time_error,
                  max_tracking_error_rad=tracking, peak_actual_torque_Nm=torque_peak,
                  max_balance_error_Nm=balance_peak, max_force_mapping_error_Nm=mapping_peak,
                  minimum_scaled_J_singular_value=scaled_min_sv, moving_contacts=0,
                  saturation_steps=0, limit_steps=0, engine_warnings=0)
    write(output / "simulation_checks.json", result)
    return result


def update_counted(observer, args, cfg, ledger):
    require(ledger["observer_calls"] < cfg["limits"]["observer_calls"], "Observer budget exhausted")
    ledger["observer_calls"] += 1
    return observer.update(*args)


def replay_current(cfg, measurement_path, output, oracle, ledger, limit=None):
    # This function never opens the diagnostic file or owns the plant context/D_true.
    packet = load_current_packet(measurement_path)
    observer = CurrentObserver(oracle.Observer(cfg), cfg["current"]["D_est_Nm_per_Aeq"])
    records = {}
    minimum_eigenvalue = float("inf")
    maximum_asymmetry = 0.0
    count = len(packet["t"]) if limit is None else min(limit, len(packet["t"]))
    for k in range(count):
        result = update_counted(observer, tuple(packet[key][k] for key in ["t", "q", "v", "current"]), cfg, ledger)
        require(all(np.isfinite(value).all() for value in result.values()), "Nonfinite current estimate")
        eigenvalue = float(np.linalg.eigvalsh(result["covariance"])[0])
        require(eigenvalue > 0, "Non-positive covariance")
        minimum_eigenvalue = min(minimum_eigenvalue, eigenvalue)
        maximum_asymmetry = max(maximum_asymmetry, float(np.abs(result["covariance"]-result["covariance"].T).max()))
        sample(records, t=packet["t"][k], **result)
    records = {key: np.asarray(value) for key, value in records.items()}
    if output is not None:
        save_series(output / "current_estimates.npz", records)
        write(output / "current_replay_checks.json", dict(passed=True, samples=count,
              minimum_covariance_eigenvalue=minimum_eigenvalue, maximum_asymmetry=maximum_asymmetry,
              feedback_fields=sorted(packet), current_filter="identity", current_units="A_eq"))
    return records


def replay_torque(cfg, measurement_path, diagnostic_path, output, oracle, ledger):
    packet = load_current_packet(measurement_path)
    with np.load(diagnostic_path, allow_pickle=False) as archive:
        tau, times = archive["tau_act"].copy(), archive["t"].copy()
    close(times, packet["t"], "reference torque timestamps", cfg)
    observer = oracle.Observer(cfg)
    records = {}
    for k, t in enumerate(times):
        result = update_counted(observer, (t, packet["q"][k], packet["v"][k], tau[k]), cfg, ledger)
        require(all(np.isfinite(value).all() for value in result.values()), "Nonfinite torque estimate")
        sample(records, t=t, **result)
    records = {key: np.asarray(value) for key, value in records.items()}
    save_series(output / "torque_estimates.npz", records)
    return records


def evaluate(cfg, case, output, current, torque, prefix):
    with np.load(output / "diagnostics.npz", allow_pickle=False) as archive:
        diagnostic = {key: archive[key].copy() for key in archive.files}
    differences = {key: close(current[key], torque[key], "paired "+key, cfg) for key in torque}
    differences["converted_torque"] = close(current["converted_torque"], diagnostic["tau_act"], "torque conversion", cfg)
    prefix_differences = {key: close(prefix[key], current[key][:len(prefix["t"])], "causal prefix "+key, cfg)
                          for key in prefix}
    require(all(value == 0 for value in prefix_differences.values()), "Prefix is not exactly identical")
    write(output / "paired_checks.json", dict(passed=True, maximum_absolute_differences=differences,
          prefix_maximum_absolute_differences=prefix_differences,
          comparison_rtol=cfg["comparison_rtol"], comparison_atol=cfg["comparison_atol"]))
    t = current["t"]
    windows = {"full": np.ones(len(t), dtype=bool), "steady": t >= cfg["evaluation"]["steady_start_s"]}
    if case["force"] == "step":
        windows["loaded"] = t >= cfg["force"]["start_s"]
    else:
        windows["after_startup"] = t >= cfg["evaluation"]["no_force_settle_s"]
    rows = []
    for window, selected in windows.items():
        for axis, name in enumerate(cfg["force"]["order"]):
            a = current["wrench"][selected, axis] - diagnostic["wrench"][selected, axis]
            b = torque["wrench"][selected, axis] - diagnostic["wrench"][selected, axis]
            rows.append(dict(case=case["name"], window=window, samples=int(selected.sum()),
                start_s=float(t[selected][0]), end_s=float(t[selected][-1]), axis=name,
                unit="N" if axis < 3 else "N m", current_rmse=float(np.sqrt(np.mean(a*a))),
                torque_rmse=float(np.sqrt(np.mean(b*b))),
                rmse_difference=float(np.sqrt(np.mean(a*a))-np.sqrt(np.mean(b*b))),
                current_bias=float(np.mean(a)), current_peak_absolute_error=float(np.max(np.abs(a))),
                maximum_paired_output_difference=float(np.max(np.abs(current["wrench"][selected, axis]-torque["wrench"][selected, axis])))))
    with (output / "six_axis_metrics.csv").open("x", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    write(output / "evaluation.json", dict(case=case["name"], six_axis=rows, differences=differences,
          units=dict(converted_torque="N m", nominal_input="N m", momentum="N m s", innovation="N m s",
                     wrench="first 3 N, last 3 N m", covariance="pairwise state-unit products")))
    return dict(case=case["name"], differences=differences, rows=rows, prefix_identical=True)


def probe(cfg, output, preflight_dir, oracle, helpers, ledger):
    previous = json.loads((preflight_dir / "metrics.json").read_text())
    require(previous["passed"] is True and previous["simulation_steps"] == 0, "Preflight did not pass")
    ledger["observer_calls"] = previous["observer_calls"]
    results = []
    for case in cfg["cases"]:
        case_dir = output / case["name"]
        case_dir.mkdir()
        started = time.perf_counter()
        sim = simulate(cfg, case, case_dir, oracle, helpers, ledger)
        current = replay_current(cfg, case_dir / "current_measurements.npz", case_dir, oracle, ledger)
        torque = replay_torque(cfg, case_dir / "current_measurements.npz", case_dir / "diagnostics.npz", case_dir, oracle, ledger)
        prefix = replay_current(cfg, case_dir / "current_measurements.npz", None, oracle, ledger, limit=101)
        result = evaluate(cfg, case, case_dir, current, torque, prefix)
        result["simulation"] = sim
        result["wall_seconds"] = time.perf_counter()-started
        results.append(result)
        print(json.dumps(dict(event="paired_case_complete", case=case["name"],
                              maximum_wrench_difference=result["differences"]["wrench"])), flush=True)
    write(output / "summary.json", dict(passed=True, results=results, cumulative_ledger=ledger))
    report = ["# 等效电流接口检查", "", "阶段：EXPLORE。No new scientific evidence was generated.", "",
        "两条轨迹各推进一次物理仿真，电流入口与力矩入口使用同一份 q/v/t 和实际驱动力矩来源。"
        "电流估计器仅读取 t/q/v/current；实际驱动力矩与外力真值另存。主对照 D 为单位映射，非均匀映射只作预检。", "",
        "| 场景 | 样本数 | 力矩往返最大差／N·m | 动量输入最大差／N·m | wrench 前三/后三分量最大差／N、N·m |",
        "|---|---:|---:|---:|---:|"]
    for result in results:
        d = result["differences"]
        report.append(f"| {result['case']} | {result['simulation']['feedback_samples']} | {d['converted_torque']:.9g} | {d['nominal_input']:.9g} | {d['wrench']:.9g}（本项仅表示相等检查，不作混合单位性能指标） |")
    report += ["", "## 六轴描述性误差", "", "以下为原生 125 Hz 的 RMSE；两入口数值相同。力与力矩分开报告，不计算混合单位向量范数。", "",
               "| 场景／窗口 | Fx／N | Fy／N | Fz／N | Mx／N·m | My／N·m | Mz／N·m |", "|---|---:|---:|---:|---:|---:|---:|"]
    for result in results:
        for window in (["after_startup"] if result["case"] == "motion" else ["loaded", "steady"]):
            rows = [r for r in result["rows"] if r["window"] == window]
            report.append("| "+result["case"]+" / "+window+" | "+" | ".join(f"{r['current_rmse']:.9g}" for r in rows)+" |")
    report += ["", "after_startup 为 0.2–6 s，loaded 为 1–6 s，steady 为 2–6 s，均包含端点。峰值、偏置与全段指标见各场景 six_axis_metrics.csv。", "",
        "## 结论与边界", "", "本轮验证等效电流接口不改变原 KF 输出，保留 16 kHz 物理、1 kHz 控制、125 Hz 观测。"
        "前缀一致性、换算及模型预检是工程验收；不证明真实电机模型正确，也未测试 MSMA。", "",
        "D 的单位为 N·m/A_eq，A_eq 不是已标定的 UR5 安培。当前无电流噪声、偏置、比例失配或电气动态。"
        "动力学采用准确参数、独立状态；控制含阻尼补偿，估计器按原方程减去 Bv 一次。", "",
        "采样瞬时电流在控制更新后记录；KF 沿用上一帧输入和 J 传播下一 8 ms。该保持近似未被修复，"
        "因此相同输出不等于零估力误差。没有使用未来反馈或高速真值推进观测器。", "",
        "下一步仅建议：另行明确并实现论文 MSMA 的 WMA/HMA 状态机、窗口历史与噪声标定，再做受控比较。"
        "本轮未执行，不更换默认模型或参数。", "", "## 计算", "",
        f"累计物理步数 {ledger['physics_steps']}，累计 KF 调用 {ledger['observer_calls']}（含预检与前缀）。",
        "实际进程墙钟见本目录 metrics.json 与 ../preflight/metrics.json；Docker 与调度耗时见 ../workflow-state.json。",
        "模型、配置与实现的本次快照见 ../inputs/；可执行请求见 ../request.snapshot.json。", ""]
    (output / "decision.md").write_text("\n".join(report))
    return dict(passed=True, cases=len(results), simulation_steps=ledger["physics_steps"],
                observer_calls=ledger["observer_calls"], feedback_samples=sum(r["simulation"]["feedback_samples"] for r in results),
                maximum_paired_wrench_difference=max(r["differences"]["wrench"] for r in results), prefix_identical=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["preflight", "probe"], required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--preflight-dir", type=Path)
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    ledger = dict(physics_steps=0, observer_calls=0)
    started = time.perf_counter()
    def timed_out(signum, frame):
        raise TimeoutError("Internal process time budget exceeded")
    signal.signal(signal.SIGALRM, timed_out)
    signal.alarm(55 if args.mode == "preflight" else 530)
    try:
        oracle = load(cfg["oracle_source"], "retained_oracle")
        helpers = load(cfg["simulation_helpers_source"], "retained_simulation_helpers")
        write(output / "resolved_config.json", cfg)
        if args.mode == "preflight":
            result = preflight(cfg, oracle, helpers, ledger, output, write)
        else:
            require(args.preflight_dir is not None, "Probe requires preflight directory")
            result = probe(cfg, output, args.preflight_dir, oracle, helpers, ledger)
        result["wall_seconds"] = time.perf_counter()-started
        write(output / "metrics.json", result)
        print(json.dumps(result), flush=True)
    except Exception as error:
        write(output / "failure.json", dict(error=repr(error), traceback=traceback.format_exc(), ledger=ledger))
        write(output / "metrics.json", dict(passed=False, error=repr(error), wall_seconds=time.perf_counter()-started, **ledger))
        raise
    finally:
        signal.alarm(0)
        write(output / "budget_ledger.json", dict(**ledger, wall_seconds=time.perf_counter()-started))


if __name__ == "__main__":
    main()
