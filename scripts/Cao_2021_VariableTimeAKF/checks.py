"""Checks specific to the equivalent-current adapter and preserved UR5 plant."""
import platform
import sys
import mujoco
import numpy as np
import scipy

from current_interface import CurrentObserver, EquivalentCurrentSensor, load_current_packet


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def close(actual, expected, name, cfg):
    require(np.allclose(actual, expected, rtol=cfg["comparison_rtol"],
                        atol=cfg["comparison_atol"]), f"Mismatch: {name}")
    return float(np.max(np.abs(np.asarray(actual) - np.asarray(expected))))


def preflight(cfg, oracle, helpers, ledger, output, write):
    versions = dict(python=platform.python_version(), mujoco=mujoco.__version__,
                    numpy=np.__version__, scipy=scipy.__version__, executable=sys.executable)
    write(output / "environment.json", versions)
    for package, expected in cfg["expected_versions"].items():
        require(versions[package] == expected, f"Environment mismatch: {package}")
    require(sys.version_info[:2] == (3, 12), "Python major/minor mismatch")
    m = mujoco.MjModel.from_binary_path(cfg["model"])
    names = [m.joint(i).name for i in range(m.njnt)]
    require((m.nq, m.nv, m.nu, m.na) == (6, 6, 6, 0), "Unexpected plant dimensions")
    require(names == cfg["joint_names"], "Joint order mismatch")
    require(np.array_equal(m.jnt_qposadr, np.arange(6)) and
            np.array_equal(m.jnt_dofadr, np.arange(6)), "Joint coordinate mapping mismatch")
    require(np.all(m.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT) and
            np.array_equal(m.actuator_trnid[:, 0], np.arange(6)), "Actuator transmission mismatch")
    close(m.actuator_gear, np.tile([1, 0, 0, 0, 0, 0], (6, 1)), "unit gears", cfg)
    require(np.all(m.actuator_dyntype == mujoco.mjtDyn.mjDYN_NONE) and
            np.all(m.actuator_gaintype == mujoco.mjtGain.mjGAIN_FIXED) and
            np.all(m.actuator_biastype == mujoco.mjtBias.mjBIAS_NONE), "Not direct torque motors")
    close(m.actuator_gainprm[:, 0], np.ones(6), "unit motor gain", cfg)
    require(m.opt.timestep == cfg["physics_dt_s"], "Physical timestep mismatch")
    require(m.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST, "Integrator mismatch")
    require(not (m.opt.disableflags & (mujoco.mjtDisableBit.mjDSBL_CONTACT |
                                      mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)), "Disabled contact guard")
    close(m.dof_damping, cfg["damping"], "damping", cfg)
    require(not np.any(m.dof_armature) and not np.any(m.dof_frictionloss), "Unexpected passive terms")
    close(m.opt.gravity, [0, 0, -9.81], "gravity", cfg)
    require(np.all(m.actuator_ctrllimited) and np.all(m.actuator_forcelimited), "Missing limits")
    limits = np.asarray([150, 150, 150, 28, 28, 28])
    close(m.actuator_ctrlrange, np.column_stack([-limits, limits]), "control limits", cfg)
    close(m.actuator_forcerange, np.column_stack([-limits, limits]), "force limits", cfg)
    control_stride = round(cfg["control_dt_s"] / m.opt.timestep)
    observer_stride = round(cfg["observer_dt_s"] / m.opt.timestep)
    require(control_stride == 16 and observer_stride == 128, "Multi-rate configuration mismatch")
    require(control_stride * m.opt.timestep == cfg["control_dt_s"] and
            observer_stride * m.opt.timestep == cfg["observer_dt_s"], "Nonintegral clock ratios")

    model = oracle.Model(cfg["model"], cfg["derivative_epsilon_s"])
    d = mujoco.MjData(m)
    mask = helpers.moving_mask(m)
    state_checks = []
    for t in [0.0, 0.25, 0.75]:
        q, v, _ = helpers.trajectory(t, cfg, True)
        M, h, Md, J = model.evaluate(q, v)
        d.qpos[:] = q
        d.qvel[:] = v
        d.ctrl[:] = h + model.B * v
        mujoco.mj_forward(m, d)
        helpers.valid(m, d, mask, d.ctrl.copy())
        passive_error = close(d.qfrc_passive, -model.B * v, "passive damping", cfg)
        balance_error = close(M @ d.qacc + h, d.qfrc_actuator + d.qfrc_passive,
                              "continuous balance", cfg)
        require(np.isfinite(Md).all() and np.linalg.matrix_rank(J) == 6, "Invalid nominal dynamics")
        state_checks.append(dict(t_s=t, passive_error_Nm=passive_error, balance_error_Nm=balance_error))

    D = np.array(cfg["nonuniform_fixture_D_Nm_per_Aeq"])
    sensor = EquivalentCurrentSensor(D)
    rows = []

    class Capture:
        def update(self, t, q, v, torque):
            return {"captured_torque": torque.copy()}

    capture = CurrentObserver(Capture(), D)
    for axis in range(6):
        for sign in [-1, 1]:
            tau = np.zeros(6)
            tau[axis] = sign * (axis + 1)
            current = sensor.measure(tau)
            expected = np.zeros(6)
            expected[axis] = sign * (axis + 1) / D[axis]
            close(current, expected, "per-axis current", cfg)
            result = capture.update(len(rows) * cfg["observer_dt_s"], np.zeros(6), np.zeros(6), current)
            err = close(result["captured_torque"], tau, "current roundtrip", cfg)
            rows.append(dict(joint=axis+1, sign=sign, torque_Nm=tau.tolist(),
                             current_Aeq=current.tolist(), maximum_error_Nm=err))

    rejected = []
    def rejects(name, operation):
        try:
            operation()
        except ValueError as error:
            rejected.append(dict(case=name, error=str(error)))
        else:
            raise RuntimeError(f"Invalid input accepted: {name}")

    for name, bad in [("wrong_D_dimension", [1]*5), ("zero_D", [0,1,1,1,1,1]),
                      ("nonfinite_D", [np.nan,1,1,1,1,1]), ("negative_D", [-1,1,1,1,1,1])]:
        rejects(name, lambda value=bad: EquivalentCurrentSensor(value))
        rejects(name+"_estimator", lambda value=bad: CurrentObserver(Capture(), value))
    rejects("wrong_torque_dimension", lambda: sensor.measure([1]*5))
    rejects("nonfinite_torque", lambda: sensor.measure([np.inf]*6))
    for key in ["q", "v", "current"]:
        args = dict(t=1.0, q=np.zeros(6), v=np.zeros(6), current=np.zeros(6))
        args[key] = np.zeros(5)
        rejects("wrong_"+key+"_dimension", lambda value=args: capture.update(**value))
        args = dict(t=1.0, q=np.zeros(6), v=np.zeros(6), current=np.zeros(6))
        args[key] = np.full(6, np.nan)
        rejects("nonfinite_"+key, lambda value=args: capture.update(**value))
    rejects("nonfinite_time", lambda: capture.update(np.nan, np.zeros(6), np.zeros(6), np.zeros(6)))
    capture.update(1.0, np.zeros(6), np.zeros(6), np.zeros(6))
    rejects("duplicate_time", lambda: capture.update(1.0, np.zeros(6), np.zeros(6), np.zeros(6)))
    rejects("backward_time", lambda: capture.update(0.9, np.zeros(6), np.zeros(6), np.zeros(6)))

    contaminated = output / "rejected_packet_fixture.npz"
    np.savez_compressed(contaminated, t=[0.0], q=np.zeros((1, 6)), v=np.zeros((1, 6)),
                        current=np.zeros((1, 6)), tau=np.zeros((1, 6)))
    rejects("extra_torque_field", lambda: load_current_packet(contaminated))

    original = oracle.Observer(cfg)
    current_observer = CurrentObserver(oracle.Observer(cfg), D)
    nonuniform_differences = []
    for n in range(3):
        t = n * cfg["observer_dt_s"]
        q, v, a = helpers.trajectory(t, cfg, True)
        M, h, _, _ = model.evaluate(q, v)
        tau = M @ a + h + model.B * v
        ledger["observer_calls"] += 2
        old = original.update(t, q, v, tau)
        new = current_observer.update(t, q, v, sensor.measure(tau))
        nonuniform_differences.append({key: close(new[key], old[key], "nonuniform KF "+key, cfg)
                                       for key in old})
    write(output / "checks.json", dict(versions=versions, joint_names=names,
          control_stride=control_stride, observer_stride=observer_stride,
          model_states=state_checks, signed_conversion=rows, rejected_inputs=rejected,
          nonuniform_KF_differences=nonuniform_differences, passed=True))
    return dict(passed=True, rejected_invalid_inputs=len(rejected), conversion_cases=len(rows),
                simulation_steps=ledger["physics_steps"], observer_calls=ledger["observer_calls"])
