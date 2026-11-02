"""Independent prerequisite checks, executed only through the approved request."""
from dataclasses import fields
from importlib.metadata import version
import json
import platform
import sys

import mujoco
import numpy as np
from scipy.integrate import quad_vec
from scipy.linalg import expm

from dynamics import KnownDynamics, apply_wrench, jacobian, load_model, mass_matrix
from observer import Measurement, correct, covariance_check, discretize
from simulation import guard, moving_bodies


def environment():
    expected = {"mujoco": "3.12.0", "numpy": "1.26.4", "scipy": "1.14.1",
                "matplotlib": "3.9.4", "torch": "2.9.1+cpu"}
    actual = {name: version(name) for name in expected}
    if actual != expected or sys.version_info[:2] != (3, 12):
        raise RuntimeError(f"Runtime differs from declared Docker dependencies: {actual}; {sys.version}")
    import torch
    if torch.version.cuda is not None:
        raise RuntimeError("Expected the existing CPU PyTorch environment")
    return {"packages": actual, "python": platform.python_version(), "platform": platform.platform(),
            "physics_steps": 0, "ros_nodes_started": 0, "pytorch_used_for_training": False}


def run_checks(root, config):
    report = {"environment": environment(), "comparisons": {}}
    tol = config["checks"]

    def compare(name, actual, expected, derivative=False):
        prefix = "derivative" if derivative else "algebra"
        np.testing.assert_allclose(actual, expected, rtol=tol[f"{prefix}_rtol"],
                                   atol=tol[f"{prefix}_atol"], err_msg=name)
        report["comparisons"][name] = {"max_abs_error": float(np.max(np.abs(actual - expected))),
                                         "rtol": tol[f"{prefix}_rtol"], "atol": tol[f"{prefix}_atol"]}

    rng = np.random.default_rng(config["seed"])
    for dt in (0.001, 0.008, 0.02):
        J = rng.normal(size=(6, 6))
        Qp, Qw = np.diag(np.linspace(0.001, 0.006, 6)), np.diag(np.linspace(0.1, 0.6, 6))
        A, B, Q = discretize(J, dt, Qp, Qw)
        Ac, Bc, Qc = np.zeros((12, 12)), np.zeros((12, 6)), np.zeros((12, 12))
        Ac[:6, 6:], Bc[:6] = J.T, np.eye(6)
        Qc[:6, :6], Qc[6:, 6:] = Qp, Qw
        augmented = np.zeros((18, 18))
        augmented[:12, :12], augmented[:12, 12:] = Ac, Bc
        transition = expm(augmented * dt)
        integrated_Q, _ = quad_vec(lambda s: expm(Ac * s) @ Qc @ expm(Ac.T * s), 0, dt)
        compare(f"A_dt_{dt}", A, transition[:12, :12])
        compare(f"B_dt_{dt}", B, transition[:12, 12:])
        compare(f"Q_dt_{dt}", Q, integrated_Q)
        covariance_check(Q, config["observer"]["covariance_psd_atol"])
        x, P, y = np.zeros(12), np.eye(12), np.zeros(6)
        for _ in range(50):
            x, P, _ = correct(x, P, y, np.zeros(6), J, dt, Qp, Qw, np.eye(6) * 1e-8)
            covariance_check(P, config["observer"]["covariance_psd_atol"])

    design = json.loads((root / config["control_design"]).read_text())
    q, v = np.array(design["q_hold_rad"]), np.array([0.11, -0.17, 0.13, 0.2, -0.14, 0.07])
    nominal = KnownDynamics(root / config["model"], config["derivative_epsilon_s"])
    terms = nominal.evaluate(q, v)
    compare("inertia_symmetry", terms.mass, terms.mass.T)
    report["inertia_min_eigenvalue"] = float(np.linalg.eigvalsh(terms.mass).min())
    gravity = nominal.evaluate(q, np.zeros(6)).bias
    for h in tol["coordinate_difference_steps_rad"]:
        derivatives = []
        for basis in np.eye(6):
            derivatives.append((nominal.mass_at(q + h * basis, nominal.plus)
                                - nominal.mass_at(q - h * basis, nominal.minus)) / (2 * h))
        dM = np.stack(derivatives, axis=-1)
        md = np.einsum("ijk,k->ij", dM, v)
        gradient = 0.5 * np.einsum("i,ijk,j->k", v, dM, v)
        C = np.empty((6, 6))
        for i in range(6):
            for j in range(6):
                C[i, j] = sum(0.5 * (dM[i, j, k] + dM[i, k, j] - dM[j, k, i]) * v[k]
                              for k in range(6))
        compare(f"directional_vs_coordinate_Mdot_{h}", terms.mass_dot, md, True)
        compare(f"Christoffel_identity_{h}", md, C + C.T, True)
        compare(f"recursive_bias_vs_energy_c_{h}", terms.bias - gravity, md @ v - gradient, True)
        compare(f"transpose_C_velocity_{h}", C.T @ v, gradient, True)

    model = load_model(root / config["model"])
    data = mujoco.MjData(model)
    data.qpos[:], data.qvel[:] = q, v
    data.ctrl[:] = gravity + model.dof_damping * v
    mujoco.mj_forward(model, data)
    wrench = np.array([2., -3., 5., 0.2, -0.1, 0.3])
    applied = apply_wrench(model, data, wrench)
    mujoco.mj_forward(model, data)
    guard(model, data, data.ctrl.copy(), moving_bodies(model), tol)
    J = jacobian(model, data)
    compare("applied_force_mapping", applied, J.T @ wrench)
    compare("viscous_passive_force", data.qfrc_passive.copy(), -model.dof_damping * v)
    balance = (mass_matrix(model, data) @ data.qacc + data.qfrc_bias
               - data.qfrc_actuator - data.qfrc_passive - applied - data.qfrc_constraint)
    compare("continuous_dynamics_balance", balance, np.zeros(6))
    compare("virtual_work", np.array(applied @ v), np.array(wrench @ (J @ v)))

    # Kinematic finite differences test both linear and angular Jacobian rows.
    h = tol["coordinate_difference_steps_rad"][-1]
    plus, minus = mujoco.MjData(model), mujoco.MjData(model)
    plus.qpos[:], minus.qpos[:] = q + h * v, q - h * v
    mujoco.mj_fwdPosition(model, plus)
    mujoco.mj_fwdPosition(model, minus)
    site = model.site("tool0").id
    linear = (plus.site_xpos[site] - minus.site_xpos[site]) / (2 * h)
    dR = (plus.site_xmat[site].reshape(3, 3) - minus.site_xmat[site].reshape(3, 3)) / (2 * h)
    angular_skew = dR @ data.site_xmat[site].reshape(3, 3).T
    angular = np.array([angular_skew[2, 1], angular_skew[0, 2], angular_skew[1, 0]])
    compare("kinematic_velocity", np.r_[linear, angular], J @ v, True)
    if [field.name for field in fields(Measurement)] != [
            "time_s", "position_rad", "velocity_rad_s", "drive_torque_Nm"]:
        raise ValueError("Measurement interface includes undeclared information")
    report["measurement_fields"] = [field.name for field in fields(Measurement)]
    report["passed"] = True
    report["comparison_count"] = len(report["comparisons"])
    return report
