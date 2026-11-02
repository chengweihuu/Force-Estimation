"""12-state momentum/wrench KF. No plant state or force reference is accepted."""
from dataclasses import dataclass

import numpy as np

from dynamics import vector


@dataclass(frozen=True)
class Measurement:
    time_s: float
    position_rad: np.ndarray
    velocity_rad_s: np.ndarray
    drive_torque_Nm: np.ndarray


def discretize(J, dt, Qp, Qw):
    """Exact frozen-coefficient discretization for a wrench random walk."""
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("Sampling interval must be finite and positive")
    D = np.asarray(J, dtype=float).T
    if D.shape != (6, 6) or not np.isfinite(D).all():
        raise ValueError("Invalid Jacobian")
    A, B = np.eye(12), np.zeros((12, 6))
    A[:6, 6:], B[:6] = dt * D, dt * np.eye(6)
    Q = np.zeros((12, 12))
    Q[:6, :6] = dt * Qp + dt**3 / 3 * D @ Qw @ D.T
    Q[:6, 6:] = dt**2 / 2 * D @ Qw
    Q[6:, :6] = Q[:6, 6:].T
    Q[6:, 6:] = dt * Qw
    return A, B, Q


def covariance_check(P, tolerance):
    if not np.isfinite(P).all():
        raise FloatingPointError("Nonfinite covariance")
    if np.max(np.abs(P - P.T)) > tolerance:
        raise ValueError("Asymmetric covariance")
    minimum = float(np.linalg.eigvalsh(P).min())
    if minimum < -tolerance:
        raise ValueError(f"Indefinite covariance: {minimum}")
    return minimum


def correct(x, P, y, previous_input, previous_J, dt, Qp, Qw, R):
    A, B, Q = discretize(previous_J, dt, Qp, Qw)
    predicted_x = A @ x + B @ previous_input
    predicted_P = A @ P @ A.T + Q
    residual = y - predicted_x[:6]
    K = np.linalg.solve(predicted_P[:6, :6] + R, predicted_P[:, :6].T).T
    C = np.eye(6, 12)
    left = np.eye(12) - K @ C
    updated_P = left @ predicted_P @ left.T + K @ R @ K.T
    return predicted_x + K @ residual, (updated_P + updated_P.T) / 2, residual


class MomentumKF:
    def __init__(self, nominal, config):
        self.nominal = nominal
        diagonals = {}
        for name in ("Qp_continuous_diag", "Qw_continuous_diag",
                     "assumed_velocity_std_rad_s", "initial_wrench_variance"):
            values = vector(config[name], name)
            if np.any(values < 0):
                raise ValueError(f"Negative {name}")
            diagonals[name] = values
        if np.any(diagonals["assumed_velocity_std_rad_s"] <= 0):
            raise ValueError("Require positive assumed velocity noise")
        self.Qp = np.diag(diagonals["Qp_continuous_diag"])
        self.Qw = np.diag(diagonals["Qw_continuous_diag"])
        self.velocity_cov = np.diag(diagonals["assumed_velocity_std_rad_s"]**2)
        self.initial_wrench_cov = np.diag(diagonals["initial_wrench_variance"])
        self.covariance_tolerance = float(config["covariance_psd_atol"])
        self.previous = None

    def update(self, measurement):
        if not isinstance(measurement, Measurement):
            raise TypeError("Estimator accepts only the declared Measurement packet")
        t = float(measurement.time_s)
        if not np.isfinite(t) or (self.previous is not None and t <= self.previous[0]):
            raise ValueError("Measurement timestamps must be finite and strictly increasing")
        q = vector(measurement.position_rad, "q")
        v = vector(measurement.velocity_rad_s, "v")
        torque = vector(measurement.drive_torque_Nm, "drive torque")
        terms = self.nominal.evaluate(q, v)
        y = terms.mass @ v
        R = terms.mass @ self.velocity_cov @ terms.mass.T
        if self.previous is None:
            self.x = np.r_[y, np.zeros(6)]
            self.P = np.zeros((12, 12))
            self.P[:6, :6], self.P[6:, 6:] = R, self.initial_wrench_cov
            innovation = np.zeros(6)
        else:
            t0, u0, J0 = self.previous
            self.x, self.P, innovation = correct(
                self.x, self.P, y, u0, J0, t - t0, self.Qp, self.Qw, R)
        if not np.isfinite(self.x).all():
            raise FloatingPointError("Nonfinite KF state")
        min_cov = covariance_check(self.P, self.covariance_tolerance)
        u = terms.momentum_input(v, torque)
        self.previous = (t, u.copy(), terms.jacobian.copy())
        return {
            "wrench": self.x[6:].copy(), "momentum": y, "input": u,
            "joint_external_torque": terms.jacobian.T @ self.x[6:],
            "innovation": innovation, "covariance": self.P.copy(),
            "covariance_min_eigenvalue": min_cov,
            "jacobian_singular_values": np.linalg.svd(terms.jacobian, compute_uv=False),
        }
