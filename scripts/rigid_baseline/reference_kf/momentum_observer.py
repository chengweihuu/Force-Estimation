"""Momentum/wrench random-walk KF and a sign-consistent GMO debug baseline.

All dynamics are supplied by an independent nominal model. Inputs to update are
only timestamp, measured q/v, and the measured joint drive-torque proxy.
"""
import numpy as np


def discretize(J, dt, Qp, Qw):
    D = J.T
    A = np.eye(12)
    A[:6, 6:] = dt*D
    B = np.zeros((12, 6))
    B[:6] = dt*np.eye(6)
    Q = np.zeros((12, 12))
    Q[:6, :6] = dt*Qp + dt**3/3*(D @ Qw @ D.T)
    Q[:6, 6:] = dt**2/2*(D @ Qw)
    Q[6:, :6] = Q[:6, 6:].T
    Q[6:, 6:] = dt*Qw
    return A, B, Q


def kf_step(x, P, y, previous_input, previous_J, dt, Qp, Qw, R):
    A, B, Q = discretize(previous_J, dt, Qp, Qw)
    xp = A @ x + B @ previous_input
    Pp = A @ P @ A.T + Q
    innovation = y - xp[:6]
    S = Pp[:6, :6] + R
    K = np.linalg.solve(S, Pp[:, :6].T).T
    C = np.eye(6, 12)
    left = np.eye(12) - K @ C
    P = left @ Pp @ left.T + K @ R @ K.T
    return xp + K @ innovation, (P + P.T)/2, innovation


class MomentumObserver:
    def __init__(self, nominal, config):
        self.nominal = nominal
        self.Qp = np.diag(config["Qp_continuous_diag"])
        self.Qw = np.diag(config["Qw_continuous_diag"])
        self.velocity_cov = np.diag(np.square(config["assumed_velocity_std_rad_s"]))
        self.wrench_initial_var = config["initial_wrench_variance"]
        self.gain = np.asarray(config["gmo_gain_s_inv"])
        self.previous = None

    def update(self, timestamp, q_meas, v_meas, torque_meas):
        q, v, torque = (np.asarray(value, dtype=float) for value in (q_meas, v_meas, torque_meas))
        assert all(value.shape == (6,) and np.isfinite(value).all() for value in (q, v, torque))
        M, h, Mdot, J = self.nominal.evaluate(q, v)
        p = M @ v
        u = Mdot @ v + torque - h  # Source model has zero passive friction.
        R = M @ self.velocity_cov @ M.T
        if self.previous is None:
            self.x = np.r_[p, np.zeros(6)]
            self.P = np.zeros((12, 12))
            self.P[:6, :6] = R
            self.P[6:, 6:] = np.diag(self.wrench_initial_var)
            self.gmo_momentum = p.copy()
            self.gmo_torque = np.zeros(6)
            innovation = np.zeros(6)
        else:
            t0, u0, J0 = self.previous
            dt = timestamp - t0
            assert dt > 0
            self.x, self.P, innovation = kf_step(self.x, self.P, p, u0, J0, dt, self.Qp, self.Qw, R)
            self.gmo_momentum += dt*(u0 + self.gmo_torque)
            self.gmo_torque = self.gain*(p - self.gmo_momentum)
        self.previous = (timestamp, u, J)
        return {
            "wrench": self.x[6:].copy(), "joint_external_torque": J.T @ self.x[6:],
            "gmo_wrench": np.linalg.solve(J.T, self.gmo_torque),
            "gmo_joint_torque": self.gmo_torque.copy(),
            "innovation": innovation.copy(), "covariance": self.P.copy(),
            "nominal_input": u, "momentum": p,
        }
