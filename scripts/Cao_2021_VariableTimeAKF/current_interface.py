"""Equivalent-current I/O only; no electrical motor model or experiment identity.

Current is joint-positive, measured in A_eq. D has units N m / A_eq.
The sensor and observer deliberately own separate copies of their coefficients.
"""
import numpy as np


def vector(value, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (6,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite six-vector")
    return result.copy()


def coefficients(value):
    result = vector(value, "D")
    if np.any(result <= 0):
        raise ValueError("D must be positive; current signs are joint-positive")
    return result


class EquivalentCurrentSensor:
    """Plant-side conversion of ACTUAL actuator joint torque, with zero noise."""

    def __init__(self, D_true):
        self.D_true = coefficients(D_true)

    def measure(self, actual_actuator_torque):
        return vector(actual_actuator_torque, "actual_actuator_torque") / self.D_true


class CurrentObserver:
    """Only t/q/v/current enter this wrapper; the inner observer owns its model."""

    def __init__(self, torque_observer, D_est):
        self._observer = torque_observer
        self.D_est = coefficients(D_est)
        self._last_time = None

    def update(self, t, q, v, current):
        if np.ndim(t) != 0 or not np.isfinite(t):
            raise ValueError("Timestamp must be a finite scalar")
        t = float(t)
        if self._last_time is not None and t <= self._last_time:
            raise ValueError("Timestamps must be strictly increasing")
        q, v, current = vector(q, "q"), vector(v, "v"), vector(current, "current")
        torque = self.D_est * current
        result = dict(self._observer.update(t, q, v, torque))
        self._last_time = t
        result["converted_torque"] = torque.copy()
        return result


def load_current_packet(path):
    """A strict measured-signal boundary; diagnostic torque/truth is rejected."""
    with np.load(path, allow_pickle=False) as archive:
        if set(archive.files) != {"t", "q", "v", "current"}:
            raise ValueError("Current feedback must contain exactly t/q/v/current")
        packet = {key: archive[key].copy() for key in archive.files}
    t = packet["t"]
    if t.ndim != 1 or len(t) == 0 or not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError("Invalid sampling timestamps")
    for key in ("q", "v", "current"):
        if packet[key].shape != (len(t), 6) or not np.isfinite(packet[key]).all():
            raise ValueError(f"Invalid feedback {key}")
    return packet
