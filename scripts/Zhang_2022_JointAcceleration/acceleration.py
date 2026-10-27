"""Zhang acceleration observers; physical units, causal Euler, no truth input."""
import numpy as np

METHODS = ('nominal', 'gm', 'hodo', 'gm_two_layer', 'gm_hodo')


class HODO:
    """Independent observer bank; each row has its own three extended states."""
    def __init__(self, gains, third_layer=(True, False, True)):
        self.gains = np.asarray(gains, dtype=float)
        assert self.gains.shape == (6, 6)
        self.third = np.asarray(third_layer, dtype=bool)
        self.reset()

    def reset(self):
        self.previous_time = None

    def update(self, t, velocity, base_acceleration):
        v = np.asarray(velocity, dtype=float)
        base = np.asarray(base_acceleration, dtype=float)
        assert v.shape == (6,) and base.shape == (len(self.third), 6)
        if self.previous_time is None:
            self.x = np.zeros((len(self.third), 3, 6))
            self.x[:, 0] = v
        else:
            dt = float(t - self.previous_time)
            assert dt > 0
            self.x = self.x + dt * self.previous_derivative
        self.x[~self.third, 2] = 0.0
        e = v[None, :] - self.x[:, 0]
        s = np.sign(e)
        k1, k2, k3, k4, k5, k6 = self.gains
        u1 = k1 * np.sqrt(np.abs(e)) * s + k2 * e
        u2 = k3 * s + k4 * e
        u3 = (k5 * s + k6 * e) * self.third[:, None]
        acceleration = base + self.x[:, 1] + u1
        derivative = np.stack((acceleration, self.x[:, 2] + u2, u3), axis=1)
        if not np.isfinite(derivative).all() or not np.isfinite(self.x).all():
            raise FloatingPointError('Nonfinite HODO state or output')
        self.previous_derivative = derivative.copy()
        self.previous_time = float(t)
        return dict(acceleration=acceleration, state=self.x.copy(), error=e,
                    correction=np.stack((u1, u2, u3), axis=1))


class AccelerationBank:
    """Only receives timestamp, q, v and actual drive torque plus nominal model."""
    def __init__(self, nominal, momentum_core, gains, damping_factor=1.0):
        self.nominal = nominal
        self.momentum_core = momentum_core
        self.B = damping_factor * nominal.B.copy()
        self.hodo = HODO(gains)

    def update(self, t, q, v, tau):
        M, h, Md, _ = self.nominal.evaluate(q, v)
        p = M @ v
        rhs = tau - h - self.B * v
        known = rhs + Md @ v
        r, phat = self.momentum_core.update(t, p, known)
        solved = np.linalg.solve(M, np.column_stack((rhs, r)))
        f, external_acc = solved.T
        fstar = f + external_acc
        bank = self.hodo.update(t, v, np.stack((f, fstar, fstar)))
        output = dict(acceleration=np.vstack((f, fstar, bank['acceleration'])),
                      state=bank['state'], correction=bank['correction'],
                      velocity_error=bank['error'], gm_residual=r, momentum=p,
                      estimated_momentum=phat, known_input=known, f=f, fstar=fstar)
        if not all(np.isfinite(x).all() for x in output.values()):
            raise FloatingPointError('Nonfinite estimator output')
        return output
