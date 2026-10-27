"""Causal current processing: paper WMA/HMA equations and Fig. 2 control flow.

Per-axis reset and continuous HMA caches are explicit local implementation choices.
No experiment identity, plant state, wrench, or noise calibration is used here.
"""
from collections import deque
import math
import numpy as np
from current_interface import vector


def wma_weights(n):
    if not isinstance(n, (int, np.integer)) or n < 1:
        raise ValueError("Window length must be a positive integer")
    return 2 * np.arange(n, 0, -1, dtype=float) / (n * (n+1))


def hma_weights(n):
    if not isinstance(n, (int, np.integer)) or n < 2:
        raise ValueError("HMA requires N >= 2")
    half, outer = n//2, math.isqrt(n)
    combined = -wma_weights(n)
    combined[:half] += 2*wma_weights(half)
    return np.convolve(wma_weights(outer), combined)  # newest coefficient first


def linear_series(values, weights):
    """Finite convolution, zero padding only outside returned valid history.

Callers must discard the first len(weights)-1 outputs; they are NOT warmup data.
"""
    x = np.asarray(values, dtype=float)
    if x.ndim != 2 or x.shape[1] != 6 or not np.isfinite(x).all():
        raise ValueError("Expected a finite [samples,6] signal")
    return np.column_stack([np.convolve(x[:,j], weights, mode="full")[:len(x)] for j in range(6)])


class CurrentFilter:
    def __init__(self, method, window, threshold):
        if method not in ("raw", "wma", "hma", "msma"):
            raise ValueError("Unknown current processing method")
        if not isinstance(window, (int, np.integer)) or window < 2:
            raise ValueError("Maximum window must be an integer >= 2")
        self.method, self.N = method, int(window)
        self.half, self.outer = self.N//2, math.isqrt(self.N)
        self.threshold = vector(threshold, "threshold")
        if np.any(self.threshold <= 0):
            raise ValueError("Thresholds must be finite and positive")
        self.raw = deque(maxlen=self.N)
        self.intermediate = deque(maxlen=self.outer)
        self.weights = {n: wma_weights(n) for n in range(1,self.N+1)}
        self.outer_weights = wma_weights(self.outer)
        self.r = np.zeros(6, dtype=int)
        self.k = -1
        self.previous = None
        self.primed = False

    def _append(self, value):
        self.raw.append(vector(value, "current"))
        if len(self.raw) == self.N:
            newest = np.asarray(self.raw)[::-1]
            a = 2*(self.weights[self.half] @ newest[:self.half]) - self.weights[self.N] @ newest
            self.intermediate.append(a)

    def prime(self, past_current):
        x = np.asarray(past_current, dtype=float)
        if self.primed or self.k >= 0:
            raise ValueError("History can be primed once, before the first output")
        if x.ndim != 2 or x.shape[1] != 6 or len(x) < self.N+self.outer-1 or not np.isfinite(x).all():
            raise ValueError("Not enough actual past measurements for HMA")
        for row in x:
            self._append(row)
        self.primed = True

    def update(self, current):
        if not self.primed:
            raise ValueError("Prime with actual past measurements first")
        x = vector(current, "current")
        self.k += 1
        self._append(x)
        newest = np.asarray(self.raw)[::-1]
        hma = self.outer_weights @ np.asarray(self.intermediate)[::-1]
        reset = np.zeros(6, dtype=bool)
        checked = np.zeros(6, dtype=bool)
        length = np.full(6, self.N, dtype=int)
        mode = np.full(6, 2, dtype=int)  # raw=0, WMA=1, HMA=2
        if self.method == "raw":
            output, mode, length = x.copy(), np.zeros(6,dtype=int), np.ones(6,dtype=int)
        elif self.method == "wma":
            output, mode = self.weights[self.N] @ newest, np.ones(6,dtype=int)
        elif self.method == "hma":
            output = hma.copy()
        else:
            output = np.empty(6)
            for j in range(6):
                age = self.k - self.r[j] + 1
                if age <= self.N:
                    mode[j], length[j] = 1, age
                    output[j] = self.weights[age] @ newest[:age,j]
                else:
                    checked[j] = True
                    if abs(x[j]-self.previous[j]) <= self.threshold[j]:
                        output[j] = hma[j]
                    else:
                        self.r[j] = self.k
                        mode[j], length[j], reset[j] = 1, 1, True
                        output[j] = x[j]
        self.previous = output.copy()
        return dict(current=output, mode=mode, length=length, reset=reset, threshold_checked=checked)


def process(values, method, window, threshold, history_count):
    filt = CurrentFilter(method, window, threshold)
    filt.prime(values[:history_count])
    records = {key: [] for key in ("current", "mode", "length", "reset", "threshold_checked")}
    for value in values[history_count:]:
        row = filt.update(value)
        for key in records:
            records[key].append(row[key])
    return {key: np.asarray(value) for key,value in records.items()}
