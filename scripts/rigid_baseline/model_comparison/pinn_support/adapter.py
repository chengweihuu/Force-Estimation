"""Reference-gravity interval adapter; supplied gravity is a fixed oracle input.

The fixed linear operator has an exact adjoint including SOS initial states,
odd endpoint padding, reversed pass, derivative boundaries and accumulation.
"""
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi, sosfiltfilt
import torch
from torch import nn
from torch.nn import functional as F


def require(ok, message):
    if not ok:
        raise ValueError(message)


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def json_value(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().tolist()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def write_json(path, value):
    payload = json.dumps(value, default=json_value, ensure_ascii=False, indent=2, allow_nan=False)+"\n"
    with Path(path).open("x") as f:
        f.write(payload)


def read_json(path):
    return json.loads(Path(path).read_text())


class IntervalOperator:
    def __init__(self, physical_steps, plan, core=None):
        self.stride = int(round(plan["feedback_dt_s"]/plan["physics_dt_s"]))
        require(physical_steps % self.stride == 0, "Incomplete feedback interval")
        self.count = int(physical_steps)
        self.n = self.count//self.stride+1
        self.dt = plan["feedback_dt_s"]
        spec = plan["filter"]
        self.pad = spec["padlen"]
        require(spec["padtype"] == "odd" and spec["edge_order"] == 2 and self.n > self.pad,
                "Only approved padding/derivative supported")
        self.sos = butter(spec["order"], spec["cutoff_Hz"], fs=1/self.dt, btype="lowpass", output="sos")
        self.zi = sosfilt_zi(self.sos)
        self.core = tuple(core) if core is not None else tuple(int(round(t/self.dt)) for t in plan["core_interval_s"])
        require(0 <= self.core[0] < self.core[1] <= self.n, "Invalid core")
        # S(x)=H*x+response*x[0] for a filter initialized at steady state x[0].
        self.initial_response = sosfilt(self.sos, np.zeros(self.n+2*self.pad), zi=self.zi)[0]

    def forward(self, impulse):
        require(impulse.shape == (self.count, 6), "Expected one six-axis impulse per physics step")
        block = impulse.reshape(-1, self.stride, 6).sum(axis=1)
        nodes = np.vstack((np.zeros((1, 6)), np.cumsum(block, axis=0)))
        filtered = sosfiltfilt(self.sos, nodes, axis=0, padtype="odd", padlen=self.pad)
        derivative = np.gradient(filtered, self.dt, axis=0, edge_order=2)
        return derivative[self.core[0]:self.core[1]].copy()

    def _filter_transpose(self, gradient):
        out = sosfilt(self.sos, gradient[::-1], axis=0)[::-1].copy()
        out[0] += self.initial_response @ gradient
        return out

    def adjoint(self, gradient):
        require(gradient.shape == (self.core[1]-self.core[0], 6), "Wrong residual gradient shape")
        g = np.zeros((self.n, 6))
        g[self.core[0]:self.core[1]] = gradient
        # Transpose of numpy.gradient(edge_order=2), including endpoints.
        d = np.zeros_like(g)
        d[:-2] -= g[1:-1]/(2*self.dt)
        d[2:] += g[1:-1]/(2*self.dt)
        d[0] -= 3*g[0]/(2*self.dt); d[1] += 2*g[0]/self.dt; d[2] -= g[0]/(2*self.dt)
        d[-1] += 3*g[-1]/(2*self.dt); d[-2] -= 2*g[-1]/self.dt; d[-3] += g[-1]/(2*self.dt)
        # Transpose of crop(reverse(S(reverse(S(odd_pad(nodes)))))).
        padded = np.zeros((self.n+2*self.pad, 6))
        padded[self.pad:-self.pad] = d
        padded = self._filter_transpose(padded[::-1])[::-1]
        padded = self._filter_transpose(padded)
        nodes = padded[self.pad:-self.pad].copy()
        nodes[0] += 2*padded[:self.pad].sum(axis=0)
        nodes[1:self.pad+1] -= padded[:self.pad][::-1]
        nodes[-1] += 2*padded[-self.pad:].sum(axis=0)
        nodes[-self.pad-1:-1] -= padded[-self.pad:][::-1]
        block = np.cumsum(nodes[:0:-1], axis=0)[::-1]
        return np.repeat(block, self.stride, axis=0)

    def torch(self, impulse):
        return _IntervalAutograd.apply(impulse, self)


class _IntervalAutograd(torch.autograd.Function):
    @staticmethod
    def forward(ctx, impulse, operator):
        require(impulse.device.type == "cpu" and impulse.dtype == torch.float64,
                "This CPU pilot requires float64")
        ctx.operator = operator
        return torch.from_numpy(operator.forward(impulse.detach().numpy()))

    @staticmethod
    def backward(ctx, grad_output):
        return torch.from_numpy(ctx.operator.adjoint(grad_output.detach().numpy())), None


class LearnedDynamics(nn.Module):
    def __init__(self, structured_class, plan, center=None, scale=None, gravity_mean=None):
        super().__init__()
        self.network = structured_class(**plan["model"], input_center_rad=center, input_scale_rad=scale)
        initial = plan["physical_friction_initial"]
        self.friction_raw = nn.Parameter(torch.full((6,), np.log(np.expm1(initial)), dtype=torch.float64))
        if gravity_mean is not None:
            with torch.no_grad():
                self.network.layers[-1].bias[21:] = torch.as_tensor(gravity_mean, dtype=torch.float64)

    def predict(self, q, v):
        result = self.network.dynamics(q, v)
        return result["M"], result["h"], F.softplus(self.friction_raw)*v


def impulses_from_predictions(mass, bias, friction, dv, tau, h, known_B):
    """B is a fixed numerical prior; physical friction remains a separate output."""
    return torch.einsum("bij,bj->bi", mass, dv)+h*(bias+friction-tau)+h*known_B*dv


def model_impulses(model, q, v, dv, tau, h, known_B, gravity):
    result = model.network.dynamics(q, v)
    mass = result['M']
    bias = result['h'] - result['G'] + gravity
    friction = F.softplus(model.friction_raw)*v
    return impulses_from_predictions(mass, bias, friction, dv, tau, h, known_B)


PACKET_FIELDS = {"t", "q", "v", "tau_actuator", "ctrl", "control_update", "control_index",
                 "feedback_sample", "physics_dt_s"}


def load_packet(path, plan):
    with np.load(path) as packet:
        require(set(packet.files) == PACKET_FIELDS, "Training schema must contain only direct motion/drive records")
        values = {k: packet[k].copy() for k in packet.files}
    h = float(values["physics_dt_s"])
    require(h == plan["physics_dt_s"], "Timestep differs")
    n = int(round(plan["duration_s"]/h))
    require(values["q"].shape == values["v"].shape == values["tau_actuator"].shape == (n+1, 6), "Packet shape differs")
    require(np.allclose(values["t"], np.arange(n+1)*h, rtol=0, atol=1e-9), "Packet clock differs")
    require(all(np.isfinite(values[k]).all() for k in ("q", "v", "tau_actuator")), "Nonfinite packet")
    return {"q": torch.from_numpy(values["q"][:-1]), "v": torch.from_numpy(values["v"][:-1]),
            "dv": torch.from_numpy(np.diff(values["v"], axis=0)),
            "tau": torch.from_numpy(values["tau_actuator"][:-1])}


def evaluate_impulses(model, data, h, known_B, chunk_size):
    pieces = []
    with torch.no_grad():
        for lo in range(0, len(data["q"]), chunk_size):
            sl = slice(lo, lo+chunk_size)
            pieces.append(model_impulses(model, *(data[k][sl] for k in ("q", "v", "dv", "tau")), h, known_B, data["gravity"][sl]).numpy())
    return np.concatenate(pieces)


def loss_and_gradient(model, data, operator, plan, scale, backward=True):
    """Exact two-pass chain rule; model parameters are fixed between the passes."""
    h = plan["physics_dt_s"]
    known_B = torch.tensor(plan["known_integrator_B"], dtype=torch.float64)
    chunk = plan["training"]["chunk_size"]
    impulses = evaluate_impulses(model, data, h, known_B, chunk)
    residual = operator.forward(impulses)
    require(np.isfinite(residual).all(), "Nonfinite model residual")
    loss = float(np.mean((residual/scale)**2))
    if backward:
        grad_r = 2*residual/(scale**2*residual.size)
        grad_e = torch.from_numpy(operator.adjoint(grad_r))
        for lo in range(0, len(data["q"]), chunk):
            sl = slice(lo, lo+chunk)
            current = model_impulses(model, *(data[k][sl] for k in ("q", "v", "dv", "tau")), h, known_B, data["gravity"][sl])
            (current*grad_e[sl]).sum().backward()
    return loss, residual
