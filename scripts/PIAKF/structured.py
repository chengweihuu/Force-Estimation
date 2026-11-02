"""Structured learned dynamics; the scientific core has no simulator dependency.

Derivative propagation follows the local paper-reading implementation, with
physical-coordinate normalization, inertia scaling, a gravity head and learned
viscous friction. This is a disclosed reconstruction, not author source code.
"""
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F


class StructuredDynamics(nn.Module):
    def __init__(self, config, statistics):
        super().__init__()
        self.config = config
        sizes = [6, *config["widths"], 27]
        self.layers = nn.ModuleList(nn.Linear(a, b, dtype=torch.float64)
                                    for a, b in zip(sizes[:-1], sizes[1:]))
        row, col = torch.tril_indices(6, 6)
        self.register_buffer("row", row)
        self.register_buffer("col", col)
        self.register_buffer("diagonal", row == col)
        for name in ("q_center", "q_scale", "gravity_center", "gravity_scale", "torque_scale"):
            value = torch.tensor(statistics[name], dtype=torch.float64)
            if value.shape != (6,) or not torch.isfinite(value).all():
                raise ValueError(f"Invalid statistics: {name}")
            if "scale" in name and not (value > 0).all():
                raise ValueError(f"Nonpositive statistics: {name}")
            self.register_buffer(name, value)
        self.register_buffer("inertia_scale", torch.tensor(config["inertia_scale"], dtype=torch.float64))
        self.epsilon = float(config["epsilon"])
        if not (self.inertia_scale.shape == (6,) and (self.inertia_scale > 0).all()
                and np.isfinite(self.epsilon) and 0 < self.epsilon < 1):
            raise ValueError("Invalid inertia parameterization")
        self.friction_raw = nn.Parameter(torch.full(
            (6,), float(np.log(np.expm1(config["friction_initial"]))), dtype=torch.float64))
        # Near-identity dimensionless inertia and training-label-mean gravity.
        with torch.no_grad():
            self.layers[-1].weight.normal_(0, config["output_weight_std"])
            self.layers[-1].bias.zero_()
            diagonal_bias = float(np.log(np.expm1(np.sqrt(1 - self.epsilon))))
            self.layers[-1].bias[:21][self.diagonal] = diagonal_bias

    def raw(self, q, derivative=False):
        z = (q - self.q_center) / self.q_scale
        dz = torch.diag(1 / self.q_scale).expand(len(q), 6, 6) if derivative else None
        for index, layer in enumerate(self.layers):
            z = layer(z)
            if derivative:
                dz = torch.einsum("oi,bik->bok", layer.weight, dz)
            if index < len(self.layers) - 1:
                if derivative:
                    dz = (z > 0).to(z.dtype).unsqueeze(-1) * dz
                z = F.relu(z)
        return (z, dz) if derivative else z

    def mass_gravity(self, q):
        raw = self.raw(q)
        entries = torch.where(self.diagonal, F.softplus(raw[:, :21]), raw[:, :21])
        L = raw.new_zeros((len(q), 6, 6))
        L[:, self.row, self.col] = entries
        M = L @ L.transpose(-1, -2) + self.epsilon * torch.eye(6, dtype=q.dtype, device=q.device)
        M = M * self.inertia_scale[None, :, None] * self.inertia_scale[None, None, :]
        return M, self.gravity_center + self.gravity_scale * raw[:, 21:]

    def dynamics(self, q, velocity, acceleration=None):
        raw, draw = self.raw(q, True)
        entries = torch.where(self.diagonal, F.softplus(raw[:, :21]), raw[:, :21])
        factors = torch.where(self.diagonal, torch.sigmoid(raw[:, :21]),
                              torch.ones_like(raw[:, :21]))
        L = raw.new_zeros((len(q), 6, 6))
        dL = raw.new_zeros((len(q), 6, 6, 6))
        L[:, self.row, self.col] = entries
        dL[:, self.row, self.col, :] = draw[:, :21, :] * factors.unsqueeze(-1)
        M = L @ L.transpose(-1, -2) + self.epsilon * torch.eye(6, dtype=q.dtype, device=q.device)
        dM = (torch.einsum("birk,bjr->bijk", dL, L)
              + torch.einsum("bir,bjrk->bijk", L, dL))
        scale = self.inertia_scale[None, :, None] * self.inertia_scale[None, None, :]
        M, dM = M * scale, dM * scale.unsqueeze(-1)
        Mdot = torch.einsum("bijk,bk->bij", dM, velocity)
        gradient_T = 0.5 * torch.einsum("bi,bijk,bj->bk", velocity, dM, velocity)
        c = torch.einsum("bij,bj->bi", Mdot, velocity) - gradient_T
        G = self.gravity_center + self.gravity_scale * raw[:, 21:]
        friction = F.softplus(self.friction_raw) * velocity
        result = {"M": M, "dM": dM, "Mdot": Mdot, "gradient_T": gradient_T,
                  "c": c, "G": G, "friction": friction}
        if acceleration is not None:
            result["tau"] = torch.einsum("bij,bj->bi", M, acceleration) + c + G + friction
        return result


def save_model(path, model, statistics, updates):
    torch.save({"network_config": model.config, "statistics": statistics,
                "updates": int(updates), "state_dict": model.state_dict()}, path)


def load_network(path):
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    model = StructuredDynamics(checkpoint["network_config"], checkpoint["statistics"])
    model.load_state_dict(checkpoint["state_dict"])
    return model.eval()


class LearnedDynamics:
    """Only kinematics comes from MuJoCo; all dynamic terms come from weights."""
    def __init__(self, network, model_path):
        import mujoco
        self.mj = mujoco
        self.geometry = mujoco.MjModel.from_xml_path(str(model_path))
        self.geometry_data = mujoco.MjData(self.geometry)
        self.site = self.geometry.site("tool0").id
        self.network = network.eval()

    def evaluate(self, q, velocity):
        from dynamics import Terms, vector
        q, velocity = vector(q, "q"), vector(velocity, "v")
        with torch.no_grad():
            result = self.network.dynamics(torch.from_numpy(q[None]), torch.from_numpy(velocity[None]))
        if not all(torch.isfinite(x).all() for x in result.values()):
            raise FloatingPointError("Nonfinite learned dynamics")
        values = {key: value[0].numpy() for key, value in result.items()}
        if np.linalg.eigvalsh(values["M"]).min() <= 0:
            raise ValueError("Nonpositive learned inertia")
        data = self.geometry_data
        data.qpos[:] = q
        self.mj.mj_kinematics(self.geometry, data)
        self.mj.mj_comPos(self.geometry, data)
        linear, angular = np.empty((3, 6)), np.empty((3, 6))
        self.mj.mj_jacSite(self.geometry, data, linear, angular, self.site)
        return Terms(values["M"], values["Mdot"], values["c"] + values["G"],
                     values["friction"], np.vstack((linear, angular)))
