"""Paper-structured dynamics, physical-unit q, no Plant or task identity."""
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F


class StructuredDynamics(nn.Module):
    def __init__(self, widths=(128, 128, 128), diagonal_floor=1e-4, mass_floor=0.,
                 input_center_rad=None, input_scale_rad=None):
        super().__init__()
        if not np.isfinite(mass_floor) or mass_floor < 0:
            raise ValueError("mass_floor must be finite and nonnegative (kg m^2)")
        sizes = [6, *widths, 27]
        self.layers = nn.ModuleList(nn.Linear(a, b, dtype=torch.float64)
                                    for a, b in zip(sizes[:-1], sizes[1:]))
        row, col = torch.tril_indices(6, 6)
        self.register_buffer("row", row)
        self.register_buffer("col", col)
        self.register_buffer("diagonal", row == col)
        self.diagonal_floor = diagonal_floor
        self.mass_floor = mass_floor
        center = torch.as_tensor([0.]*6 if input_center_rad is None else input_center_rad,
                                 dtype=torch.float64)
        scale = torch.as_tensor([1.]*6 if input_scale_rad is None else input_scale_rad,
                                dtype=torch.float64)
        if (center.shape != (6,) or scale.shape != (6,)
                or not torch.isfinite(center).all() or not torch.isfinite(scale).all()
                or not (scale > 0).all()):
            raise ValueError("Input center/scale must be six finite values; scales must be positive")
        # Configuration owns these constants; old weight files remain loadable.
        self.register_buffer("input_center", center, persistent=False)
        self.register_buffer("input_scale", scale, persistent=False)

    def raw(self, q, with_jacobian=False):
        z = (q-self.input_center)/self.input_scale
        # Propagate derivatives with respect to physical q, not normalized z.
        derivative = torch.diag(1/self.input_scale).expand(len(q), 6, 6)
        for i, layer in enumerate(self.layers):
            z = layer(z)
            if with_jacobian:
                derivative = torch.einsum("oi,bik->bok", layer.weight, derivative)
            if i < len(self.layers)-1:
                if with_jacobian:
                    derivative = (z > 0).to(z.dtype).unsqueeze(-1)*derivative
                z = F.relu(z)  # derivative at zero = 0, matching PyTorch.
        return (z, derivative) if with_jacobian else z

    def mass_gravity(self, q):
        raw = self.raw(q)
        entries = torch.where(self.diagonal, F.softplus(raw[:, :21])+self.diagonal_floor,
                              raw[:, :21])
        L = raw.new_zeros((len(q), 6, 6))
        L[:, self.row, self.col] = entries
        M = L @ L.transpose(-1, -2)
        M = M + self.mass_floor*torch.eye(6, dtype=q.dtype, device=q.device)
        return M, raw[:, 21:]

    def dynamics(self, q, v, a=None):
        raw, draw = self.raw(q, True)
        entries = torch.where(self.diagonal, F.softplus(raw[:, :21])+self.diagonal_floor,
                              raw[:, :21])
        scales = torch.where(self.diagonal, torch.sigmoid(raw[:, :21]),
                             torch.ones_like(raw[:, :21]))
        L = raw.new_zeros((len(q), 6, 6))
        dL = raw.new_zeros((len(q), 6, 6, 6))
        L[:, self.row, self.col] = entries
        dL[:, self.row, self.col, :] = draw[:, :21, :]*scales.unsqueeze(-1)
        M = L @ L.transpose(-1, -2)
        M = M + self.mass_floor*torch.eye(6, dtype=q.dtype, device=q.device)
        # The configured constant inertia has zero position derivative.
        dM = (torch.einsum("birk,bjr->bijk", dL, L)
              + torch.einsum("bir,bjrk->bijk", L, dL))
        Mdot = torch.einsum("bijk,bk->bij", dM, v)
        gradient_T = .5*torch.einsum("bi,bijk,bj->bk", v, dM, v)
        h = torch.einsum("bij,bj->bi", Mdot, v)-gradient_T+raw[:, 21:]
        result = dict(M=M, h=h, Mdot=Mdot, dM=dM, G=raw[:, 21:],
                      gradient_T=gradient_T)
        if a is not None:
            result["tau"] = torch.einsum("bij,bj->bi", M, a)+h
        return result


class Geometry:
    """Only kinematics and site Jacobian; never queries simulated dynamics."""
    def __init__(self, model_path):
        import mujoco
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.site = self.model.site("tool0").id

    def jacobian(self, q):
        mj = self.mujoco
        self.data.qpos[:] = q
        mj.mj_kinematics(self.model, self.data)
        mj.mj_comPos(self.model, self.data)
        jp, jr = np.zeros((3, 6)), np.zeros((3, 6))
        mj.mj_jacSite(self.model, self.data, jp, jr, self.site)
        return np.vstack((jp, jr))


class LearnedNominal:
    def __init__(self, network, model_path):
        self.network = network.eval()
        self.geometry = Geometry(model_path)

    def evaluate(self, q, v):
        # Input derivatives are explicitly propagated by dynamics(); no autograd
        # tape is needed for inference. Training still differentiates its graph.
        with torch.no_grad():
            r = self.network.dynamics(torch.as_tensor(q).reshape(1, 6),
                                      torch.as_tensor(v).reshape(1, 6))
        if not all(torch.isfinite(x).all() for x in r.values()):
            raise RuntimeError("Non-finite learned dynamics")
        if torch.linalg.eigvalsh(r["M"][0]).min() <= 0:
            raise RuntimeError("Non-positive learned inertia")
        return (r["M"][0].numpy(), r["h"][0].numpy(), r["Mdot"][0].numpy(),
                self.geometry.jacobian(q))


def load_network(path, network_config):
    net = StructuredDynamics(**network_config)
    net.load_state_dict(torch.load(path, map_location="cpu", weights_only=True))
    return net.eval()
