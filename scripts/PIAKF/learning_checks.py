"""Independent chain-rule, parameter-gradient and checkpoint checks."""
import numpy as np
import torch

from checks import environment
from structured import StructuredDynamics, load_network, save_model


def run_learning_checks(config, output):
    torch.set_num_threads(1)
    torch.manual_seed(config["seed"])
    statistics = {"q_center": [0.] * 6, "q_scale": [.1, .2, .15, .1, .2, .1],
                  "gravity_center": [0.] * 6, "gravity_scale": [1.] * 6, "torque_scale": [1.] * 6}
    model = StructuredDynamics(config["network"], statistics)
    rng = np.random.default_rng(config["seed"])
    comparisons = {}

    def compare(name, actual, expected, gradient=False):
        actual, expected = np.asarray(actual), np.asarray(expected)
        rtol = config["verification"]["gradient_rtol" if gradient else "derivative_rtol"]
        atol = config["verification"]["gradient_atol" if gradient else "derivative_atol"]
        np.testing.assert_allclose(actual, expected, rtol=rtol, atol=atol, err_msg=name)
        comparisons[name] = {"max_abs_error": float(np.max(np.abs(actual - expected))),
                             "rtol": rtol, "atol": atol}

    min_eigenvalue = float("inf")
    for index in range(3):
        q = torch.tensor(rng.normal(0, .03, 6), dtype=torch.float64)
        v = torch.tensor(rng.normal(0, .2, 6), dtype=torch.float64)
        result = model.dynamics(q[None], v[None])
        reference = torch.autograd.functional.jacobian(lambda x: model.mass_gravity(x[None])[0][0], q)
        compare(f"physical_q_jacobian_{index}", result["dM"][0].detach(), reference.detach())
        h = config["verification"]["difference_step"]
        finite = []
        with torch.no_grad():
            for direction in torch.eye(6, dtype=torch.float64):
                finite.append((model.mass_gravity((q + h * direction)[None])[0][0]
                               - model.mass_gravity((q - h * direction)[None])[0][0]) / (2 * h))
        compare(f"finite_difference_{index}", result["dM"][0].detach(), torch.stack(finite, dim=-1))
        dM, md = result["dM"][0], result["Mdot"][0]
        C = torch.empty((6, 6), dtype=torch.float64)
        for i in range(6):
            for j in range(6):
                C[i, j] = sum(.5 * (dM[i, j, k] + dM[i, k, j] - dM[j, k, i]) * v[k]
                              for k in range(6))
        compare(f"Christoffel_{index}", md.detach(), (C + C.T).detach())
        compare(f"velocity_term_{index}", result["c"][0].detach(), (C @ v).detach())
        compare(f"kinetic_gradient_{index}", result["gradient_T"][0].detach(), (C.T @ v).detach())
        M = result["M"][0].detach().numpy()
        compare(f"symmetry_{index}", M, M.T)
        min_eigenvalue = min(min_eigenvalue, float(np.linalg.eigvalsh(M).min()))
    if min_eigenvalue <= 0:
        raise ValueError("Nonpositive inertia in network checks")

    q, v = q[None], v[None]
    acceleration = torch.tensor([[.3, -.2, .4, -.3, .2, -.1]], dtype=torch.float64)
    def loss():
        return (100 * model.dynamics(q, v, acceleration)["tau"].square()).sum()
    model.zero_grad()
    loss().backward()
    if not all(p.grad is not None and torch.isfinite(p.grad).all() for p in model.parameters()):
        raise FloatingPointError("Missing or nonfinite parameter gradients")
    for label, parameter in (("first_layer", model.layers[0].weight), ("output_layer", model.layers[-1].weight)):
        flat = int(parameter.grad.abs().argmax())
        coordinates = np.unravel_index(flat, parameter.shape)
        analytic = float(parameter.grad[coordinates])
        if analytic == 0:
            raise ValueError("Parameter gradient probe is uninformative")
        h = config["verification"]["difference_step"]
        original = float(parameter[coordinates].detach())
        with torch.no_grad():
            parameter[coordinates] = original + h
            plus = float(loss())
            parameter[coordinates] = original - h
            minus = float(loss())
            parameter[coordinates] = original
        compare(f"parameter_gradient_{label}", analytic, (plus - minus) / (2 * h), True)
    path = output / "roundtrip.pt"
    save_model(path, model, statistics, 0)
    restored = load_network(path)
    compare("checkpoint_roundtrip", model.dynamics(q, v, acceleration)["tau"].detach(),
            restored.dynamics(q, v, acceleration)["tau"].detach())
    return {"passed": True, "environment": environment(), "comparisons": comparisons,
            "comparison_count": len(comparisons), "inertia_min_eigenvalue": min_eigenvalue,
            "training_updates": 0}
