"""Build-time installation check only; no project models or research data."""
from importlib.metadata import version
import sys

import matplotlib
import mujoco
import numpy as np
import pinocchio as pin
import scipy

assert sys.version_info[:2] == (3, 12), sys.version
assert pin.__version__ == "3.8.0", pin.__version__
assert mujoco.__version__ == "3.12.0", mujoco.__version__
assert np.__version__ == "2.3.5", np.__version__
assert version("cmeel-urdfdom") == "4.0.1"
assert version("cmeel-tinyxml2") == "10.0.0"

# Importing pinocchio already loads its native library dependencies. Also exercise
# the actual regression API instead of treating successful pip resolution as proof.
model = pin.buildSampleModelManipulator()
model.gravity.linear[:] = 0.0
data = model.createData()
q = pin.neutral(model)
v = np.linspace(0.1, 0.6, model.nv)
a = np.linspace(-0.3, 0.2, model.nv)
theta = np.concatenate(
    [model.inertias[j].toDynamicParameters() for j in range(1, model.njoints)]
)
Y = pin.computeJointTorqueRegressor(model, data, q, v, a).copy()
tau = pin.rnea(model, data, q, v, a).copy()
np.testing.assert_allclose(Y @ theta, tau, rtol=1e-10, atol=1e-10)
assert np.isfinite(tau).all()
M = pin.crba(model, data, q).copy()
Yp = pin.computeJointTorqueRegressor(model, data, q, np.zeros(model.nv), v)
np.testing.assert_allclose(Yp @ theta, M @ v, rtol=1e-10, atol=1e-10)
for package in ("pin", "mujoco", "numpy", "scipy", "matplotlib",
                "cmeel-urdfdom", "cmeel-tinyxml2"):
    print(f"{package}=={version(package)}")
print("Rigid environment check passed: native imports and regressor identities.")

# Installation-only positive definite cone and quadratic objective check.
import cvxpy as cp
assert version("cvxpy") == "1.6.5"
assert version("clarabel") == "0.11.1"
x = cp.Variable((2, 2), symmetric=True)
problem = cp.Problem(cp.Minimize(cp.sum_squares(x - np.eye(2))), [x >> 0.1*np.eye(2)])
problem.solve(solver="CLARABEL", max_iter=200, tol_gap_abs=1e-9,
              tol_gap_rel=1e-9, tol_feas=1e-9, warm_start=False)
assert problem.status == "optimal", problem.status
assert np.linalg.eigvalsh(x.value).min() >= 0.1 - 1e-8
assert np.linalg.norm(x.value - np.eye(2)) < 1e-6
print("SDP environment check passed: CVXPY 1.6.5 / Clarabel 0.11.1")
