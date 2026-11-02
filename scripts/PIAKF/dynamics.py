"""Known-model prerequisite for PIAKF, not the learned dynamics implementation.

All six joints are revolute. Wrenches act ON the robot, at tool0, expressed
in world axes, ordered [Fx,Fy,Fz,Mx,My,Mz]. Nominal and plant objects are separate.
"""
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np


def vector(value, name):
    value = np.asarray(value, dtype=np.float64)
    if value.shape != (6,) or not np.isfinite(value).all():
        raise ValueError(f"{name} must contain six finite values")
    return value


def load_model(path):
    model = mujoco.MjModel.from_xml_path(str(Path(path).resolve(strict=True)))
    if (model.nq, model.nv, model.nu, model.njnt) != (6, 6, 6, 6):
        raise ValueError("Expected a fixed-base six-joint model with six motors")
    if not np.all(model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE):
        raise ValueError("Only six revolute joints are supported")
    for actual in (model.jnt_qposadr, model.jnt_dofadr, model.actuator_trnid[:, 0]):
        np.testing.assert_array_equal(actual, np.arange(6))
    if not np.all(model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT):
        raise ValueError("Expected joint motors")
    if not (np.all(model.actuator_dyntype == 0)
            and np.all(model.actuator_gaintype == 0)
            and np.all(model.actuator_biastype == 0)
            and np.all(model.actuator_gainprm[:, 0] == 1)):
        raise ValueError("Expected unit-gain, stateless torque motors")
    np.testing.assert_array_equal(model.actuator_gear,
                                  np.tile([1., 0., 0., 0., 0., 0.], (6, 1)))
    if np.any(model.dof_armature) or np.any(model.dof_frictionloss):
        raise ValueError("This first probe expects zero armature and dry friction")
    if model.ntendon or model.nflex or model.neq:
        raise ValueError("Tendons, flex bodies and equality constraints are outside this probe")
    if model.opt.integrator != mujoco.mjtIntegrator.mjINT_IMPLICITFAST:
        raise ValueError("Preserve the declared implicitfast integrator")
    if not (np.isfinite(model.dof_damping).all() and np.all(model.dof_damping >= 0)):
        raise ValueError("Invalid viscous damping")
    model.site("tool0")  # Require the declared reference point.
    return model


def mass_matrix(model, data):
    """Use mass-vector products to avoid version-specific mj_fullM bindings."""
    matrix = np.empty((6, 6))
    column = np.empty(6)
    for index, basis in enumerate(np.eye(6)):
        mujoco.mj_mulM(model, data, column, basis)
        matrix[:, index] = column
    return matrix


def jacobian(model, data):
    linear, angular = np.empty((3, 6)), np.empty((3, 6))
    mujoco.mj_jacSite(model, data, linear, angular, model.site("tool0").id)
    return np.vstack((linear, angular))


def apply_wrench(model, data, wrench):
    """Replace, rather than accumulate, the only external action for this step."""
    wrench = vector(wrench, "wrench")
    site = model.site("tool0").id
    data.qfrc_applied[:] = 0
    data.xfrc_applied[:] = 0
    mujoco.mj_applyFT(model, data, wrench[:3], wrench[3:], data.site_xpos[site],
                      int(model.site_bodyid[site]), data.qfrc_applied)
    return data.qfrc_applied.copy()


@dataclass(frozen=True)
class Terms:
    mass: np.ndarray
    mass_dot: np.ndarray
    bias: np.ndarray  # c + G, excludes passive viscous damping.
    friction: np.ndarray  # Positive resisting term B*v on the equation's left.
    jacobian: np.ndarray

    def momentum_input(self, velocity, drive_torque):
        return (self.mass_dot @ velocity + drive_torque
                - self.bias - self.friction)


class KnownDynamics:
    """An independent model driven ONLY by measured q and v.

    Matching plant parameters here is intentional and disclosed. This class is
    a diagnostic baseline, never a hidden substitute for learned inertia.
    """
    def __init__(self, model_path, derivative_epsilon_s):
        self.model = load_model(model_path)
        self.data = mujoco.MjData(self.model)
        self.plus = mujoco.MjData(self.model)
        self.minus = mujoco.MjData(self.model)
        self.epsilon = float(derivative_epsilon_s)
        if not np.isfinite(self.epsilon) or self.epsilon <= 0:
            raise ValueError("Derivative epsilon must be positive")

    def mass_at(self, q, data=None):
        data = self.data if data is None else data
        data.qpos[:] = vector(q, "q")
        data.qvel[:] = 0
        mujoco.mj_fwdPosition(self.model, data)
        return mass_matrix(self.model, data)

    def evaluate(self, q, velocity):
        q, velocity = vector(q, "q"), vector(velocity, "velocity")
        mass = self.mass_at(q)
        self.data.qvel[:] = velocity
        mujoco.mj_fwdVelocity(self.model, self.data)
        bias = self.data.qfrc_bias.copy()
        J = jacobian(self.model, self.data)
        mass_dot = (self.mass_at(q + self.epsilon * velocity, self.plus)
                    - self.mass_at(q - self.epsilon * velocity, self.minus)) / (2 * self.epsilon)
        if not all(np.isfinite(x).all() for x in (mass, mass_dot, bias, J)):
            raise FloatingPointError("Nonfinite nominal dynamics")
        if np.linalg.eigvalsh(mass).min() <= 0:
            raise ValueError("Nonpositive nominal inertia")
        return Terms(mass, mass_dot, bias, self.model.dof_damping * velocity, J)
