"""Shared base-frame task geometry in ``[angular; linear]`` order."""

import math

import numpy as np

from armbycontroller.modeling.lie import skew


def spatial_vector(values, name, *, positive=False, nonnegative=False):
    """Validate one six-axis ``[Mx, My, Mz, Fx, Fy, Fz]`` vector."""
    result = np.asarray(values, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain six finite values")
    if positive and np.any(result <= 0.0):
        raise ValueError(f"{name} must contain six positive values")
    if nonnegative and np.any(result < 0.0):
        raise ValueError(f"{name} must contain six nonnegative values")
    return result


def transform_matrix(values, name):
    """Validate and return one finite SE(3) transform."""
    result = np.asarray(values, dtype=float)
    if (
        result.shape != (4, 4)
        or not np.all(np.isfinite(result))
        or not np.allclose(result[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9)
        or not np.allclose(
            result[:3, :3].T @ result[:3, :3], np.eye(3), atol=1e-7
        )
        or not np.isclose(np.linalg.det(result[:3, :3]), 1.0, atol=1e-7)
    ):
        raise ValueError(f"{name} must be a finite SE(3) transform")
    return result


def geometric_jacobian(model, joint_positions):
    """Return base-frame tool-origin ``Jg`` in ``[angular; linear]`` order."""
    joints = np.asarray(joint_positions, dtype=float)
    if joints.ndim != 1 or joints.size < 1 or not np.all(np.isfinite(joints)):
        raise ValueError("joint_positions must be a finite vector")
    pose = transform_matrix(model.forward_kinematics(joints), "current_pose")
    space = np.asarray(model.space_jacobian(joints), dtype=float)
    if space.shape != (6, joints.size) or not np.all(np.isfinite(space)):
        raise ValueError("model must return a finite 6xn space Jacobian")

    # Modern Robotics space twists use [omega; v]. The tool-origin linear
    # velocity is p_dot=v+omega×p, hence v-[p]x*omega below.
    jacobian = np.vstack((
        space[:3],
        space[3:] - skew(pose[:3, 3]) @ space[:3],
    ))
    return jacobian, pose


def joint_torque_from_wrench(jacobian, wrench):
    """Map a tool-origin wrench through virtual work: ``tau=Jg.T*wrench``."""
    matrix = np.asarray(jacobian, dtype=float)
    task_wrench = spatial_vector(wrench, "wrench")
    if (
        matrix.ndim != 2
        or matrix.shape[0] != 6
        or matrix.shape[1] < 1
        or not np.all(np.isfinite(matrix))
    ):
        raise ValueError("jacobian must be a finite 6xn matrix")
    return matrix.T @ task_wrench


def wrench_from_joint_torque(jacobian, joint_torque, damping=0.05):
    """Solve ``tau=Jg.T*wrench`` with damped least squares."""
    matrix = np.asarray(jacobian, dtype=float)
    torque = np.asarray(joint_torque, dtype=float)
    damping = float(damping)
    if (
        matrix.ndim != 2
        or matrix.shape[0] != 6
        or matrix.shape[1] < 1
        or not np.all(np.isfinite(matrix))
        or torque.shape != (matrix.shape[1],)
        or not np.all(np.isfinite(torque))
        or not math.isfinite(damping)
        or damping <= 0.0
    ):
        raise ValueError(
            "wrench estimation inputs must be finite and compatible"
        )
    regularized = matrix @ matrix.T + damping ** 2 * np.eye(6)
    try:
        return np.linalg.solve(regularized, matrix @ torque)
    except np.linalg.LinAlgError as error:
        raise ValueError("wrench estimation solve failed") from error
