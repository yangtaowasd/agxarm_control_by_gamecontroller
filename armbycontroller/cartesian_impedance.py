"""Task-space impedance helpers for torque-controlled AGX arms."""

from dataclasses import dataclass

import modern_robotics as mr
import numpy as np

from armbycontroller.lie import skew


@dataclass(frozen=True)
class CartesianImpedanceResult:
    """One Cartesian impedance evaluation in the robot base frame."""

    joint_torque: np.ndarray
    wrench: np.ndarray
    pose_error: np.ndarray
    twist: np.ndarray
    jacobian: np.ndarray


def geometric_jacobian(model, joint_positions):
    """Return ``[angular; linear]`` base-frame end-effector Jacobian."""
    joints = np.asarray(joint_positions, dtype=float)
    pose = np.asarray(model.forward_kinematics(joints), dtype=float)
    space = np.asarray(model.space_jacobian(joints), dtype=float)
    if (
        pose.shape != (4, 4)
        or space.shape != (6, joints.size)
        or not np.all(np.isfinite(pose))
        or not np.all(np.isfinite(space))
    ):
        raise ValueError("kinematic model returned an invalid pose or Jacobian")
    return np.vstack((
        space[:3],
        space[3:] - skew(pose[:3, 3]) @ space[:3],
    )), pose


def _task_vector(values, name, nonnegative=True):
    result = np.asarray(values, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain six finite values")
    if nonnegative and np.any(result < 0.0):
        raise ValueError(f"{name} values must be nonnegative")
    return result


def _rotation_error(current, target):
    error = mr.so3ToVec(mr.MatrixLog3(target @ current.T))
    error = np.asarray(error, dtype=float)
    if error.shape != (3,) or not np.all(np.isfinite(error)):
        raise ValueError("orientation error is invalid")
    return error


def _redundant_nullspace_torque(
    jacobian,
    joint_positions,
    joint_velocities,
    reference_positions,
    stiffness,
    damping,
):
    """Project a posture spring into the exact kinematic null space."""
    joint_count = joint_positions.size
    if joint_count <= 6 or (stiffness == 0.0 and damping == 0.0):
        return np.zeros(joint_count, dtype=float)
    _, singular_values, right_vectors = np.linalg.svd(
        jacobian, full_matrices=True
    )
    tolerance = (
        max(jacobian.shape)
        * np.finfo(float).eps
        * (float(singular_values[0]) if singular_values.size else 0.0)
    )
    rank = int(np.sum(singular_values > tolerance))
    basis = right_vectors[rank:].T
    if basis.size == 0:
        return np.zeros(joint_count, dtype=float)
    posture_torque = (
        float(stiffness) * (reference_positions - joint_positions)
        - float(damping) * joint_velocities
    )
    return basis @ (basis.T @ posture_torque)


def cartesian_impedance_torque(
    model,
    joint_positions,
    joint_velocities,
    target_position,
    target_rotation,
    stiffness,
    damping,
    *,
    nullspace_reference=None,
    nullspace_stiffness=0.0,
    nullspace_damping=0.0,
):
    """
    Map a base-frame Cartesian spring-damper wrench through ``J.T``.

    Task vectors use ``[rx, ry, rz, x, y, z]`` order. Rotational stiffness
    therefore has N.m/rad units and translational stiffness has N/m units.
    The target is stationary, so damping opposes the measured tool twist.
    """
    joints = np.asarray(joint_positions, dtype=float)
    velocities = np.asarray(joint_velocities, dtype=float)
    target_position = np.asarray(target_position, dtype=float)
    target_rotation = np.asarray(target_rotation, dtype=float)
    stiffness = _task_vector(stiffness, "stiffness")
    damping = _task_vector(damping, "damping")
    if (
        joints.ndim != 1
        or joints.size < 1
        or velocities.shape != joints.shape
        or target_position.shape != (3,)
        or target_rotation.shape != (3, 3)
        or not np.all(np.isfinite(joints))
        or not np.all(np.isfinite(velocities))
        or not np.all(np.isfinite(target_position))
        or not np.all(np.isfinite(target_rotation))
        or not np.allclose(target_rotation.T @ target_rotation, np.eye(3), atol=1e-6)
        or np.linalg.det(target_rotation) < 0.999999
    ):
        raise ValueError("Cartesian impedance state or target is invalid")
    if (
        not np.isfinite(nullspace_stiffness)
        or not np.isfinite(nullspace_damping)
        or nullspace_stiffness < 0.0
        or nullspace_damping < 0.0
    ):
        raise ValueError("nullspace gains must be finite and nonnegative")

    jacobian, current_pose = geometric_jacobian(model, joints)
    pose_error = np.concatenate((
        _rotation_error(current_pose[:3, :3], target_rotation),
        target_position - current_pose[:3, 3],
    ))
    twist = jacobian @ velocities
    wrench = stiffness * pose_error - damping * twist
    joint_torque = jacobian.T @ wrench

    if nullspace_reference is not None:
        reference = np.asarray(nullspace_reference, dtype=float)
        if reference.shape != joints.shape or not np.all(np.isfinite(reference)):
            raise ValueError("nullspace reference must match the joint state")
        joint_torque += _redundant_nullspace_torque(
            jacobian,
            joints,
            velocities,
            reference,
            float(nullspace_stiffness),
            float(nullspace_damping),
        )
    return CartesianImpedanceResult(
        joint_torque=joint_torque,
        wrench=wrench,
        pose_error=pose_error,
        twist=twist,
        jacobian=jacobian,
    )
