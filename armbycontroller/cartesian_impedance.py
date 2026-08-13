"""Frame-consistent Cartesian impedance mapped by virtual work."""

from dataclasses import dataclass

import numpy as np

from armbycontroller.lie import rotation_vector
from armbycontroller.lie import skew


@dataclass(frozen=True)
class CartesianImpedanceCommand:
    """One evaluation of the complete Cartesian impedance equation."""

    pose_error: np.ndarray
    desired_twist: np.ndarray
    measured_twist: np.ndarray
    commanded_wrench: np.ndarray
    geometric_jacobian: np.ndarray
    task_torque: np.ndarray
    nullspace_torque: np.ndarray
    model_torque: np.ndarray
    command_torque: np.ndarray


def cartesian_impedance_diagonals(
    rotation_stiffness,
    base_z_rotation_stiffness,
    translation_stiffness,
    rotation_damping,
    translation_damping,
):
    """Build base-frame ``[Rx, Ry, Rz, X, Y, Z]`` diagonal gains."""
    stiffness = np.asarray(
        [rotation_stiffness, rotation_stiffness,
         base_z_rotation_stiffness,
         translation_stiffness, translation_stiffness,
         translation_stiffness],
        dtype=float,
    )
    damping = np.asarray(
        [rotation_damping] * 3 + [translation_damping] * 3,
        dtype=float,
    )
    return stiffness, damping


def _finite_vector(values, size, name):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def _nonnegative_joint_gain(values, size, name):
    result = np.asarray(values, dtype=float)
    if result.ndim == 0:
        result = np.full(size, float(result))
    if (
        result.shape != (size,)
        or not np.all(np.isfinite(result))
        or np.any(result < 0.0)
    ):
        raise ValueError(
            f"{name} must be one nonnegative value or {size} values"
        )
    return result


def _transform(values, name):
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


def _symmetric_psd_matrix(values, size, name):
    result = np.asarray(values, dtype=float)
    if result.shape == (size,):
        result = np.diag(result)
    if (
        result.shape != (size, size)
        or not np.all(np.isfinite(result))
        or not np.allclose(result, result.T, atol=1e-10)
    ):
        raise ValueError(
            f"{name} must be a symmetric positive semidefinite "
            f"{size}x{size} matrix"
        )
    symmetric = 0.5 * (result + result.T)
    if float(np.min(np.linalg.eigvalsh(symmetric))) < -1e-10:
        raise ValueError(
            f"{name} must be a symmetric positive semidefinite "
            f"{size}x{size} matrix"
        )
    return symmetric


def _impedance_matrix(values, name):
    return _symmetric_psd_matrix(values, 6, name)


def geometric_jacobian(model, joint_positions):
    """Return the base-frame tool-origin Jacobian in ``[angular; linear]``."""
    joints = np.asarray(joint_positions, dtype=float)
    if joints.ndim != 1 or joints.size < 1 or not np.all(np.isfinite(joints)):
        raise ValueError("joint_positions must be a finite vector")
    pose = _transform(model.forward_kinematics(joints), "current_pose")
    space = np.asarray(model.space_jacobian(joints), dtype=float)
    if space.shape != (6, joints.size) or not np.all(np.isfinite(space)):
        raise ValueError("model must return a finite 6xn space Jacobian")

    # A Modern Robotics space twist is [omega; v], where the velocity of the
    # tool origin is p_dot = v + omega x p.  Cartesian impedance acts at that
    # origin, so convert it before using wrench/velocity power duality.
    jacobian = np.vstack((
        space[:3],
        space[3:] - skew(pose[:3, 3]) @ space[:3],
    ))
    return jacobian, pose


def cartesian_pose_error(current_pose, desired_pose):
    """Return tool-origin error ``[SO(3) rotation vector; position]``."""
    current = _transform(current_pose, "current_pose")
    desired = _transform(desired_pose, "desired_pose")
    rotation_error = rotation_vector(
        desired[:3, :3] @ current[:3, :3].T
    )
    error = np.concatenate((
        np.asarray(rotation_error, dtype=float),
        desired[:3, 3] - current[:3, 3],
    ))
    if not np.all(np.isfinite(error)):
        raise ValueError("Cartesian pose error is not finite")
    return error


def equivalent_joint_impedance(
    geometric_jacobian_matrix, cartesian_stiffness, cartesian_damping
):
    """Return the full local matrices ``Kq=J.T Kx J`` and ``Dq=J.T Dx J``."""
    jacobian = np.asarray(geometric_jacobian_matrix, dtype=float)
    if (
        jacobian.ndim != 2
        or jacobian.shape[0] != 6
        or jacobian.shape[1] < 1
        or not np.all(np.isfinite(jacobian))
    ):
        raise ValueError("geometric_jacobian must be a finite 6xn matrix")
    stiffness = _impedance_matrix(cartesian_stiffness, "stiffness")
    damping = _impedance_matrix(cartesian_damping, "damping")
    return jacobian.T @ stiffness @ jacobian, jacobian.T @ damping @ jacobian


def equivalent_cartesian_impedance(
    geometric_jacobian_matrix, joint_stiffness, joint_damping
):
    """Exactly recover ``Kx=J^-T Kq J^-1`` for a regular six-axis pose."""
    jacobian = np.asarray(geometric_jacobian_matrix, dtype=float)
    if (
        jacobian.shape != (6, 6)
        or not np.all(np.isfinite(jacobian))
        or np.linalg.matrix_rank(jacobian) != 6
    ):
        raise ValueError(
            "exact joint-to-Cartesian conversion requires a nonsingular 6x6 "
            "geometric Jacobian"
        )
    stiffness = _symmetric_psd_matrix(
        joint_stiffness, 6, "joint_stiffness"
    )
    damping = _symmetric_psd_matrix(
        joint_damping, 6, "joint_damping"
    )
    inverse = np.linalg.solve(jacobian, np.eye(6))
    cartesian_stiffness = inverse.T @ stiffness @ inverse
    cartesian_damping = inverse.T @ damping @ inverse
    return (
        0.5 * (cartesian_stiffness + cartesian_stiffness.T),
        0.5 * (cartesian_damping + cartesian_damping.T),
    )


def cartesian_impedance_command(
    kinematic_model,
    dynamics_model,
    joint_positions,
    joint_velocities,
    desired_pose,
    desired_twist,
    cartesian_stiffness,
    cartesian_damping,
    *,
    nullspace_reference=None,
    nullspace_reference_velocity=None,
    nullspace_stiffness=0.0,
    nullspace_damping=0.0,
):
    """
    Evaluate strict Cartesian impedance and map it to joint torque.

    All task quantities use the robot base frame and ``[angular; linear]``
    order.  The implemented equations are::

        e_x     = [Log(R_d R.T)^vee; p_d - p]
        x_dot   = J_g(q) q_dot
        wrench  = K_x e_x + D_x (x_dot_d - x_dot)
        tau_x   = J_g(q).T wrench
        tau_0   = K_0 (q_0 - q) + D_0 (qdot_0 - qdot)
        N_tau   = I - J.T (J M^-1 J.T)^+ J M^-1
        tau_n   = N_tau tau_0
        tau_m   = C(q, q_dot) q_dot + g(q)
        tau_cmd = tau_x + tau_n + tau_m

    ``tau_m`` is obtained from URDF inverse dynamics with zero acceleration.
    The optional dynamically consistent nullspace term is intended for a
    redundant arm such as seven-axis Nero.  It uses the URDF mass matrix and
    satisfies ``J M^-1 tau_n = 0`` up to numerical tolerance.  No diagonal
    approximation, saturation, torque-rate limit, or hardware-specific MIT
    gain is hidden in this formula-level function.  The function is
    deliberately stateless: each returned torque is the immediate result of
    the displayed equations and never depends on a previous command.
    """
    positions = np.asarray(joint_positions, dtype=float)
    if positions.ndim != 1 or positions.size < 1:
        raise ValueError("joint_positions must be a finite vector")
    velocities = _finite_vector(
        joint_velocities, positions.size, "joint_velocities"
    )
    if not np.all(np.isfinite(positions)):
        raise ValueError("joint_positions must be a finite vector")
    target = _transform(desired_pose, "desired_pose")
    target_twist = _finite_vector(desired_twist, 6, "desired_twist")
    stiffness = _impedance_matrix(cartesian_stiffness, "stiffness")
    damping = _impedance_matrix(cartesian_damping, "damping")

    jacobian, current_pose = geometric_jacobian(
        kinematic_model, positions
    )
    pose_error = cartesian_pose_error(current_pose, target)
    measured_twist = jacobian @ velocities
    wrench = (
        stiffness @ pose_error
        + damping @ (target_twist - measured_twist)
    )
    task_torque = jacobian.T @ wrench

    nullspace_torque = np.zeros(positions.size, dtype=float)
    if nullspace_reference is not None:
        reference = _finite_vector(
            nullspace_reference, positions.size, "nullspace_reference"
        )
        reference_velocity = _finite_vector(
            np.zeros(positions.size)
            if nullspace_reference_velocity is None
            else nullspace_reference_velocity,
            positions.size,
            "nullspace_reference_velocity",
        )
        joint_stiffness = _nonnegative_joint_gain(
            nullspace_stiffness,
            positions.size,
            "nullspace_stiffness",
        )
        joint_damping = _nonnegative_joint_gain(
            nullspace_damping,
            positions.size,
            "nullspace_damping",
        )
        mass_model = (
            dynamics_model
            if dynamics_model is not None
            else kinematic_model
        )
        if not hasattr(mass_model, "mass_matrix"):
            raise ValueError(
                "nullspace impedance requires a dynamics mass_matrix"
            )
        mass = np.asarray(mass_model.mass_matrix(positions), dtype=float)
        if (
            mass.shape != (positions.size, positions.size)
            or not np.all(np.isfinite(mass))
            or not np.allclose(mass, mass.T, atol=1e-9)
            or float(np.min(np.linalg.eigvalsh(mass))) <= 0.0
        ):
            raise ValueError(
                "dynamics mass_matrix must be finite, symmetric, and "
                "positive definite"
            )
        try:
            mass_inverse_jacobian_transpose = np.linalg.solve(
                mass, jacobian.T
            )
        except np.linalg.LinAlgError as error:
            raise ValueError("dynamics mass_matrix must be nonsingular") from error
        jacobian_mass_inverse = mass_inverse_jacobian_transpose.T
        operational_inverse_inertia = (
            jacobian @ mass_inverse_jacobian_transpose
        )
        torque_projector = np.eye(positions.size) - (
            jacobian.T
            @ np.linalg.pinv(operational_inverse_inertia, rcond=1e-8)
            @ jacobian_mass_inverse
        )
        unprojected = (
            joint_stiffness * (reference - positions)
            + joint_damping * (reference_velocity - velocities)
        )
        nullspace_torque = torque_projector @ unprojected

    if dynamics_model is None:
        model_torque = np.zeros(positions.size, dtype=float)
    else:
        model_torque = np.asarray(
            dynamics_model.inverse_dynamics(
                positions,
                velocities,
                np.zeros(positions.size, dtype=float),
            ),
            dtype=float,
        )
        if (
            model_torque.shape != positions.shape
            or not np.all(np.isfinite(model_torque))
        ):
            raise ValueError(
                "dynamics model must return one finite torque per joint"
            )
    command_torque = task_torque + nullspace_torque + model_torque
    if not all(
        np.all(np.isfinite(values))
        for values in (
            measured_twist, wrench, task_torque, nullspace_torque,
            command_torque,
        )
    ):
        raise ValueError("Cartesian impedance evaluation is not finite")

    return CartesianImpedanceCommand(
        pose_error=pose_error.copy(),
        desired_twist=target_twist.copy(),
        measured_twist=measured_twist.copy(),
        commanded_wrench=wrench.copy(),
        geometric_jacobian=jacobian.copy(),
        task_torque=task_torque.copy(),
        nullspace_torque=nullspace_torque.copy(),
        model_torque=model_torque.copy(),
        command_torque=command_torque.copy(),
    )
