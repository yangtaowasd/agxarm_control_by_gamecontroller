"""Numerical inverse kinematics using a URDF model's PoE space screws."""

from dataclasses import dataclass
import math

import modern_robotics as mr
import numpy as np

from armbycontroller.lie import skew
from armbycontroller.lie import transform


class ScrewIkFailure(RuntimeError):
    """A safe rejection of an invalid or unreachable Cartesian target."""


@dataclass(frozen=True)
class ScrewIkResult:
    """One FK-verified inverse-kinematics solution."""

    joints: np.ndarray
    position_error: float
    orientation_error: float
    iterations: int


class ScrewVelocityIk:
    """One-step resolved-rate IK for a 100 Hz Cartesian servo."""

    def __init__(
        self,
        model,
        joint_limits=None,
        base_damping=1e-3,
        singular_damping=5e-2,
        singular_threshold=5e-2,
        nullspace_gain=0.35,
        max_joint_velocity=0.5,
    ):
        self.model = model
        self.joint_limits = np.asarray(
            model.joint_limits if joint_limits is None else joint_limits,
            dtype=float,
        )
        self.joint_count = model.joint_count
        self.base_damping = float(base_damping)
        self.singular_damping = float(singular_damping)
        self.singular_threshold = float(singular_threshold)
        self.nullspace_gain = float(nullspace_gain)
        self.max_joint_velocity = float(max_joint_velocity)
        if self.joint_limits.shape != (self.joint_count, 2):
            raise ValueError("joint_limits shape does not match the model")

    def step(
        self, joints, linear_velocity, angular_velocity, period
    ):
        """Integrate one base-frame Cartesian velocity command."""
        q = np.asarray(joints, dtype=float)
        linear = np.asarray(linear_velocity, dtype=float)
        angular = np.asarray(angular_velocity, dtype=float)
        period = float(period)
        if (
            q.shape != (self.joint_count,)
            or linear.shape != (3,)
            or angular.shape != (3,)
            or not np.all(np.isfinite(q))
            or not np.all(np.isfinite(linear))
            or not np.all(np.isfinite(angular))
            or not math.isfinite(period)
            or period <= 0.0
        ):
            raise ValueError("velocity IK inputs are invalid")

        pose = self.model.forward_kinematics(q)
        space = self.model.space_jacobian(q)
        geometric = np.vstack((
            space[:3],
            space[3:] - skew(pose[:3, 3]) @ space[:3],
        ))
        singular_values = np.linalg.svd(geometric, compute_uv=False)
        minimum = float(singular_values[-1])
        singular_fraction = np.clip(
            (self.singular_threshold - minimum) / self.singular_threshold,
            0.0,
            1.0,
        )
        damping = (
            self.base_damping
            + singular_fraction * self.singular_damping
        )
        inverse = geometric.T @ np.linalg.solve(
            geometric @ geometric.T + damping ** 2 * np.eye(6),
            np.eye(6),
        )
        twist = np.concatenate((angular, linear))
        velocity = inverse @ twist

        if np.linalg.norm(twist) > 1e-12 and singular_fraction > 0.0:
            centers = np.mean(self.joint_limits, axis=1)
            spans = self.joint_limits[:, 1] - self.joint_limits[:, 0]
            center_velocity = (
                self.nullspace_gain * (centers - q) / spans
            )
            nullspace = np.eye(self.joint_count) - inverse @ geometric
            velocity += singular_fraction * nullspace @ center_velocity

        maximum = float(np.max(np.abs(velocity)))
        if maximum > self.max_joint_velocity:
            velocity *= self.max_joint_velocity / maximum
        next_q = q + velocity * period
        return np.clip(
            next_q,
            self.joint_limits[:, 0],
            self.joint_limits[:, 1],
        )


def _rotation_vector(rotation):
    return mr.so3ToVec(mr.MatrixLog3(rotation))


class ScrewIkSolver:
    """Damped least-squares IK from the model's analytic space Jacobian."""

    def __init__(
        self,
        model,
        joint_limits=None,
        max_iterations=120,
        position_tolerance=1e-5,
        orientation_tolerance=1e-4,
        damping=2e-3,
        singular_damping=1e-2,
        singular_threshold=5e-2,
        continuity_gain=0.2,
        max_joint_step=0.12,
        workspace_radius=math.inf,
    ):
        self.model = model
        self.joint_limits = np.asarray(
            model.joint_limits if joint_limits is None else joint_limits,
            dtype=float,
        )
        self.joint_count = model.joint_count
        self.dof = self.joint_count
        self.max_iterations = int(max_iterations)
        self.position_tolerance = float(position_tolerance)
        self.orientation_tolerance = float(orientation_tolerance)
        self.damping = float(damping)
        self.singular_damping = float(singular_damping)
        self.singular_threshold = float(singular_threshold)
        self.continuity_gain = float(continuity_gain)
        self.max_joint_step = float(max_joint_step)
        self.workspace_radius = float(workspace_radius)
        if self.joint_limits.shape != (self.joint_count, 2):
            raise ValueError("joint_limits shape does not match the model")
        if np.any(self.joint_limits[:, 0] >= self.joint_limits[:, 1]):
            raise ValueError(
                "each lower joint limit must be below its upper limit"
            )
        if (
            self.damping <= 0.0
            or self.singular_damping < 0.0
            or self.singular_threshold <= 0.0
            or self.continuity_gain < 0.0
        ):
            raise ValueError("IK damping and continuity settings are invalid")

    def _damped_inverse(self, jacobian):
        """Return an adaptive DLS inverse and the minimum singular value."""
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        minimum = float(singular_values[-1])
        singular_fraction = float(np.clip(
            (self.singular_threshold - minimum)
            / self.singular_threshold,
            0.0,
            1.0,
        ))
        damping = self.damping + singular_fraction * self.singular_damping
        regularized = (
            jacobian @ jacobian.T + damping ** 2 * np.eye(6)
        )
        try:
            inverse = jacobian.T @ np.linalg.solve(
                regularized, np.eye(6)
            )
        except np.linalg.LinAlgError as error:
            raise ScrewIkFailure("IK Jacobian solve failed") from error
        return inverse, minimum

    @staticmethod
    def _target(target):
        target = np.asarray(target, dtype=float)
        if target.shape != (4, 4) or not np.all(np.isfinite(target)):
            raise ScrewIkFailure("target must be a finite 4x4 transform")
        if not np.allclose(target[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
            raise ScrewIkFailure("target has an invalid homogeneous row")
        rotation = target[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
            raise ScrewIkFailure("target rotation is not orthonormal")
        if np.linalg.det(rotation) < 0.999999:
            raise ScrewIkFailure("target rotation is not right-handed")
        return target

    def _errors(self, current, target):
        position_error = target[:3, 3] - current[:3, 3]
        orientation_error = _rotation_vector(
            target[:3, :3] @ current[:3, :3].T
        )
        return position_error, orientation_error

    def solve(self, target, seed):
        """Solve a full six-dimensional target and verify it with PoE FK."""
        target = self._target(target)
        if np.linalg.norm(target[:3, 3]) > self.workspace_radius:
            raise ScrewIkFailure("target lies outside the robot workspace")
        q = np.asarray(seed, dtype=float)
        if q.shape != (self.joint_count,) or not np.all(np.isfinite(q)):
            raise ScrewIkFailure(
                f"IK seed must contain {self.joint_count} finite values"
            )
        q = np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])
        reference = q.copy()

        for iteration in range(1, self.max_iterations + 1):
            current = self.model.forward_kinematics(q)
            position_error, orientation_error = self._errors(current, target)
            position_norm = float(np.linalg.norm(position_error))
            orientation_norm = float(np.linalg.norm(orientation_error))
            if (
                position_norm <= self.position_tolerance
                and orientation_norm <= self.orientation_tolerance
            ):
                return ScrewIkResult(
                    q.copy(), position_norm, orientation_norm, iteration
                )

            space = self.model.space_jacobian(q)
            position = current[:3, 3]
            geometric = np.vstack((
                space[:3],
                space[3:] - skew(position) @ space[:3],
            ))
            error = np.concatenate((orientation_error, position_error))
            inverse, _ = self._damped_inverse(geometric)
            step = inverse @ error
            if self.continuity_gain > 0.0:
                nullspace = np.eye(self.joint_count) - inverse @ geometric
                step += (
                    self.continuity_gain
                    * nullspace
                    @ (reference - q)
                )
            maximum = float(np.max(np.abs(step)))
            if maximum > self.max_joint_step:
                step *= self.max_joint_step / maximum

            old_cost = position_norm + 0.3 * orientation_norm
            accepted = False
            scale = 1.0
            for _ in range(8):
                candidate = np.clip(
                    q + scale * step,
                    self.joint_limits[:, 0],
                    self.joint_limits[:, 1],
                )
                pose = self.model.forward_kinematics(candidate)
                p_error, r_error = self._errors(pose, target)
                cost = np.linalg.norm(p_error) + 0.3 * np.linalg.norm(r_error)
                if cost < old_cost:
                    q = candidate
                    accepted = True
                    break
                scale *= 0.5
            if not accepted:
                break

        current = self.model.forward_kinematics(q)
        position_error, orientation_error = self._errors(current, target)
        raise ScrewIkFailure(
            "IK did not converge: position=%.6g m, orientation=%.6g rad"
            % (
                np.linalg.norm(position_error),
                np.linalg.norm(orientation_error),
            )
        )

    def ik(self, position, rotation, seed_jnt_values):
        """Provide the solver interface consumed by ``AgxIkEngine``."""
        target = transform(rotation, position)
        try:
            return self.solve(target, seed_jnt_values).joints
        except ScrewIkFailure:
            return None

    def fk(self, joints):
        """Return position and rotation through the shared solver interface."""
        pose = self.model.forward_kinematics(joints)
        return pose[:3, 3].copy(), pose[:3, :3].copy()
