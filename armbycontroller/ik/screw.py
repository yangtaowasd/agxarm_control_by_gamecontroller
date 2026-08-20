"""Numerical inverse kinematics using a URDF model's PoE space screws."""

from dataclasses import dataclass
import math

import numpy as np

from armbycontroller.cartesian import geometric_jacobian
from armbycontroller.modeling.lie import space_pose_error
from armbycontroller.modeling.lie import transform


class ScrewIkFailure(RuntimeError):
    """A safe rejection of an invalid or unreachable Cartesian target."""


@dataclass(frozen=True)
class ScrewIkResult:
    """One FK-verified inverse-kinematics solution."""

    joints: np.ndarray
    position_error: float
    orientation_error: float
    iterations: int


class BoundedScrewVelocityIk:
    """Bounded weighted DLS over a PoE model's screw Jacobian."""

    def __init__(
        self,
        model,
        velocity_limits,
        damping=0.02,
        task_weights=None,
        joint_limit_margin=0.03,
        singularity_slow_threshold=0.05,
        singularity_stop_threshold=0.01,
        singularity_damping=0.08,
    ):
        self.model = model
        self.joint_count = int(model.joint_count)
        self.joint_limits = np.asarray(model.joint_limits, dtype=float)
        self.velocity_limits = self._vector(
            velocity_limits, self.joint_count, "velocity_limits", positive=True
        )
        self.task_weights = self._vector(
            np.ones(6) if task_weights is None else task_weights,
            6,
            "task_weights",
            positive=True,
        )
        self.damping = float(damping)
        self.singularity_slow_threshold = float(
            singularity_slow_threshold
        )
        self.singularity_stop_threshold = float(
            singularity_stop_threshold
        )
        self.singularity_damping = float(singularity_damping)
        self.joint_limit_margin = float(joint_limit_margin)
        if (
            self.joint_limits.shape != (self.joint_count, 2)
            or not np.all(np.isfinite(self.joint_limits))
            or np.any(self.joint_limits[:, 0] >= self.joint_limits[:, 1])
            or not math.isfinite(self.damping)
            or self.damping <= 0.0
            or not math.isfinite(self.singularity_slow_threshold)
            or not math.isfinite(self.singularity_stop_threshold)
            or not math.isfinite(self.singularity_damping)
            or self.singularity_stop_threshold <= 0.0
            or self.singularity_slow_threshold
            <= self.singularity_stop_threshold
            or self.singularity_damping < 0.0
            or not math.isfinite(self.joint_limit_margin)
            or self.joint_limit_margin < 0.0
        ):
            raise ValueError("invalid bounded screw velocity IK settings")
        self.last_minimum_singular_value = math.inf
        self.last_velocity_scale = 1.0
        self.last_damping = self.damping

    @staticmethod
    def _vector(values, size, name, *, positive=False):
        result = np.asarray(values, dtype=float)
        if (
            result.shape != (size,)
            or not np.all(np.isfinite(result))
            or (positive and np.any(result <= 0.0))
        ):
            qualifier = " positive" if positive else ""
            raise ValueError(
                f"{name} must be a finite{qualifier} {size}-vector"
            )
        return result.copy()

    def _bounds(self, joint_positions, period):
        lower_position = (
            self.joint_limits[:, 0]
            + self.joint_limit_margin
            - joint_positions
        ) / period
        upper_position = (
            self.joint_limits[:, 1]
            - self.joint_limit_margin
            - joint_positions
        ) / period
        lower = np.maximum(-self.velocity_limits, lower_position)
        upper = np.minimum(self.velocity_limits, upper_position)
        for index in np.flatnonzero(lower > upper):
            if (
                joint_positions[index]
                <= self.joint_limits[index, 0] + self.joint_limit_margin
            ):
                lower[index], upper[index] = (
                    0.0,
                    self.velocity_limits[index],
                )
            else:
                lower[index], upper[index] = (
                    -self.velocity_limits[index],
                    0.0,
                )
        return lower, upper

    def solve(self, target_twist, joint_positions, period):
        """Map one tool twist to a bounded joint-velocity reference."""
        target = self._vector(target_twist, 6, "target_twist")
        joints = self._vector(
            joint_positions, self.joint_count, "joint_positions"
        )
        period = float(period)
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("period must be finite and positive")
        jacobian, _ = geometric_jacobian(self.model, joints)
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        minimum = float(singular_values[-1])
        scale = float(np.clip(
            (minimum - self.singularity_stop_threshold)
            / (
                self.singularity_slow_threshold
                - self.singularity_stop_threshold
            ),
            0.0,
            1.0,
        ))
        self.last_minimum_singular_value = minimum
        self.last_velocity_scale = scale
        self.last_damping = (
            self.damping + (1.0 - scale) * self.singularity_damping
        )
        if scale == 0.0 and np.linalg.norm(target) > 1e-12:
            raise ScrewIkFailure(
                "bounded screw velocity IK reached its singularity stop "
                f"threshold (sigma_min={minimum:.6g})"
            )
        target = scale * target
        weighted_jacobian = self.task_weights[:, None] * jacobian
        weighted_target = self.task_weights * target
        lower, upper = self._bounds(joints, period)

        velocity = np.zeros(self.joint_count)
        free = np.ones(self.joint_count, dtype=bool)
        for _ in range(self.joint_count + 1):
            if not np.any(free):
                break
            fixed = ~free
            residual = weighted_target.copy()
            if np.any(fixed):
                residual -= weighted_jacobian[:, fixed] @ velocity[fixed]
            reduced = weighted_jacobian[:, free]
            normal = (
                reduced.T @ reduced
                + self.last_damping ** 2 * np.eye(np.count_nonzero(free))
            )
            try:
                velocity[free] = np.linalg.solve(
                    normal, reduced.T @ residual
                )
            except np.linalg.LinAlgError as error:
                raise ScrewIkFailure(
                    "bounded screw velocity IK solve failed"
                ) from error
            below = free & (velocity < lower)
            above = free & (velocity > upper)
            if not np.any(below | above):
                break
            violation = np.maximum(lower - velocity, velocity - upper)
            index = int(np.argmax(np.where(
                below | above, violation, -np.inf
            )))
            velocity[index] = (
                lower[index]
                if velocity[index] < lower[index]
                else upper[index]
            )
            free[index] = False
        return np.clip(velocity, lower, upper)


class ScrewIkSolver:
    """DLS IK from a full SE(3) space error and analytic space Jacobian."""

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
        error_twist = space_pose_error(current, target)
        return error_twist[3:], error_twist[:3]

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
            error = np.concatenate((orientation_error, position_error))
            inverse, _ = self._damped_inverse(space)
            step = inverse @ error
            if self.continuity_gain > 0.0:
                nullspace = np.eye(self.joint_count) - inverse @ space
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
