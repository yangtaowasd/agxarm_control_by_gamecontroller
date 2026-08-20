"""Shared validation for one measured control cycle."""

import math

import numpy as np

from armbycontroller.control.core import ControlSafetyError


INTERACTION_TORQUE_LIMIT_MAX = 8.0


class InteractionSafetyLimits:
    """One validated source for every MIT interaction-mode boundary."""

    def __init__(
        self,
        *,
        torque_limit,
        reference_velocity_limit,
        measured_velocity_stop_limit,
        measured_velocity_hard_limit,
        measured_velocity_violation_cycles,
        joint_limit_margin,
    ):
        torque = np.asarray(torque_limit, dtype=float)
        if torque.ndim != 1 or torque.size < 1:
            raise ValueError("interaction torque limit must be a joint vector")
        self.joint_count = torque.size
        self.torque_limit = self._positive_vector(
            torque, "torque_limit"
        )
        if np.any(self.torque_limit > INTERACTION_TORQUE_LIMIT_MAX):
            raise ValueError(
                "interaction torque limits must be in (0, 8] N.m"
            )
        self.reference_velocity_limit = self._positive_vector(
            reference_velocity_limit, "reference_velocity_limit"
        )
        self.measured_velocity_stop_limit = self._positive_vector(
            measured_velocity_stop_limit,
            "measured_velocity_stop_limit",
        )
        self.measured_velocity_hard_limit = self._positive_vector(
            measured_velocity_hard_limit,
            "measured_velocity_hard_limit",
        )
        if np.any(
            self.measured_velocity_stop_limit
            <= self.reference_velocity_limit
        ) or np.any(
            self.measured_velocity_hard_limit
            <= self.measured_velocity_stop_limit
        ):
            raise ValueError(
                "measured stop limits must exceed reference limits and "
                "hard limits must exceed stop limits"
            )
        self.measured_velocity_violation_cycles = int(
            measured_velocity_violation_cycles
        )
        if self.measured_velocity_violation_cycles < 1:
            raise ValueError(
                "measured_velocity_violation_cycles must be positive"
            )
        self.joint_limit_margin = float(joint_limit_margin)
        if (
            not math.isfinite(self.joint_limit_margin)
            or self.joint_limit_margin < 0.0
        ):
            raise ValueError(
                "joint_limit_margin must be finite and nonnegative"
            )

    def _positive_vector(self, values, name):
        result = np.asarray(values, dtype=float)
        if (
            result.shape != (self.joint_count,)
            or not np.all(np.isfinite(result))
            or np.any(result <= 0.0)
        ):
            raise ValueError(
                f"{name} must contain {self.joint_count} positive values"
            )
        return result.copy()

    def velocity_guard(self):
        """Create independent violation history under shared thresholds."""
        return SustainedVelocityGuard(
            self.measured_velocity_stop_limit,
            self.measured_velocity_hard_limit,
            self.measured_velocity_violation_cycles,
        )


class SustainedVelocityGuard:
    """Debounce tracking overspeed while retaining an immediate hard stop."""

    def __init__(
        self,
        sustained_limits,
        hard_limits,
        violation_cycles=3,
    ):
        sustained = np.asarray(sustained_limits, dtype=float)
        hard = np.asarray(hard_limits, dtype=float)
        if (
            sustained.ndim != 1
            or sustained.size < 1
            or hard.shape != sustained.shape
            or not np.all(np.isfinite(sustained))
            or not np.all(np.isfinite(hard))
            or np.any(sustained <= 0.0)
            or np.any(hard <= sustained)
        ):
            raise ValueError(
                "velocity limits must be finite positive vectors with "
                "hard limits above sustained limits"
            )
        self.sustained_limits = sustained.copy()
        self.hard_limits = hard.copy()
        self.joint_count = sustained.size
        self.violation_cycles = int(violation_cycles)
        if self.violation_cycles < 1:
            raise ValueError("violation_cycles must be positive")
        self._violation_counts = np.zeros(self.joint_count, dtype=int)

    def reset(self):
        """Clear consecutive-violation history on controller entry."""
        self._violation_counts.fill(0)

    def validate(self, velocity):
        """Accept one measured velocity or raise a reasoned safety error."""
        velocity = np.asarray(velocity, dtype=float)
        if (
            velocity.shape != (self.joint_count,)
            or not np.all(np.isfinite(velocity))
        ):
            raise ValueError(
                "measured velocity must be a finite joint vector"
            )
        magnitude = np.abs(velocity)
        if np.any(magnitude > self.hard_limits):
            self.reset()
            raise ControlSafetyError(
                "measured joint velocity exceeded its hard limit",
                reason="measured_velocity_hard_limit",
            )
        violated = magnitude > self.sustained_limits
        self._violation_counts = np.where(
            violated, self._violation_counts + 1, 0
        )
        if np.any(self._violation_counts >= self.violation_cycles):
            self.reset()
            raise ControlSafetyError(
                "measured joint velocity remained above its stop limit",
                reason="measured_velocity_limit",
            )


class ControlCycleGuard:
    """Reject incomplete or out-of-envelope measured joint state."""

    def __init__(
        self,
        joint_count,
        *,
        require_position=True,
        require_velocity=True,
        require_effort=False,
        joint_limits=None,
        position_tolerance=0.0,
        velocity_limits=None,
        maximum_period=None,
    ):
        self.joint_count = int(joint_count)
        if self.joint_count < 1:
            raise ValueError("joint_count must be positive")
        self.require_position = bool(require_position)
        self.require_velocity = bool(require_velocity)
        self.require_effort = bool(require_effort)
        self.joint_limits = self._joint_limits(joint_limits)
        self.position_tolerance = float(position_tolerance)
        if (
            not math.isfinite(self.position_tolerance)
            or self.position_tolerance < 0.0
        ):
            raise ValueError(
                "position_tolerance must be finite and nonnegative"
            )
        self.velocity_limits = self._positive_vector(
            velocity_limits, "velocity_limits"
        )
        self.maximum_period = (
            None if maximum_period is None else float(maximum_period)
        )
        if (
            self.maximum_period is not None
            and (
                not math.isfinite(self.maximum_period)
                or self.maximum_period <= 0.0
            )
        ):
            raise ValueError("maximum_period must be positive and finite")

    def _joint_limits(self, values):
        if values is None:
            return None
        result = np.asarray(values, dtype=float)
        if (
            result.shape != (self.joint_count, 2)
            or not np.all(np.isfinite(result))
            or np.any(result[:, 0] >= result[:, 1])
        ):
            raise ValueError("joint_limits must be a finite Nx2 matrix")
        return result.copy()

    def _positive_vector(self, values, name):
        if values is None:
            return None
        result = np.asarray(values, dtype=float)
        if (
            result.shape != (self.joint_count,)
            or not np.all(np.isfinite(result))
            or np.any(result <= 0.0)
        ):
            raise ValueError(
                f"{name} must contain {self.joint_count} positive values"
            )
        return result.copy()

    def validate(self, sample):
        """Validate one Control Input or raise a reasoned safety error."""
        state = sample.state
        if state.joint_count != self.joint_count:
            raise ControlSafetyError(
                "control state joint count changed",
                reason="joint_count_mismatch",
            )
        missing = []
        if self.require_position and not state.position_valid:
            missing.append("q")
        if self.require_velocity and not state.velocity_valid:
            missing.append("dq")
        if self.require_effort and not state.effort_valid:
            missing.append("effort")
        if missing:
            raise ControlSafetyError(
                "control cycle requires complete %s feedback"
                % "/".join(missing),
                reason="incomplete_feedback",
            )
        if (
            self.maximum_period is not None
            and sample.period > self.maximum_period
        ):
            raise ControlSafetyError(
                "control cycle period exceeded its limit",
                reason="control_period_limit",
            )
        if (
            state.position_valid
            and self.joint_limits is not None
            and np.any(
                (
                    state.position
                    < self.joint_limits[:, 0] - self.position_tolerance
                )
                | (
                    state.position
                    > self.joint_limits[:, 1] + self.position_tolerance
                )
            )
        ):
            raise ControlSafetyError(
                "measured joint position exceeded its limit",
                reason="measured_position_limit",
            )
        if (
            state.velocity_valid
            and self.velocity_limits is not None
            and np.any(
                np.abs(state.velocity) > self.velocity_limits
            )
        ):
            raise ControlSafetyError(
                "measured joint velocity exceeded its limit",
                reason="measured_velocity_limit",
            )
