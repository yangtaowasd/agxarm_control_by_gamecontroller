"""Controller-neutral MIT command construction and torque safety envelope."""

from dataclasses import dataclass

import numpy as np

from armbycontroller.control.core import ControlSafetyError
from armbycontroller.control.core import MitCommand


def bounded_model_feedforward(model_torque, scale, torque_limit):
    """Scale and bound model torque independently for every joint."""
    model_torque = np.asarray(model_torque, dtype=float)
    torque_limit = np.asarray(torque_limit, dtype=float)
    scale = float(scale)
    if (
        model_torque.shape != torque_limit.shape
        or model_torque.ndim != 1
        or not np.all(np.isfinite(model_torque))
        or not np.all(np.isfinite(torque_limit))
        or np.any(torque_limit <= 0.0)
        or not np.isfinite(scale)
    ):
        raise ValueError("model feedforward inputs are invalid")
    return np.clip(scale * model_torque, -torque_limit, torque_limit)


def limit_mit_combined_torque(
    feedforward,
    reference_position,
    reference_velocity,
    measured_position,
    measured_velocity,
    kp,
    kd,
    torque_limit,
):
    """Adjust MIT feedforward so the estimated total respects a limit."""
    arrays = [
        np.asarray(values, dtype=float)
        for values in (
            feedforward,
            reference_position,
            reference_velocity,
            measured_position,
            measured_velocity,
            kp,
            kd,
            torque_limit,
        )
    ]
    if (
        len({value.shape for value in arrays}) != 1
        or arrays[0].ndim != 1
        or not all(np.all(np.isfinite(value)) for value in arrays)
        or np.any(arrays[-1] <= 0.0)
    ):
        raise ValueError(
            "MIT torque limiter inputs must be finite equal arrays"
        )
    feed, q_ref, dq_ref, q, dq, kp_value, kd_value, limit = arrays
    feedback = kp_value * (q_ref - q) + kd_value * (dq_ref - dq)
    bounded_total = np.clip(feedback + feed, -limit, limit)
    bounded_feedforward = np.clip(
        bounded_total - feedback, -limit, limit
    )
    return bounded_feedforward, feedback + bounded_feedforward


def _joint_vector(values, size, name):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite values")
    return result.copy()


@dataclass(frozen=True)
class MitTorqueResult:
    """One MIT command plus a common total-torque decomposition."""

    command: MitCommand
    feedback_torque: np.ndarray
    feedforward_requested: np.ndarray
    total_requested: np.ndarray
    saturated: bool
    feasible: bool
    saturation_reason: str
    torque_limit: np.ndarray
    rate_limited: bool = False
    torque_rate_limit: np.ndarray | None = None

    def signals(
        self,
        *,
        model_torque=None,
        task_torque=None,
        auxiliary_torque=None,
    ):
        """Return comparable torque fields for every MIT controller."""
        size = self.command.position.size
        zeros = np.zeros(size)
        model = _joint_vector(
            zeros if model_torque is None else model_torque,
            size,
            "model_torque",
        )
        task = _joint_vector(
            zeros if task_torque is None else task_torque,
            size,
            "task_torque",
        )
        auxiliary = _joint_vector(
            zeros if auxiliary_torque is None else auxiliary_torque,
            size,
            "auxiliary_torque",
        )
        if not np.allclose(
            model + task + auxiliary,
            self.feedforward_requested,
            atol=1e-9,
            rtol=1e-9,
        ):
            raise ValueError(
                "torque diagnostic parts must sum to requested feedforward"
            )
        return {
            "torque_feedback": self.feedback_torque,
            "torque_model_requested": model,
            "torque_task_requested": task,
            "torque_auxiliary_requested": auxiliary,
            "torque_feedforward_requested": self.feedforward_requested,
            "torque_feedforward_sent": self.command.feedforward,
            "torque_total_requested": self.total_requested,
            "torque_total_estimated": self.command.estimated_torque,
            "torque_limit": self.torque_limit,
            "torque_saturated": self.saturated,
            "torque_feasible": self.feasible,
            "torque_saturation_reason": self.saturation_reason,
            "torque_rate_limited": self.rate_limited,
            "torque_rate_limit": self.torque_rate_limit,
        }


class MitTorqueEnvelope:
    """Build MIT commands under absolute and optional torque-rate limits."""

    def __init__(self, kp, kd, torque_limit, torque_rate_limit=None):
        kp = np.asarray(kp, dtype=float)
        if kp.ndim != 1 or kp.size < 1:
            raise ValueError("kp must be a finite joint vector")
        self.joint_count = kp.size
        self.kp = _joint_vector(kp, self.joint_count, "kp")
        self.kd = _joint_vector(kd, self.joint_count, "kd")
        self.torque_limit = _joint_vector(
            torque_limit, self.joint_count, "torque_limit"
        )
        if (
            np.any(self.kp < 0.0)
            or np.any(self.kd < 0.0)
            or np.any(self.torque_limit <= 0.0)
        ):
            raise ValueError(
                "MIT gains must be nonnegative and torque limits positive"
            )
        self.torque_rate_limit = (
            None
            if torque_rate_limit is None
            else _joint_vector(
                torque_rate_limit, self.joint_count, "torque_rate_limit"
            )
        )
        if (
            self.torque_rate_limit is not None
            and np.any(self.torque_rate_limit <= 0.0)
        ):
            raise ValueError("MIT torque-rate limits must be positive")
        self.previous_torque = None

    def reset(self, initial_torque=None):
        """Reset rate history from measured torque or a zero-torque start."""
        if self.torque_rate_limit is None:
            self.previous_torque = None
            return
        initial = (
            np.zeros(self.joint_count)
            if initial_torque is None
            else _joint_vector(
                initial_torque, self.joint_count, "initial_torque"
            )
        )
        self.previous_torque = np.clip(
            initial, -self.torque_limit, self.torque_limit
        )

    def command(
        self,
        reference_position,
        reference_velocity,
        measured_position,
        measured_velocity,
        feedforward,
        *,
        reject_infeasible=True,
        period=None,
    ):
        """Return one bounded MIT command and its torque decomposition."""
        size = self.joint_count
        reference_position = _joint_vector(
            reference_position, size, "reference_position"
        )
        reference_velocity = _joint_vector(
            reference_velocity, size, "reference_velocity"
        )
        measured_position = _joint_vector(
            measured_position, size, "measured_position"
        )
        measured_velocity = _joint_vector(
            measured_velocity, size, "measured_velocity"
        )
        feedforward = _joint_vector(
            feedforward, size, "feedforward"
        )
        feedback = (
            self.kp * (reference_position - measured_position)
            + self.kd * (reference_velocity - measured_velocity)
        )
        requested_total = feedback + feedforward
        bounded_feedforward, estimated = limit_mit_combined_torque(
            feedforward,
            reference_position,
            reference_velocity,
            measured_position,
            measured_velocity,
            self.kp,
            self.kd,
            self.torque_limit,
        )
        absolute_feasible = bool(np.all(
            np.abs(estimated) <= self.torque_limit + 1e-9
        ))
        absolute_saturated = bool(not np.allclose(
            bounded_feedforward, feedforward, atol=1e-12, rtol=0.0
        ))
        if reject_infeasible and not absolute_feasible:
            raise ControlSafetyError(
                "MIT feedback alone exceeds the torque limit"
            )
        rate_limited = False
        rate_feasible = True
        if self.torque_rate_limit is not None:
            if period is None or not np.isfinite(period) or period <= 0.0:
                raise ValueError(
                    "period must be finite and positive with torque-rate "
                    "limits"
                )
            if self.previous_torque is None:
                self.reset()
            delta = self.torque_rate_limit * float(period)
            rate_target = np.clip(
                estimated,
                self.previous_torque - delta,
                self.previous_torque + delta,
            )
            rate_limited = bool(not np.allclose(
                rate_target, estimated, atol=1e-12, rtol=0.0
            ))
            bounded_feedforward = np.clip(
                rate_target - feedback,
                -self.torque_limit,
                self.torque_limit,
            )
            estimated = feedback + bounded_feedforward
            rate_feasible = bool(np.all(
                np.abs(estimated - self.previous_torque) <= delta + 1e-9
            ))
            if reject_infeasible and not rate_feasible:
                raise ControlSafetyError(
                    "MIT feedback alone exceeds the torque-rate limit",
                    reason="torque_rate_infeasible",
                )
            self.previous_torque = estimated.copy()
        feasible = absolute_feasible and rate_feasible
        saturated = absolute_saturated or rate_limited
        if not feasible:
            reason = "feedback_uncontrollable"
        elif absolute_saturated and rate_limited:
            reason = "total_and_rate_limit"
        elif absolute_saturated:
            reason = "total_limit"
        elif rate_limited:
            reason = "torque_rate_limit"
        else:
            reason = "none"
        command = MitCommand(
            reference_position,
            reference_velocity,
            self.kp,
            self.kd,
            bounded_feedforward,
            estimated,
        )
        return MitTorqueResult(
            command,
            feedback,
            feedforward,
            requested_total,
            saturated,
            feasible,
            reason,
            self.torque_limit,
            rate_limited,
            None if self.torque_rate_limit is None else self.torque_rate_limit,
        )
