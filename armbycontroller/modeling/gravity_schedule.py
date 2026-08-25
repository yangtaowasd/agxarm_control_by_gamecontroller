"""Smooth empirical scheduling for modeled joint-gravity torque."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class SignedGravityCalibration:
    """Scale and bias used on the negative and positive sides of one joint."""

    negative_scale: float = 1.0
    positive_scale: float = 1.0
    negative_bias_nm: float = 0.0
    positive_bias_nm: float = 0.0

    def __post_init__(self):
        values = (
            self.negative_scale,
            self.positive_scale,
            self.negative_bias_nm,
            self.positive_bias_nm,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("gravity calibration values must be finite")
        if not (
            0.0 <= self.negative_scale <= 2.0
            and 0.0 <= self.positive_scale <= 2.0
        ):
            raise ValueError("gravity calibration scales must be in [0, 2]")
        if (
            abs(self.negative_bias_nm) > 2.0
            or abs(self.positive_bias_nm) > 2.0
        ):
            raise ValueError(
                "gravity calibration biases must be in [-2, 2] N.m"
            )


class SmoothJointGravitySchedule:
    """Blend signed per-joint gravity calibration without a zero-angle jump."""

    def __init__(self, joint_count, transition_angle, calibrations):
        self.joint_count = int(joint_count)
        if self.joint_count < 1:
            raise ValueError("joint_count must be positive")
        self.transition_angle = float(transition_angle)
        if (
            not math.isfinite(self.transition_angle)
            or not math.radians(0.5)
            <= self.transition_angle
            <= math.radians(15.0)
        ):
            raise ValueError(
                "gravity transition angle must be in [0.5, 15] degrees"
            )
        self.calibrations = {}
        for joint_index, calibration in dict(calibrations).items():
            index = int(joint_index)
            if not 0 <= index < self.joint_count:
                raise ValueError("gravity calibration joint index is invalid")
            if not isinstance(calibration, SignedGravityCalibration):
                raise TypeError(
                    "gravity calibrations must be SignedGravityCalibration"
                )
            self.calibrations[index] = calibration

    def _positive_weight(self, angle):
        normalized = np.clip(
            (float(angle) + self.transition_angle)
            / (2.0 * self.transition_angle),
            0.0,
            1.0,
        )
        return float(normalized * normalized * (3.0 - 2.0 * normalized))

    def parameters(self, joint_positions):
        """Return the smoothly scheduled scale and bias joint vectors."""
        position = np.asarray(joint_positions, dtype=float)
        if (
            position.shape != (self.joint_count,)
            or not np.all(np.isfinite(position))
        ):
            raise ValueError(
                f"joint_positions must contain {self.joint_count} "
                "finite values"
            )
        scale = np.ones(self.joint_count)
        bias = np.zeros(self.joint_count)
        for index, calibration in self.calibrations.items():
            weight = self._positive_weight(position[index])
            scale[index] = (
                (1.0 - weight) * calibration.negative_scale
                + weight * calibration.positive_scale
            )
            bias[index] = (
                (1.0 - weight) * calibration.negative_bias_nm
                + weight * calibration.positive_bias_nm
            )
        return scale, bias

    def apply(self, joint_positions, gravity_torque):
        """Apply the scheduled calibration to one raw gravity vector."""
        torque = np.asarray(gravity_torque, dtype=float)
        if (
            torque.shape != (self.joint_count,)
            or not np.all(np.isfinite(torque))
        ):
            raise ValueError(
                f"gravity_torque must contain {self.joint_count} "
                "finite values"
            )
        scale, bias = self.parameters(joint_positions)
        return scale * torque + bias


def create_nero_horizontal_gravity_schedule(
    joint_count,
    transition_angle,
    j2_scale,
    j2_bias_nm,
    j4_scale,
    j4_bias_nm,
):
    """Build the agreed independent signed J2/J4 horizontal schedule."""

    def pair(values, name):
        result = np.asarray(values, dtype=float).reshape(-1)
        if result.size == 1:
            result = np.repeat(result, 2)
        if result.size != 2 or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must contain one or two finite values")
        return result

    if int(joint_count) < 4:
        raise ValueError("Nero horizontal gravity scheduling requires J4")
    j2_scale = pair(j2_scale, "J2 gravity scale")
    j2_bias_nm = pair(j2_bias_nm, "J2 gravity bias")
    j4_scale = pair(j4_scale, "J4 gravity scale")
    j4_bias_nm = pair(j4_bias_nm, "J4 gravity bias")
    return SmoothJointGravitySchedule(
        joint_count,
        transition_angle,
        {
            1: SignedGravityCalibration(
                j2_scale[0],
                j2_scale[1],
                j2_bias_nm[0],
                j2_bias_nm[1],
            ),
            3: SignedGravityCalibration(
                j4_scale[0],
                j4_scale[1],
                j4_bias_nm[0],
                j4_bias_nm[1],
            ),
        },
    )


class ScheduledGravityModel:
    """Decorate a dynamics model while changing only its gravity component."""

    def __init__(self, model, schedule):
        if model is None:
            raise ValueError("scheduled gravity requires a dynamics model")
        if not isinstance(schedule, SmoothJointGravitySchedule):
            raise TypeError("schedule must be SmoothJointGravitySchedule")
        self.model = model
        self.schedule = schedule

    def __getattr__(self, name):
        return getattr(self.model, name)

    def gravity_torque(self, joint_positions):
        raw = self.model.gravity_torque(joint_positions)
        return self.schedule.apply(joint_positions, raw)

    def inverse_dynamics(
        self, joint_positions, joint_velocities, joint_accelerations
    ):
        total = np.asarray(
            self.model.inverse_dynamics(
                joint_positions, joint_velocities, joint_accelerations
            ),
            dtype=float,
        )
        if (
            total.shape != (self.schedule.joint_count,)
            or not np.all(np.isfinite(total))
        ):
            raise ValueError(
                "inverse dynamics must return one finite joint vector"
            )
        raw_gravity = np.asarray(
            self.model.gravity_torque(joint_positions), dtype=float
        )
        scheduled_gravity = self.schedule.apply(
            joint_positions, raw_gravity
        )
        return total - raw_gravity + scheduled_gravity

    def compensation(self, joint_positions):
        """Keep the legacy compensation alias gravity-only."""
        return self.gravity_torque(joint_positions)

    def momentum_observer_terms(self, joint_positions, joint_velocities):
        """Schedule only gravity inside the observer's beta term."""
        momentum, beta = self.model.momentum_observer_terms(
            joint_positions, joint_velocities
        )
        momentum = np.asarray(momentum, dtype=float)
        beta = np.asarray(beta, dtype=float)
        expected = (self.schedule.joint_count,)
        if (
            momentum.shape != expected
            or beta.shape != expected
            or not np.all(np.isfinite(momentum))
            or not np.all(np.isfinite(beta))
        ):
            raise ValueError(
                "momentum observer terms must return finite joint vectors"
            )
        raw_gravity = np.asarray(
            self.model.gravity_torque(joint_positions), dtype=float
        )
        scheduled_gravity = self.schedule.apply(
            joint_positions, raw_gravity
        )
        return momentum, beta - raw_gravity + scheduled_gravity
