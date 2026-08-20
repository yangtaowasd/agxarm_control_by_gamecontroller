"""Bounded joint-reference trajectory generation."""

import math

import numpy as np


class JointTrajectoryState:
    """Generate bounded q/dq/ddq references for a moving joint-space goal."""

    def __init__(self, joint_count, max_velocity, max_acceleration, max_jerk):
        self.joint_count = int(joint_count)
        self.max_velocity = np.asarray(max_velocity, dtype=float)
        self.max_acceleration = np.asarray(max_acceleration, dtype=float)
        self.max_jerk = np.asarray(max_jerk, dtype=float)
        expected = (self.joint_count,)
        if any(
            values.shape != expected or not np.all(np.isfinite(values))
            or np.any(values <= 0.0)
            for values in (
                self.max_velocity, self.max_acceleration, self.max_jerk
            )
        ):
            raise ValueError(
                "trajectory limits must be positive per-joint values"
            )
        self.reset(np.zeros(self.joint_count))

    def reset(self, positions):
        """Reset the trajectory to a stationary measured position."""
        positions = np.asarray(positions, dtype=float)
        if positions.shape != (self.joint_count,) or not np.all(
            np.isfinite(positions)
        ):
            raise ValueError("trajectory reset positions are invalid")
        self.position = positions.copy()
        self.velocity = np.zeros(self.joint_count, dtype=float)
        self.acceleration = np.zeros(self.joint_count, dtype=float)

    def step(self, goals, period):
        """Advance one jerk-, acceleration-, and velocity-limited cycle."""
        goals = np.asarray(goals, dtype=float)
        period = float(period)
        if (
            goals.shape != (self.joint_count,)
            or not np.all(np.isfinite(goals))
            or not math.isfinite(period)
            or period <= 0.0
        ):
            raise ValueError("trajectory goal or period is invalid")
        natural_frequency = 8.0
        acceleration_delta = np.clip(
            natural_frequency ** 3 * (goals - self.position)
            - 3.0 * natural_frequency ** 2 * self.velocity
            - 3.0 * natural_frequency * self.acceleration,
            -self.max_jerk * period,
            self.max_jerk * period,
        )
        new_acceleration = np.clip(
            self.acceleration + acceleration_delta,
            -self.max_acceleration,
            self.max_acceleration,
        )
        new_velocity = np.clip(
            self.velocity + 0.5 * (
                self.acceleration + new_acceleration
            ) * period,
            -self.max_velocity,
            self.max_velocity,
        )
        new_position = self.position + 0.5 * (
            self.velocity + new_velocity
        ) * period
        # Snap only inside a one-frame terminal region to remove the tiny
        # limit cycle while preserving the jerk bound.
        settled = (
            (
                np.abs(goals - new_position)
                <= 25.0 * self.max_jerk * period ** 3
            )
            & (np.abs(new_velocity) <= 4.0 * self.max_jerk * period ** 2)
            & (np.abs(new_acceleration) <= self.max_jerk * period)
        )
        new_position = np.where(settled, goals, new_position)
        new_velocity = np.where(settled, 0.0, new_velocity)
        new_acceleration = np.where(settled, 0.0, new_acceleration)
        self.position = new_position
        self.velocity = new_velocity
        self.acceleration = new_acceleration
        return (
            self.position.copy(),
            self.velocity.copy(),
            self.acceleration.copy(),
        )
