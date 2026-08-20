"""Normalize and estimate joint feedback without ROS or SDK side effects."""

from collections.abc import Iterable
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class MotorFeedback:
    """One cached SDK feedback sample used by all 100 Hz consumers."""

    position: np.ndarray
    velocity: np.ndarray
    torque: np.ndarray


def estimate_joint_velocity(
    previous_position,
    current_position,
    previous_velocity,
    period,
    time_constant,
):
    """Estimate joint velocity with a first-order low-pass differentiator."""
    previous = np.asarray(previous_position, dtype=float)
    current = np.asarray(current_position, dtype=float)
    velocity = np.asarray(previous_velocity, dtype=float)
    period = float(period)
    time_constant = float(time_constant)
    if (
        previous.ndim != 1
        or current.shape != previous.shape
        or velocity.shape != previous.shape
        or not all(
            np.all(np.isfinite(values))
            for values in (previous, current, velocity)
        )
        or not math.isfinite(period)
        or period <= 0.0
        or not math.isfinite(time_constant)
        or time_constant < 0.0
    ):
        raise ValueError("velocity estimator inputs are invalid")
    raw_velocity = (current - previous) / period
    if time_constant == 0.0:
        return raw_velocity
    alpha = period / (time_constant + period)
    return velocity + alpha * (raw_velocity - velocity)


def extract_joint_angles(result, joint_count):
    """Extract a complete joint vector from supported SDK result shapes."""
    if result is None:
        return None
    value = result.msg if hasattr(result, "msg") else result
    if isinstance(value, Iterable) and not isinstance(
        value, (str, bytes, dict)
    ):
        joints = [float(item) for item in value]
        return joints[:joint_count] if len(joints) >= joint_count else None
    if isinstance(value, dict):
        for key in ("joint_angles", "angles", "data", "msg"):
            joints = value.get(key)
            if (
                isinstance(joints, (list, tuple))
                and len(joints) >= joint_count
            ):
                return [float(item) for item in joints[:joint_count]]
    return None
