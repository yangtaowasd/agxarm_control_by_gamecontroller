"""Conservative low-speed friction assist from observer residual torque."""

import numpy as np


def _joint_vector(values, joint_count, name):
    result = np.asarray(values, dtype=float)
    if result.ndim == 0 or result.size == 1:
        result = np.full(joint_count, float(result.reshape(-1)[0]))
    if result.shape != (joint_count,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {joint_count} finite values")
    return result.copy()


class ObserverFrictionAssist:
    """Cancel a bounded fraction of low-speed opposing observer residual.

    The momentum residual contains contact torque as well as friction.  Assist
    is therefore admitted only when it opposes an already-existing restoring
    torque, and is disabled once measured motion exceeds the configured low-
    speed window.  It never creates a direction of motion by itself.
    """

    def __init__(
        self,
        joint_count,
        gain,
        torque_limit,
        velocity_threshold,
        restoring_torque_threshold,
    ):
        self.joint_count = int(joint_count)
        if self.joint_count < 1:
            raise ValueError("joint_count must be positive")
        self.gain = _joint_vector(gain, self.joint_count, "gain")
        self.torque_limit = _joint_vector(
            torque_limit, self.joint_count, "torque_limit"
        )
        self.velocity_threshold = _joint_vector(
            velocity_threshold, self.joint_count, "velocity_threshold"
        )
        self.restoring_torque_threshold = _joint_vector(
            restoring_torque_threshold,
            self.joint_count,
            "restoring_torque_threshold",
        )
        if np.any(self.gain < 0.0) or np.any(self.gain >= 1.0):
            raise ValueError("gain values must be in [0, 1)")
        if np.any(self.torque_limit < 0.0):
            raise ValueError("torque_limit values must be nonnegative")
        if np.any(self.velocity_threshold <= 0.0):
            raise ValueError("velocity_threshold values must be positive")
        if np.any(self.restoring_torque_threshold < 0.0):
            raise ValueError(
                "restoring_torque_threshold values must be nonnegative"
            )

    def evaluate(
        self,
        restoring_torque,
        measured_velocity,
        external_torque,
        *,
        observation_valid,
    ):
        restoring = _joint_vector(
            restoring_torque, self.joint_count, "restoring_torque"
        )
        velocity = _joint_vector(
            measured_velocity, self.joint_count, "measured_velocity"
        )
        residual = _joint_vector(
            external_torque, self.joint_count, "external_torque"
        )
        if not observation_valid:
            return np.zeros(self.joint_count)

        raw_assist = np.clip(
            -self.gain * residual,
            -self.torque_limit,
            self.torque_limit,
        )
        active = (
            (np.abs(velocity) < self.velocity_threshold)
            & (np.abs(restoring) >= self.restoring_torque_threshold)
            & (raw_assist * restoring > 0.0)
        )
        return np.where(active, raw_assist, 0.0)
