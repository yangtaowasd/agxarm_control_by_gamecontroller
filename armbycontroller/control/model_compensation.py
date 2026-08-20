"""Shared URDF model compensation for MIT controller adapters."""

from dataclasses import dataclass

import numpy as np


MODEL_COMPENSATION_MODES = (
    "gravity",
    "bias",
    "inverse_dynamics",
)


def _joint_vector(values, joint_count, name):
    result = np.asarray(values, dtype=float)
    if (
        result.shape != (joint_count,)
        or not np.all(np.isfinite(result))
    ):
        raise ValueError(
            f"{name} must contain {joint_count} finite values"
        )
    return result.copy()


def _joint_scale(values, joint_count):
    result = np.asarray(values, dtype=float)
    if result.ndim == 0 or result.size == 1:
        result = np.full(joint_count, float(result.reshape(-1)[0]))
    result = _joint_vector(result, joint_count, "model_scale")
    if np.any(result < 0.0) or np.any(result > 1.0):
        raise ValueError("model_scale values must be in [0, 1]")
    return result


@dataclass(frozen=True)
class ModelCompensation:
    """Raw and scaled model torque for one control cycle."""

    mode: str
    raw_torque: np.ndarray
    requested_torque: np.ndarray
    scale: np.ndarray

    def __post_init__(self):
        raw = np.asarray(self.raw_torque, dtype=float)
        if raw.ndim != 1 or raw.size < 1 or not np.all(np.isfinite(raw)):
            raise ValueError("raw_torque must be a finite joint vector")
        size = raw.size
        object.__setattr__(self, "raw_torque", raw.copy())
        object.__setattr__(
            self,
            "requested_torque",
            _joint_vector(self.requested_torque, size, "requested_torque"),
        )
        object.__setattr__(
            self, "scale", _joint_scale(self.scale, size)
        )

    def signals(self):
        """Return the common model-compensation diagnostic fields."""
        return {
            "model_compensation_mode": self.mode,
            "model_compensation_active": True,
            "model_torque_raw": self.raw_torque,
            "model_torque": self.requested_torque,
            "model_scale": self.scale,
        }


class ModelCompensator:
    """Evaluate gravity, bias, or inverse-dynamics model torque."""

    def __init__(self, model, joint_count, mode, scale=1.0):
        if model is None:
            raise ValueError("model compensation requires a dynamics model")
        self.model = model
        self.joint_count = int(joint_count)
        if self.joint_count < 1:
            raise ValueError("joint_count must be positive")
        self.mode = str(mode).strip().lower()
        if self.mode not in MODEL_COMPENSATION_MODES:
            raise ValueError(
                "model compensation mode must be gravity, bias, or "
                "inverse_dynamics"
            )
        self.scale = _joint_scale(scale, self.joint_count)

    def _gravity(self, position):
        if hasattr(self.model, "gravity_torque"):
            return self.model.gravity_torque(position)
        zeros = np.zeros(self.joint_count)
        return self.model.inverse_dynamics(position, zeros, zeros)

    def _bias(self, position, velocity):
        if hasattr(self.model, "inverse_dynamics"):
            return self.model.inverse_dynamics(
                position, velocity, np.zeros(self.joint_count)
            )
        if not (
            hasattr(self.model, "gravity_torque")
            and hasattr(self.model, "coriolis_torque")
        ):
            raise ValueError(
                "bias compensation requires inverse dynamics or gravity and "
                "Coriolis model methods"
            )
        return (
            np.asarray(self.model.gravity_torque(position), dtype=float)
            + np.asarray(
                self.model.coriolis_torque(position, velocity), dtype=float
            )
        )

    def evaluate(self, position, velocity=None, acceleration=None):
        """Return one finite, scaled model-compensation request."""
        position = _joint_vector(
            position, self.joint_count, "joint_positions"
        )
        zeros = np.zeros(self.joint_count)
        velocity = _joint_vector(
            zeros if velocity is None else velocity,
            self.joint_count,
            "joint_velocities",
        )
        acceleration = _joint_vector(
            zeros if acceleration is None else acceleration,
            self.joint_count,
            "joint_accelerations",
        )
        if self.mode == "gravity":
            raw = self._gravity(position)
        elif self.mode == "bias":
            raw = self._bias(position, velocity)
        else:
            if hasattr(self.model, "inverse_dynamics"):
                raw = self.model.inverse_dynamics(
                    position, velocity, acceleration
                )
            elif hasattr(self.model, "compensation"):
                raw = self.model.compensation(position)
            else:
                raise ValueError(
                    "inverse-dynamics compensation requires inverse_dynamics"
                )
        raw = _joint_vector(raw, self.joint_count, "model_torque")
        return ModelCompensation(
            self.mode,
            raw,
            self.scale * raw,
            self.scale,
        )
