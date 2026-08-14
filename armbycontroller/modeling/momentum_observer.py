"""Acceleration-free generalized-momentum external-torque observer."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class MomentumObservation:
    """One observer sample in joint coordinates."""

    external_torque: np.ndarray
    momentum: np.ndarray
    predicted_momentum: np.ndarray
    beta: np.ndarray
    reset: bool = False


class GeneralizedMomentumObserver:
    """Estimate external joint torque without differentiating velocity."""

    def __init__(self, joint_count, gain, maximum_period=0.05):
        self.joint_count = int(joint_count)
        if self.joint_count < 1:
            raise ValueError("joint_count must be positive")
        self.gain = self._vector(gain, "gain")
        if np.any(self.gain <= 0.0):
            raise ValueError("gain must be positive")
        self.maximum_period = float(maximum_period)
        if (
            not math.isfinite(self.maximum_period)
            or self.maximum_period <= 0.0
        ):
            raise ValueError("maximum_period must be positive and finite")
        if np.any(self.gain * self.maximum_period >= 2.0):
            raise ValueError(
                "gain * maximum_period must stay below the explicit-Euler "
                "stability limit of 2"
            )
        self._initial_momentum = None
        self._integral = np.zeros(self.joint_count)
        self._residual = np.zeros(self.joint_count)

    def _vector(self, values, name):
        result = np.asarray(values, dtype=float)
        if result.ndim == 0:
            result = np.full(self.joint_count, float(result))
        if (
            result.shape != (self.joint_count,)
            or not np.all(np.isfinite(result))
        ):
            raise ValueError(
                f"{name} must be one finite value or {self.joint_count} values"
            )
        return result

    @property
    def initialized(self):
        return self._initial_momentum is not None

    def reset(self, momentum=None):
        """Clear the integral and optionally anchor it at measured momentum."""
        self._integral = np.zeros(self.joint_count)
        self._residual = np.zeros(self.joint_count)
        self._initial_momentum = (
            None if momentum is None
            else self._vector(momentum, "momentum").copy()
        )

    def update(self, momentum, actuator_torque, beta, period):
        """Advance ``r=K[p-p0-integral(tau-beta+r)]`` by one sample."""
        momentum = self._vector(momentum, "momentum")
        actuator = self._vector(actuator_torque, "actuator_torque")
        beta = self._vector(beta, "beta")
        period = float(period)
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("period must be positive and finite")
        if not self.initialized or period > self.maximum_period:
            self.reset(momentum)
            return MomentumObservation(
                self._residual.copy(),
                momentum.copy(),
                momentum.copy(),
                beta.copy(),
                reset=True,
            )
        self._integral += period * (
            actuator - beta + self._residual
        )
        predicted = self._initial_momentum + self._integral
        self._residual = self.gain * (momentum - predicted)
        return MomentumObservation(
            self._residual.copy(),
            momentum.copy(),
            predicted.copy(),
            beta.copy(),
        )
