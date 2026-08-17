"""Shared bounded integration for Cartesian-admittance modes."""

from dataclasses import dataclass
import math

import numpy as np

from armbycontroller.cartesian import spatial_vector
from armbycontroller.cartesian import transform_matrix
from armbycontroller.modeling.lie import rotation_from_vector


@dataclass(frozen=True)
class CartesianAdmittanceState:
    """One base-frame ``[rotation vector; translation]`` state."""

    mode: str
    offset: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    applied_wrench: np.ndarray
    resisting_wrench: np.ndarray
    desired_pose: np.ndarray
    desired_twist: np.ndarray


class CartesianAdmittance:
    """Integrate mode-specific virtual resistance with shared hard bounds."""

    mode = "generic"

    def __init__(
        self,
        virtual_mass,
        wrench_deadband,
        wrench_limit,
        offset_limit,
        velocity_limit,
        wrench_filter_hz=5.0,
    ):
        self.virtual_mass = spatial_vector(
            virtual_mass, "virtual_mass", positive=True
        )
        self.wrench_deadband = spatial_vector(
            wrench_deadband, "wrench_deadband", nonnegative=True
        )
        self.wrench_limit = spatial_vector(
            wrench_limit, "wrench_limit", positive=True
        )
        self.offset_limit = spatial_vector(
            offset_limit, "offset_limit", positive=True
        )
        self.velocity_limit = spatial_vector(
            velocity_limit, "velocity_limit", positive=True
        )
        self.wrench_filter_hz = float(wrench_filter_hz)
        if (
            not math.isfinite(self.wrench_filter_hz)
            or self.wrench_filter_hz < 0.0
        ):
            raise ValueError("wrench_filter_hz must be finite and nonnegative")
        self.anchor_pose = None
        self.offset = np.zeros(6)
        self.velocity = np.zeros(6)
        self.filtered_wrench = np.zeros(6)

    def reset(self, anchor_pose):
        """Capture a new anchor and clear all virtual motion."""
        self.anchor_pose = transform_matrix(
            anchor_pose, "anchor_pose"
        ).copy()
        self.offset = np.zeros(6)
        self.velocity = np.zeros(6)
        self.filtered_wrench = np.zeros(6)

    def _bounded_wrench(self, external_wrench, period):
        wrench = spatial_vector(external_wrench, "external_wrench")
        wrench = np.clip(wrench, -self.wrench_limit, self.wrench_limit)
        if self.wrench_filter_hz > 0.0:
            alpha = 1.0 - math.exp(
                -2.0 * math.pi * self.wrench_filter_hz * period
            )
            self.filtered_wrench += alpha * (
                wrench - self.filtered_wrench
            )
        else:
            self.filtered_wrench = wrench.copy()
        return np.sign(self.filtered_wrench) * np.maximum(
            np.abs(self.filtered_wrench) - self.wrench_deadband,
            0.0,
        )

    def _mode_acceleration(self, applied_wrench, period):
        raise NotImplementedError("admittance mode must define its dynamics")

    def step(self, external_wrench, period):
        """Advance one sample and return a bounded SE(3) reference."""
        if self.anchor_pose is None:
            raise RuntimeError("admittance must be reset with an anchor pose")
        period = float(period)
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("period must be positive and finite")
        applied = self._bounded_wrench(external_wrench, period)
        acceleration, resisting_wrench = self._mode_acceleration(
            applied, period
        )
        acceleration = spatial_vector(acceleration, "acceleration")
        resisting_wrench = spatial_vector(
            resisting_wrench, "resisting_wrench"
        )
        self.velocity = np.clip(
            self.velocity + period * acceleration,
            -self.velocity_limit,
            self.velocity_limit,
        )
        candidate = self.offset + period * self.velocity
        clipped = np.clip(candidate, -self.offset_limit, self.offset_limit)
        outward = (candidate != clipped) & (
            np.sign(self.velocity) == np.sign(candidate)
        )
        self.velocity[outward] = 0.0
        self.offset = clipped

        desired = self.anchor_pose.copy()
        desired[:3, :3] = (
            rotation_from_vector(self.offset[:3])
            @ self.anchor_pose[:3, :3]
        )
        desired[:3, 3] = self.anchor_pose[:3, 3] + self.offset[3:]
        return CartesianAdmittanceState(
            mode=self.mode,
            offset=self.offset.copy(),
            velocity=self.velocity.copy(),
            acceleration=acceleration.copy(),
            applied_wrench=applied.copy(),
            resisting_wrench=resisting_wrench.copy(),
            desired_pose=desired,
            desired_twist=self.velocity.copy(),
        )
