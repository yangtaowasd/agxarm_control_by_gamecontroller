"""Cartesian admittance driven by an estimated external wrench."""

from dataclasses import dataclass
import math

import numpy as np

from armbycontroller.lie import rotation_from_vector


def _vector(values, name, *, positive=False, nonnegative=False):
    result = np.asarray(values, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain six finite values")
    if positive and np.any(result <= 0.0):
        raise ValueError(f"{name} must contain six positive values")
    if nonnegative and np.any(result < 0.0):
        raise ValueError(f"{name} must contain six nonnegative values")
    return result


def _pose(values):
    pose = np.asarray(values, dtype=float)
    if (
        pose.shape != (4, 4)
        or not np.all(np.isfinite(pose))
        or not np.allclose(pose[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9)
        or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-7)
        or not np.isclose(np.linalg.det(pose[:3, :3]), 1.0, atol=1e-7)
    ):
        raise ValueError("anchor_pose must be a finite SE(3) transform")
    return pose


def estimate_cartesian_wrench(jacobian, external_joint_torque, damping=0.05):
    """Solve ``tau_ext=J.T wrench`` with damped least squares."""
    jacobian = np.asarray(jacobian, dtype=float)
    torque = np.asarray(external_joint_torque, dtype=float)
    damping = float(damping)
    if (
        jacobian.ndim != 2
        or jacobian.shape[0] != 6
        or jacobian.shape[1] < 1
        or not np.all(np.isfinite(jacobian))
        or torque.shape != (jacobian.shape[1],)
        or not np.all(np.isfinite(torque))
        or not math.isfinite(damping)
        or damping <= 0.0
    ):
        raise ValueError(
            "wrench estimation inputs must be finite and compatible"
        )
    regularized = jacobian @ jacobian.T + damping ** 2 * np.eye(6)
    try:
        return np.linalg.solve(regularized, jacobian @ torque)
    except np.linalg.LinAlgError as error:
        raise ValueError("wrench estimation solve failed") from error


@dataclass(frozen=True)
class CartesianAdmittanceState:
    """One base-frame ``[rotation vector; translation]`` admittance state."""

    offset: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    applied_wrench: np.ndarray
    desired_pose: np.ndarray
    desired_twist: np.ndarray


class CartesianAdmittance:
    """Integrate ``M*xdd + D*xd + K*x = wrench_ext`` at a fixed-rate seam."""

    def __init__(
        self,
        virtual_mass,
        damping,
        stiffness,
        wrench_deadband,
        wrench_limit,
        offset_limit,
        velocity_limit,
        wrench_filter_hz=5.0,
    ):
        self.virtual_mass = _vector(
            virtual_mass, "virtual_mass", positive=True
        )
        self.damping = _vector(damping, "damping", nonnegative=True)
        self.stiffness = _vector(
            stiffness, "stiffness", nonnegative=True
        )
        self.wrench_deadband = _vector(
            wrench_deadband, "wrench_deadband", nonnegative=True
        )
        self.wrench_limit = _vector(
            wrench_limit, "wrench_limit", positive=True
        )
        self.offset_limit = _vector(
            offset_limit, "offset_limit", positive=True
        )
        self.velocity_limit = _vector(
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
        self.anchor_pose = _pose(anchor_pose).copy()
        self.offset = np.zeros(6)
        self.velocity = np.zeros(6)
        self.filtered_wrench = np.zeros(6)

    def _bounded_wrench(self, external_wrench, period):
        wrench = _vector(external_wrench, "external_wrench")
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

    def step(self, external_wrench, period):
        """Advance one sample and return a bounded SE(3) reference."""
        if self.anchor_pose is None:
            raise RuntimeError("admittance must be reset with an anchor pose")
        period = float(period)
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("period must be positive and finite")
        applied = self._bounded_wrench(external_wrench, period)
        acceleration = (
            applied
            - self.damping * self.velocity
            - self.stiffness * self.offset
        ) / self.virtual_mass
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
            offset=self.offset.copy(),
            velocity=self.velocity.copy(),
            acceleration=acceleration.copy(),
            applied_wrench=applied.copy(),
            desired_pose=desired,
            desired_twist=self.velocity.copy(),
        )
