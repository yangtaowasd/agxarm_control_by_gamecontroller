"""Velocity-admittance reference generation over low-gain MIT control."""

import math

import numpy as np

from armbycontroller.control.core import ControlResult
from armbycontroller.control.mit import MitTorqueEnvelope
from armbycontroller.control.model_compensation import ModelCompensator
from armbycontroller.control.safety import ControlCycleGuard
from armbycontroller.control.safety import INTERACTION_TORQUE_LIMIT_MAX
from armbycontroller.control.safety import SustainedVelocityGuard
from armbycontroller.ik.screw import BoundedScrewVelocityIk


ADMITTANCE_MIT_TORQUE_LIMIT_MAX = INTERACTION_TORQUE_LIMIT_MAX


def _vector(values, size, name, *, positive=False, nonnegative=False):
    result = np.asarray(values, dtype=float)
    if (
        result.shape != (size,)
        or not np.all(np.isfinite(result))
        or (positive and np.any(result <= 0.0))
        or (nonnegative and np.any(result < 0.0))
    ):
        qualifier = (
            " positive"
            if positive
            else " nonnegative" if nonnegative else ""
        )
        raise ValueError(
            f"{name} must be a finite{qualifier} {size}-vector"
        )
    return result.copy()


class CartesianAdmittanceController:
    """Convert Cartesian admittance twist into a bounded MIT reference."""

    name = "cartesian_admittance"

    def __init__(
        self,
        model,
        admittance,
        velocity_ik: BoundedScrewVelocityIk,
        kp,
        kd,
        torque_limit,
        *,
        model_scale=1.0,
        joint_count=None,
        measured_velocity_limit=None,
        measured_velocity_hard_limit=None,
        measured_velocity_violation_cycles=3,
    ):
        self.model = model
        self.admittance = admittance
        self.velocity_ik = velocity_ik
        self.joint_count = int(
            model.joint_count if joint_count is None else joint_count
        )
        self.kp = _vector(
            kp, self.joint_count, "kp", nonnegative=True
        )
        self.kd = _vector(
            kd, self.joint_count, "kd", nonnegative=True
        )
        self.torque_limit = _vector(
            torque_limit,
            self.joint_count,
            "torque_limit",
            positive=True,
        )
        if np.any(
            self.torque_limit > ADMITTANCE_MIT_TORQUE_LIMIT_MAX
        ):
            raise ValueError("torque_limit values must be in (0, 8] N.m")
        self.model_scale = float(model_scale)
        if (
            not math.isfinite(self.model_scale)
            or not 0.0 <= self.model_scale <= 1.0
        ):
            raise ValueError("model_scale must be finite and in [0, 1]")
        if self.velocity_ik.joint_count != self.joint_count:
            raise ValueError("velocity IK joint count does not match model")
        if self.velocity_ik.model is not self.model:
            raise ValueError("velocity IK and admittance must share one model")
        self.model_compensator = ModelCompensator(
            self.model,
            self.joint_count,
            "gravity",
            self.model_scale,
        )
        self.mit_envelope = MitTorqueEnvelope(
            self.kp, self.kd, self.torque_limit
        )
        self.cycle_guard = ControlCycleGuard(
            self.joint_count,
            joint_limits=self.velocity_ik.joint_limits,
            position_tolerance=self.velocity_ik.joint_limit_margin,
        )
        sustained_velocity_limit = (
            2.0 * self.velocity_ik.velocity_limits
            if measured_velocity_limit is None
            else measured_velocity_limit
        )
        hard_velocity_limit = (
            4.0 * self.velocity_ik.velocity_limits
            if measured_velocity_hard_limit is None
            else measured_velocity_hard_limit
        )
        self.measured_velocity_guard = SustainedVelocityGuard(
            sustained_velocity_limit,
            hard_velocity_limit,
            measured_velocity_violation_cycles,
        )

    def reset(self, state):
        if state.joint_count != self.joint_count or not state.position_valid:
            raise ValueError(
                "admittance reset requires complete joint position"
            )
        self.admittance.reset(self.model.forward_kinematics(state.position))
        self.measured_velocity_guard.reset()

    def step(self, sample):
        state = sample.state
        self.cycle_guard.validate(sample)
        self.measured_velocity_guard.validate(state.velocity)
        admittance_state = self.admittance.step(
            sample.reference.external_wrench, sample.period
        )
        reference_velocity = self.velocity_ik.solve(
            admittance_state.desired_twist,
            state.position,
            sample.period,
        )
        reference_position = np.clip(
            state.position + reference_velocity * sample.period,
            self.velocity_ik.joint_limits[:, 0],
            self.velocity_ik.joint_limits[:, 1],
        )
        compensation = self.model_compensator.evaluate(
            state.position
        )
        torque = self.mit_envelope.command(
            reference_position,
            reference_velocity,
            state.position,
            state.velocity,
            compensation.requested_torque,
        )
        commanded_pose = self.model.forward_kinematics(reference_position)
        command = torque.command
        zeros = np.zeros(6)
        signals = {
            "admittance_offset": getattr(
                admittance_state, "offset", zeros
            ),
            "admittance_velocity": getattr(
                admittance_state, "velocity", zeros
            ),
            "admittance_acceleration": getattr(
                admittance_state, "acceleration", zeros
            ),
            "applied_wrench": getattr(
                admittance_state, "applied_wrench", zeros
            ),
            "resisting_wrench": getattr(
                admittance_state, "resisting_wrench", zeros
            ),
            "desired_pose": commanded_pose,
            "desired_twist": getattr(
                admittance_state, "desired_twist", zeros
            ),
            "admittance_mode": getattr(
                admittance_state,
                "mode",
                getattr(self.admittance, "mode", "unknown"),
            ),
            "joint_velocity": reference_velocity,
        }
        signals.update(compensation.signals())
        signals.update(torque.signals(
            model_torque=compensation.requested_torque
        ))
        return ControlResult(
            self.name,
            command,
            signals,
            raw=admittance_state,
        )
