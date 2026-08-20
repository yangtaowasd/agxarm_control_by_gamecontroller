"""One-owner hybrid Cartesian controller over complementary task subspaces."""

from dataclasses import dataclass

import numpy as np

from armbycontroller.cartesian import spatial_vector
from armbycontroller.control.core import ControlResult
from armbycontroller.control.mit import MitTorqueEnvelope
from armbycontroller.control.model_compensation import ModelCompensator
from armbycontroller.control.safety import ControlCycleGuard
from armbycontroller.control.safety import INTERACTION_TORQUE_LIMIT_MAX
from armbycontroller.control.safety import SustainedVelocityGuard
from armbycontroller.hybrid.selection import task_axis_mask
from armbycontroller.ik.screw import BoundedScrewVelocityIk
from armbycontroller.impedance.cartesian import cartesian_impedance_command


HYBRID_MIT_TORQUE_LIMIT_MAX = INTERACTION_TORQUE_LIMIT_MAX


def _joint_vector(values, size, name, *, nonnegative=False, positive=False):
    result = np.asarray(values, dtype=float)
    if (
        result.shape != (size,)
        or not np.all(np.isfinite(result))
        or (nonnegative and np.any(result < 0.0))
        or (positive and np.any(result <= 0.0))
    ):
        raise ValueError(f"{name} must be a valid {size}-joint vector")
    return result.copy()


def _task_matrix(values, name):
    result = np.asarray(values, dtype=float)
    if result.shape == (6,):
        result = np.diag(result)
    if result.shape != (6, 6) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 6-vector or 6x6 matrix")
    return result.copy()


@dataclass(frozen=True)
class HybridCartesianState:
    """One hybrid evaluation with separate admittance and impedance states."""

    admittance: object
    impedance: object
    desired_twist: np.ndarray
    reference_joint_velocity: np.ndarray


class HybridCartesianController:
    """Combine admittance Twist and complementary impedance in one command."""

    name = "hybrid_cartesian"

    def __init__(
        self,
        model,
        admittance,
        velocity_ik: BoundedScrewVelocityIk,
        cartesian_stiffness,
        cartesian_damping,
        kp,
        kd,
        torque_limit,
        *,
        torque_rate_limit=None,
        admittance_axes="z",
        desired_wrench=None,
        model_scale=1.0,
        nullspace_stiffness=None,
        nullspace_damping=None,
        nullspace_enabled=False,
        measured_velocity_limit=None,
        measured_velocity_hard_limit=None,
        measured_velocity_violation_cycles=3,
    ):
        self.model = model
        self.admittance = admittance
        self.velocity_ik = velocity_ik
        self.joint_count = int(model.joint_count)
        if velocity_ik.model is not model:
            raise ValueError("hybrid controller and velocity IK must share model")
        if velocity_ik.joint_count != self.joint_count:
            raise ValueError("hybrid velocity IK joint count does not match")

        self.admittance_axis_mask = task_axis_mask(admittance_axes)
        self.impedance_axis_mask = 1.0 - self.admittance_axis_mask
        self.admittance_selection = np.diag(self.admittance_axis_mask)
        self.impedance_selection = np.diag(self.impedance_axis_mask)
        stiffness = _task_matrix(cartesian_stiffness, "cartesian_stiffness")
        damping = _task_matrix(cartesian_damping, "cartesian_damping")
        self.impedance_stiffness = (
            self.impedance_selection @ stiffness @ self.impedance_selection
        )
        self.impedance_damping = (
            self.impedance_selection @ damping @ self.impedance_selection
        )
        self.desired_wrench = spatial_vector(
            np.zeros(6) if desired_wrench is None else desired_wrench,
            "desired_wrench",
        )
        kp = _joint_vector(kp, self.joint_count, "kp", nonnegative=True)
        kd = _joint_vector(kd, self.joint_count, "kd", nonnegative=True)
        torque_limit = _joint_vector(
            torque_limit,
            self.joint_count,
            "torque_limit",
            positive=True,
        )
        if np.any(torque_limit > HYBRID_MIT_TORQUE_LIMIT_MAX):
            raise ValueError("hybrid torque limits must be in (0, 8] N.m")
        self.mit_envelope = MitTorqueEnvelope(
            kp, kd, torque_limit, torque_rate_limit
        )
        self.model_compensator = ModelCompensator(
            model, self.joint_count, "bias", model_scale
        )
        self.nullspace_enabled = bool(nullspace_enabled)
        self.nullspace_stiffness = _joint_vector(
            np.zeros(self.joint_count)
            if nullspace_stiffness is None
            else nullspace_stiffness,
            self.joint_count,
            "nullspace_stiffness",
            nonnegative=True,
        )
        self.nullspace_damping = _joint_vector(
            np.zeros(self.joint_count)
            if nullspace_damping is None
            else nullspace_damping,
            self.joint_count,
            "nullspace_damping",
            nonnegative=True,
        )
        self.cycle_guard = ControlCycleGuard(
            self.joint_count,
            joint_limits=velocity_ik.joint_limits,
            position_tolerance=velocity_ik.joint_limit_margin,
        )
        sustained = (
            2.0 * velocity_ik.velocity_limits
            if measured_velocity_limit is None
            else measured_velocity_limit
        )
        hard = (
            4.0 * velocity_ik.velocity_limits
            if measured_velocity_hard_limit is None
            else measured_velocity_hard_limit
        )
        self.measured_velocity_guard = SustainedVelocityGuard(
            sustained, hard, measured_velocity_violation_cycles
        )
        self.anchor_pose = None
        self.anchor_position = None

    def reset(self, state):
        """Capture one nominal pose shared by both complementary subspaces."""
        if state.joint_count != self.joint_count or not state.position_valid:
            raise ValueError("hybrid reset requires complete joint position")
        self.anchor_position = state.position.copy()
        self.anchor_pose = self.model.forward_kinematics(state.position)
        self.admittance.reset(self.anchor_pose)
        self.measured_velocity_guard.reset()
        self.mit_envelope.reset(
            state.effort if state.effort_valid else None
        )

    def step(self, sample):
        """Evaluate one Twist-admittance and complementary-impedance cycle."""
        if self.anchor_pose is None:
            raise RuntimeError("hybrid controller must be reset before step")
        state = sample.state
        self.cycle_guard.validate(sample)
        self.measured_velocity_guard.validate(state.velocity)

        wrench_error = self.admittance_selection @ (
            sample.reference.external_wrench - self.desired_wrench
        )
        admittance_state = self.admittance.step(
            wrench_error, sample.period
        )
        admittance_twist = self.admittance_selection @ spatial_vector(
            admittance_state.desired_twist, "admittance_desired_twist"
        )
        reference_velocity = self.velocity_ik.solve(
            admittance_twist, state.position, sample.period
        )
        reference_position = np.clip(
            state.position + reference_velocity * sample.period,
            self.velocity_ik.joint_limits[:, 0],
            self.velocity_ik.joint_limits[:, 1],
        )

        nullspace = {}
        if self.nullspace_enabled:
            nullspace = {
                "nullspace_reference": self.anchor_position,
                "nullspace_reference_velocity": np.zeros(self.joint_count),
                "nullspace_stiffness": self.nullspace_stiffness,
                "nullspace_damping": self.nullspace_damping,
            }
        impedance = cartesian_impedance_command(
            self.model,
            self.model,
            state.position,
            state.velocity,
            self.anchor_pose,
            np.zeros(6),
            self.impedance_stiffness,
            self.impedance_damping,
            model_torque_override=np.zeros(self.joint_count),
            **nullspace,
        )
        compensation = self.model_compensator.evaluate(
            state.position, state.velocity
        )
        feedforward = (
            impedance.task_torque
            + impedance.nullspace_torque
            + compensation.requested_torque
        )
        torque = self.mit_envelope.command(
            reference_position,
            reference_velocity,
            state.position,
            state.velocity,
            feedforward,
            period=sample.period,
        )
        commanded_pose = self.model.forward_kinematics(reference_position)
        signals = {
            "admittance_axis_mask": self.admittance_axis_mask,
            "impedance_axis_mask": self.impedance_axis_mask,
            "admittance_twist": admittance_twist,
            "desired_twist": admittance_twist,
            "joint_velocity": reference_velocity,
            "desired_pose": commanded_pose,
            "desired_wrench": self.desired_wrench,
            "wrench_error": wrench_error,
            "admittance_offset": getattr(
                admittance_state, "offset", np.zeros(6)
            ),
            "admittance_velocity": getattr(
                admittance_state, "velocity", np.zeros(6)
            ),
            "admittance_acceleration": getattr(
                admittance_state, "acceleration", np.zeros(6)
            ),
            "applied_wrench": getattr(
                admittance_state, "applied_wrench", wrench_error
            ),
            "resisting_wrench": getattr(
                admittance_state, "resisting_wrench", np.zeros(6)
            ),
            "pose_error": impedance.pose_error,
            "measured_twist": impedance.measured_twist,
            "commanded_wrench": impedance.commanded_wrench,
            "task_torque": impedance.task_torque,
            "nullspace_torque": impedance.nullspace_torque,
        }
        signals.update(compensation.signals())
        signals.update(torque.signals(
            model_torque=compensation.requested_torque,
            task_torque=impedance.task_torque,
            auxiliary_torque=impedance.nullspace_torque,
        ))
        return ControlResult(
            self.name,
            torque.command,
            signals,
            raw=HybridCartesianState(
                admittance_state,
                impedance,
                admittance_twist.copy(),
                reference_velocity.copy(),
            ),
        )
