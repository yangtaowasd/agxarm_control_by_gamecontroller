"""Controller adapters for joint and Cartesian impedance."""

import numpy as np

from armbycontroller.cartesian import geometric_jacobian
from armbycontroller.control.core import ControlResult
from armbycontroller.control.mit import MitTorqueEnvelope
from armbycontroller.control.model_compensation import ModelCompensator
from armbycontroller.control.safety import ControlCycleGuard
from armbycontroller.control.safety import INTERACTION_TORQUE_LIMIT_MAX
from armbycontroller.impedance.cartesian import cartesian_impedance_command
from armbycontroller.impedance.cartesian import limit_cartesian_wrench


def _joint_vector(values, joint_count, name, *, positive=False):
    result = np.asarray(values, dtype=float)
    if (
        result.shape != (joint_count,)
        or not np.all(np.isfinite(result))
        or (positive and np.any(result <= 0.0))
    ):
        qualifier = " positive" if positive else ""
        raise ValueError(f"{name} must be a finite{qualifier} joint vector")
    return result.copy()


class JointMitController:
    """Joint MIT impedance with optional URDF inverse-dynamics support."""

    name = "joint_impedance"

    def __init__(
        self,
        joint_count,
        kp,
        kd,
        feedforward,
        torque_limit,
        dynamics_model=None,
        model_scale=1.0,
        position_tolerance=0.0,
        torque_rate_limit=None,
    ):
        self.joint_count = int(joint_count)
        self.kp = _joint_vector(kp, self.joint_count, "kp")
        self.kd = _joint_vector(kd, self.joint_count, "kd")
        self.feedforward = _joint_vector(
            feedforward, self.joint_count, "feedforward"
        )
        self.torque_limit = _joint_vector(
            torque_limit, self.joint_count, "torque_limit", positive=True
        )
        if np.any(self.torque_limit > INTERACTION_TORQUE_LIMIT_MAX):
            raise ValueError("torque_limit values must be in (0, 8] N.m")
        if np.any(self.kp < 0.0) or np.any(self.kd < 0.0):
            raise ValueError("MIT gains must be nonnegative")
        self.dynamics_model = dynamics_model
        self.model_scale = float(model_scale)
        self.model_compensator = (
            None
            if dynamics_model is None
            else ModelCompensator(
                dynamics_model,
                self.joint_count,
                "inverse_dynamics",
                self.model_scale,
            )
        )
        self.mit_envelope = MitTorqueEnvelope(
            self.kp, self.kd, self.torque_limit, torque_rate_limit
        )
        self.cycle_guard = ControlCycleGuard(
            self.joint_count,
            require_position=False,
            require_velocity=False,
            joint_limits=getattr(dynamics_model, "joint_limits", None),
            position_tolerance=position_tolerance,
        )

    def reset(self, state):
        if state.joint_count != self.joint_count:
            raise ValueError("joint controller reset size mismatch")
        self.mit_envelope.reset(
            state.effort if state.effort_valid else None
        )

    def step(self, sample):
        self.cycle_guard.validate(sample)
        reference = sample.reference
        state = sample.state
        model_torque = self.feedforward.copy()
        model_active = False
        compensation = None
        if self.dynamics_model is not None and state.position_valid:
            compensation = self.model_compensator.evaluate(
                state.position,
                reference.velocity,
                reference.acceleration,
            )
            model_torque = compensation.requested_torque
            model_active = True
        combined_limit_active = state.position_valid
        torque = self.mit_envelope.command(
            reference.position,
            reference.velocity,
            state.position,
            state.velocity,
            model_torque,
            reject_infeasible=combined_limit_active,
            period=sample.period,
        )
        signals = {
            "model_torque": model_torque,
            "model_active": model_active,
            "combined_limit_active": combined_limit_active,
            "torque_limit": self.torque_limit,
        }
        if compensation is not None:
            signals.update(compensation.signals())
        signals.update(torque.signals(
            model_torque=(
                model_torque if model_active else np.zeros(self.joint_count)
            ),
            auxiliary_torque=(
                np.zeros(self.joint_count)
                if model_active else model_torque
            ),
        ))
        return ControlResult(
            self.name,
            torque.command,
            signals,
        )


class CartesianImpedanceController:
    """Cartesian impedance with optional joint-posture torque via MIT."""

    name = "cartesian_impedance"

    def __init__(
        self,
        model,
        stiffness,
        damping,
        torque_limit,
        *,
        torque_rate_limit=None,
        nullspace_stiffness=None,
        nullspace_damping=None,
        nullspace_enabled=False,
        joint_posture_stiffness=None,
        joint_posture_damping=None,
        model_scale=1.0,
        position_tolerance=0.0,
        maximum_force=float("inf"),
        maximum_torque=float("inf"),
    ):
        self.model = model
        self.stiffness = np.asarray(stiffness, dtype=float).copy()
        self.damping = np.asarray(damping, dtype=float).copy()
        self.joint_count = np.asarray(torque_limit).size
        self.torque_limit = _joint_vector(
            torque_limit, self.joint_count, "torque_limit", positive=True
        )
        if np.any(self.torque_limit > INTERACTION_TORQUE_LIMIT_MAX):
            raise ValueError("torque_limit values must be in (0, 8] N.m")
        self.nullspace_enabled = bool(nullspace_enabled)
        self.maximum_force = float(maximum_force)
        self.maximum_torque = float(maximum_torque)
        limit_cartesian_wrench(
            np.zeros(6), self.maximum_force, self.maximum_torque
        )
        scale = np.asarray(model_scale, dtype=float)
        if scale.ndim == 0 or scale.size == 1:
            scale = np.full(self.joint_count, float(scale.reshape(-1)[0]))
        self.model_scale = _joint_vector(
            scale, self.joint_count, "model_scale"
        )
        if np.any(self.model_scale < 0.0) or np.any(self.model_scale > 1.0):
            raise ValueError("model_scale values must be in [0, 1]")
        self.model_compensator = ModelCompensator(
            self.model,
            self.joint_count,
            "bias",
            self.model_scale,
        )
        self.mit_envelope = MitTorqueEnvelope(
            np.zeros(self.joint_count),
            np.zeros(self.joint_count),
            self.torque_limit,
            torque_rate_limit,
        )
        self.cycle_guard = ControlCycleGuard(
            self.joint_count,
            joint_limits=getattr(self.model, "joint_limits", None),
            position_tolerance=position_tolerance,
        )
        self.nullspace_stiffness = _joint_vector(
            np.zeros(self.joint_count)
            if nullspace_stiffness is None else nullspace_stiffness,
            self.joint_count,
            "nullspace_stiffness",
        )
        self.nullspace_damping = _joint_vector(
            np.zeros(self.joint_count)
            if nullspace_damping is None else nullspace_damping,
            self.joint_count,
            "nullspace_damping",
        )
        self.joint_posture_stiffness = _joint_vector(
            np.zeros(self.joint_count)
            if joint_posture_stiffness is None
            else joint_posture_stiffness,
            self.joint_count,
            "joint_posture_stiffness",
        )
        self.joint_posture_damping = _joint_vector(
            np.zeros(self.joint_count)
            if joint_posture_damping is None
            else joint_posture_damping,
            self.joint_count,
            "joint_posture_damping",
        )
        if (
            np.any(self.nullspace_stiffness < 0.0)
            or np.any(self.nullspace_damping < 0.0)
            or np.any(self.joint_posture_stiffness < 0.0)
            or np.any(self.joint_posture_damping < 0.0)
        ):
            raise ValueError("Cartesian joint gains must be nonnegative")
        self._reference_position = None
        self._reference_pose = None
        self._reference_jacobian = None

    def reset(self, state):
        if state.joint_count != self.joint_count:
            raise ValueError("Cartesian controller reset size mismatch")
        self._reference_position = None
        self._reference_pose = None
        self._reference_jacobian = None
        self.mit_envelope.reset(
            state.effort if state.effort_valid else None
        )

    def step(self, sample):
        state = sample.state
        reference = sample.reference
        self.cycle_guard.validate(sample)
        reference_position = np.asarray(reference.position, dtype=float)
        if (
            self._reference_position is None
            or not np.array_equal(
                reference_position, self._reference_position
            )
        ):
            reference_jacobian, desired_pose = geometric_jacobian(
                self.model, reference_position
            )
            self._reference_position = reference_position.copy()
            self._reference_pose = desired_pose
            self._reference_jacobian = reference_jacobian
        else:
            desired_pose = self._reference_pose
            reference_jacobian = self._reference_jacobian
        desired_twist = reference_jacobian @ reference.velocity
        nullspace = {}
        if self.nullspace_enabled:
            nullspace = {
                "nullspace_reference": reference.position,
                "nullspace_reference_velocity": reference.velocity,
                "nullspace_stiffness": self.nullspace_stiffness,
                "nullspace_damping": self.nullspace_damping,
            }
        compensation = self.model_compensator.evaluate(
            state.position, state.velocity
        )
        raw = cartesian_impedance_command(
            self.model,
            self.model,
            state.position,
            state.velocity,
            desired_pose,
            desired_twist,
            self.stiffness,
            self.damping,
            model_torque_override=compensation.requested_torque,
            maximum_force=self.maximum_force,
            maximum_torque=self.maximum_torque,
            **nullspace,
        )
        joint_posture_torque = (
            self.joint_posture_stiffness
            * (reference.position - state.position)
            + self.joint_posture_damping
            * (reference.velocity - state.velocity)
        )
        raw_command_torque = raw.command_torque + joint_posture_torque
        zeros = np.zeros(self.joint_count)
        torque = self.mit_envelope.command(
            state.position,
            zeros,
            state.position,
            state.velocity,
            raw_command_torque,
            period=sample.period,
        )
        signals = {
            "pose_error": raw.pose_error,
            "desired_twist": raw.desired_twist,
            "measured_twist": raw.measured_twist,
            "commanded_wrench": raw.commanded_wrench,
            "raw_commanded_wrench": raw.raw_commanded_wrench,
            "wrench_limited": raw.wrench_limited,
            "task_torque": raw.task_torque,
            "nullspace_torque": raw.nullspace_torque,
            "joint_posture_torque": joint_posture_torque,
            "raw_command_torque": raw_command_torque,
            "torque_limit": self.torque_limit,
            "torque_clipped": torque.saturated,
        }
        signals.update(compensation.signals())
        signals.update(torque.signals(
            model_torque=raw.model_torque,
            task_torque=raw.task_torque,
            auxiliary_torque=(
                raw.nullspace_torque + joint_posture_torque
            ),
        ))
        return ControlResult(
            self.name,
            torque.command,
            signals,
            raw=raw,
        )
