"""Controller adapters for joint and Cartesian impedance."""

import numpy as np

from armbycontroller.cartesian import geometric_jacobian
from armbycontroller.control.core import ControlResult
from armbycontroller.control.core import ControlSafetyError
from armbycontroller.control.core import MitCommand
from armbycontroller.impedance.cartesian import cartesian_impedance_command


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


def bounded_model_feedforward(model_torque, scale, torque_limit):
    """Scale and bound model torque independently for every joint."""
    model_torque = np.asarray(model_torque, dtype=float)
    torque_limit = np.asarray(torque_limit, dtype=float)
    scale = float(scale)
    if (
        model_torque.shape != torque_limit.shape
        or model_torque.ndim != 1
        or not np.all(np.isfinite(model_torque))
        or not np.all(np.isfinite(torque_limit))
        or np.any(torque_limit <= 0.0)
        or not np.isfinite(scale)
    ):
        raise ValueError("model feedforward inputs are invalid")
    return np.clip(scale * model_torque, -torque_limit, torque_limit)


def limit_mit_combined_torque(
    feedforward,
    reference_position,
    reference_velocity,
    measured_position,
    measured_velocity,
    kp,
    kd,
    torque_limit,
):
    """Adjust MIT feedforward so the estimated total respects a limit."""
    arrays = [
        np.asarray(values, dtype=float)
        for values in (
            feedforward,
            reference_position,
            reference_velocity,
            measured_position,
            measured_velocity,
            kp,
            kd,
            torque_limit,
        )
    ]
    if (
        len({value.shape for value in arrays}) != 1
        or arrays[0].ndim != 1
        or not all(np.all(np.isfinite(value)) for value in arrays)
        or np.any(arrays[-1] <= 0.0)
    ):
        raise ValueError(
            "MIT torque limiter inputs must be finite equal arrays"
        )
    feed, q_ref, dq_ref, q, dq, kp_value, kd_value, limit = arrays
    feedback = kp_value * (q_ref - q) + kd_value * (dq_ref - dq)
    desired_total = feedback + feed
    bounded_total = np.clip(desired_total, -limit, limit)
    bounded_feedforward = np.clip(bounded_total - feedback, -limit, limit)
    return bounded_feedforward, feedback + bounded_feedforward


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
        if np.any(self.kp < 0.0) or np.any(self.kd < 0.0):
            raise ValueError("MIT gains must be nonnegative")
        self.dynamics_model = dynamics_model
        self.model_scale = float(model_scale)

    def reset(self, state):
        if state.joint_count != self.joint_count:
            raise ValueError("joint controller reset size mismatch")

    def step(self, sample):
        if sample.state.joint_count != self.joint_count:
            raise ValueError("joint controller input size mismatch")
        reference = sample.reference
        state = sample.state
        model_torque = self.feedforward.copy()
        model_active = False
        if self.dynamics_model is not None and state.position_valid:
            if hasattr(self.dynamics_model, "inverse_dynamics"):
                raw_model = self.dynamics_model.inverse_dynamics(
                    state.position,
                    reference.velocity,
                    reference.acceleration,
                )
            else:
                raw_model = self.dynamics_model.compensation(state.position)
            model_torque = bounded_model_feedforward(
                raw_model, self.model_scale, self.torque_limit
            )
            model_active = True
        estimated = (
            self.kp * (reference.position - state.position)
            + self.kd * (reference.velocity - state.velocity)
            + model_torque
        )
        combined_limit_active = state.position_valid
        if combined_limit_active:
            model_torque, estimated = limit_mit_combined_torque(
                model_torque,
                reference.position,
                reference.velocity,
                state.position,
                state.velocity,
                self.kp,
                self.kd,
                self.torque_limit,
            )
        command = MitCommand(
            reference.position,
            reference.velocity,
            self.kp,
            self.kd,
            model_torque,
            estimated,
        )
        return ControlResult(
            self.name,
            command,
            {
                "model_torque": model_torque,
                "model_active": model_active,
                "combined_limit_active": combined_limit_active,
                "torque_limit": self.torque_limit,
            },
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
        nullspace_stiffness=None,
        nullspace_damping=None,
        nullspace_enabled=False,
        joint_posture_stiffness=None,
        joint_posture_damping=None,
        model_scale=1.0,
    ):
        self.model = model
        self.stiffness = np.asarray(stiffness, dtype=float).copy()
        self.damping = np.asarray(damping, dtype=float).copy()
        self.joint_count = np.asarray(torque_limit).size
        self.torque_limit = _joint_vector(
            torque_limit, self.joint_count, "torque_limit", positive=True
        )
        self.nullspace_enabled = bool(nullspace_enabled)
        scale = np.asarray(model_scale, dtype=float)
        if scale.ndim == 0 or scale.size == 1:
            scale = np.full(self.joint_count, float(scale.reshape(-1)[0]))
        self.model_scale = _joint_vector(
            scale, self.joint_count, "model_scale"
        )
        if np.any(self.model_scale < 0.0) or np.any(self.model_scale > 1.0):
            raise ValueError("model_scale values must be in [0, 1]")
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

    def step(self, sample):
        state = sample.state
        reference = sample.reference
        if not state.position_valid or not state.velocity_valid:
            raise ControlSafetyError(
                "Cartesian impedance requires complete q/dq feedback"
            )
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
        raw = cartesian_impedance_command(
            self.model,
            self.model,
            state.position,
            state.velocity,
            desired_pose,
            desired_twist,
            self.stiffness,
            self.damping,
            model_scale=self.model_scale,
            **nullspace,
        )
        joint_posture_torque = (
            self.joint_posture_stiffness
            * (reference.position - state.position)
            + self.joint_posture_damping
            * (reference.velocity - state.velocity)
        )
        raw_command_torque = raw.command_torque + joint_posture_torque
        torque = np.clip(
            raw_command_torque, -self.torque_limit, self.torque_limit
        )
        zeros = np.zeros(self.joint_count)
        command = MitCommand(
            state.position,
            zeros,
            zeros,
            zeros,
            torque,
            torque,
        )
        return ControlResult(
            self.name,
            command,
            {
                "pose_error": raw.pose_error,
                "desired_twist": raw.desired_twist,
                "measured_twist": raw.measured_twist,
                "commanded_wrench": raw.commanded_wrench,
                "task_torque": raw.task_torque,
                "nullspace_torque": raw.nullspace_torque,
                "joint_posture_torque": joint_posture_torque,
                "model_torque": raw.model_torque,
                "raw_command_torque": raw_command_torque,
                "torque_limit": self.torque_limit,
                "torque_clipped": bool(np.any(raw_command_torque != torque)),
            },
            raw=raw,
        )
