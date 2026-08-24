"""Controller adapters, normalized cycle input, and telemetry helpers."""

import time

import numpy as np

from armbycontroller.admittance.controller import CartesianAdmittanceController
from armbycontroller.control import ControlEngine
from armbycontroller.control import ControlInput
from armbycontroller.control import ControlReference
from armbycontroller.control import ControlState
from armbycontroller.hardware import extract_joint_angles
from armbycontroller.hybrid import HybridCartesianController
from armbycontroller.ik.screw import BoundedScrewVelocityIk
from armbycontroller.impedance.controllers import CartesianImpedanceController
from armbycontroller.impedance.controllers import JointMitController
from armbycontroller.ros.telemetry import RosTelemetry


class ControllerRuntimeMixin:
    """Build controller adapters and normalize their ROS-facing inputs."""

    def _build_control_engine(self):
        """Create pure controller adapters from the validated node settings."""
        controllers = []
        joint_count = int(getattr(
            self, "joint_count", len(self.jog.target_joints)
        ))
        joint_settings = ("mit_kp", "mit_kd", "mit_feedforward")
        if all(hasattr(self, name) for name in joint_settings):
            controllers.append(JointMitController(
                joint_count,
                self.mit_kp,
                self.mit_kd,
                self.mit_feedforward,
                getattr(
                    self,
                    "mit_gravity_torque_limit",
                    [8.0] * joint_count,
                ),
                dynamics_model=getattr(self, "gravity_model", None),
                model_scale=getattr(self, "mit_gravity_scale", 1.0),
                position_tolerance=getattr(
                    getattr(self, "interaction_safety", None),
                    "joint_limit_margin",
                    0.0,
                ),
                torque_rate_limit=getattr(
                    getattr(self, "interaction_safety", None),
                    "torque_rate_limit",
                    None,
                ),
            ))

        model = getattr(self, "gravity_model", None)
        if model is None:
            model = getattr(getattr(self, "ik_solver", None), "model", None)
        cartesian_settings = (
            "cartesian_stiffness",
            "cartesian_damping",
            "cartesian_torque_limit",
        )
        if model is not None and all(
            hasattr(self, name) for name in cartesian_settings
        ):
            controllers.append(CartesianImpedanceController(
                model,
                self.cartesian_stiffness,
                self.cartesian_damping,
                self.cartesian_torque_limit,
                torque_rate_limit=getattr(
                    getattr(self, "interaction_safety", None),
                    "torque_rate_limit",
                    None,
                ),
                nullspace_stiffness=getattr(
                    self, "cartesian_nullspace_stiffness", None
                ),
                nullspace_damping=getattr(
                    self, "cartesian_nullspace_damping", None
                ),
                nullspace_enabled=(
                    getattr(self, "robot_model", "piper_l") == "nero"
                ),
                joint_posture_stiffness=getattr(
                    self, "cartesian_joint_posture_stiffness", None
                ),
                joint_posture_damping=getattr(
                    self, "cartesian_joint_posture_damping", None
                ),
                model_scale=getattr(
                    self,
                    "cartesian_model_scale",
                    np.ones(joint_count),
                ),
                position_tolerance=getattr(
                    getattr(self, "interaction_safety", None),
                    "joint_limit_margin",
                    0.0,
                ),
                maximum_force=getattr(
                    self, "cartesian_max_force", float("inf")
                ),
                maximum_torque=getattr(
                    self, "cartesian_max_torque", float("inf")
                ),
                position_integral_gain=getattr(
                    self, "cartesian_position_integral_gain", None
                ),
                position_integral_deadband=getattr(
                    self, "cartesian_position_integral_deadband", None
                ),
                position_integral_max_rotation_error=getattr(
                    self,
                    "cartesian_position_integral_max_rotation_error",
                    float("inf"),
                ),
                position_integral_max_translation_error=getattr(
                    self,
                    "cartesian_position_integral_max_translation_error",
                    float("inf"),
                ),
                position_integral_max_force=getattr(
                    self, "cartesian_position_integral_max_force", 1.0
                ),
                position_integral_max_torque=getattr(
                    self, "cartesian_position_integral_max_torque", 0.25
                ),
                position_integral_leak_rate=getattr(
                    self, "cartesian_position_integral_leak_rate", 0.0
                ),
                position_integral_saturation_leak_rate=getattr(
                    self,
                    "cartesian_position_integral_saturation_leak_rate",
                    0.1,
                ),
                position_integral_external_force_gate=getattr(
                    self,
                    "cartesian_position_integral_external_force_gate",
                    float("inf"),
                ),
                position_integral_external_force_release=getattr(
                    self,
                    "cartesian_position_integral_external_force_release",
                    0.5,
                ),
                position_integral_external_torque_gate=getattr(
                    self,
                    "cartesian_position_integral_external_torque_gate",
                    float("inf"),
                ),
                position_integral_external_torque_release=getattr(
                    self,
                    "cartesian_position_integral_external_torque_release",
                    0.1,
                ),
                position_integral_requires_external_wrench=getattr(
                    self,
                    "cartesian_position_integral_requires_external_wrench",
                    False,
                ),
            ))
        admittance_settings = (
            "admittance_mit_kp",
            "admittance_mit_kd",
            "admittance_mit_torque_limit",
            "admittance_joint_velocity_limit",
            "admittance_measured_joint_velocity_stop_limit",
            "admittance_measured_joint_velocity_hard_limit",
            "admittance_measured_velocity_violation_cycles",
            "admittance_task_weights",
            "admittance_velocity_dls_damping",
            "admittance_joint_limit_margin",
            "admittance_singularity_slow_threshold",
            "admittance_singularity_stop_threshold",
            "admittance_singularity_damping",
        )
        if (
            model is not None
            and hasattr(self, "admittance_controller")
            and all(hasattr(self, name) for name in admittance_settings)
        ):
            velocity_ik = BoundedScrewVelocityIk(
                model,
                self.admittance_joint_velocity_limit,
                damping=self.admittance_velocity_dls_damping,
                task_weights=self.admittance_task_weights,
                joint_limit_margin=self.admittance_joint_limit_margin,
                singularity_slow_threshold=(
                    self.admittance_singularity_slow_threshold
                ),
                singularity_stop_threshold=(
                    self.admittance_singularity_stop_threshold
                ),
                singularity_damping=self.admittance_singularity_damping,
            )
            controllers.append(CartesianAdmittanceController(
                model,
                self.admittance_controller,
                velocity_ik,
                self.admittance_mit_kp,
                self.admittance_mit_kd,
                self.admittance_mit_torque_limit,
                torque_rate_limit=getattr(
                    getattr(self, "interaction_safety", None),
                    "torque_rate_limit",
                    None,
                ),
                model_scale=getattr(
                    self, "admittance_mit_model_scale", 1.0
                ),
                joint_count=joint_count,
                measured_velocity_limit=(
                    self.admittance_measured_joint_velocity_stop_limit
                ),
                measured_velocity_hard_limit=(
                    self.admittance_measured_joint_velocity_hard_limit
                ),
                measured_velocity_violation_cycles=(
                    self.admittance_measured_velocity_violation_cycles
                ),
            ))
        hybrid_settings = admittance_settings + (
            "cartesian_stiffness",
            "cartesian_damping",
            "cartesian_model_scale",
            "cartesian_nullspace_stiffness",
            "cartesian_nullspace_damping",
            "hybrid_admittance_axes",
            "hybrid_admittance_frame",
            "hybrid_admittance_frame_rotation",
            "hybrid_desired_wrench",
        )
        if (
            model is not None
            and hasattr(self, "hybrid_admittance_controller")
            and all(hasattr(self, name) for name in hybrid_settings)
        ):
            hybrid_velocity_ik = BoundedScrewVelocityIk(
                model,
                self.admittance_joint_velocity_limit,
                damping=self.admittance_velocity_dls_damping,
                task_weights=self.admittance_task_weights,
                joint_limit_margin=self.admittance_joint_limit_margin,
                singularity_slow_threshold=(
                    self.admittance_singularity_slow_threshold
                ),
                singularity_stop_threshold=(
                    self.admittance_singularity_stop_threshold
                ),
                singularity_damping=self.admittance_singularity_damping,
            )
            controllers.append(HybridCartesianController(
                model,
                self.hybrid_admittance_controller,
                hybrid_velocity_ik,
                self.cartesian_stiffness,
                self.cartesian_damping,
                self.admittance_mit_kp,
                self.admittance_mit_kd,
                self.admittance_mit_torque_limit,
                torque_rate_limit=getattr(
                    getattr(self, "interaction_safety", None),
                    "torque_rate_limit",
                    None,
                ),
                admittance_axes=self.hybrid_admittance_axes,
                admittance_frame=self.hybrid_admittance_frame,
                admittance_frame_rotation=(
                    self.hybrid_admittance_frame_rotation
                ),
                desired_wrench=self.hybrid_desired_wrench,
                model_scale=self.cartesian_model_scale,
                nullspace_stiffness=self.cartesian_nullspace_stiffness,
                nullspace_damping=self.cartesian_nullspace_damping,
                nullspace_enabled=(
                    getattr(self, "robot_model", "piper_l") == "nero"
                ),
                measured_velocity_limit=(
                    self.admittance_measured_joint_velocity_stop_limit
                ),
                measured_velocity_hard_limit=(
                    self.admittance_measured_joint_velocity_hard_limit
                ),
                measured_velocity_violation_cycles=(
                    self.admittance_measured_velocity_violation_cycles
                ),
                maximum_force=getattr(
                    self, "cartesian_max_force", float("inf")
                ),
                maximum_torque=getattr(
                    self, "cartesian_max_torque", float("inf")
                ),
            ))
        return ControlEngine(controllers)

    def _get_control_engine(self, controller_name):
        engine = getattr(self, "control_engine", None)
        if engine is None or controller_name not in engine.available:
            engine = self._build_control_engine()
            self.control_engine = engine
        if controller_name not in engine.available:
            raise RuntimeError(
                f"controller adapter {controller_name!r} is unavailable"
            )
        return engine

    def _next_control_input(self, feedback=None, external_wrench=None):
        """Capture one shared sample for a controller adapter."""
        now = time.monotonic()
        nominal = 1.0 / getattr(self, "mit_command_rate", 100.0)
        previous = getattr(self, "last_mit_tick_time", None)
        period = (
            nominal
            if previous is None
            else float(np.clip(now - previous, 0.25 * nominal, 2.0 * nominal))
        )
        self.last_mit_tick_time = now
        trajectory = getattr(self, "mit_trajectory", None)
        if trajectory is None:
            reference_position = np.asarray(
                self.jog.target_joints, dtype=float
            )
            reference_velocity = np.zeros(self.joint_count)
            reference_acceleration = np.zeros(self.joint_count)
        else:
            (
                reference_position,
                reference_velocity,
                reference_acceleration,
            ) = trajectory.step(self.jog.target_joints, period)

        position = feedback.position if feedback is not None else None
        velocity = feedback.velocity if feedback is not None else None
        effort = feedback.torque if feedback is not None else None
        if position is None:
            try:
                position = extract_joint_angles(
                    self.arm.get_joint_angles(), self.joint_count
                )
            except Exception:
                position = None
        if velocity is None:
            velocity = self.read_joint_velocities()
        position_valid = position is not None
        velocity_valid = velocity is not None
        effort_valid = effort is not None
        if position is None:
            position = reference_position
        if velocity is None:
            velocity = np.zeros(self.joint_count)
        if effort is None:
            effort = np.zeros(self.joint_count)
        state = ControlState(
            position,
            velocity,
            effort,
            position_valid=position_valid,
            velocity_valid=velocity_valid,
            effort_valid=effort_valid,
        )
        reference = ControlReference(
            reference_position,
            reference_velocity,
            reference_acceleration,
            np.zeros(6) if external_wrench is None else external_wrench,
            external_wrench is not None,
        )
        return ControlInput(now, period, state, reference)

    def _publish_control_result(self, sample, result, interaction_mode):
        self._telemetry().publish_control_result(
            sample, result, interaction_mode
        )

    def _publish_control_event(self, event, **fields):
        self._telemetry().publish_control_event(event, **fields)

    def _telemetry(self):
        """Return the owned telemetry adapter or a test compatibility view."""
        telemetry = getattr(self, "telemetry", None)
        if telemetry is None:
            telemetry = RosTelemetry.from_controller_publishers(self)
        return telemetry

    def _current_interaction_mode(self):
        """Return the one synchronized interaction mode."""
        return self._check_interaction_mode_invariant()

    def _publish_interaction_state(self, reason):
        self._telemetry().publish_interaction_state(reason)
