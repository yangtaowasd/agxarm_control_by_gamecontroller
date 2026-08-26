#!/usr/bin/env python3
"""Unified ROS 2 keyboard controller for Nero and Piper-L arms."""

import math
import time
from collections import deque

import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger

from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD

from armbycontroller.api import InteractionModeInterface
from armbycontroller.admittance import ADMITTANCE_MODES
from armbycontroller.admittance import create_cartesian_admittance
from armbycontroller.control import InteractionModeLifecycle
from armbycontroller.control import JointTrajectoryState
from armbycontroller.control.mit import (
    bounded_model_feedforward as _bounded_model,
)
from armbycontroller.control.mit import (
    limit_mit_combined_torque as _limit_mit,
)
from armbycontroller.impedance.cartesian import cartesian_impedance_diagonals
from armbycontroller.modeling.gravity_schedule import ScheduledGravityModel
from armbycontroller.modeling.gravity_schedule import (
    create_nero_horizontal_gravity_schedule,
)
from armbycontroller.modeling.screw_model import UrdfScrewModel
from armbycontroller.ik.core import AgxIkEngine
from armbycontroller.ik.core import create_screw_solver
from armbycontroller.ik.core import resolve_urdf_path
from armbycontroller.ik.core import resolve_tool_urdf_path
from armbycontroller.hybrid import compliance_frame_rotation
from armbycontroller.hybrid import task_axis_mask
from armbycontroller.modeling.screw_model import project_gravity_vector
from armbycontroller.ros.control_cycle import ControlCycleMixin
from armbycontroller.ros.controller_runtime import ControllerRuntimeMixin
from armbycontroller.ros.hardware_session import HardwareSessionMixin
from armbycontroller.ros.interaction_runtime import InteractionRuntimeMixin
from armbycontroller.ros.parameters import declare_controller_parameters
from armbycontroller.ros.parameters import (  # noqa: F401
    default_mit_feedforward,
)
from armbycontroller.ros.parameters import default_mit_gains  # noqa: F401
from armbycontroller.ros.parameters import expand_joint_values
from armbycontroller.ros.settings import ControllerSettings
from armbycontroller.ros.telemetry import RosTelemetry
from armbycontroller.teleop import ArmJointJogState
from armbycontroller.teleop import JogUpdate as JogUpdate  # noqa: F401
from armbycontroller.teleop import KEY_COUNT
from armbycontroller.teleop import KEY_HOME as KEY_HOME  # noqa: F401
from armbycontroller.teleop import KEY_JOINT_1 as KEY_JOINT_1  # noqa: F401
from armbycontroller.teleop import KEY_JOINT_7 as KEY_JOINT_7  # noqa: F401
from armbycontroller.teleop import (  # noqa: F401
    KEY_MODE_TOGGLE as KEY_MODE_TOGGLE,
)


def bounded_model_feedforward(model_torque, scale, torque_limit):
    """Compatibility export; implementation lives at the controller seam."""
    return _bounded_model(model_torque, scale, torque_limit)


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
    """Compatibility export; implementation lives at the controller seam."""
    return _limit_mit(
        feedforward,
        reference_position,
        reference_velocity,
        measured_position,
        measured_velocity,
        kp,
        kd,
        torque_limit,
    )


class ArmKeyboardController(
    ControllerRuntimeMixin,
    InteractionRuntimeMixin,
    HardwareSessionMixin,
    ControlCycleMixin,
    Node,
):
    def __init__(self):
        super().__init__("arm_keyboard_controller")

        declare_controller_parameters(self)

        self.settings = ControllerSettings.from_node(self)
        self.settings.install_on(self)
        start_in_impedance = self.settings.start_in_impedance
        self.interaction_velocity_guard = (
            self.interaction_safety.velocity_guard()
        )
        self.mit_kp = expand_joint_values(
            self.get_parameter("mit_kp").value, self.joint_count, "mit_kp"
        )
        self.mit_kd = expand_joint_values(
            self.get_parameter("mit_kd").value, self.joint_count, "mit_kd"
        )
        self.mit_feedforward = expand_joint_values(
            self.get_parameter("mit_feedforward").value,
            self.joint_count,
            "mit_feedforward",
        )
        self.mit_gravity_compensation_enabled = bool(
            self.get_parameter("mit_gravity_compensation_enabled").value
        )
        self.gravity_urdf_path = str(
            self.get_parameter("gravity_urdf_path").value
        )
        self.mit_gravity_scale = float(
            self.get_parameter("mit_gravity_scale").value
        )
        self.nero_horizontal_gravity_schedule_enabled = bool(
            self.get_parameter(
                "nero_horizontal_gravity_schedule_enabled"
            ).value
        )
        self.nero_horizontal_gravity_transition_angle = float(
            self.get_parameter(
                "nero_horizontal_gravity_transition_angle"
            ).value
        )
        self.nero_horizontal_gravity_j2_scale = expand_joint_values(
            self.get_parameter(
                "nero_horizontal_gravity_j2_scale"
            ).value,
            2,
            "nero_horizontal_gravity_j2_scale",
        )
        self.nero_horizontal_gravity_j2_bias_nm = expand_joint_values(
            self.get_parameter(
                "nero_horizontal_gravity_j2_bias_nm"
            ).value,
            2,
            "nero_horizontal_gravity_j2_bias_nm",
        )
        self.nero_horizontal_gravity_j4_scale = expand_joint_values(
            self.get_parameter(
                "nero_horizontal_gravity_j4_scale"
            ).value,
            2,
            "nero_horizontal_gravity_j4_scale",
        )
        self.nero_horizontal_gravity_j4_bias_nm = expand_joint_values(
            self.get_parameter(
                "nero_horizontal_gravity_j4_bias_nm"
            ).value,
            2,
            "nero_horizontal_gravity_j4_bias_nm",
        )
        self.mit_gravity_torque_limit = (
            self.interaction_safety.torque_limit.tolist()
        )
        rotation_stiffness = float(
            self.get_parameter(
                "cartesian_impedance_rotation_stiffness"
            ).value
        )
        base_z_rotation_stiffness = float(
            self.get_parameter(
                "cartesian_impedance_base_z_rotation_stiffness"
            ).value
        )
        translation_stiffness = float(
            self.get_parameter(
                "cartesian_impedance_translation_stiffness"
            ).value
        )
        rotation_damping = float(
            self.get_parameter(
                "cartesian_impedance_rotation_damping"
            ).value
        )
        translation_damping = float(
            self.get_parameter(
                "cartesian_impedance_translation_damping"
            ).value
        )
        (
            self.cartesian_stiffness,
            self.cartesian_damping,
        ) = cartesian_impedance_diagonals(
            rotation_stiffness,
            base_z_rotation_stiffness,
            translation_stiffness,
            rotation_damping,
            translation_damping,
        )
        self.cartesian_max_force = float(
            self.get_parameter("cartesian_impedance_max_force").value
        )
        self.cartesian_max_torque = float(
            self.get_parameter("cartesian_impedance_max_torque").value
        )
        self.cartesian_position_integral_gain = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_position_integral_gain"
                ).value,
                6,
                "cartesian_impedance_position_integral_gain",
            ),
            dtype=float,
        )
        self.cartesian_position_integral_deadband = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_position_integral_deadband"
                ).value,
                6,
                "cartesian_impedance_position_integral_deadband",
            ),
            dtype=float,
        )
        integral_scalar_names = (
            "max_rotation_error",
            "max_translation_error",
            "max_force",
            "max_torque",
            "leak_rate",
            "saturation_leak_rate",
            "external_force_gate",
            "external_force_release",
            "external_torque_gate",
            "external_torque_release",
        )
        for suffix in integral_scalar_names:
            setattr(
                self,
                f"cartesian_position_integral_{suffix}",
                float(self.get_parameter(
                    f"cartesian_impedance_position_integral_{suffix}"
                ).value),
            )
        self.cartesian_position_integral_requires_external_wrench = bool(
            self.get_parameter(
                "cartesian_impedance_position_integral_requires_"
                "external_wrench"
            ).value
        )
        self.cartesian_nullspace_stiffness = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_nullspace_stiffness"
                ).value,
                self.joint_count,
                "cartesian_impedance_nullspace_stiffness",
            ),
            dtype=float,
        )
        self.cartesian_nullspace_damping = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_nullspace_damping"
                ).value,
                self.joint_count,
                "cartesian_impedance_nullspace_damping",
            ),
            dtype=float,
        )
        self.cartesian_joint_posture_stiffness = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_joint_posture_stiffness"
                ).value,
                self.joint_count,
                "cartesian_impedance_joint_posture_stiffness",
            ),
            dtype=float,
        )
        self.cartesian_joint_posture_damping = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_joint_posture_damping"
                ).value,
                self.joint_count,
                "cartesian_impedance_joint_posture_damping",
            ),
            dtype=float,
        )
        self.cartesian_torque_limit = (
            self.interaction_safety.torque_limit.copy()
        )
        self.cartesian_model_scale = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_model_scale"
                ).value,
                self.joint_count,
                "cartesian_impedance_model_scale",
            ),
            dtype=float,
        )
        task_parameter_names = (
            "admittance_virtual_mass",
            "admittance_zero_force_damping",
            "admittance_zero_force_holding_stiffness",
            "admittance_zero_force_friction",
            "admittance_zero_force_stiction_velocity",
            "admittance_resistive_damping",
            "admittance_resistive_stiffness",
            "admittance_wrench_deadband",
            "admittance_wrench_limit",
            "admittance_offset_limit",
            "admittance_velocity_limit",
        )
        admittance_values = {
            name: expand_joint_values(
                self.get_parameter(name).value, 6, name
            )
            for name in task_parameter_names
        }
        self.admittance_mode = str(
            self.get_parameter("admittance_mode").value
        ).strip().lower()
        if self.admittance_mode not in ADMITTANCE_MODES:
            raise ValueError(
                "admittance_mode must be zero_force or resistive"
            )
        self.admittance_wrench_filter_hz = float(
            self.get_parameter("admittance_wrench_filter_hz").value
        )
        self.admittance_wrench_dls_damping = float(
            self.get_parameter("admittance_wrench_dls_damping").value
        )
        self.admittance_wrench_timeout = float(
            self.get_parameter("admittance_wrench_timeout").value
        )
        self.admittance_mit_kp = expand_joint_values(
            self.get_parameter("admittance_mit_kp").value,
            self.joint_count,
            "admittance_mit_kp",
        )
        self.admittance_mit_kd = expand_joint_values(
            self.get_parameter("admittance_mit_kd").value,
            self.joint_count,
            "admittance_mit_kd",
        )
        self.admittance_mit_torque_limit = (
            self.interaction_safety.torque_limit.tolist()
        )
        self.admittance_mit_model_scale = float(
            self.get_parameter("admittance_mit_model_scale").value
        )
        self.admittance_joint_velocity_limit = (
            self.interaction_safety.reference_velocity_limit.tolist()
        )
        self.admittance_measured_joint_velocity_stop_limit = (
            self.interaction_safety.measured_velocity_stop_limit.tolist()
        )
        self.admittance_measured_joint_velocity_hard_limit = (
            self.interaction_safety.measured_velocity_hard_limit.tolist()
        )
        self.admittance_measured_velocity_violation_cycles = (
            self.interaction_safety.measured_velocity_violation_cycles
        )
        self.admittance_task_weights = np.asarray(
            expand_joint_values(
                self.get_parameter("admittance_task_weights").value,
                6,
                "admittance_task_weights",
            ),
            dtype=float,
        )
        self.admittance_velocity_dls_damping = float(
            self.get_parameter("admittance_velocity_dls_damping").value
        )
        self.admittance_singularity_slow_threshold = float(
            self.get_parameter(
                "admittance_singularity_slow_threshold"
            ).value
        )
        self.admittance_singularity_stop_threshold = float(
            self.get_parameter(
                "admittance_singularity_stop_threshold"
            ).value
        )
        self.admittance_singularity_damping = float(
            self.get_parameter("admittance_singularity_damping").value
        )
        self.admittance_joint_limit_margin = (
            self.interaction_safety.joint_limit_margin
        )
        self.hybrid_admittance_axes = str(
            self.get_parameter("hybrid_admittance_axes").value
        ).strip().lower()
        task_axis_mask(self.hybrid_admittance_axes)
        self.hybrid_admittance_frame = str(
            self.get_parameter("hybrid_admittance_frame").value
        ).strip().lower()
        self.hybrid_admittance_frame_rotation = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "hybrid_admittance_frame_rotation"
                ).value,
                3,
                "hybrid_admittance_frame_rotation",
            ),
            dtype=float,
        )
        compliance_frame_rotation(
            self.hybrid_admittance_frame,
            np.eye(4),
            self.hybrid_admittance_frame_rotation,
        )
        self.hybrid_desired_wrench = np.asarray(
            expand_joint_values(
                self.get_parameter("hybrid_desired_wrench").value,
                6,
                "hybrid_desired_wrench",
            ),
            dtype=float,
        )
        admittance_factory_settings = dict(
            virtual_mass=admittance_values["admittance_virtual_mass"],
            zero_force_damping=admittance_values[
                "admittance_zero_force_damping"
            ],
            zero_force_holding_stiffness=admittance_values[
                "admittance_zero_force_holding_stiffness"
            ],
            zero_force_friction=admittance_values[
                "admittance_zero_force_friction"
            ],
            zero_force_stiction_velocity=admittance_values[
                "admittance_zero_force_stiction_velocity"
            ],
            resistive_damping=admittance_values[
                "admittance_resistive_damping"
            ],
            resistive_stiffness=admittance_values[
                "admittance_resistive_stiffness"
            ],
            wrench_deadband=admittance_values[
                "admittance_wrench_deadband"
            ],
            wrench_limit=admittance_values["admittance_wrench_limit"],
            offset_limit=admittance_values["admittance_offset_limit"],
            velocity_limit=admittance_values[
                "admittance_velocity_limit"
            ],
            wrench_filter_hz=self.admittance_wrench_filter_hz,
        )
        self.admittance_controller = create_cartesian_admittance(
            self.admittance_mode,
            **admittance_factory_settings,
        )
        self.hybrid_admittance_controller = create_cartesian_admittance(
            self.admittance_mode,
            **admittance_factory_settings,
        )
        configured_gravity = self.get_parameter("gravity_vector").value
        if self.robot_model == "nero" and self.nero_mount:
            self.gravity_vector = np.asarray(
                project_gravity_vector(self.nero_mount), dtype=float
            )
        else:
            self.gravity_vector = np.asarray(
                configured_gravity, dtype=float
            )
        self.mit_trajectory_max_velocity = (
            self.interaction_safety.reference_velocity_limit.tolist()
        )
        self.mit_trajectory_max_acceleration = expand_joint_values(
            self.get_parameter("mit_trajectory_max_acceleration").value,
            self.joint_count,
            "mit_trajectory_max_acceleration",
        )
        self.mit_trajectory_max_jerk = expand_joint_values(
            self.get_parameter("mit_trajectory_max_jerk").value,
            self.joint_count,
            "mit_trajectory_max_jerk",
        )
        self.mit_max_joint_step = float(
            self.get_parameter("mit_max_joint_step").value
        )
        if self.mit_max_joint_step <= 0.0:
            raise ValueError("MIT rate and maximum joint step must be > 0")

        if (
            self.admittance_wrench_dls_damping <= 0.0
            or self.admittance_wrench_timeout <= 0.0
            or self.admittance_velocity_dls_damping <= 0.0
            or self.admittance_singularity_stop_threshold <= 0.0
            or self.admittance_singularity_slow_threshold
            <= self.admittance_singularity_stop_threshold
            or self.admittance_singularity_damping < 0.0
        ):
            raise ValueError(
                "admittance DLS, singularity, and wrench timeout settings "
                "are invalid"
            )
        if self.cartesian_max_force <= 0.0 or self.cartesian_max_torque <= 0.0:
            raise ValueError("Cartesian wrench limits must be positive")
        if (
            np.any(self.cartesian_position_integral_gain < 0.0)
            or np.any(self.cartesian_position_integral_deadband < 0.0)
            or self.cartesian_position_integral_max_rotation_error <= 0.0
            or self.cartesian_position_integral_max_translation_error <= 0.0
            or self.cartesian_position_integral_max_force <= 0.0
            or self.cartesian_position_integral_max_torque <= 0.0
            or self.cartesian_position_integral_leak_rate < 0.0
            or self.cartesian_position_integral_saturation_leak_rate <= 0.0
            or self.cartesian_position_integral_external_force_gate <= 0.0
            or self.cartesian_position_integral_external_force_release <= 0.0
            or self.cartesian_position_integral_external_torque_gate <= 0.0
            or self.cartesian_position_integral_external_torque_release <= 0.0
            or self.cartesian_position_integral_external_force_release
            >= self.cartesian_position_integral_external_force_gate
            or self.cartesian_position_integral_external_torque_release
            >= self.cartesian_position_integral_external_torque_gate
        ):
            raise ValueError(
                "Cartesian position-integral settings are invalid"
            )
        if any(
            not 0.0 <= value <= 500.0
            for value in self.admittance_mit_kp
        ):
            raise ValueError("admittance_mit_kp values must be in [0, 500]")
        if any(
            not 0.0 <= value <= 5.0
            for value in self.admittance_mit_kd
        ):
            raise ValueError("admittance_mit_kd values must be in [0, 5]")
        if not 0.0 <= self.admittance_mit_model_scale <= 1.0:
            raise ValueError(
                "admittance_mit_model_scale must be in [0, 1]"
            )
        if np.any(self.admittance_task_weights <= 0.0):
            raise ValueError("admittance task weights must be positive")
        if not 0.0 <= self.mit_gravity_scale <= 1.0:
            raise ValueError("mit_gravity_scale must be in [0, 1]")
        if self.gravity_vector.shape != (3,) or not np.all(
            np.isfinite(self.gravity_vector)
        ):
            raise ValueError("gravity_vector must contain three finite values")
        if any(not 0.0 <= value <= 500.0 for value in self.mit_kp):
            raise ValueError("mit_kp values must be in [0, 500]")
        if any(not 0.0 <= value <= 5.0 for value in self.mit_kd):
            raise ValueError("mit_kd values must be in [0, 5]")
        if any(abs(value) > 10.0 for value in self.mit_feedforward):
            raise ValueError(
                "mit_feedforward values must be in [-10, 10] N·m"
            )
        if (
            not np.all(np.isfinite(self.cartesian_stiffness))
            or not np.all(np.isfinite(self.cartesian_damping))
            or np.any(self.cartesian_stiffness < 0.0)
            or np.any(self.cartesian_damping < 0.0)
            or np.any(self.cartesian_nullspace_stiffness < 0.0)
            or np.any(self.cartesian_nullspace_damping < 0.0)
            or np.any(self.cartesian_joint_posture_stiffness < 0.0)
            or np.any(self.cartesian_joint_posture_damping < 0.0)
        ):
            raise ValueError(
                "Cartesian, nullspace, and joint-posture gains must be "
                "nonnegative"
            )
        if (
            not np.all(np.isfinite(self.cartesian_model_scale))
            or np.any(self.cartesian_model_scale < 0.0)
            or np.any(self.cartesian_model_scale > 1.0)
        ):
            raise ValueError(
                "Cartesian model scales must be in [0, 1]"
            )
        if any(
            value <= 0.0
            for values in (
                self.mit_trajectory_max_velocity,
                self.mit_trajectory_max_acceleration,
                self.mit_trajectory_max_jerk,
            )
            for value in values
        ):
            raise ValueError("MIT trajectory limits must be greater than zero")

        self.joint_limits = [
            tuple(
                ROBOT_JOINT_LIMIT_PRESET_RAD[
                    self.robot_model
                ][f"joint{index}"]
            )
            for index in range(1, self.joint_count + 1)
        ]
        legacy_rate_scale = 20.0 / self.control_rate
        self.jog = ArmJointJogState(
            self.joint_limits, self.step_rad * legacy_rate_scale
        )
        self.cartesian_step_per_tick = (
            self.cartesian_step * legacy_rate_scale
        )
        self.orientation_step_per_tick = (
            self.orientation_step_rad * legacy_rate_scale
        )
        self.mit_trajectory = JointTrajectoryState(
            self.joint_count,
            self.mit_trajectory_max_velocity,
            self.mit_trajectory_max_acceleration,
            self.mit_trajectory_max_jerk,
        )
        urdf_path = resolve_urdf_path(
            self.urdf_path, self.robot_model
        )
        equipped_urdf_path = resolve_tool_urdf_path(
            urdf_path,
            self.robot_model,
            self.tool_configuration,
            self.gravity_urdf_path,
        )
        self.gravity_model = None
        if self.mit_gravity_compensation_enabled:
            self.gravity_model = UrdfScrewModel(
                equipped_urdf_path,
                self.base_frame,
                self.tip_link,
                self.joint_count,
                self.gravity_vector,
            )
            if (
                self.robot_model == "nero"
                and self.nero_mount == "horizontal"
                and self.nero_horizontal_gravity_schedule_enabled
            ):
                j2_scale = self.nero_horizontal_gravity_j2_scale
                j2_bias = self.nero_horizontal_gravity_j2_bias_nm
                j4_scale = self.nero_horizontal_gravity_j4_scale
                j4_bias = self.nero_horizontal_gravity_j4_bias_nm
                schedule = create_nero_horizontal_gravity_schedule(
                    self.joint_count,
                    self.nero_horizontal_gravity_transition_angle,
                    j2_scale,
                    j2_bias,
                    j4_scale,
                    j4_bias,
                )
                self.gravity_model = ScheduledGravityModel(
                    self.gravity_model, schedule
                )
                self.get_logger().info(
                    "Nero horizontal gravity scheduling active: "
                    "J2/J4 [negative, positive], smooth transition="
                    f"{math.degrees(schedule.transition_angle):.3f} deg"
                )
            self.get_logger().info(
                "URDF inverse dynamics ready: "
                f"{equipped_urdf_path}; "
                f"modeled_mass={self.gravity_model.modeled_mass:.3f} kg; "
                f"gravity={self.gravity_vector.tolist()}; "
                f"joints={self.gravity_model.movable_joint_names}"
            )
        self.ik_solver = create_screw_solver(
            equipped_urdf_path,
            self.base_frame,
            self.tip_link,
            self.joint_count,
            self.ik_timeout,
            self.ik_tolerance,
        )
        self.ik_engine = AgxIkEngine(
            self.ik_solver,
            self.joint_count,
            self.workspace_min_radius,
            self.workspace_max_radius,
            self.fk_position_tolerance,
            self.fk_rotation_tolerance,
            self.pointing_roll_samples,
            pointing_axis_only=False,
        )
        self.control_engine = self._build_control_engine()
        self.ik_target_rotation = None
        self.control_mode = "joint"
        self.ik_target_position = None
        self.ik_valid_history = deque(maxlen=10)
        self.ik_recovery_until = -1.0
        self.key_state = [0] * KEY_COUNT
        self.last_keyboard_time = 0.0
        self.arm = None
        self.firmware_probe_arm = None
        self.arm_connected = False
        self.arm_ready = False
        self.emergency_stopped = False
        self.interaction_transitioning = False
        self.interaction_transition_target = ""
        self.interaction_fault_reason = ""
        self.last_mit_tick_time = None
        self.admittance_enabled = False
        self.hybrid_enabled = False
        self.interaction_lifecycle = InteractionModeLifecycle()
        self.public_interaction_mode_interface = InteractionModeInterface(
            current_mode=self._current_interaction_mode,
            enter_normal=self._enter_normal_interaction_mode,
            enter_impedance=self._enter_public_impedance,
            enter_admittance=self._enter_public_admittance,
        )
        self.admittance_previous_control_mode = "joint"
        self.last_admittance_tick_time = None
        self.last_hybrid_tick_time = None
        self.feedback_previous_position = None
        self.feedback_previous_velocity = np.zeros(self.joint_count)
        self.feedback_previous_time = None
        self.feedback_source_timestamps = {}
        self.last_complete_motor_feedback = None
        self.last_complete_motor_feedback_at = -math.inf
        self.latest_external_wrench = np.zeros(6)
        self.latest_external_wrench_received_at = -math.inf
        self.latest_external_wrench_source_time = -math.inf

        self.keyboard_sub = self.create_subscription(
            Int32MultiArray,
            self.keyboard_topic,
            self.keyboard_callback,
            qos_profile_sensor_data,
        )
        self.telemetry = RosTelemetry(
            self,
            dynamics_state_topic=self.dynamics_state_topic,
            control_sample_topic=self.control_sample_topic,
            control_event_topic=self.control_event_topic,
            interaction_state_topic=self.interaction_state_topic,
        )
        self.normal_mode_service_server = self.create_service(
            Trigger,
            self.normal_mode_service_name,
            self._normal_mode_service_callback,
        )
        self.impedance_mode_service_server = self.create_service(
            Trigger,
            self.impedance_mode_service_name,
            self._impedance_mode_service_callback,
        )
        self.admittance_mode_service_server = self.create_service(
            Trigger,
            self.admittance_mode_service_name,
            self._admittance_mode_service_callback,
        )
        self.external_torque_sub = self.create_subscription(
            JointState,
            self.external_torque_topic,
            self.external_torque_callback,
            20,
        )
        self.connect_arm()
        self.mit_trajectory.reset(self.jog.target_joints)
        if start_in_impedance:
            self.toggle_impedance()
        self._publish_interaction_state("startup")
        self.timer = self.create_timer(
            1.0 / self.control_rate, self.control_tick
        )
        self.dynamics_timer = self.create_timer(
            1.0 / self.mit_command_rate, self.mit_tick
        )

        if self.execute_motion and not self.arm_ready:
            self.get_logger().error(
                f"{self.robot_model} motion unavailable: hardware "
                "initialization failed; keyboard commands are disabled"
            )
        else:
            self.get_logger().info(
                f"{self.robot_model} ready: P=joint/IK; "
                f"joint 1-{self.joint_count}+A/D; "
                "IK W/S+A/D+Z/X; I="
                f"{self.impedance_backend} impedance via MIT; "
                "O=admittance; H=hybrid; SPACE=home; E=E-stop"
            )

    def keyboard_callback(self, message):
        data = list(message.data)
        if len(data) < KEY_COUNT:
            self.get_logger().warning(
                f"keyboard state length {len(data)} < {KEY_COUNT}",
                throttle_duration_sec=1.0,
            )
            return
        self.key_state = [1 if value else 0 for value in data[:KEY_COUNT]]
        self.last_keyboard_time = time.monotonic()
