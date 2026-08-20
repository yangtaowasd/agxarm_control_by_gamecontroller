"""Validated immutable startup settings for the main ROS controller."""

from dataclasses import dataclass
from dataclasses import fields
import math
from typing import Any
from typing import Mapping

from armbycontroller.control import InteractionSafetyLimits
from armbycontroller.ik.core import resolve_firmware_name
from armbycontroller.ros.parameters import expand_joint_values
from armbycontroller.ros.parameters import MODEL_PROFILES


@dataclass(frozen=True)
class ControllerSettings:
    """Robot, startup, kinematic, safety, and ROS interface settings."""

    robot_model: str
    profile: Mapping[str, Any]
    joint_count: int
    keyboard_topic: str
    can_interface: str
    requested_firmware_name: str
    firmware_name: str
    firmware: Any
    firmware_probe_timeout: float
    firmware_probe_poll_period: float
    firmware_reconnect_delay: float
    nero_mount: str
    tool_configuration: str
    nero_velocity_estimation_enabled: bool
    velocity_filter_time_constant: float
    control_rate: float
    step_rad: float
    speed_percent: int
    joint_max_acceleration: float
    joint_acc_timeout: float
    position_mode_timeout: float
    keyboard_timeout: float
    enable_timeout: float
    feedback_timeout: float
    move_home_on_start: bool
    startup_home_timeout: float
    startup_home_tolerance: float
    clear_errors_on_enable: bool
    reset_emergency_stop_on_start: bool
    emergency_reset_timeout: float
    execute_motion: bool
    disable_arm_on_shutdown: bool
    urdf_path: str
    base_frame: str
    tip_link: str
    cartesian_step: float
    ik_timeout: float
    ik_tolerance: float
    fk_position_tolerance: float
    fk_rotation_tolerance: float
    pointing_roll_samples: int
    orientation_step_rad: float
    workspace_min_radius: float
    workspace_max_radius: float
    ik_recovery_pause: float
    start_in_impedance: bool
    impedance_enabled: bool
    impedance_backend: str
    mit_command_rate: float
    interaction_safety: InteractionSafetyLimits
    dynamics_state_topic: str
    external_torque_topic: str
    control_sample_topic: str
    control_event_topic: str
    interaction_state_topic: str
    normal_mode_service_name: str
    impedance_mode_service_name: str
    admittance_mode_service_name: str

    @classmethod
    def from_node(cls, node):
        """Read and validate settings after parameters are declared."""

        def value(name):
            return node.get_parameter(name).value

        robot_model = str(value("robot_model")).lower()
        if robot_model not in MODEL_PROFILES:
            raise ValueError("robot_model must be nero or piper_l")
        profile = MODEL_PROFILES[robot_model]
        joint_count = int(profile["joint_count"])

        requested_firmware_name = str(value("firmware")).lower()
        firmware_name = resolve_firmware_name(
            robot_model, requested_firmware_name
        )
        firmwares = profile["firmwares"]
        firmware = firmwares.get(firmware_name)
        if firmware is None:
            raise ValueError(
                f"unsupported {robot_model} firmware {firmware_name!r}; "
                f"choose auto or one of {sorted(firmwares)}"
            )

        robot_min = float(value("robot_min_reach"))
        robot_max = float(value("robot_max_reach"))
        robot_min = profile["min_reach"] if robot_min < 0.0 else robot_min
        robot_max = profile["max_reach"] if robot_max < 0.0 else robot_max
        workspace_min_radius = robot_min + float(
            value("workspace_inner_margin")
        )
        workspace_max_radius = robot_max - float(
            value("workspace_outer_margin")
        )

        interaction_safety = InteractionSafetyLimits(
            torque_limit=expand_joint_values(
                value("interaction_torque_limit"),
                joint_count,
                "interaction_torque_limit",
            ),
            torque_rate_limit=expand_joint_values(
                value("interaction_torque_rate_limit"),
                joint_count,
                "interaction_torque_rate_limit",
            ),
            reference_velocity_limit=expand_joint_values(
                value("interaction_reference_joint_velocity_limit"),
                joint_count,
                "interaction_reference_joint_velocity_limit",
            ),
            measured_velocity_stop_limit=expand_joint_values(
                value("interaction_measured_joint_velocity_stop_limit"),
                joint_count,
                "interaction_measured_joint_velocity_stop_limit",
            ),
            measured_velocity_hard_limit=expand_joint_values(
                value("interaction_measured_joint_velocity_hard_limit"),
                joint_count,
                "interaction_measured_joint_velocity_hard_limit",
            ),
            measured_velocity_violation_cycles=int(
                value("interaction_measured_velocity_violation_cycles")
            ),
            joint_limit_margin=float(value("interaction_joint_limit_margin")),
        )

        settings = cls(
            robot_model=robot_model,
            profile=profile,
            joint_count=joint_count,
            keyboard_topic=str(value("keyboard_topic")),
            can_interface=str(value("can_interface")),
            requested_firmware_name=requested_firmware_name,
            firmware_name=firmware_name,
            firmware=firmware,
            firmware_probe_timeout=float(value("firmware_probe_timeout")),
            firmware_probe_poll_period=float(
                value("firmware_probe_poll_period")
            ),
            firmware_reconnect_delay=float(
                value("firmware_reconnect_delay")
            ),
            nero_mount=str(value("nero_mount")).strip().lower(),
            tool_configuration=(
                str(value("tool_configuration")).strip().lower()
            ),
            nero_velocity_estimation_enabled=bool(
                value("nero_velocity_estimation_enabled")
            ),
            velocity_filter_time_constant=float(
                value("velocity_filter_time_constant")
            ),
            control_rate=float(value("control_rate")),
            step_rad=float(value("step_rad")),
            speed_percent=int(value("speed_percent")),
            joint_max_acceleration=float(value("joint_max_acceleration")),
            joint_acc_timeout=float(value("joint_acc_timeout")),
            position_mode_timeout=float(value("position_mode_timeout")),
            keyboard_timeout=float(value("keyboard_timeout")),
            enable_timeout=float(value("enable_timeout")),
            feedback_timeout=float(value("feedback_timeout")),
            move_home_on_start=bool(value("move_home_on_start")),
            startup_home_timeout=float(value("startup_home_timeout")),
            startup_home_tolerance=float(value("startup_home_tolerance")),
            clear_errors_on_enable=bool(value("clear_errors_on_enable")),
            reset_emergency_stop_on_start=bool(
                value("reset_emergency_stop_on_start")
            ),
            emergency_reset_timeout=float(value("emergency_reset_timeout")),
            execute_motion=bool(value("execute_motion")),
            disable_arm_on_shutdown=bool(value("disable_arm_on_shutdown")),
            urdf_path=str(value("urdf_path")),
            base_frame=str(value("base_frame")),
            tip_link=str(value("tip_link")) or str(profile["tip_link"]),
            cartesian_step=float(value("cartesian_step")),
            ik_timeout=float(value("ik_timeout")),
            ik_tolerance=float(value("ik_tolerance")),
            fk_position_tolerance=float(value("fk_position_tolerance")),
            fk_rotation_tolerance=float(value("fk_rotation_tolerance")),
            pointing_roll_samples=int(value("pointing_roll_samples")),
            orientation_step_rad=float(value("orientation_step_rad")),
            workspace_min_radius=workspace_min_radius,
            workspace_max_radius=workspace_max_radius,
            ik_recovery_pause=float(value("ik_recovery_pause")),
            start_in_impedance=bool(value("impedance_enabled")),
            impedance_enabled=False,
            impedance_backend=str(value("impedance_backend")).lower(),
            mit_command_rate=float(value("mit_command_rate")),
            interaction_safety=interaction_safety,
            dynamics_state_topic=str(value("dynamics_state_topic")),
            external_torque_topic=str(value("external_torque_topic")),
            control_sample_topic=str(value("control_sample_topic")),
            control_event_topic=str(value("control_event_topic")),
            interaction_state_topic=str(value("interaction_state_topic")),
            normal_mode_service_name=str(value("normal_mode_service")),
            impedance_mode_service_name=str(value("impedance_mode_service")),
            admittance_mode_service_name=str(
                value("admittance_mode_service")
            ),
        )
        settings._validate()
        return settings

    def install_on(self, node):
        """Expose legacy node attributes while callers migrate to settings."""
        for field in fields(self):
            setattr(node, field.name, getattr(self, field.name))

    def _validate(self):
        if self.control_rate <= 0.0:
            raise ValueError("control_rate must be > 0")
        if (
            not math.isfinite(self.velocity_filter_time_constant)
            or self.velocity_filter_time_constant < 0.0
        ):
            raise ValueError(
                "velocity_filter_time_constant must be finite and >= 0"
            )
        if not 1 <= self.speed_percent <= 100:
            raise ValueError("speed_percent must be in [1, 100]")
        if self.joint_max_acceleration <= 0.0:
            raise ValueError("joint_max_acceleration must be > 0")
        if self.joint_acc_timeout <= 0.0:
            raise ValueError("joint_acc_timeout must be > 0")
        if self.position_mode_timeout <= 0.0:
            raise ValueError("position_mode_timeout must be > 0")
        if (
            not math.isfinite(self.firmware_probe_timeout)
            or self.firmware_probe_timeout <= 0.0
        ):
            raise ValueError("firmware_probe_timeout must be finite and > 0")
        if (
            not math.isfinite(self.firmware_probe_poll_period)
            or self.firmware_probe_poll_period < 0.0
        ):
            raise ValueError(
                "firmware_probe_poll_period must be finite and >= 0"
            )
        if (
            not math.isfinite(self.firmware_reconnect_delay)
            or self.firmware_reconnect_delay < 0.0
        ):
            raise ValueError(
                "firmware_reconnect_delay must be finite and >= 0"
            )
        if self.cartesian_step <= 0.0:
            raise ValueError("cartesian_step must be > 0")
        if self.orientation_step_rad <= 0.0:
            raise ValueError("orientation_step_rad must be > 0")
        if self.workspace_min_radius >= self.workspace_max_radius:
            raise ValueError("workspace margins leave no usable region")
        if not 1 <= self.pointing_roll_samples <= 72:
            raise ValueError("pointing_roll_samples must be in [1, 72]")
        if self.keyboard_timeout <= 0.0:
            raise ValueError("keyboard_timeout must be > 0")
        if self.startup_home_timeout <= 0.0:
            raise ValueError("startup_home_timeout must be > 0")
        if self.startup_home_tolerance <= 0.0:
            raise ValueError("startup_home_tolerance must be > 0")
        if self.emergency_reset_timeout <= 0.0:
            raise ValueError("emergency_reset_timeout must be > 0")
        if self.mit_command_rate <= 0.0:
            raise ValueError("MIT rate and maximum joint step must be > 0")
        if self.impedance_backend not in ("joint", "cartesian"):
            raise ValueError("impedance_backend must be joint or cartesian")
