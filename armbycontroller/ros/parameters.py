"""Robot profiles and ROS parameter declarations for the main arm node."""

import numpy as np
from rcl_interfaces.msg import ParameterDescriptor

from pyAgxArm import ArmModel, NeroFW, PiperFW


MODEL_PROFILES = {
    "nero": {
        "arm_model": ArmModel.NERO,
        "joint_count": 7,
        "tip_link": "link7",
        "min_reach": 0.1447354,
        "max_reach": 0.7374482,
        "firmwares": {
            "default": NeroFW.DEFAULT,
            "v111": NeroFW.V111,
            "v112": NeroFW.V112,
            "v120": NeroFW.V120,
        },
    },
    "piper_l": {
        "arm_model": ArmModel.PIPER_L,
        "joint_count": 6,
        "tip_link": "link6",
        "min_reach": 0.0,
        "max_reach": 0.8738043,
        "firmwares": {
            "default": PiperFW.DEFAULT,
            "v183": PiperFW.V183,
            "v188": PiperFW.V188,
            "v189": PiperFW.V189,
        },
    },
}

MIT_GAIN_PROFILES = {
    "nero": {
        "kp": [1.0] * 7,
        "kd": [0.2] * 7,
        "t_ff": [0.0] * 7,
    },
    "piper_l": {
        "kp": [0.3, 0.5, 0.5, 0.5, 1.0, 0.3],
        "kd": [0.01, 0.01, 0.01, 0.01, 0.01, 0.01],
        # Dynamic URDF inverse dynamics supplies the nominal torque.
        "t_ff": [0.0] * 6,
    },
}

ADMITTANCE_MIT_GAIN_PROFILES = {
    "nero": {
        "kp": [0.32, 0.24, 0.32, 0.24, 0.32, 0.28, 0.28],
        "kd": [0.08] * 7,
    },
    "piper_l": {
        "kp": [0.3, 0.5, 0.5, 0.5, 1.0, 0.3],
        "kd": [0.01] * 6,
    },
}


def default_mit_gains(robot_model):
    """Return independent per-joint MIT gains for a supported arm."""
    profile = MIT_GAIN_PROFILES[robot_model]
    return list(profile["kp"]), list(profile["kd"])


def default_mit_feedforward(robot_model):
    """Return per-joint MIT feedforward torque for a supported arm."""
    return list(MIT_GAIN_PROFILES[robot_model]["t_ff"])


def expand_joint_values(values, joint_count, name):
    """Expand one value to all joints or validate a per-joint list."""
    if np.isscalar(values):
        result = [float(values)]
    else:
        result = [float(value) for value in values]
    if len(result) == 1:
        result *= joint_count
    if len(result) != joint_count or not np.all(np.isfinite(result)):
        raise ValueError(
            f"{name} must contain 1 or {joint_count} finite values"
        )
    return result


def declare_controller_parameters(node):
    """Declare the complete parameter surface without starting hardware."""
    node.declare_parameter("robot_model", "nero")
    declared_model = str(node.get_parameter("robot_model").value).lower()
    gain_model = (
        declared_model if declared_model in MODEL_PROFILES else "nero"
    )
    default_mit_kp, default_mit_kd = default_mit_gains(gain_model)
    default_mit_feedforward_values = default_mit_feedforward(gain_model)
    admittance_mit_profile = ADMITTANCE_MIT_GAIN_PROFILES[gain_model]

    node.declare_parameter("keyboard_topic", "/arm_keyboard_state")
    node.declare_parameter("can_interface", "can0")
    node.declare_parameter("firmware", "auto")
    node.declare_parameter("firmware_probe_timeout", 5.0)
    node.declare_parameter("firmware_probe_poll_period", 0.1)
    node.declare_parameter("firmware_reconnect_delay", 0.5)
    node.declare_parameter("nero_mount", "")
    node.declare_parameter("tool_configuration", "auto")
    node.declare_parameter("nero_velocity_estimation_enabled", True)
    node.declare_parameter("velocity_filter_time_constant", 0.03)
    node.declare_parameter("control_rate", 100.0)
    node.declare_parameter("step_rad", 0.005)
    node.declare_parameter("speed_percent", 40)
    node.declare_parameter("joint_max_acceleration", 1.0)
    node.declare_parameter("joint_acc_timeout", 2.0)
    node.declare_parameter("position_mode_timeout", 2.0)
    node.declare_parameter("keyboard_timeout", 0.3)
    node.declare_parameter("enable_timeout", 5.0)
    node.declare_parameter("feedback_timeout", 3.0)
    node.declare_parameter("move_home_on_start", False)
    node.declare_parameter("startup_home_timeout", 30.0)
    node.declare_parameter("startup_home_tolerance", 0.01)
    node.declare_parameter("clear_errors_on_enable", True)
    node.declare_parameter("reset_emergency_stop_on_start", False)
    node.declare_parameter("emergency_reset_timeout", 5.0)
    node.declare_parameter("execute_motion", True)
    node.declare_parameter("disable_arm_on_shutdown", False)
    node.declare_parameter("urdf_path", "")
    node.declare_parameter("base_frame", "base_link")
    node.declare_parameter("tip_link", "")
    node.declare_parameter("cartesian_step", 0.005)
    node.declare_parameter("ik_timeout", 0.01)
    node.declare_parameter("ik_tolerance", 1e-5)
    node.declare_parameter("fk_position_tolerance", 1e-4)
    node.declare_parameter("fk_rotation_tolerance", 1e-3)
    node.declare_parameter("pointing_roll_samples", 8)
    node.declare_parameter("orientation_step_rad", 0.02)
    node.declare_parameter("robot_min_reach", -1.0)
    node.declare_parameter("robot_max_reach", -1.0)
    node.declare_parameter("workspace_inner_margin", 0.05)
    node.declare_parameter("workspace_outer_margin", 0.10)
    node.declare_parameter("ik_recovery_pause", 2.0)
    node.declare_parameter("impedance_enabled", False)
    node.declare_parameter("impedance_backend", "cartesian")
    node.declare_parameter("mit_command_rate", 100.0)

    scalar_or_array = ParameterDescriptor(dynamic_typing=True)
    node.declare_parameter(
        "interaction_torque_limit", [8.0], scalar_or_array
    )
    node.declare_parameter(
        "interaction_torque_rate_limit", [20.0], scalar_or_array
    )
    node.declare_parameter(
        "interaction_reference_joint_velocity_limit",
        [1.0],
        scalar_or_array,
    )
    node.declare_parameter(
        "interaction_measured_joint_velocity_stop_limit",
        [1.5],
        scalar_or_array,
    )
    node.declare_parameter(
        "interaction_measured_joint_velocity_hard_limit",
        [2.5],
        scalar_or_array,
    )
    node.declare_parameter(
        "interaction_measured_velocity_violation_cycles", 3
    )
    node.declare_parameter("interaction_joint_limit_margin", 0.03)
    node.declare_parameter("dynamics_state_topic", "/arm_dynamics_state")
    node.declare_parameter(
        "external_torque_topic", "/arm_external_joint_torque"
    )
    node.declare_parameter("control_sample_topic", "/arm_control_sample")
    node.declare_parameter("control_event_topic", "/arm_control_event")
    node.declare_parameter(
        "interaction_state_topic", "/arm/interaction_state"
    )
    node.declare_parameter("normal_mode_service", "/arm/set_normal_mode")
    node.declare_parameter(
        "impedance_mode_service", "/arm/set_impedance_mode"
    )
    node.declare_parameter(
        "admittance_mode_service", "/arm/set_admittance_mode"
    )
    node.declare_parameter("mit_kp", default_mit_kp)
    node.declare_parameter("mit_kd", default_mit_kd)
    node.declare_parameter("mit_feedforward", default_mit_feedforward_values)
    node.declare_parameter("mit_gravity_compensation_enabled", True)
    node.declare_parameter("gravity_urdf_path", "")
    node.declare_parameter("mit_gravity_scale", 1.0)
    node.declare_parameter(
        "nero_horizontal_gravity_schedule_enabled", True
    )
    node.declare_parameter(
        "nero_horizontal_gravity_transition_angle", 0.03490658503988659
    )
    node.declare_parameter(
        "nero_horizontal_gravity_j2_scale", [1.0, 1.0]
    )
    node.declare_parameter(
        "nero_horizontal_gravity_j2_bias_nm", [0.0, 0.0]
    )
    node.declare_parameter(
        "nero_horizontal_gravity_j4_scale", [1.0, 1.0]
    )
    node.declare_parameter(
        "nero_horizontal_gravity_j4_bias_nm", [0.0, 0.0]
    )
    node.declare_parameter("cartesian_impedance_rotation_stiffness", 0.4)
    node.declare_parameter(
        "cartesian_impedance_base_z_rotation_stiffness", 4.0
    )
    node.declare_parameter("cartesian_impedance_translation_stiffness", 10.0)
    node.declare_parameter("cartesian_impedance_rotation_damping", 0.08)
    node.declare_parameter("cartesian_impedance_translation_damping", 0.8)
    node.declare_parameter("cartesian_impedance_max_force", 10.0)
    node.declare_parameter("cartesian_impedance_max_torque", 4.0)
    node.declare_parameter(
        "cartesian_impedance_position_integral_gain",
        [0.0],
        scalar_or_array,
    )
    node.declare_parameter(
        "cartesian_impedance_position_integral_deadband",
        [0.0],
        scalar_or_array,
    )
    node.declare_parameter(
        "cartesian_impedance_position_integral_max_rotation_error", 0.05
    )
    node.declare_parameter(
        "cartesian_impedance_position_integral_max_translation_error", 0.02
    )
    node.declare_parameter(
        "cartesian_impedance_position_integral_max_force", 0.75
    )
    node.declare_parameter(
        "cartesian_impedance_position_integral_max_torque", 0.2
    )
    node.declare_parameter(
        "cartesian_impedance_position_integral_leak_rate", 0.05
    )
    node.declare_parameter(
        "cartesian_impedance_position_integral_saturation_leak_rate", 0.1
    )
    node.declare_parameter(
        "cartesian_impedance_position_integral_external_force_gate", 1.0
    )
    node.declare_parameter(
        "cartesian_impedance_position_integral_external_force_release", 0.5
    )
    node.declare_parameter(
        "cartesian_impedance_position_integral_external_torque_gate", 0.2
    )
    node.declare_parameter(
        "cartesian_impedance_position_integral_external_torque_release", 0.1
    )
    node.declare_parameter(
        "cartesian_impedance_position_integral_requires_external_wrench",
        True,
    )
    node.declare_parameter(
        "cartesian_impedance_nullspace_stiffness",
        [0.4],
        scalar_or_array,
    )
    node.declare_parameter(
        "cartesian_impedance_nullspace_damping",
        [0.1],
        scalar_or_array,
    )
    node.declare_parameter(
        "cartesian_impedance_joint_posture_stiffness",
        [0.0],
        scalar_or_array,
    )
    node.declare_parameter(
        "cartesian_impedance_joint_posture_damping",
        [0.0],
        scalar_or_array,
    )
    node.declare_parameter(
        "cartesian_impedance_model_scale", [1.0], scalar_or_array
    )
    node.declare_parameter(
        "admittance_virtual_mass", [0.2, 0.2, 0.2, 2.0, 2.0, 2.0]
    )
    node.declare_parameter("admittance_mode", "zero_force")
    node.declare_parameter(
        "admittance_zero_force_damping",
        [2.0, 2.0, 2.0, 18.0, 18.0, 18.0],
    )
    node.declare_parameter(
        "admittance_zero_force_holding_stiffness",
        [0.25, 0.25, 0.25, 5.0, 5.0, 5.0],
    )
    node.declare_parameter(
        "admittance_zero_force_friction",
        [0.03, 0.03, 0.03, 0.35, 0.35, 0.35],
    )
    node.declare_parameter(
        "admittance_zero_force_stiction_velocity",
        [0.015, 0.015, 0.015, 0.005, 0.005, 0.005],
    )
    node.declare_parameter(
        "admittance_resistive_damping",
        [2.5, 2.5, 2.5, 25.0, 25.0, 25.0],
    )
    node.declare_parameter(
        "admittance_resistive_stiffness",
        [0.8, 0.8, 0.8, 12.0, 12.0, 12.0],
    )
    node.declare_parameter(
        "admittance_wrench_deadband",
        [0.05, 0.05, 0.05, 0.25, 0.25, 0.25],
    )
    node.declare_parameter(
        "admittance_wrench_limit", [1.5, 1.5, 1.5, 6.0, 6.0, 6.0]
    )
    node.declare_parameter(
        "admittance_offset_limit",
        [0.25, 0.25, 0.25, 0.08, 0.08, 0.08],
    )
    node.declare_parameter(
        "admittance_velocity_limit",
        [0.12, 0.12, 0.12, 0.05, 0.05, 0.05],
    )
    node.declare_parameter("admittance_wrench_filter_hz", 5.0)
    node.declare_parameter("admittance_wrench_dls_damping", 0.05)
    node.declare_parameter("admittance_wrench_timeout", 0.10)
    node.declare_parameter("admittance_mit_kp", admittance_mit_profile["kp"])
    node.declare_parameter("admittance_mit_kd", admittance_mit_profile["kd"])
    node.declare_parameter("admittance_mit_model_scale", 1.0)
    node.declare_parameter(
        "admittance_task_weights", [0.4, 0.4, 0.4, 1.0, 1.0, 1.0]
    )
    node.declare_parameter("admittance_velocity_dls_damping", 0.02)
    node.declare_parameter(
        "admittance_singularity_slow_threshold", 0.05
    )
    node.declare_parameter(
        "admittance_singularity_stop_threshold", 0.01
    )
    node.declare_parameter("admittance_singularity_damping", 0.08)
    node.declare_parameter("hybrid_admittance_axes", "z")
    node.declare_parameter("hybrid_admittance_frame", "base")
    node.declare_parameter(
        "hybrid_admittance_frame_rotation", [0.0, 0.0, 0.0]
    )
    node.declare_parameter("hybrid_desired_wrench", [0.0] * 6)
    node.declare_parameter("gravity_vector", [0.0, 0.0, -9.80665])
    node.declare_parameter(
        "mit_trajectory_max_acceleration", [1.0], scalar_or_array
    )
    node.declare_parameter(
        "mit_trajectory_max_jerk", [5.0], scalar_or_array
    )
    node.declare_parameter("mit_max_joint_step", 0.05)
