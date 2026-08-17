"""Launch one keyboard protocol for a Nero or Piper-L controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from armbycontroller.modeling.screw_model import project_gravity_vector


COMMON = {
    "can_interface": "can0",
    "execute_motion": "true",
    "dynamics_state_topic": "/arm_dynamics_state",
    "external_torque_topic": "/arm_external_joint_torque",
    "control_sample_topic": "/arm_control_sample",
    "control_event_topic": "/arm_control_event",
    "joint_acc_timeout": "2.0",
    "position_mode_timeout": "2.0",
    "reset_emergency_stop_on_start": "true",
    "emergency_reset_timeout": "5.0",
}
OBSERVER = {
    "momentum_observer_enabled": "true",
}
EXPERIMENT = {
    "experiment_recording_enabled": "false",
    "experiment_output_directory": "~/.ros/armbycontroller/experiments",
    "experiment_name": "manual_control",
    "experiment_flush_every": "1",
}
OPTIONAL = {
    "urdf_path": "",
    "gravity_urdf_path": "",
}
STARTUP = {
    "move_home_on_start": "true",
    "startup_home_timeout": "30.0",
    "startup_home_tolerance": "0.01",
}
MODELS = ("nero", "piper_l")
USE_CONFIG = "__config__"
USE_ROBOT_CONFIG = "__robot__"
ROBOT_CONFIG_FILENAMES = {
    "nero": "nero.yaml",
    "piper_l": "piper_l.yaml",
}
CONFIGURED_PARAMETERS = {
    "firmware",
    "firmware_probe_timeout",
    "firmware_probe_poll_period",
    "firmware_reconnect_delay",
    "impedance_enabled",
    "impedance_backend",
    "control_rate",
    "mit_command_rate",
    "mit_kp",
    "mit_kd",
    "mit_feedforward",
    "mit_gravity_compensation_enabled",
    "mit_gravity_scale",
    "mit_gravity_torque_limit",
    "cartesian_impedance_rotation_stiffness",
    "cartesian_impedance_base_z_rotation_stiffness",
    "cartesian_impedance_translation_stiffness",
    "cartesian_impedance_rotation_damping",
    "cartesian_impedance_translation_damping",
    "cartesian_impedance_nullspace_stiffness",
    "cartesian_impedance_nullspace_damping",
    "cartesian_impedance_joint_posture_stiffness",
    "cartesian_impedance_joint_posture_damping",
    "cartesian_impedance_torque_limit",
    "cartesian_impedance_model_scale",
    "admittance_mode",
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
    "admittance_wrench_filter_hz",
    "admittance_wrench_dls_damping",
    "admittance_wrench_timeout",
    "mit_trajectory_max_velocity",
    "mit_trajectory_max_acceleration",
    "mit_trajectory_max_jerk",
    "nero_mount",
    "tool_configuration",
    "nero_velocity_estimation_enabled",
    "velocity_filter_time_constant",
}
CONFIGURED_OBSERVER_PARAMETERS = {
    "momentum_observer_rate",
    "momentum_observer_gain",
    "momentum_observer_max_period",
}


def _nodes(context):
    model = LaunchConfiguration("robot_model").perform(context).lower()
    if model not in MODELS:
        raise ValueError("robot_model must be nero or piper_l")
    common_config_path = LaunchConfiguration("common_config").perform(
        context
    ).strip()
    robot_config_path = LaunchConfiguration("controller_config").perform(
        context
    ).strip()
    if robot_config_path == USE_ROBOT_CONFIG:
        robot_config_path = PathJoinSubstitution([
            FindPackageShare("armbycontroller"),
            "config",
            ROBOT_CONFIG_FILENAMES[model],
        ]).perform(context)
    parameter_files = [common_config_path, robot_config_path]
    parameter_names = [*COMMON, *STARTUP, *CONFIGURED_PARAMETERS]
    controller_parameters = {
        "robot_model": model,
    }
    for name in parameter_names:
        value = LaunchConfiguration(name).perform(context)
        if value != USE_CONFIG:
            controller_parameters[name] = LaunchConfiguration(name)
    mount = LaunchConfiguration("nero_mount").perform(context).lower()
    if model == "nero" and mount != USE_CONFIG:
        try:
            gravity = project_gravity_vector(mount)
        except ValueError as error:
            raise ValueError(
                "nero_mount must be horizontal or side"
            ) from error
        controller_parameters["gravity_vector"] = list(gravity)
    for name in OPTIONAL:
        if LaunchConfiguration(name).perform(context).strip():
            controller_parameters[name] = LaunchConfiguration(name)
    nodes = [
        Node(
            package="armbycontroller", executable="keyboard",
            name="arm_keyboard_reader", output="screen",
            parameters=[{"device": LaunchConfiguration("device")}],
        ),
        Node(
            package="armbycontroller",
            executable="keyboard_controller_node.py",
            name="arm_keyboard_controller", output="screen",
            parameters=[*parameter_files, controller_parameters],
        ),
    ]
    observer_enabled = (
        LaunchConfiguration("momentum_observer_enabled")
        .perform(context).lower() in ("true", "1", "yes", "on")
    )
    if observer_enabled:
        observer_parameters = {
            "robot_model": model,
            "dynamics_state_topic": LaunchConfiguration(
                "dynamics_state_topic"
            ),
            "external_torque_topic": LaunchConfiguration(
                "external_torque_topic"
            ),
        }
        for name in CONFIGURED_OBSERVER_PARAMETERS:
            value = LaunchConfiguration(name).perform(context)
            if value != USE_CONFIG:
                observer_parameters[name] = LaunchConfiguration(name)
        if "gravity_vector" in controller_parameters:
            observer_parameters["gravity_vector"] = (
                controller_parameters["gravity_vector"]
            )
        if model == "nero" and mount != USE_CONFIG:
            observer_parameters["nero_mount"] = mount
            observer_parameters["gravity_vector"] = list(gravity)
        for name in OPTIONAL:
            if LaunchConfiguration(name).perform(context).strip():
                observer_parameters[name] = LaunchConfiguration(name)
        nodes.append(Node(
            package="armbycontroller",
            executable="momentum_observer_node.py",
            name="arm_momentum_observer",
            output="screen",
            parameters=[*parameter_files, observer_parameters],
        ))
    experiment_enabled = (
        LaunchConfiguration("experiment_recording_enabled")
        .perform(context).lower() in ("true", "1", "yes", "on")
    )
    if experiment_enabled:
        nodes.append(Node(
            package="armbycontroller",
            executable="experiment_recorder_node.py",
            name="arm_experiment_recorder",
            output="screen",
            parameters=[{
                "sample_topic": LaunchConfiguration(
                    "control_sample_topic"
                ),
                "event_topic": LaunchConfiguration(
                    "control_event_topic"
                ),
                "output_directory": LaunchConfiguration(
                    "experiment_output_directory"
                ),
                "experiment_name": LaunchConfiguration("experiment_name"),
                "robot_model": model,
                "start_on_launch": True,
                "flush_every": LaunchConfiguration(
                    "experiment_flush_every"
                ),
            }],
        ))
    return nodes


def generate_launch_description():
    arguments = {
        "robot_model": "nero",
        "device": "/dev/input/event3",
        "common_config": PathJoinSubstitution([
            FindPackageShare("armbycontroller"),
            "config",
            "common.yaml",
        ]),
        "controller_config": USE_ROBOT_CONFIG,
    }
    arguments.update(COMMON)
    arguments.update(STARTUP)
    arguments.update(OPTIONAL)
    arguments.update(OBSERVER)
    arguments.update(EXPERIMENT)
    for name in CONFIGURED_PARAMETERS:
        arguments[name] = USE_CONFIG
    for name in CONFIGURED_OBSERVER_PARAMETERS:
        arguments[name] = USE_CONFIG
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in arguments.items()
        ],
        OpaqueFunction(function=_nodes),
    ])
