"""Launch one keyboard protocol for a Nero or Piper-L controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from armbycontroller.screw_model import project_gravity_vector


COMMON = {
    "can_interface": "can0",
    "firmware": "auto",
    "execute_motion": "true",
    "impedance_enabled": "false",
    "impedance_backend": "cartesian",
    "control_rate": "100.0",
    "mit_command_rate": "100.0",
    "dynamics_state_topic": "/arm_dynamics_state",
    "external_torque_topic": "/arm_external_joint_torque",
    "mit_gravity_compensation_enabled": "true",
    "mit_gravity_scale": "1.0",
    "mit_gravity_torque_limit": "10.0",
    "cartesian_impedance_rotation_stiffness": "0.4",
    "cartesian_impedance_base_z_rotation_stiffness": "4.0",
    "cartesian_impedance_translation_stiffness": "10.0",
    "cartesian_impedance_rotation_damping": "0.08",
    "cartesian_impedance_translation_damping": "0.8",
    "cartesian_impedance_nullspace_stiffness": "0.4",
    "cartesian_impedance_nullspace_damping": "0.1",
    "cartesian_impedance_torque_limit": "8.0",
    "admittance_virtual_mass": "[0.12, 0.12, 0.12, 1.5, 1.5, 1.5]",
    "admittance_damping": "[0.8, 0.8, 0.8, 8.0, 8.0, 8.0]",
    "admittance_stiffness": "[0.8, 0.8, 0.8, 8.0, 8.0, 8.0]",
    "admittance_wrench_deadband": "[0.03, 0.03, 0.03, 0.15, 0.15, 0.15]",
    "admittance_wrench_limit": "[2.0, 2.0, 2.0, 8.0, 8.0, 8.0]",
    "admittance_offset_limit": "[0.35, 0.35, 0.35, 0.10, 0.10, 0.10]",
    "admittance_velocity_limit": "[0.5, 0.5, 0.5, 0.15, 0.15, 0.15]",
    "admittance_wrench_filter_hz": "5.0",
    "admittance_wrench_dls_damping": "0.05",
    "admittance_wrench_timeout": "0.10",
    "mit_trajectory_max_velocity": "0.5",
    "mit_trajectory_max_acceleration": "1.0",
    "mit_trajectory_max_jerk": "5.0",
    "joint_acc_timeout": "2.0",
    "position_mode_timeout": "2.0",
    "reset_emergency_stop_on_start": "true",
    "emergency_reset_timeout": "5.0",
}
OBSERVER = {
    "momentum_observer_enabled": "true",
    "momentum_observer_rate": "100.0",
    "momentum_observer_gain": "10.0",
    "momentum_observer_max_period": "0.05",
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


def _nodes(context):
    model = LaunchConfiguration("robot_model").perform(context).lower()
    if model not in MODELS:
        raise ValueError("robot_model must be nero or piper_l")
    parameter_names = [*COMMON, *STARTUP]
    controller_parameters = {
        "robot_model": model,
        **{name: LaunchConfiguration(name) for name in parameter_names},
    }
    if model == "nero":
        mount = LaunchConfiguration("nero_mount").perform(context).lower()
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
            parameters=[controller_parameters],
        ),
    ]
    observer_enabled = (
        LaunchConfiguration("momentum_observer_enabled")
        .perform(context).lower() in ("true", "1", "yes", "on")
    )
    if observer_enabled:
        observer_parameters = {
            "robot_model": model,
            "gravity_vector": controller_parameters.get(
                "gravity_vector", [0.0, 0.0, -9.80665]
            ),
            "dynamics_state_topic": LaunchConfiguration(
                "dynamics_state_topic"
            ),
            "external_torque_topic": LaunchConfiguration(
                "external_torque_topic"
            ),
            **{
                name: LaunchConfiguration(name)
                for name in OBSERVER if name != "momentum_observer_enabled"
            },
        }
        for name in OPTIONAL:
            if LaunchConfiguration(name).perform(context).strip():
                observer_parameters[name] = LaunchConfiguration(name)
        nodes.append(Node(
            package="armbycontroller",
            executable="momentum_observer_node.py",
            name="arm_momentum_observer",
            output="screen",
            parameters=[observer_parameters],
        ))
    return nodes


def generate_launch_description():
    arguments = {
        "robot_model": "nero",
        "device": "/dev/input/event3",
        "nero_mount": "select",
    }
    arguments.update(COMMON)
    arguments.update(STARTUP)
    arguments.update(OPTIONAL)
    arguments.update(OBSERVER)
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in arguments.items()
        ],
        OpaqueFunction(function=_nodes),
    ])
