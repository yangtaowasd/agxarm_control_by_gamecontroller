"""Launch one keyboard protocol for a Nero or Piper-L controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from armbycontroller.gravity_compensation import nero_mount_gravity


COMMON = {
    "can_interface": "can0",
    "firmware": "auto",
    "execute_motion": "true",
    "impedance_enabled": "false",
    "control_rate": "100.0",
    "mit_command_rate": "100.0",
    "mit_gravity_compensation_enabled": "true",
    "mit_gravity_scale": "1.0",
    "mit_gravity_torque_limit": "10.0",
    "cartesian_stiffness": "[4.0, 4.0, 4.0, 80.0, 80.0, 80.0]",
    "cartesian_damping": "[1.5, 1.5, 1.5, 12.0, 12.0, 12.0]",
    "cartesian_nullspace_stiffness": "0.5",
    "cartesian_nullspace_damping": "0.1",
    "joint_acc_timeout": "2.0",
    "position_mode_timeout": "2.0",
    "reset_emergency_stop_on_start": "true",
    "emergency_reset_timeout": "5.0",
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
        controller_parameters["gravity_vector"] = nero_mount_gravity(mount)
    for name in OPTIONAL:
        if LaunchConfiguration(name).perform(context).strip():
            controller_parameters[name] = LaunchConfiguration(name)
    return [
        Node(
            package="armbycontroller", executable="keyboard",
            name="arm_keyboard_reader", output="screen",
            parameters=[{"device": LaunchConfiguration("device")}],
        ),
        Node(
            package="armbycontroller", executable="keyboard_controller.py",
            name="arm_keyboard_controller", output="screen",
            parameters=[controller_parameters],
        ),
    ]


def generate_launch_description():
    arguments = {
        "robot_model": "nero",
        "device": "/dev/input/event3",
        "nero_mount": "select",
    }
    arguments.update(COMMON)
    arguments.update(STARTUP)
    arguments.update(OPTIONAL)
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in arguments.items()
        ],
        OpaqueFunction(function=_nodes),
    ])
