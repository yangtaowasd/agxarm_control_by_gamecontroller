"""Launch the profile-driven keyboard reader with a Piper or Nero controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


COMMON = {
    "can_interface": "can0",
    "firmware": "default",
    "execute_motion": "true",
}
NERO_STARTUP = {
    "move_home_on_start": "true",
    "startup_home_timeout": "30.0",
    "startup_home_tolerance": "0.01",
}
PROFILES = {
    "piper": "piper_keyboard_controller.py",
    "nero": "nero_joint_keyboard_controller.py",
}


def _nodes(context):
    profile = LaunchConfiguration("robot_model").perform(context).lower()
    if profile not in PROFILES:
        raise ValueError("robot_model must be piper or nero")
    controller = PROFILES[profile]
    parameter_names = list(COMMON)
    if profile == "nero":
        parameter_names.extend(NERO_STARTUP)
    return [
        Node(
            package="armbycontroller", executable="keyboard",
            name=f"{profile}_keyboard_reader", output="screen",
            parameters=[{
                "device": LaunchConfiguration("device"), "profile": profile,
            }],
        ),
        Node(
            package="armbycontroller", executable=controller,
            name=f"{profile}_keyboard_controller", output="screen",
            parameters=[{
                name: LaunchConfiguration(name) for name in parameter_names
            }],
        ),
    ]


def generate_launch_description():
    arguments = {"robot_model": "piper", "device": "/dev/input/event3"}
    arguments.update(COMMON)
    arguments.update(NERO_STARTUP)
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in arguments.items()
        ],
        OpaqueFunction(function=_nodes),
    ])
