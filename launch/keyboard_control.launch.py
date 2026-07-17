"""Launch one keyboard protocol for a Nero or Piper-L controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


COMMON = {
    "can_interface": "can0",
    "firmware": "auto",
    "execute_motion": "true",
    "impedance_enabled": "false",
    "mit_command_rate": "100.0",
    "mit_handover_duration": "0.5",
    "mit_kd_max": "0.6",
    "mit_damping_transition_velocity": "0.3",
    "mit_damping_torque_limit": "1.0",
    "joint_acc_timeout": "2.0",
    "position_mode_timeout": "2.0",
    "reset_emergency_stop_on_start": "true",
    "emergency_reset_timeout": "5.0",
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
    return [
        Node(
            package="armbycontroller", executable="keyboard",
            name="arm_keyboard_reader", output="screen",
            parameters=[{"device": LaunchConfiguration("device")}],
        ),
        Node(
            package="armbycontroller", executable="keyboard_controller.py",
            name="arm_keyboard_controller", output="screen",
            parameters=[{
                "robot_model": model,
                **{
                    name: LaunchConfiguration(name) for name in parameter_names
                },
            }],
        ),
    ]


def generate_launch_description():
    arguments = {"robot_model": "nero", "device": "/dev/input/event3"}
    arguments.update(COMMON)
    arguments.update(STARTUP)
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in arguments.items()
        ],
        OpaqueFunction(function=_nodes),
    ])
