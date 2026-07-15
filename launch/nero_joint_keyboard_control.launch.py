from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = {
    "device": "/dev/input/event3",
    "can_interface": "can0",
    "firmware": "default",
    "control_rate": "20.0",
    "step_rad": "0.005",
    "speed_percent": "20",
    "keyboard_timeout": "0.3",
    "enable_timeout": "5.0",
    "feedback_timeout": "3.0",
    "move_home_on_start": "true",
    "startup_home_timeout": "30.0",
    "startup_home_tolerance": "0.01",
    "clear_errors_on_enable": "true",
    "execute_motion": "true",
}


def generate_launch_description():
    values = {name: LaunchConfiguration(name) for name in ARGUMENTS}
    declarations = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in ARGUMENTS.items()
    ]
    controller_parameters = {
        name: value for name, value in values.items() if name != "device"
    }
    return LaunchDescription(
        [
            *declarations,
            Node(
                package="armbycontroller",
                executable="nero_keyboard",
                name="nero_keyboard_reader",
                output="screen",
                parameters=[{"device": values["device"]}],
            ),
            Node(
                package="armbycontroller",
                executable="nero_joint_keyboard_controller.py",
                name="nero_joint_keyboard_controller",
                output="screen",
                parameters=[controller_parameters],
            ),
        ]
    )
