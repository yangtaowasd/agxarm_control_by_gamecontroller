from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LAUNCH_ARGUMENTS = {
    "device": "/dev/input/event3",
    "can_interface": "can0",
    "firmware": "default",
    "step_rad": "0.01",
    "arrow_mode": "orientation",
    "orientation_mapping": "square",
    "orientation_square_limit": "1.0",
    "orientation_square_step": "0.02",
    "orientation_square_max_tilt_deg": "80.0",
    "joint4_limit_deg": "127.0",
    "joint5_limit_deg": "89.5",
    "joint6_limit_deg": "170.0",
    "orientation_ik_damping": "0.2",
    "orientation_ik_max_iterations": "40",
    "orientation_ik_max_joint_step": "0.02",
    "orientation_ik_fd_eps": "0.001",
    "orientation_ik_line_search_steps": "6",
    "orientation_ik_tolerance": "0.002",
    "orientation_ik_refine_iterations": "10",
    "orientation_ik_refine_damping": "0.05",
    "orientation_ik_coordinate_iterations": "8",
    "orientation_j5_limit_deg": "80.0",
    "orientation_yaw_joint_step_scale": "2.0",
    "orientation_singularity_yaw_boost": "2.0",
    "avoid_j5_zero_deg": "0.0",
    "speed_percent": "40",
    "j4_j6_speed_percent": "80",
    "j3_correction_step_rad": "0.01",
    "j3_correction_limit_deg": "10.0",
    "enable_timeout": "5.0",
    "clear_errors_on_enable": "true",
    "shutdown_j5_deg": "30.0",
    "shutdown_home_tolerance": "0.0001",
    "shutdown_home_poll_interval": "0.05",
    "execute_motion": "true",
}


def generate_launch_description():
    config = {
        name: LaunchConfiguration(name)
        for name in LAUNCH_ARGUMENTS
    }
    declarations = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in LAUNCH_ARGUMENTS.items()
    ]
    controller_parameters = {
        name: value
        for name, value in config.items()
        if name != "device"
    }

    return LaunchDescription(
        [
            *declarations,
            Node(
                package="armbycontroller",
                executable="keyboard",
                name="keyboard_reader",
                output="screen",
                parameters=[{"device": config["device"]}],
            ),
            Node(
                package="armbycontroller",
                executable="piper_keyboard_controller.py",
                name="piper_keyboard_controller",
                output="screen",
                parameters=[controller_parameters],
            ),
        ]
    )
