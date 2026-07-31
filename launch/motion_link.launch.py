"""Launch pose control with the Motion Link phone bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PROFILES = {
    "nero": {
        "topic_prefix": "/nero",
        "tip_link": "link7",
        "initial_joint_positions": [0.0, 1.2, 0.0, 0.8, 0.0, 0.0, 0.0],
        "robot_min_reach": 0.1447354,
        "robot_max_reach": 0.7374482,
    },
    "piper_l": {
        "topic_prefix": "/piper_l",
        "tip_link": "link6",
        "initial_joint_positions": [
            0.0, 1.3939753, -1.0158306, 0.0, 1.2799181, 0.0
        ],
        "robot_min_reach": 0.0,
        "robot_max_reach": 0.8738043,
    },
}


def _parse_bool(value):
    """Parse one ROS launch Boolean without accepting ambiguous text."""
    normalized = str(value).strip().lower()
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"expected a Boolean value, got {value!r}")


def _nodes(context):
    """Create model-specific controller and bridge nodes."""
    model = LaunchConfiguration("robot_model").perform(context).lower()
    if model not in PROFILES:
        raise ValueError(f"robot_model must be one of {sorted(PROFILES)}")
    simulation_mode = _parse_bool(
        LaunchConfiguration("simulation_mode").perform(context)
    )
    execute_motion = _parse_bool(
        LaunchConfiguration("execute_motion").perform(context)
    )
    enable_commands = _parse_bool(
        LaunchConfiguration("enable_commands").perform(context)
    )
    if execute_motion and not enable_commands:
        raise ValueError(
            "execute_motion=true requires enable_commands=true"
        )
    profile = PROFILES[model]
    common = {
        "robot_model": model,
        "topic_prefix": profile["topic_prefix"],
    }
    controller_parameters = {
        **common,
        **profile,
        "simulation_mode": simulation_mode,
        "execute_motion": execute_motion,
        "can_interface": LaunchConfiguration("can_interface"),
        # The publisher remains 30 Hz. A 30 ms acceptance gate leaves
        # scheduler jitter below the nominal 33.3 ms sample interval.
        "command_period": 0.03,
        "state_period": 1.0 / 30.0,
        "pointing_axis_only": False,
        "workspace_limit_enabled": True,
        "workspace_inner_margin": 0.05,
        "workspace_outer_margin": 0.10,
    }
    bridge_parameters = {
        **common,
        "server_url": LaunchConfiguration("server_url"),
        "enable_commands": enable_commands,
        "publish_rate_hz": 30.0,
        "maximum_rotation_rad": LaunchConfiguration(
            "maximum_rotation_rad"
        ),
        "end_effector": LaunchConfiguration("end_effector"),
    }
    return [
        Node(
            package="armbycontroller",
            executable="pose_controller.py",
            name=f"{model}_pose_controller",
            output="screen",
            parameters=[controller_parameters],
        ),
        Node(
            package="armbycontroller",
            executable="motion_link_bridge.py",
            name=f"{model}_motion_link_bridge",
            output="screen",
            parameters=[bridge_parameters],
        ),
    ]


def generate_launch_description():
    """Declare safe defaults and launch phone-controlled pose handling."""
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="nero"),
        DeclareLaunchArgument(
            "server_url", default_value="http://127.0.0.1:8080"
        ),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("simulation_mode", default_value="true"),
        DeclareLaunchArgument("execute_motion", default_value="false"),
        DeclareLaunchArgument("enable_commands", default_value="false"),
        DeclareLaunchArgument(
            "maximum_rotation_rad", default_value="0.6"
        ),
        DeclareLaunchArgument(
            "end_effector", default_value="gripper"
        ),
        OpaqueFunction(function=_nodes),
    ])
