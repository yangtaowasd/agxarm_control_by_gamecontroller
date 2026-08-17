"""Launch unified web/phone keyboard control with live state feedback."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from armbycontroller.model_profiles import get_arm_profile


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
    profile = get_arm_profile(model)
    simulation_mode = _parse_bool(
        LaunchConfiguration("simulation_mode").perform(context)
    )
    execute_motion = _parse_bool(
        LaunchConfiguration("execute_motion").perform(context)
    )
    enable_commands = _parse_bool(
        LaunchConfiguration("enable_commands").perform(context)
    )
    backend_api_enabled = _parse_bool(
        LaunchConfiguration("backend_api_enabled").perform(context)
    )
    backend_transport = LaunchConfiguration(
        "backend_transport"
    ).perform(context).strip().lower()
    if backend_transport not in ("motion_link", "http", "both"):
        raise ValueError(
            "backend_transport must be motion_link, http or both"
        )
    if backend_transport in ("http", "both") and not backend_api_enabled:
        raise ValueError(
            "HTTP command transport requires backend_api_enabled=true"
        )
    if execute_motion and not enable_commands:
        raise ValueError(
            "execute_motion=true requires enable_commands=true"
        )
    common = {
        "robot_model": model,
        "topic_prefix": profile.topic_prefix,
    }
    controller_parameters = {
        "robot_model": model,
        "simulation_mode": simulation_mode,
        "execute_motion": execute_motion,
        "can_interface": LaunchConfiguration("can_interface"),
        "initial_joint_positions": list(profile.initial_joint_positions),
        "firmware": LaunchConfiguration("firmware"),
        "firmware_probe_timeout": LaunchConfiguration(
            "firmware_probe_timeout"
        ),
        "firmware_probe_poll_period": LaunchConfiguration(
            "firmware_probe_poll_period"
        ),
        "firmware_reconnect_delay": LaunchConfiguration(
            "firmware_reconnect_delay"
        ),
        # The publisher remains 30 Hz. A 30 ms acceptance gate leaves
        # scheduler jitter below the nominal 33.3 ms sample interval.
        "command_period": 0.03,
        "state_period": 1.0 / 30.0,
        "robot_min_reach": profile.min_reach,
        "robot_max_reach": profile.max_reach,
        "workspace_inner_margin": 0.05,
        "workspace_outer_margin": 0.10,
        "move_home_on_start": False,
        "reset_emergency_stop_on_start": False,
    }
    bridge_parameters = {
        **common,
        "server_url": LaunchConfiguration("server_url"),
        "enable_commands": (
            enable_commands
            and backend_transport in ("motion_link", "both")
        ),
        "simulation_mode": simulation_mode,
        "publish_rate_hz": 30.0,
        "maximum_rotation_rad": LaunchConfiguration(
            "maximum_rotation_rad"
        ),
        "end_effector": LaunchConfiguration("end_effector"),
    }
    nodes = [
        Node(
            package="armbycontroller",
            executable="keyboard_controller.py",
            name=f"{model}_keyboard_controller",
            output="screen",
            parameters=[controller_parameters],
        ),
    ]
    if backend_transport in ("motion_link", "both"):
        nodes.append(Node(
            package="armbycontroller",
            executable="motion_link_bridge.py",
            name=f"{model}_motion_link_bridge",
            output="screen",
            parameters=[bridge_parameters],
        ))
    if backend_api_enabled:
        nodes.append(
            Node(
                package="armbycontroller",
                executable="backend_api.py",
                name=f"{model}_backend_api",
                output="screen",
                parameters=[{
                    "robot_model": model,
                    "api_host": LaunchConfiguration("backend_api_host"),
                    "api_port": LaunchConfiguration("backend_api_port"),
                    "api_token": LaunchConfiguration("backend_api_token"),
                    "enable_commands": (
                        enable_commands
                        and backend_transport in ("http", "both")
                    ),
                }],
            )
        )
    return nodes


def generate_launch_description():
    """Declare safe defaults and launch phone-controlled pose handling."""
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="nero"),
        DeclareLaunchArgument(
            "server_url", default_value="http://127.0.0.1:8080"
        ),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("firmware", default_value="auto"),
        DeclareLaunchArgument(
            "firmware_probe_timeout", default_value="5.0"
        ),
        DeclareLaunchArgument(
            "firmware_probe_poll_period", default_value="0.1"
        ),
        DeclareLaunchArgument(
            "firmware_reconnect_delay", default_value="0.5"
        ),
        DeclareLaunchArgument("simulation_mode", default_value="true"),
        DeclareLaunchArgument("execute_motion", default_value="false"),
        DeclareLaunchArgument("enable_commands", default_value="false"),
        DeclareLaunchArgument("backend_api_enabled", default_value="true"),
        DeclareLaunchArgument(
            "backend_transport", default_value="motion_link"
        ),
        DeclareLaunchArgument(
            "backend_api_host", default_value="127.0.0.1"
        ),
        DeclareLaunchArgument("backend_api_port", default_value="8765"),
        DeclareLaunchArgument("backend_api_token", default_value=""),
        DeclareLaunchArgument(
            "maximum_rotation_rad", default_value="0.6"
        ),
        DeclareLaunchArgument(
            "end_effector", default_value="gripper"
        ),
        OpaqueFunction(function=_nodes),
    ])
