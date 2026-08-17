"""Launch Nero or Piper-L pose control simulation in RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from armbycontroller.ik.core import resolve_urdf_path


PROFILES = {
    "nero": {
        "topic_prefix": "/nero",
        "firmware": "auto",
        "tip_link": "link7",
        "initial_joint_positions": [0.0, 1.2, 0.0, 0.8, 0.0, 0.0, 0.0],
        "robot_min_reach": 0.1447354,
        "robot_max_reach": 0.7374482,
    },
    "piper_l": {
        "topic_prefix": "/piper_l",
        "firmware": "auto",
        "tip_link": "link6",
        # link6 is near [0.30, 0.00, 0.30] m with local +Z downward.
        "initial_joint_positions": [
            0.0, 1.3939753, -1.0158306, 0.0, 1.2799181, 0.0
        ],
        "robot_min_reach": 0.0,
        "robot_max_reach": 0.8738043,
    },
}


def _rviz_config():
    """Resolve the display config independently of the selected URDF owner."""
    candidates = []
    try:
        candidates.append(
            Path(get_package_share_directory("agx_arm_description"))
            / "rviz" / "display.rviz"
        )
    except PackageNotFoundError:
        pass
    candidates.append(
        Path.home() / "agx_arm_ws" / "src" / "agx_arm_ros" / "src"
        / "agx_arm_description" / "rviz" / "display.rviz"
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise RuntimeError(
        "AGX RViz config was not found; source agx_arm_description"
    )


def _nodes(context):
    """Create model-specific nodes after resolving the launch argument."""
    model = LaunchConfiguration("robot_model").perform(context).lower()
    if model not in PROFILES:
        raise ValueError(f"robot_model must be one of {sorted(PROFILES)}")
    profile = PROFILES[model]
    urdf_path = resolve_urdf_path("", model)
    rviz_path = _rviz_config()

    description = urdf_path.read_text(encoding="utf-8").replace(
        "package://agx_arm_description/agx_arm_urdf",
        f"file://{urdf_path.parents[2]}",
    )
    common = {
        "robot_model": model,
        "simulation_mode": True,
        "execute_motion": False,
        "command_period": 0.02,
        "state_period": 0.02,
        "joint_max_velocity": 1.0,
        "joint_max_acceleration": 1.0,
        "valid_history_size": 10,
        "recovery_pause": 2.0,
        "pointing_axis_only": False,
        "workspace_limit_enabled": True,
        "workspace_inner_margin": 0.05,
        "workspace_outer_margin": 0.10,
    }
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", str(rviz_path)],
        ),
        Node(
            package="armbycontroller",
            executable="pose_controller.py",
            name=f"{model}_ik_controller",
            output="screen",
            parameters=[common | profile],
        ),
    ]


def generate_launch_description():
    """Declare the model selector and create the shared simulation stack."""
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="nero"),
        OpaqueFunction(function=_nodes),
    ])
