"""Canonical static configuration for supported arm models."""

from dataclasses import dataclass


@dataclass(frozen=True)
class ArmModelProfile:
    """Model facts shared by controllers, adapters, and launch files."""

    name: str
    joint_count: int
    tip_link: str
    initial_joint_positions: tuple
    min_reach: float
    max_reach: float

    @property
    def topic_prefix(self):
        """Return the model's stable absolute ROS topic prefix."""
        return f"/{self.name}"

    def pose_parameters(self):
        """Return parameters accepted by pose-oriented controllers."""
        return {
            "robot_model": self.name,
            "topic_prefix": self.topic_prefix,
            "tip_link": self.tip_link,
            "initial_joint_positions": list(self.initial_joint_positions),
            "robot_min_reach": self.min_reach,
            "robot_max_reach": self.max_reach,
        }


ARM_MODEL_PROFILES = {
    "nero": ArmModelProfile(
        name="nero",
        joint_count=7,
        tip_link="link7",
        initial_joint_positions=(0.0, 1.2, 0.0, 0.8, 0.0, 0.0, 0.0),
        min_reach=0.1447354,
        max_reach=0.7374482,
    ),
    "piper_l": ArmModelProfile(
        name="piper_l",
        joint_count=6,
        tip_link="link6",
        initial_joint_positions=(
            0.0, 1.3939753, -1.0158306, 0.0, 1.2799181, 0.0
        ),
        min_reach=0.0,
        max_reach=0.8738043,
    ),
}


def get_arm_profile(robot_model):
    """Return one canonical profile or raise a consistent model error."""
    name = str(robot_model).strip().lower()
    try:
        return ARM_MODEL_PROFILES[name]
    except KeyError as error:
        choices = ", ".join(sorted(ARM_MODEL_PROFILES))
        raise ValueError(f"robot_model must be one of: {choices}") from error
