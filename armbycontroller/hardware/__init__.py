"""Hardware transport and device-discovery helpers."""

from armbycontroller.hardware.connection import ArmConnection
from armbycontroller.hardware.connection import connect_arm_two_stage
from armbycontroller.hardware.connection import firmware_profile_from_info
from armbycontroller.hardware.feedback import estimate_joint_velocity
from armbycontroller.hardware.feedback import extract_joint_angles
from armbycontroller.hardware.feedback import MotorFeedback

__all__ = [
    "ArmConnection",
    "connect_arm_two_stage",
    "estimate_joint_velocity",
    "extract_joint_angles",
    "firmware_profile_from_info",
    "MotorFeedback",
]
