#!/usr/bin/env python3
"""Unified ROS 2 keyboard controller for Nero and Piper-L arms."""

import math
import time
from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, PiperFW
from pyAgxArm import create_agx_arm_config
from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD

from armbycontroller.gravity_compensation import UrdfGravityModel
from armbycontroller.ik_core import AgxIkEngine
from armbycontroller.ik_core import create_tracik_solver
from armbycontroller.ik_core import IkFailure
from armbycontroller.ik_core import increment_tool_orientation
from armbycontroller.ik_core import prepare_planned_joint_mode
from armbycontroller.ik_core import resolve_urdf_path
from armbycontroller.ik_core import resolve_firmware_name
from armbycontroller.ik_core import set_joint_acceleration_limits


KEY_JOINT_1, KEY_JOINT_7 = 0, 6
KEY_DECREASE, KEY_INCREASE = 7, 8
KEY_HOME, KEY_ESTOP, KEY_MODE_TOGGLE = 9, 10, 11
KEY_FORWARD, KEY_BACKWARD, KEY_Z_UP, KEY_Z_DOWN = 12, 13, 14, 15
KEY_IMPEDANCE_TOGGLE = 16
KEY_ARROW_UP, KEY_ARROW_DOWN = 17, 18
KEY_ARROW_LEFT, KEY_ARROW_RIGHT = 19, 20
KEY_ROLL_LEFT, KEY_ROLL_RIGHT = 21, 22
KEY_COUNT = 23


def clamp(value, low, high):
    return min(max(value, low), high)


@dataclass(frozen=True)
class JogUpdate:
    selected_joint: int
    target_changed: bool = False
    selection_changed: bool = False
    home_requested: bool = False
    estop_requested: bool = False
    mode_toggle_requested: bool = False
    impedance_toggle_requested: bool = False


class ArmJointJogState:
    """Track selection, key edges, and a limit-safe joint target."""

    def __init__(self, joint_limits, step_rad, initial_joints=None):
        if not 1 <= len(joint_limits) <= 7 or step_rad <= 0.0:
            raise ValueError("arm requires 1-7 limits and step_rad > 0")
        self.joint_count = len(joint_limits)
        self.joint_limits = [tuple(map(float, limit)) for limit in joint_limits]
        if any(low >= high for low, high in self.joint_limits):
            raise ValueError("each joint limit must satisfy low < high")
        self.step_rad = float(step_rad)
        self.selected_joint = 0
        self.target_joints = [0.0] * self.joint_count
        self.previous_keys = [0] * KEY_COUNT
        if initial_joints is not None:
            self.sync_target(initial_joints)

    def sync_target(
        self, joints: Sequence[float], clamp_to_limits: bool = True
    ):
        if len(joints) != self.joint_count:
            raise ValueError("joint target count does not match the arm")
        values = [float(value) for value in joints]
        if clamp_to_limits:
            values = [
                clamp(value, low, high)
                for value, (low, high) in zip(values, self.joint_limits)
            ]
        self.target_joints = values

    def update(self, keys: Sequence[int]):
        if len(keys) < KEY_COUNT:
            raise ValueError(f"keyboard state must contain {KEY_COUNT} values")
        pressed = [bool(value) for value in keys[:KEY_COUNT]]
        rising = [
            current and not previous
            for current, previous in zip(pressed, self.previous_keys)
        ]
        old_selection = self.selected_joint
        for index in range(KEY_JOINT_1, self.joint_count):
            if rising[index]:
                self.selected_joint = index

        home = rising[KEY_HOME]
        estop = rising[KEY_ESTOP]
        toggle = rising[KEY_MODE_TOGGLE]
        impedance_toggle = rising[KEY_IMPEDANCE_TOGGLE]
        changed = False
        if home:
            target = [clamp(0.0, low, high) for low, high in self.joint_limits]
            changed, self.target_joints = target != self.target_joints, target
        elif not estop and not toggle and not impedance_toggle:
            direction = pressed[KEY_INCREASE] - pressed[KEY_DECREASE]
            if direction:
                low, high = self.joint_limits[self.selected_joint]
                old = self.target_joints[self.selected_joint]
                if old < low:
                    new = min(old + self.step_rad, low) if direction > 0 else old
                elif old > high:
                    new = max(old - self.step_rad, high) if direction < 0 else old
                else:
                    new = clamp(old + direction * self.step_rad, low, high)
                self.target_joints[self.selected_joint] = new
                changed = new != old
        self.previous_keys = pressed
        return JogUpdate(
            self.selected_joint,
            changed,
            self.selected_joint != old_selection,
            home,
            estop,
            toggle,
            impedance_toggle,
        )


MODEL_PROFILES = {
    "nero": {
        "arm_model": ArmModel.NERO,
        "joint_count": 7,
        "tip_link": "link7",
        "min_reach": 0.1447354,
        "max_reach": 0.7374482,
        "firmwares": {
            "default": NeroFW.DEFAULT, "v111": NeroFW.V111,
            "v112": NeroFW.V112, "v120": NeroFW.V120,
        },
    },
    "piper_l": {
        "arm_model": ArmModel.PIPER_L,
        "joint_count": 6,
        "tip_link": "link6",
        "min_reach": 0.0,
        "max_reach": 0.8738043,
        "firmwares": {
            "default": PiperFW.DEFAULT, "v183": PiperFW.V183,
            "v188": PiperFW.V188, "v189": PiperFW.V189,
        },
    },
}

MIT_GAIN_PROFILES = {
    "nero": {
        "kp": [1.0] * 7,
        "kd": [0.2] * 7,
        "t_ff": [0.0] * 7,
        "handover_kp": [10.0] * 7,
    },
    "piper_l": {
        "kp": [0.3, 0.5, 0.5, 0.5, 1.0, 0.3],
        "kd": [0.01, 0.01, 0.01, 0.01, 0.01, 0.01],
        "kd_max": [0.01, 0.01, 0.01, 0.01, 0.01, 0.01],
        # Dynamic URDF inverse dynamics supplies the nominal torque.
        # This remains a residual calibration bias and defaults to zero.
        "t_ff": [0.0] * 6,
        "handover_kp": [6.0, 10.0, 10.0, 6.0, 10.0, 6.0],
    },
}


def default_mit_gains(robot_model):
    """Return independent per-joint MIT gains for a supported arm."""
    profile = MIT_GAIN_PROFILES[robot_model]
    return list(profile["kp"]), list(profile["kd"])


def default_mit_handover_kp(robot_model):
    """Return per-joint stiffness used while entering MIT mode."""
    return list(MIT_GAIN_PROFILES[robot_model]["handover_kp"])


def default_mit_feedforward(robot_model):
    """Return per-joint MIT feedforward torque for a supported arm."""
    return list(MIT_GAIN_PROFILES[robot_model]["t_ff"])


def extract_joint_angles(result, joint_count):
    if result is None:
        return None
    value = result.msg if hasattr(result, "msg") else result
    if isinstance(value, Iterable) and not isinstance(value, (str, bytes, dict)):
        joints = [float(item) for item in value]
        return joints[:joint_count] if len(joints) >= joint_count else None
    if isinstance(value, dict):
        for key in ("joint_angles", "angles", "data", "msg"):
            joints = value.get(key)
            if isinstance(joints, (list, tuple)) and len(joints) >= joint_count:
                return [float(item) for item in joints[:joint_count]]
    return None


def expand_joint_values(values, joint_count, name):
    """Expand one gain to all joints or validate a per-joint list."""
    if np.isscalar(values):
        result = [float(values)]
    else:
        result = [float(value) for value in values]
    if len(result) == 1:
        result *= joint_count
    if len(result) != joint_count or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain 1 or {joint_count} finite values")
    return result


def blend_handover_kp(configured, handover, elapsed, duration):
    """Smoothly reduce takeover stiffness to the configured soft stiffness."""
    configured = np.asarray(configured, dtype=float)
    handover = np.asarray(handover, dtype=float)
    blend = float(np.clip(float(elapsed) / float(duration), 0.0, 1.0))
    blend = blend * blend * (3.0 - 2.0 * blend)
    return (handover + blend * (configured - handover)).tolist()


def bounded_gravity_feedforward(
    gravity_torque, residual_torque, elapsed, duration, scale, torque_limit
):
    """Ramp and bound URDF model torque plus a residual calibration bias."""
    gravity_torque = np.asarray(gravity_torque, dtype=float)
    residual_torque = np.asarray(residual_torque, dtype=float)
    torque_limit = np.asarray(torque_limit, dtype=float)
    blend = float(np.clip(float(elapsed) / float(duration), 0.0, 1.0))
    blend = blend * blend * (3.0 - 2.0 * blend)
    gravity_torque = np.clip(
        float(scale) * blend * gravity_torque, -torque_limit, torque_limit
    )
    return np.clip(
        residual_torque + gravity_torque, -torque_limit, torque_limit
    )


class JointTrajectoryState:
    """Generate bounded q/dq/ddq references for a moving joint-space goal."""

    def __init__(self, joint_count, max_velocity, max_acceleration, max_jerk):
        self.joint_count = int(joint_count)
        self.max_velocity = np.asarray(max_velocity, dtype=float)
        self.max_acceleration = np.asarray(max_acceleration, dtype=float)
        self.max_jerk = np.asarray(max_jerk, dtype=float)
        expected = (self.joint_count,)
        if any(
            values.shape != expected or not np.all(np.isfinite(values))
            or np.any(values <= 0.0)
            for values in (
                self.max_velocity, self.max_acceleration, self.max_jerk
            )
        ):
            raise ValueError("trajectory limits must be positive per-joint values")
        self.reset(np.zeros(self.joint_count))

    def reset(self, positions):
        positions = np.asarray(positions, dtype=float)
        if positions.shape != (self.joint_count,) or not np.all(
            np.isfinite(positions)
        ):
            raise ValueError("trajectory reset positions are invalid")
        self.position = positions.copy()
        self.velocity = np.zeros(self.joint_count, dtype=float)
        self.acceleration = np.zeros(self.joint_count, dtype=float)

    def step(self, goals, period):
        goals = np.asarray(goals, dtype=float)
        period = float(period)
        if (
            goals.shape != (self.joint_count,)
            or not np.all(np.isfinite(goals))
            or not math.isfinite(period)
            or period <= 0.0
        ):
            raise ValueError("trajectory goal or period is invalid")
        natural_frequency = 8.0
        acceleration_delta = np.clip(
            natural_frequency ** 3 * (goals - self.position)
            - 3.0 * natural_frequency ** 2 * self.velocity
            - 3.0 * natural_frequency * self.acceleration,
            -self.max_jerk * period,
            self.max_jerk * period,
        )
        new_acceleration = np.clip(
            self.acceleration + acceleration_delta,
            -self.max_acceleration,
            self.max_acceleration,
        )
        new_velocity = np.clip(
            self.velocity + 0.5 * (
                self.acceleration + new_acceleration
            ) * period,
            -self.max_velocity,
            self.max_velocity,
        )
        new_position = self.position + 0.5 * (
            self.velocity + new_velocity
        ) * period
        # The jerk-limited discrete filter otherwise forms a tiny limit cycle
        # around the goal. Snap only inside a one-frame terminal region, where
        # removing acceleration still respects the jerk bound.
        settled = (
            (np.abs(goals - new_position) <= 25.0 * self.max_jerk * period ** 3)
            & (np.abs(new_velocity) <= 4.0 * self.max_jerk * period ** 2)
            & (np.abs(new_acceleration) <= self.max_jerk * period)
        )
        new_position = np.where(settled, goals, new_position)
        new_velocity = np.where(settled, 0.0, new_velocity)
        new_acceleration = np.where(settled, 0.0, new_acceleration)
        self.position = new_position
        self.velocity = new_velocity
        self.acceleration = new_acceleration
        return (
            self.position.copy(),
            self.velocity.copy(),
            self.acceleration.copy(),
        )


def bounded_velocity_damping(
    velocity, damping_min, damping_max, transition_velocity, torque_limit
):
    """Return an equivalent Kd for smooth speed damping with torque limits."""
    velocity = np.asarray(velocity, dtype=float)
    damping_min = np.asarray(damping_min, dtype=float)
    damping_max = np.asarray(damping_max, dtype=float)
    transition_velocity = np.asarray(transition_velocity, dtype=float)
    torque_limit = np.asarray(torque_limit, dtype=float)
    speed_squared = np.square(np.abs(velocity))
    transition_squared = np.square(transition_velocity)
    damping = damping_min + (damping_max - damping_min) * (
        speed_squared / (transition_squared + speed_squared)
    )
    normalized_torque = damping * velocity / torque_limit
    damping_torque = torque_limit * np.tanh(normalized_torque)
    return np.divide(
        damping_torque,
        velocity,
        out=damping_min.copy(),
        where=np.abs(velocity) > 1e-9,
    )


class ArmKeyboardController(Node):
    def __init__(self):
        super().__init__("arm_keyboard_controller")

        self.declare_parameter("robot_model", "nero")
        declared_model = str(
            self.get_parameter("robot_model").value
        ).lower()
        gain_model = declared_model if declared_model in MODEL_PROFILES else "nero"
        default_mit_kp, default_mit_kd = default_mit_gains(gain_model)
        default_mit_kd_max = list(
            MIT_GAIN_PROFILES[gain_model].get(
                "kd_max", [3.0 * value for value in default_mit_kd]
            )
        )
        default_handover_kp = default_mit_handover_kp(gain_model)
        default_mit_feedforward_values = default_mit_feedforward(gain_model)
        self.declare_parameter("keyboard_topic", "/arm_keyboard_state")
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("firmware", "auto")
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("step_rad", 0.005)
        self.declare_parameter("speed_percent", 20)
        self.declare_parameter("joint_max_acceleration", 1.0)
        self.declare_parameter("joint_acc_timeout", 2.0)
        self.declare_parameter("position_mode_timeout", 2.0)
        self.declare_parameter("keyboard_timeout", 0.3)
        self.declare_parameter("enable_timeout", 5.0)
        self.declare_parameter("feedback_timeout", 3.0)
        self.declare_parameter("move_home_on_start", True)
        self.declare_parameter("startup_home_timeout", 30.0)
        self.declare_parameter("startup_home_tolerance", 0.01)
        self.declare_parameter("clear_errors_on_enable", True)
        self.declare_parameter("reset_emergency_stop_on_start", True)
        self.declare_parameter("emergency_reset_timeout", 5.0)
        self.declare_parameter("execute_motion", True)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tip_link", "")
        self.declare_parameter("cartesian_step", 0.005)
        self.declare_parameter("ik_timeout", 0.01)
        self.declare_parameter("ik_tolerance", 1e-5)
        self.declare_parameter("fk_position_tolerance", 1e-4)
        self.declare_parameter("fk_rotation_tolerance", 1e-3)
        self.declare_parameter("pointing_roll_samples", 8)
        self.declare_parameter("orientation_step_rad", 0.02)
        self.declare_parameter("robot_min_reach", -1.0)
        self.declare_parameter("robot_max_reach", -1.0)
        self.declare_parameter("workspace_inner_margin", 0.05)
        self.declare_parameter("workspace_outer_margin", 0.10)
        self.declare_parameter("ik_recovery_pause", 2.0)
        self.declare_parameter("impedance_enabled", False)
        self.declare_parameter("mit_command_rate", 100.0)
        self.declare_parameter("mit_kp", default_mit_kp)
        self.declare_parameter("mit_kd", default_mit_kd)
        scalar_or_array = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter(
            "mit_kd_max", default_mit_kd_max, scalar_or_array
        )
        self.declare_parameter(
            "mit_damping_transition_velocity", [0.3], scalar_or_array
        )
        self.declare_parameter(
            "mit_damping_torque_limit", [1.0], scalar_or_array
        )
        self.declare_parameter(
            "mit_feedforward", default_mit_feedforward_values
        )
        self.declare_parameter("mit_gravity_compensation_enabled", True)
        self.declare_parameter("gravity_urdf_path", "")
        self.declare_parameter("mit_gravity_scale", 1.0)
        self.declare_parameter("mit_gravity_ramp_duration", 1.0)
        self.declare_parameter(
            "mit_gravity_torque_limit", [10.0], scalar_or_array
        )
        self.declare_parameter("gravity_vector", [0.0, 0.0, -9.80665])
        self.declare_parameter(
            "mit_trajectory_max_velocity", [0.5], scalar_or_array
        )
        self.declare_parameter(
            "mit_trajectory_max_acceleration", [1.0], scalar_or_array
        )
        self.declare_parameter(
            "mit_trajectory_max_jerk", [5.0], scalar_or_array
        )
        self.declare_parameter("mit_max_joint_step", 0.05)
        self.declare_parameter("mit_handover_duration", 0.5)
        self.declare_parameter("mit_handover_kp", default_handover_kp)

        self.robot_model = str(
            self.get_parameter("robot_model").value
        ).lower()
        if self.robot_model not in MODEL_PROFILES:
            raise ValueError("robot_model must be nero or piper_l")
        self.profile = MODEL_PROFILES[self.robot_model]
        self.joint_count = self.profile["joint_count"]
        self.keyboard_topic = str(self.get_parameter("keyboard_topic").value)
        self.can_interface = str(self.get_parameter("can_interface").value)
        self.firmware_name = resolve_firmware_name(
            self.robot_model, self.get_parameter("firmware").value
        )
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.step_rad = float(self.get_parameter("step_rad").value)
        self.speed_percent = int(self.get_parameter("speed_percent").value)
        self.joint_max_acceleration = float(
            self.get_parameter("joint_max_acceleration").value
        )
        self.joint_acc_timeout = float(
            self.get_parameter("joint_acc_timeout").value
        )
        self.position_mode_timeout = float(
            self.get_parameter("position_mode_timeout").value
        )
        self.keyboard_timeout = float(self.get_parameter("keyboard_timeout").value)
        self.enable_timeout = float(self.get_parameter("enable_timeout").value)
        self.feedback_timeout = float(self.get_parameter("feedback_timeout").value)
        self.move_home_on_start = bool(
            self.get_parameter("move_home_on_start").value
        )
        self.startup_home_timeout = float(
            self.get_parameter("startup_home_timeout").value
        )
        self.startup_home_tolerance = float(
            self.get_parameter("startup_home_tolerance").value
        )
        self.clear_errors_on_enable = bool(
            self.get_parameter("clear_errors_on_enable").value
        )
        self.reset_emergency_stop_on_start = bool(
            self.get_parameter("reset_emergency_stop_on_start").value
        )
        self.emergency_reset_timeout = float(
            self.get_parameter("emergency_reset_timeout").value
        )
        self.execute_motion = bool(self.get_parameter("execute_motion").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tip_link = (
            str(self.get_parameter("tip_link").value)
            or self.profile["tip_link"]
        )
        self.cartesian_step = float(
            self.get_parameter("cartesian_step").value
        )
        self.ik_timeout = float(self.get_parameter("ik_timeout").value)
        self.ik_tolerance = float(self.get_parameter("ik_tolerance").value)
        self.fk_position_tolerance = float(
            self.get_parameter("fk_position_tolerance").value
        )
        self.fk_rotation_tolerance = float(
            self.get_parameter("fk_rotation_tolerance").value
        )
        self.pointing_roll_samples = int(
            self.get_parameter("pointing_roll_samples").value
        )
        self.orientation_step_rad = float(
            self.get_parameter("orientation_step_rad").value
        )
        robot_min = float(self.get_parameter("robot_min_reach").value)
        robot_max = float(self.get_parameter("robot_max_reach").value)
        robot_min = self.profile["min_reach"] if robot_min < 0.0 else robot_min
        robot_max = self.profile["max_reach"] if robot_max < 0.0 else robot_max
        self.workspace_min_radius = robot_min + float(
            self.get_parameter("workspace_inner_margin").value
        )
        self.workspace_max_radius = robot_max - float(
            self.get_parameter("workspace_outer_margin").value
        )
        self.ik_recovery_pause = float(
            self.get_parameter("ik_recovery_pause").value
        )
        self.impedance_enabled = bool(
            self.get_parameter("impedance_enabled").value
        )
        self.mit_command_rate = float(
            self.get_parameter("mit_command_rate").value
        )
        self.mit_kp = expand_joint_values(
            self.get_parameter("mit_kp").value, self.joint_count, "mit_kp"
        )
        self.mit_kd = expand_joint_values(
            self.get_parameter("mit_kd").value, self.joint_count, "mit_kd"
        )
        self.mit_kd_max = expand_joint_values(
            self.get_parameter("mit_kd_max").value,
            self.joint_count,
            "mit_kd_max",
        )
        self.mit_damping_transition_velocity = expand_joint_values(
            self.get_parameter("mit_damping_transition_velocity").value,
            self.joint_count,
            "mit_damping_transition_velocity",
        )
        self.mit_damping_torque_limit = expand_joint_values(
            self.get_parameter("mit_damping_torque_limit").value,
            self.joint_count,
            "mit_damping_torque_limit",
        )
        self.mit_feedforward = expand_joint_values(
            self.get_parameter("mit_feedforward").value,
            self.joint_count,
            "mit_feedforward",
        )
        self.mit_gravity_compensation_enabled = bool(
            self.get_parameter("mit_gravity_compensation_enabled").value
        )
        self.gravity_urdf_path = str(
            self.get_parameter("gravity_urdf_path").value
        )
        self.mit_gravity_scale = float(
            self.get_parameter("mit_gravity_scale").value
        )
        self.mit_gravity_ramp_duration = float(
            self.get_parameter("mit_gravity_ramp_duration").value
        )
        self.mit_gravity_torque_limit = expand_joint_values(
            self.get_parameter("mit_gravity_torque_limit").value,
            self.joint_count,
            "mit_gravity_torque_limit",
        )
        self.gravity_vector = np.asarray(
            self.get_parameter("gravity_vector").value, dtype=float
        )
        self.mit_trajectory_max_velocity = expand_joint_values(
            self.get_parameter("mit_trajectory_max_velocity").value,
            self.joint_count,
            "mit_trajectory_max_velocity",
        )
        self.mit_trajectory_max_acceleration = expand_joint_values(
            self.get_parameter("mit_trajectory_max_acceleration").value,
            self.joint_count,
            "mit_trajectory_max_acceleration",
        )
        self.mit_trajectory_max_jerk = expand_joint_values(
            self.get_parameter("mit_trajectory_max_jerk").value,
            self.joint_count,
            "mit_trajectory_max_jerk",
        )
        self.mit_max_joint_step = float(
            self.get_parameter("mit_max_joint_step").value
        )
        self.mit_handover_duration = float(
            self.get_parameter("mit_handover_duration").value
        )
        self.mit_handover_kp = expand_joint_values(
            self.get_parameter("mit_handover_kp").value,
            self.joint_count,
            "mit_handover_kp",
        )

        if self.control_rate <= 0.0:
            raise ValueError("control_rate must be > 0")
        if not 1 <= self.speed_percent <= 100:
            raise ValueError("speed_percent must be in [1, 100]")
        if self.joint_max_acceleration <= 0.0:
            raise ValueError("joint_max_acceleration must be > 0")
        if self.joint_acc_timeout <= 0.0:
            raise ValueError("joint_acc_timeout must be > 0")
        if self.position_mode_timeout <= 0.0:
            raise ValueError("position_mode_timeout must be > 0")
        if self.cartesian_step <= 0.0:
            raise ValueError("cartesian_step must be > 0")
        if self.orientation_step_rad <= 0.0:
            raise ValueError("orientation_step_rad must be > 0")
        if self.workspace_min_radius >= self.workspace_max_radius:
            raise ValueError("workspace margins leave no usable region")
        if not 1 <= self.pointing_roll_samples <= 72:
            raise ValueError("pointing_roll_samples must be in [1, 72]")
        if self.keyboard_timeout <= 0.0:
            raise ValueError("keyboard_timeout must be > 0")
        if self.startup_home_timeout <= 0.0:
            raise ValueError("startup_home_timeout must be > 0")
        if self.startup_home_tolerance <= 0.0:
            raise ValueError("startup_home_tolerance must be > 0")
        if self.emergency_reset_timeout <= 0.0:
            raise ValueError("emergency_reset_timeout must be > 0")
        if self.mit_command_rate <= 0.0 or self.mit_max_joint_step <= 0.0:
            raise ValueError("MIT rate and maximum joint step must be > 0")
        if self.mit_handover_duration <= 0.0:
            raise ValueError("mit_handover_duration must be > 0")
        if self.mit_gravity_ramp_duration <= 0.0:
            raise ValueError("mit_gravity_ramp_duration must be > 0")
        if not 0.0 <= self.mit_gravity_scale <= 1.0:
            raise ValueError("mit_gravity_scale must be in [0, 1]")
        if self.gravity_vector.shape != (3,) or not np.all(
            np.isfinite(self.gravity_vector)
        ):
            raise ValueError("gravity_vector must contain three finite values")
        if any(not 0.0 <= value <= 500.0 for value in self.mit_kp):
            raise ValueError("mit_kp values must be in [0, 500]")
        if any(not 0.0 <= value <= 500.0 for value in self.mit_handover_kp):
            raise ValueError("mit_handover_kp values must be in [0, 500]")
        if any(not 0.0 <= value <= 5.0 for value in self.mit_kd):
            raise ValueError("mit_kd values must be in [0, 5]")
        if any(
            minimum > maximum or not 0.0 <= maximum <= 5.0
            for minimum, maximum in zip(self.mit_kd, self.mit_kd_max)
        ):
            raise ValueError("mit_kd_max must be in [mit_kd, 5]")
        if any(
            value <= 0.0 for value in self.mit_damping_transition_velocity
        ):
            raise ValueError("MIT damping transition velocities must be > 0")
        if any(
            not 0.0 < value <= 8.0
            for value in self.mit_damping_torque_limit
        ):
            raise ValueError("MIT damping torque limits must be in (0, 8] N·m")
        if any(abs(value) > 10.0 for value in self.mit_feedforward):
            raise ValueError("mit_feedforward values must be in [-10, 10] N·m")
        if any(
            not 0.0 < value <= 10.0
            for value in self.mit_gravity_torque_limit
        ):
            raise ValueError(
                "MIT model torque limits must be in (0, 10] N·m"
            )
        if any(
            value <= 0.0
            for values in (
                self.mit_trajectory_max_velocity,
                self.mit_trajectory_max_acceleration,
                self.mit_trajectory_max_jerk,
            )
            for value in values
        ):
            raise ValueError("MIT trajectory limits must be greater than zero")

        firmwares = self.profile["firmwares"]
        self.firmware = firmwares.get(self.firmware_name)
        if self.firmware is None:
            raise ValueError(
                f"unsupported {self.robot_model} firmware "
                f"{self.firmware_name!r}; choose auto or one of "
                f"{sorted(firmwares)}"
            )

        self.joint_limits = [
            tuple(ROBOT_JOINT_LIMIT_PRESET_RAD[self.robot_model][f"joint{index}"])
            for index in range(1, self.joint_count + 1)
        ]
        legacy_rate_scale = 20.0 / self.control_rate
        self.jog = ArmJointJogState(
            self.joint_limits, self.step_rad * legacy_rate_scale
        )
        self.cartesian_step_per_tick = (
            self.cartesian_step * legacy_rate_scale
        )
        self.orientation_step_per_tick = (
            self.orientation_step_rad * legacy_rate_scale
        )
        self.mit_trajectory = JointTrajectoryState(
            self.joint_count,
            self.mit_trajectory_max_velocity,
            self.mit_trajectory_max_acceleration,
            self.mit_trajectory_max_jerk,
        )
        urdf_path = resolve_urdf_path(
            str(self.get_parameter("urdf_path").value), self.robot_model
        )
        self.gravity_model = None
        if self.mit_gravity_compensation_enabled:
            gravity_urdf_path = (
                Path(self.gravity_urdf_path).expanduser().resolve()
                if self.gravity_urdf_path
                else urdf_path.parent / {
                    "piper_l": "piper_l_with_gripper_description.xacro",
                    "nero": "nero_with_left_revo2_description.xacro",
                }[self.robot_model]
            )
            self.gravity_model = UrdfGravityModel(
                gravity_urdf_path,
                self.base_frame,
                self.tip_link,
                self.joint_count,
                self.gravity_vector,
            )
            self.get_logger().info(
                "URDF inverse dynamics ready: "
                f"{gravity_urdf_path}; "
                f"modeled_mass={self.gravity_model.modeled_mass:.3f} kg; "
                f"joints={self.gravity_model.movable_joint_names}"
            )
        self.ik_solver = create_tracik_solver(
            urdf_path,
            self.base_frame,
            self.tip_link,
            self.joint_count,
            self.ik_timeout,
            self.ik_tolerance,
        )
        self.ik_engine = AgxIkEngine(
            self.ik_solver,
            self.joint_count,
            self.workspace_min_radius,
            self.workspace_max_radius,
            self.fk_position_tolerance,
            self.fk_rotation_tolerance,
            self.pointing_roll_samples,
            pointing_axis_only=False,
        )
        self.ik_target_rotation = None
        self.control_mode = "joint"
        self.ik_target_position = None
        self.ik_valid_history = deque(maxlen=10)
        self.ik_recovery_until = -1.0
        self.key_state = [0] * KEY_COUNT
        self.last_keyboard_time = 0.0
        self.arm = None
        self.arm_connected = False
        self.arm_ready = False
        self.emergency_stopped = False
        self.mit_handover_started = (
            time.monotonic() if self.impedance_enabled else float("-inf")
        )
        self.last_mit_tick_time = None

        self.keyboard_sub = self.create_subscription(
            Int32MultiArray,
            self.keyboard_topic,
            self.keyboard_callback,
            qos_profile_sensor_data,
        )
        self.connect_arm()
        self.mit_trajectory.reset(self.jog.target_joints)
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_tick)
        self.mit_timer = self.create_timer(
            1.0 / self.mit_command_rate, self.mit_tick
        )

        self.get_logger().info(
            f"{self.robot_model} ready: P=joint/IK; "
            f"joint 1-{self.joint_count}+A/D; "
            "IK W/S+A/D+Z/X; I=MIT impedance; SPACE=home; E=E-stop"
        )

    def keyboard_callback(self, message):
        data = list(message.data)
        if len(data) < KEY_COUNT:
            self.get_logger().warning(
                f"keyboard state length {len(data)} < {KEY_COUNT}",
                throttle_duration_sec=1.0,
            )
            return
        self.key_state = [1 if value else 0 for value in data[:KEY_COUNT]]
        self.last_keyboard_time = time.monotonic()

    def connect_arm(self):
        config = create_agx_arm_config(
            robot=self.profile["arm_model"],
            firmeware_version=self.firmware,
            interface="socketcan",
            channel=self.can_interface,
        )
        self.arm = AgxArmFactory.create_arm(config)
        self.arm.set_joint_limits_enabled(True)

        if not self.execute_motion:
            self.get_logger().warning(
                f"dry-run mode: {self.robot_model} commands are disabled"
            )
            self.arm_ready = True
            return

        try:
            self.arm.connect()
            self.arm_connected = True
            self.get_logger().info(
                f"connected {self.robot_model} on {self.can_interface} "
                f"with firmware profile {self.firmware_name}"
            )
            try:
                firmware = self.arm.get_firmware(
                    timeout=1.0, min_interval=0.0
                )
                if firmware:
                    self.get_logger().info(f"device firmware: {firmware}")
            except Exception as exc:
                self.get_logger().warning(f"firmware query failed: {exc}")
            if self.arm_reports_emergency_stop():
                if not self.reset_emergency_stop_on_start:
                    self.get_logger().error(
                        "arm reports EMERGENCY_STOP; motion is disabled. "
                        "Inspect the arm and restart with "
                        "reset_emergency_stop_on_start:=true to explicitly "
                        "reset the controller."
                    )
                    return
                if not self.reset_emergency_stop():
                    self.get_logger().error(
                        "emergency-stop reset timed out; motion is disabled"
                    )
                    return
            else:
                self.get_logger().info(
                    "startup safety check: no electronic emergency stop reported"
                )
            if not self.enable_arm():
                self.get_logger().error("enable timed out; motion is disabled")
                return
            limits_ok, failed_joint = set_joint_acceleration_limits(
                self.arm,
                self.joint_count,
                self.joint_max_acceleration,
                self.joint_acc_timeout,
            )
            if not limits_ok:
                self.get_logger().error(
                    "failed to set/read back acceleration limit for joint "
                    f"{failed_joint}"
                )
                return
            self.get_logger().info(
                "joint acceleration limits verified individually: "
                f"{self.joint_max_acceleration:.3f} rad/s²"
            )
            self.arm.set_speed_percent(self.speed_percent)
            if not prepare_planned_joint_mode(
                self.arm, self.position_mode_timeout
            ):
                self.get_logger().error(
                    "CAN_CTRL/MOVE_J mode timed out; motion is disabled"
                )
                return
            self.get_logger().info(
                "planned joint mode confirmed: CAN_CTRL/MOVE_J"
            )
            joints = self.wait_for_joint_feedback()
            if joints is None:
                self.get_logger().error(
                    f"no complete {self.joint_count}-joint feedback; "
                    "motion is disabled for safety"
                )
                return
            self.jog.sync_target(joints, clamp_to_limits=False)
            self.get_logger().info(
                f"synced current {self.robot_model} joints: "
                f"{[round(value, 4) for value in self.jog.target_joints]}"
            )
            outside = [
                index
                for index, (value, (low, high)) in enumerate(
                    zip(joints, self.joint_limits), start=1
                )
                if value < low or value > high
            ]
            if outside:
                self.get_logger().warning(
                    "feedback outside configured limits on joints "
                    f"{outside}; use joint mode to jog each axis inward. "
                    "The first jog command will not jump to the limit."
                )
            if self.move_home_on_start and not self.move_home_and_wait():
                self.get_logger().error(
                    "startup home was not reached; keyboard motion is disabled"
                )
                return
            self.arm_ready = True
        except Exception as exc:
            self.arm_ready = False
            self.get_logger().error(
                f"failed to initialize {self.robot_model}: {exc}"
            )

    def arm_reports_emergency_stop(self):
        status = self.arm.get_arm_status()
        if status is None or not hasattr(status, "msg"):
            return False
        return "EMERGENCY_STOP" in str(
            getattr(status.msg, "arm_status", "")
        )

    def reset_emergency_stop(self):
        self.get_logger().warning(
            "explicitly resetting the electronic emergency stop"
        )
        self.arm.reset()
        deadline = time.monotonic() + self.emergency_reset_timeout
        while time.monotonic() < deadline:
            if not self.arm_reports_emergency_stop():
                self.get_logger().info("electronic emergency stop reset")
                return True
            time.sleep(0.05)
        return False

    def enable_arm(self):
        deadline = time.monotonic() + self.enable_timeout
        while time.monotonic() < deadline:
            try:
                if self.clear_errors_on_enable:
                    self.arm.clear_joint_error()
                if self.arm.enable():
                    return True
                states = self.arm.get_joints_enable_status_list()
                if (len(states) >= self.joint_count and
                        all(states[:self.joint_count])):
                    return True
            except Exception as exc:
                self.get_logger().warning(
                    f"enable attempt failed: {exc}", throttle_duration_sec=1.0
                )
            time.sleep(0.2)
        return False

    def wait_for_joint_feedback(self):
        deadline = time.monotonic() + self.feedback_timeout
        while time.monotonic() < deadline:
            try:
                joints = extract_joint_angles(
                    self.arm.get_joint_angles(), self.joint_count
                )
                if joints is not None:
                    return joints
            except Exception as exc:
                self.get_logger().warning(
                    f"joint feedback failed: {exc}", throttle_duration_sec=1.0
                )
            time.sleep(0.05)
        return None

    def move_home_and_wait(self):
        joints = extract_joint_angles(
            self.arm.get_joint_angles(), self.joint_count
        )
        if joints is None:
            self.get_logger().error(
                "cannot start sequential zeroing without joint feedback"
            )
            return False

        target = list(joints)
        limits_were_enabled = self.arm.get_joint_limits_enabled()
        # Preserve every non-active joint at its raw feedback position, even
        # when startup begins slightly outside the configured soft limits.
        self.arm.set_joint_limits_enabled(False)
        try:
            for index in range(self.joint_count):
                target[index] = 0.0
                self.get_logger().info(
                    f"forcing startup zero joint {index + 1}/"
                    f"{self.joint_count}: "
                    f"{[round(value, 4) for value in target]}"
                )
                self.arm.move_j(list(target))
                deadline = time.monotonic() + self.startup_home_timeout

                while time.monotonic() < deadline:
                    feedback = extract_joint_angles(
                        self.arm.get_joint_angles(), self.joint_count
                    )
                    if (feedback is not None and
                            abs(feedback[index]) <=
                            self.startup_home_tolerance):
                        self.get_logger().info(
                            f"joint {index + 1} zero reached: "
                            f"error={abs(feedback[index]):.6f} rad"
                        )
                        break
                    time.sleep(0.05)
                else:
                    self.get_logger().error(
                        f"joint {index + 1} sequential zero timed out"
                    )
                    return False

            home = [0.0] * self.joint_count
            self.jog.sync_target(home)
            self.get_logger().info("sequential startup zero complete")
            return True
        finally:
            self.arm.set_joint_limits_enabled(limits_were_enabled)

    def control_tick(self):
        keys = self.key_state
        if time.monotonic() - self.last_keyboard_time > self.keyboard_timeout:
            keys = [0] * KEY_COUNT

        state_keys = list(keys)
        if self.control_mode == "ik":
            # A/D belong to Cartesian Y in IK mode, not joint jogging.
            state_keys[KEY_DECREASE] = 0
            state_keys[KEY_INCREASE] = 0
        update = self.jog.update(state_keys)
        if update.selection_changed:
            self.get_logger().info(
                f"selected joint {update.selected_joint + 1}"
            )

        if update.estop_requested:
            self.trigger_emergency_stop()
            return
        if self.emergency_stopped:
            return

        if update.impedance_toggle_requested:
            self.toggle_impedance()
            return

        if update.mode_toggle_requested:
            self.toggle_control_mode()
            return

        if update.home_requested:
            self.ik_target_position = None
            self.ik_target_rotation = None
            if update.target_changed:
                self.send_target("home")
            return

        if self.control_mode == "ik":
            self.apply_cartesian_step(keys)
            return
        if not update.target_changed:
            return

        self.send_target(f"joint {update.selected_joint + 1} jog")

    def toggle_control_mode(self):
        if self.control_mode == "joint":
            joints = self.current_or_target_joints()
            position, rotation = self.ik_solver.fk(joints)
            self.ik_target_position = np.asarray(position, dtype=float)
            self.ik_target_rotation = np.asarray(rotation, dtype=float)
            self.ik_valid_history.clear()
            self.remember_ik_valid(
                self.ik_target_position, self.ik_target_rotation, joints
            )
            self.control_mode = "ik"
            self.get_logger().info(
                "mode=IK + backend="
                f"{'MIT' if self.impedance_enabled else 'planned'}: "
                "XYZ=W/S+A/D+Z/X; arrows=pointing; PgUp/PgDn=tilt"
            )
        else:
            joints = self.current_or_target_joints()
            self.jog.sync_target(joints)
            self.control_mode = "joint"
            self.get_logger().info(
                f"mode=JOINT: 1-{self.joint_count} select, A/D jog"
            )

    def toggle_impedance(self):
        """Switch safely between planned move_j and MIT joint impedance."""
        joints = self.current_or_target_joints()
        self.jog.sync_target(joints)
        self.impedance_enabled = not self.impedance_enabled
        if self.impedance_enabled:
            self.mit_handover_started = time.monotonic()
            self.last_mit_tick_time = None
            self.mit_trajectory.reset(joints)
            self.get_logger().warning(
                "backend=MIT impedance + control="
                f"{self.control_mode.upper()}: holding current joints"
            )
            self.mit_tick()
            return
        self.get_logger().info("backend=planned position control")
        self.send_target("MIT exit hold")

    def read_joint_velocities(self):
        """Read cached motor velocity for speed-dependent MIT damping."""
        if not hasattr(self.arm, "get_motor_states"):
            return None
        velocities = []
        for joint_index in range(1, self.joint_count + 1):
            try:
                state = self.arm.get_motor_states(joint_index)
                message = getattr(state, "msg", state)
                velocity = float(getattr(message, "velocity"))
            except Exception:
                return None
            if not np.isfinite(velocity):
                return None
            velocities.append(velocity)
        return np.asarray(velocities, dtype=float)

    def mit_tick(self):
        """Continuously refresh all MIT joint impedance commands."""
        if not self.impedance_enabled or self.emergency_stopped:
            return
        if not self.execute_motion:
            self.get_logger().info(
                "dry-run MIT hold active", throttle_duration_sec=1.0
            )
            return
        if not self.arm_ready:
            return
        try:
            now = time.monotonic()
            handover_elapsed = now - self.mit_handover_started
            nominal_period = 1.0 / getattr(self, "mit_command_rate", 100.0)
            last_tick = getattr(self, "last_mit_tick_time", None)
            trajectory_period = (
                nominal_period if last_tick is None
                else float(np.clip(
                    now - last_tick, 0.25 * nominal_period,
                    2.0 * nominal_period,
                ))
            )
            self.last_mit_tick_time = now
            trajectory = getattr(self, "mit_trajectory", None)
            if trajectory is None:
                reference_position = np.asarray(
                    self.jog.target_joints, dtype=float
                )
                reference_velocity = np.zeros(self.joint_count, dtype=float)
                reference_acceleration = np.zeros(
                    self.joint_count, dtype=float
                )
            else:
                (
                    reference_position,
                    reference_velocity,
                    reference_acceleration,
                ) = trajectory.step(
                    self.jog.target_joints, trajectory_period
                )
            kp_values = blend_handover_kp(
                self.mit_kp,
                self.mit_handover_kp,
                handover_elapsed,
                self.mit_handover_duration,
            )
            joint_velocity = self.read_joint_velocities()
            if joint_velocity is None:
                joint_velocity = np.zeros(self.joint_count, dtype=float)
                self.get_logger().warning(
                    "motor velocity unavailable; using minimum MIT damping",
                    throttle_duration_sec=1.0,
                )
            adaptive_kd = bounded_velocity_damping(
                joint_velocity,
                self.mit_kd,
                self.mit_kd_max,
                self.mit_damping_transition_velocity,
                self.mit_damping_torque_limit,
            )
            feedforward = np.asarray(self.mit_feedforward, dtype=float)
            gravity_model = getattr(self, "gravity_model", None)
            if gravity_model is not None:
                joints = None
                try:
                    joints = extract_joint_angles(
                        self.arm.get_joint_angles(), self.joint_count
                    )
                except Exception:
                    pass
                if joints is None:
                    self.get_logger().warning(
                        "joint feedback unavailable; URDF model torque is zero",
                        throttle_duration_sec=1.0,
                    )
                else:
                    if hasattr(gravity_model, "inverse_dynamics"):
                        model_torque = gravity_model.inverse_dynamics(
                            joints,
                            reference_velocity,
                            reference_acceleration,
                        )
                    else:
                        model_torque = gravity_model.compensation(joints)
                    feedforward = bounded_gravity_feedforward(
                        model_torque,
                        self.mit_feedforward,
                        handover_elapsed,
                        self.mit_gravity_ramp_duration,
                        self.mit_gravity_scale,
                        self.mit_gravity_torque_limit,
                    )
                    self.get_logger().info(
                        "URDF inverse dynamics raw=%s commanded=%s N·m"
                        % (
                            np.round(model_torque, 3).tolist(),
                            np.round(feedforward, 3).tolist(),
                        ),
                        throttle_duration_sec=2.0,
                    )
            for index, position in enumerate(reference_position):
                self.arm.move_mit(
                    joint_index=index + 1,
                    p_des=float(position),
                    v_des=float(reference_velocity[index]),
                    kp=kp_values[index],
                    kd=float(adaptive_kd[index]),
                    t_ff=float(feedforward[index]),
                )
        except Exception as exc:
            self.get_logger().error(f"MIT command failed: {exc}")
            self.trigger_emergency_stop()

    def current_or_target_joints(self):
        if self.execute_motion and self.arm_ready:
            try:
                joints = extract_joint_angles(
                    self.arm.get_joint_angles(), self.joint_count
                )
                if joints is not None:
                    return np.asarray(joints, dtype=float)
            except Exception as exc:
                self.get_logger().warning(
                    f"joint feedback unavailable; using last target: {exc}",
                    throttle_duration_sec=1.0,
                )
        return np.asarray(self.jog.target_joints, dtype=float)

    def remember_ik_valid(self, position, rotation, joints):
        self.ik_valid_history.append(
            (np.asarray(position, dtype=float).copy(),
             np.asarray(rotation, dtype=float).copy(),
             np.asarray(joints, dtype=float).copy())
        )

    def recover_ik(self, reason):
        self.ik_recovery_until = time.monotonic() + self.ik_recovery_pause
        if self.ik_valid_history:
            position, rotation, joints = self.ik_valid_history[-1]
            self.ik_target_position = position.copy()
            self.ik_target_rotation = rotation.copy()
            self.jog.sync_target(joints)
            self.send_target("IK recovery to last valid target")
        self.get_logger().warning(
            f"IK stuck: {reason}; retained last valid target and paused "
            f"{self.ik_recovery_pause:.1f} s"
        )

    def apply_cartesian_step(self, keys):
        remaining = self.ik_recovery_until - time.monotonic()
        if remaining > 0.0:
            return
        direction = np.asarray(
            [
                keys[KEY_FORWARD] - keys[KEY_BACKWARD],
                keys[KEY_DECREASE] - keys[KEY_INCREASE],
                keys[KEY_Z_UP] - keys[KEY_Z_DOWN],
            ],
            dtype=float,
        )
        pitch = (
            keys[KEY_ARROW_DOWN] - keys[KEY_ARROW_UP]
        ) * self.orientation_step_per_tick
        yaw = (
            keys[KEY_ARROW_LEFT] - keys[KEY_ARROW_RIGHT]
        ) * self.orientation_step_per_tick
        roll = (
            keys[KEY_ROLL_LEFT] - keys[KEY_ROLL_RIGHT]
        ) * self.orientation_step_per_tick
        if not np.any(direction) and pitch == yaw == roll == 0.0:
            return
        if self.ik_target_position is None or self.ik_target_rotation is None:
            joints = self.current_or_target_joints()
            position, rotation = self.ik_solver.fk(joints)
            self.ik_target_position = np.asarray(position, dtype=float)
            self.ik_target_rotation = np.asarray(rotation, dtype=float)
        candidate = (
            self.ik_target_position + direction * self.cartesian_step_per_tick
        )
        candidate_rotation = increment_tool_orientation(
            self.ik_target_rotation, pitch, yaw, roll
        )
        seed = np.asarray(self.jog.target_joints, dtype=float)
        try:
            result = self.ik_engine.solve(
                candidate, candidate_rotation, seed
            )
        except IkFailure as error:
            self.recover_ik(str(error))
            return
        if (self.impedance_enabled and
                float(np.max(np.abs(result.joints - seed))) >
                self.mit_max_joint_step):
            self.recover_ik("MIT IK joint step exceeds configured limit")
            return
        self.ik_target_position = candidate
        self.ik_target_rotation = candidate_rotation
        self.jog.sync_target(result.joints)
        self.remember_ik_valid(candidate, candidate_rotation, result.joints)
        self.send_target("IK pose jog")

    def send_target(self, reason):
        target = [float(value) for value in self.jog.target_joints]
        self.get_logger().info(
            f"{reason}: {[round(value, 4) for value in target]}",
            throttle_duration_sec=0.25,
        )
        if not self.execute_motion:
            return
        if self.impedance_enabled:
            return
        if not self.arm_ready:
            self.get_logger().warning(
                "arm is not ready; command skipped", throttle_duration_sec=1.0
            )
            return
        try:
            self.arm.move_j(target)
        except Exception as exc:
            self.get_logger().error(f"move_j failed: {exc}")

    def trigger_emergency_stop(self):
        self.emergency_stopped = True
        self.arm_ready = False
        self.get_logger().error("ELECTRONIC EMERGENCY STOP requested")
        if self.execute_motion and self.arm_connected:
            try:
                self.arm.electronic_emergency_stop()
            except Exception as exc:
                self.get_logger().error(f"emergency stop command failed: {exc}")

    def destroy_node(self):
        if self.arm is not None and self.arm_connected:
            try:
                self.get_logger().info(
                    "Ctrl-C shutdown: disconnecting without motion or enable/disable commands"
                )
                self.arm.disconnect()
            except Exception as exc:
                self.get_logger().error(
                    f"failed to disconnect {self.robot_model}: {exc}"
                )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmKeyboardController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
