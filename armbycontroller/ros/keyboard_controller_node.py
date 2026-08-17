#!/usr/bin/env python3
"""Unified ROS 2 keyboard controller for Nero and Piper-L arms."""

import json
import math
import time
from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Sequence

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, PiperFW
from pyAgxArm import create_agx_arm_config
from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD

from armbycontroller.impedance.cartesian import cartesian_impedance_diagonals
from armbycontroller.impedance.cartesian import geometric_jacobian
from armbycontroller.impedance.admittance import CartesianAdmittance
from armbycontroller.impedance.admittance import estimate_cartesian_wrench
from armbycontroller.control import CartesianAdmittanceController
from armbycontroller.control import CartesianImpedanceController
from armbycontroller.control import ControlEngine
from armbycontroller.control import ControlInput
from armbycontroller.control import ControlReference
from armbycontroller.control import ControlSafetyError
from armbycontroller.control import ControlState
from armbycontroller.control import JointMitController
from armbycontroller.control import MitCommand
from armbycontroller.control import PositionCommand
from armbycontroller.control import bounded_model_feedforward as _bounded_model
from armbycontroller.control import control_sample
from armbycontroller.control import limit_mit_combined_torque as _limit_mit
from armbycontroller.hardware import connect_arm_two_stage
from armbycontroller.modeling.screw_model import UrdfScrewModel
from armbycontroller.ik.core import AgxIkEngine
from armbycontroller.ik.core import create_screw_solver
from armbycontroller.ik.core import IkFailure
from armbycontroller.ik.core import increment_tool_orientation
from armbycontroller.ik.core import prepare_planned_joint_mode
from armbycontroller.ik.core import resolve_urdf_path
from armbycontroller.ik.core import resolve_firmware_name
from armbycontroller.ik.core import resolve_tool_urdf_path
from armbycontroller.ik.core import set_joint_acceleration_limits
from armbycontroller.modeling.screw_model import project_gravity_vector


KEY_JOINT_1, KEY_JOINT_7 = 0, 6
KEY_DECREASE, KEY_INCREASE = 7, 8
KEY_HOME, KEY_ESTOP, KEY_MODE_TOGGLE = 9, 10, 11
KEY_FORWARD, KEY_BACKWARD, KEY_Z_UP, KEY_Z_DOWN = 12, 13, 14, 15
KEY_IMPEDANCE_TOGGLE = 16
KEY_ARROW_UP, KEY_ARROW_DOWN = 17, 18
KEY_ARROW_LEFT, KEY_ARROW_RIGHT = 19, 20
KEY_ROLL_LEFT, KEY_ROLL_RIGHT = 21, 22
KEY_ADMITTANCE_TOGGLE = 23
KEY_COUNT = 24


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
    admittance_toggle_requested: bool = False


@dataclass(frozen=True)
class MotorFeedback:
    """One cached SDK feedback sample used by all 100 Hz consumers."""

    position: np.ndarray
    velocity: np.ndarray
    torque: np.ndarray


def estimate_joint_velocity(
    previous_position,
    current_position,
    previous_velocity,
    period,
    time_constant,
):
    """Estimate joint velocity with a first-order low-pass differentiator."""
    previous = np.asarray(previous_position, dtype=float)
    current = np.asarray(current_position, dtype=float)
    velocity = np.asarray(previous_velocity, dtype=float)
    period = float(period)
    time_constant = float(time_constant)
    if (
        previous.ndim != 1
        or current.shape != previous.shape
        or velocity.shape != previous.shape
        or not all(
            np.all(np.isfinite(values))
            for values in (previous, current, velocity)
        )
        or not math.isfinite(period)
        or period <= 0.0
        or not math.isfinite(time_constant)
        or time_constant < 0.0
    ):
        raise ValueError("velocity estimator inputs are invalid")
    raw_velocity = (current - previous) / period
    if time_constant == 0.0:
        return raw_velocity
    alpha = period / (time_constant + period)
    return velocity + alpha * (raw_velocity - velocity)


class ArmJointJogState:
    """Track selection, key edges, and a limit-safe joint target."""

    def __init__(self, joint_limits, step_rad, initial_joints=None):
        if not 1 <= len(joint_limits) <= 7 or step_rad <= 0.0:
            raise ValueError("arm requires 1-7 limits and step_rad > 0")
        self.joint_count = len(joint_limits)
        self.joint_limits = [
            tuple(map(float, limit)) for limit in joint_limits
        ]
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
        admittance_toggle = rising[KEY_ADMITTANCE_TOGGLE]
        changed = False
        if home:
            target = [clamp(0.0, low, high) for low, high in self.joint_limits]
            changed, self.target_joints = target != self.target_joints, target
        elif not any((estop, toggle, impedance_toggle, admittance_toggle)):
            direction = pressed[KEY_INCREASE] - pressed[KEY_DECREASE]
            if direction:
                low, high = self.joint_limits[self.selected_joint]
                old = self.target_joints[self.selected_joint]
                if old < low:
                    new = (
                        min(old + self.step_rad, low)
                        if direction > 0 else old
                    )
                elif old > high:
                    new = (
                        max(old - self.step_rad, high)
                        if direction < 0 else old
                    )
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
            admittance_toggle,
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
    },
    "piper_l": {
        "kp": [0.3, 0.5, 0.5, 0.5, 1.0, 0.3],
        "kd": [0.01, 0.01, 0.01, 0.01, 0.01, 0.01],
        # Dynamic URDF inverse dynamics supplies the nominal torque.
        # Additional feedforward is zero; no calibration bias is applied.
        "t_ff": [0.0] * 6,
    },
}


def default_mit_gains(robot_model):
    """Return independent per-joint MIT gains for a supported arm."""
    profile = MIT_GAIN_PROFILES[robot_model]
    return list(profile["kp"]), list(profile["kd"])


def default_mit_feedforward(robot_model):
    """Return per-joint MIT feedforward torque for a supported arm."""
    return list(MIT_GAIN_PROFILES[robot_model]["t_ff"])


def extract_joint_angles(result, joint_count):
    if result is None:
        return None
    value = result.msg if hasattr(result, "msg") else result
    if isinstance(value, Iterable) and not isinstance(
        value, (str, bytes, dict)
    ):
        joints = [float(item) for item in value]
        return joints[:joint_count] if len(joints) >= joint_count else None
    if isinstance(value, dict):
        for key in ("joint_angles", "angles", "data", "msg"):
            joints = value.get(key)
            if (
                isinstance(joints, (list, tuple))
                and len(joints) >= joint_count
            ):
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
        raise ValueError(
            f"{name} must contain 1 or {joint_count} finite values"
        )
    return result


def bounded_model_feedforward(model_torque, scale, torque_limit):
    """Compatibility export; implementation lives at the controller seam."""
    return _bounded_model(model_torque, scale, torque_limit)


def limit_mit_combined_torque(
    feedforward,
    reference_position,
    reference_velocity,
    measured_position,
    measured_velocity,
    kp,
    kd,
    torque_limit,
):
    """Compatibility export; implementation lives at the controller seam."""
    return _limit_mit(
        feedforward,
        reference_position,
        reference_velocity,
        measured_position,
        measured_velocity,
        kp,
        kd,
        torque_limit,
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
            raise ValueError(
                "trajectory limits must be positive per-joint values"
            )
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
            (
                np.abs(goals - new_position)
                <= 25.0 * self.max_jerk * period ** 3
            )
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


class ArmKeyboardController(Node):
    def __init__(self):
        super().__init__("arm_keyboard_controller")

        self.declare_parameter("robot_model", "nero")
        declared_model = str(
            self.get_parameter("robot_model").value
        ).lower()
        gain_model = (
            declared_model if declared_model in MODEL_PROFILES else "nero"
        )
        default_mit_kp, default_mit_kd = default_mit_gains(gain_model)
        default_mit_feedforward_values = default_mit_feedforward(gain_model)
        self.declare_parameter("keyboard_topic", "/arm_keyboard_state")
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("firmware", "auto")
        self.declare_parameter("firmware_probe_timeout", 5.0)
        self.declare_parameter("firmware_probe_poll_period", 0.1)
        self.declare_parameter("firmware_reconnect_delay", 0.5)
        self.declare_parameter("nero_mount", "")
        self.declare_parameter("tool_configuration", "auto")
        self.declare_parameter("nero_velocity_estimation_enabled", True)
        self.declare_parameter("velocity_filter_time_constant", 0.03)
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
        self.declare_parameter("impedance_backend", "cartesian")
        self.declare_parameter("mit_command_rate", 100.0)
        self.declare_parameter(
            "dynamics_state_topic", "/arm_dynamics_state"
        )
        self.declare_parameter(
            "external_torque_topic", "/arm_external_joint_torque"
        )
        self.declare_parameter("control_sample_topic", "/arm_control_sample")
        self.declare_parameter("control_event_topic", "/arm_control_event")
        self.declare_parameter("mit_kp", default_mit_kp)
        self.declare_parameter("mit_kd", default_mit_kd)
        scalar_or_array = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter(
            "mit_feedforward", default_mit_feedforward_values
        )
        self.declare_parameter("mit_gravity_compensation_enabled", True)
        self.declare_parameter("gravity_urdf_path", "")
        self.declare_parameter("mit_gravity_scale", 1.0)
        self.declare_parameter(
            "mit_gravity_torque_limit", [10.0], scalar_or_array
        )
        self.declare_parameter(
            "cartesian_impedance_rotation_stiffness", 0.4
        )
        self.declare_parameter(
            "cartesian_impedance_base_z_rotation_stiffness", 4.0
        )
        self.declare_parameter(
            "cartesian_impedance_translation_stiffness", 10.0
        )
        self.declare_parameter(
            "cartesian_impedance_rotation_damping", 0.08
        )
        self.declare_parameter(
            "cartesian_impedance_translation_damping", 0.8
        )
        self.declare_parameter(
            "cartesian_impedance_nullspace_stiffness",
            [0.4],
            scalar_or_array,
        )
        self.declare_parameter(
            "cartesian_impedance_nullspace_damping",
            [0.1],
            scalar_or_array,
        )
        self.declare_parameter(
            "cartesian_impedance_joint_posture_stiffness",
            [0.0],
            scalar_or_array,
        )
        self.declare_parameter(
            "cartesian_impedance_joint_posture_damping",
            [0.0],
            scalar_or_array,
        )
        self.declare_parameter(
            "cartesian_impedance_torque_limit", [8.0], scalar_or_array
        )
        self.declare_parameter(
            "cartesian_impedance_model_scale", [1.0], scalar_or_array
        )
        self.declare_parameter(
            "admittance_virtual_mass",
            [0.12, 0.12, 0.12, 1.5, 1.5, 1.5],
        )
        self.declare_parameter(
            "admittance_damping", [0.8, 0.8, 0.8, 8.0, 8.0, 8.0]
        )
        self.declare_parameter(
            "admittance_stiffness", [0.8, 0.8, 0.8, 8.0, 8.0, 8.0]
        )
        self.declare_parameter(
            "admittance_wrench_deadband",
            [0.03, 0.03, 0.03, 0.15, 0.15, 0.15],
        )
        self.declare_parameter(
            "admittance_wrench_limit", [2.0, 2.0, 2.0, 8.0, 8.0, 8.0]
        )
        self.declare_parameter(
            "admittance_offset_limit",
            [0.35, 0.35, 0.35, 0.10, 0.10, 0.10],
        )
        self.declare_parameter(
            "admittance_velocity_limit",
            [0.5, 0.5, 0.5, 0.15, 0.15, 0.15],
        )
        self.declare_parameter("admittance_wrench_filter_hz", 5.0)
        self.declare_parameter("admittance_wrench_dls_damping", 0.05)
        self.declare_parameter("admittance_wrench_timeout", 0.10)
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

        self.robot_model = str(
            self.get_parameter("robot_model").value
        ).lower()
        if self.robot_model not in MODEL_PROFILES:
            raise ValueError("robot_model must be nero or piper_l")
        self.profile = MODEL_PROFILES[self.robot_model]
        self.joint_count = self.profile["joint_count"]
        self.keyboard_topic = str(self.get_parameter("keyboard_topic").value)
        self.can_interface = str(self.get_parameter("can_interface").value)
        self.requested_firmware_name = str(
            self.get_parameter("firmware").value
        ).lower()
        self.firmware_name = resolve_firmware_name(
            self.robot_model, self.requested_firmware_name
        )
        self.firmware_probe_timeout = float(
            self.get_parameter("firmware_probe_timeout").value
        )
        self.firmware_probe_poll_period = float(
            self.get_parameter("firmware_probe_poll_period").value
        )
        self.firmware_reconnect_delay = float(
            self.get_parameter("firmware_reconnect_delay").value
        )
        self.nero_mount = str(
            self.get_parameter("nero_mount").value
        ).strip().lower()
        self.tool_configuration = str(
            self.get_parameter("tool_configuration").value
        ).strip().lower()
        self.nero_velocity_estimation_enabled = bool(
            self.get_parameter("nero_velocity_estimation_enabled").value
        )
        self.velocity_filter_time_constant = float(
            self.get_parameter("velocity_filter_time_constant").value
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
        self.keyboard_timeout = float(
            self.get_parameter("keyboard_timeout").value
        )
        self.enable_timeout = float(self.get_parameter("enable_timeout").value)
        self.feedback_timeout = float(
            self.get_parameter("feedback_timeout").value
        )
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
        start_in_impedance = self.impedance_enabled
        self.impedance_enabled = False
        self.impedance_backend = str(
            self.get_parameter("impedance_backend").value
        ).lower()
        self.mit_command_rate = float(
            self.get_parameter("mit_command_rate").value
        )
        self.dynamics_state_topic = str(
            self.get_parameter("dynamics_state_topic").value
        )
        self.external_torque_topic = str(
            self.get_parameter("external_torque_topic").value
        )
        self.control_sample_topic = str(
            self.get_parameter("control_sample_topic").value
        )
        self.control_event_topic = str(
            self.get_parameter("control_event_topic").value
        )
        self.mit_kp = expand_joint_values(
            self.get_parameter("mit_kp").value, self.joint_count, "mit_kp"
        )
        self.mit_kd = expand_joint_values(
            self.get_parameter("mit_kd").value, self.joint_count, "mit_kd"
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
        self.mit_gravity_torque_limit = expand_joint_values(
            self.get_parameter("mit_gravity_torque_limit").value,
            self.joint_count,
            "mit_gravity_torque_limit",
        )
        rotation_stiffness = float(
            self.get_parameter(
                "cartesian_impedance_rotation_stiffness"
            ).value
        )
        base_z_rotation_stiffness = float(
            self.get_parameter(
                "cartesian_impedance_base_z_rotation_stiffness"
            ).value
        )
        translation_stiffness = float(
            self.get_parameter(
                "cartesian_impedance_translation_stiffness"
            ).value
        )
        rotation_damping = float(
            self.get_parameter(
                "cartesian_impedance_rotation_damping"
            ).value
        )
        translation_damping = float(
            self.get_parameter(
                "cartesian_impedance_translation_damping"
            ).value
        )
        (
            self.cartesian_stiffness,
            self.cartesian_damping,
        ) = cartesian_impedance_diagonals(
            rotation_stiffness,
            base_z_rotation_stiffness,
            translation_stiffness,
            rotation_damping,
            translation_damping,
        )
        self.cartesian_nullspace_stiffness = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_nullspace_stiffness"
                ).value,
                self.joint_count,
                "cartesian_impedance_nullspace_stiffness",
            ),
            dtype=float,
        )
        self.cartesian_nullspace_damping = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_nullspace_damping"
                ).value,
                self.joint_count,
                "cartesian_impedance_nullspace_damping",
            ),
            dtype=float,
        )
        self.cartesian_joint_posture_stiffness = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_joint_posture_stiffness"
                ).value,
                self.joint_count,
                "cartesian_impedance_joint_posture_stiffness",
            ),
            dtype=float,
        )
        self.cartesian_joint_posture_damping = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_joint_posture_damping"
                ).value,
                self.joint_count,
                "cartesian_impedance_joint_posture_damping",
            ),
            dtype=float,
        )
        self.cartesian_torque_limit = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_torque_limit"
                ).value,
                self.joint_count,
                "cartesian_impedance_torque_limit",
            ),
            dtype=float,
        )
        self.cartesian_model_scale = np.asarray(
            expand_joint_values(
                self.get_parameter(
                    "cartesian_impedance_model_scale"
                ).value,
                self.joint_count,
                "cartesian_impedance_model_scale",
            ),
            dtype=float,
        )
        task_parameter_names = (
            "admittance_virtual_mass",
            "admittance_damping",
            "admittance_stiffness",
            "admittance_wrench_deadband",
            "admittance_wrench_limit",
            "admittance_offset_limit",
            "admittance_velocity_limit",
        )
        admittance_values = {
            name: expand_joint_values(
                self.get_parameter(name).value, 6, name
            )
            for name in task_parameter_names
        }
        self.admittance_wrench_filter_hz = float(
            self.get_parameter("admittance_wrench_filter_hz").value
        )
        self.admittance_wrench_dls_damping = float(
            self.get_parameter("admittance_wrench_dls_damping").value
        )
        self.admittance_wrench_timeout = float(
            self.get_parameter("admittance_wrench_timeout").value
        )
        self.admittance_controller = CartesianAdmittance(
            virtual_mass=admittance_values["admittance_virtual_mass"],
            damping=admittance_values["admittance_damping"],
            stiffness=admittance_values["admittance_stiffness"],
            wrench_deadband=admittance_values[
                "admittance_wrench_deadband"
            ],
            wrench_limit=admittance_values["admittance_wrench_limit"],
            offset_limit=admittance_values["admittance_offset_limit"],
            velocity_limit=admittance_values[
                "admittance_velocity_limit"
            ],
            wrench_filter_hz=self.admittance_wrench_filter_hz,
        )
        configured_gravity = self.get_parameter("gravity_vector").value
        if self.robot_model == "nero" and self.nero_mount:
            self.gravity_vector = np.asarray(
                project_gravity_vector(self.nero_mount), dtype=float
            )
        else:
            self.gravity_vector = np.asarray(
                configured_gravity, dtype=float
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

        if self.control_rate <= 0.0:
            raise ValueError("control_rate must be > 0")
        if (
            not math.isfinite(self.velocity_filter_time_constant)
            or self.velocity_filter_time_constant < 0.0
        ):
            raise ValueError(
                "velocity_filter_time_constant must be finite and >= 0"
            )
        if not 1 <= self.speed_percent <= 100:
            raise ValueError("speed_percent must be in [1, 100]")
        if self.joint_max_acceleration <= 0.0:
            raise ValueError("joint_max_acceleration must be > 0")
        if self.joint_acc_timeout <= 0.0:
            raise ValueError("joint_acc_timeout must be > 0")
        if self.position_mode_timeout <= 0.0:
            raise ValueError("position_mode_timeout must be > 0")
        if (
            not math.isfinite(self.firmware_probe_timeout)
            or self.firmware_probe_timeout <= 0.0
        ):
            raise ValueError("firmware_probe_timeout must be finite and > 0")
        if (
            not math.isfinite(self.firmware_probe_poll_period)
            or self.firmware_probe_poll_period < 0.0
        ):
            raise ValueError(
                "firmware_probe_poll_period must be finite and >= 0"
            )
        if (
            not math.isfinite(self.firmware_reconnect_delay)
            or self.firmware_reconnect_delay < 0.0
        ):
            raise ValueError(
                "firmware_reconnect_delay must be finite and >= 0"
            )
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
        if (
            self.admittance_wrench_dls_damping <= 0.0
            or self.admittance_wrench_timeout <= 0.0
        ):
            raise ValueError(
                "admittance DLS damping and wrench timeout must be positive"
            )
        if self.impedance_backend not in ("joint", "cartesian"):
            raise ValueError(
                "impedance_backend must be joint or cartesian"
            )
        if not 0.0 <= self.mit_gravity_scale <= 1.0:
            raise ValueError("mit_gravity_scale must be in [0, 1]")
        if self.gravity_vector.shape != (3,) or not np.all(
            np.isfinite(self.gravity_vector)
        ):
            raise ValueError("gravity_vector must contain three finite values")
        if any(not 0.0 <= value <= 500.0 for value in self.mit_kp):
            raise ValueError("mit_kp values must be in [0, 500]")
        if any(not 0.0 <= value <= 5.0 for value in self.mit_kd):
            raise ValueError("mit_kd values must be in [0, 5]")
        if any(abs(value) > 10.0 for value in self.mit_feedforward):
            raise ValueError(
                "mit_feedforward values must be in [-10, 10] N·m"
            )
        if any(
            not 0.0 < value <= 10.0
            for value in self.mit_gravity_torque_limit
        ):
            raise ValueError(
                "MIT model torque limits must be in (0, 10] N·m"
            )
        if (
            not np.all(np.isfinite(self.cartesian_stiffness))
            or not np.all(np.isfinite(self.cartesian_damping))
            or np.any(self.cartesian_stiffness < 0.0)
            or np.any(self.cartesian_damping < 0.0)
            or np.any(self.cartesian_nullspace_stiffness < 0.0)
            or np.any(self.cartesian_nullspace_damping < 0.0)
            or np.any(self.cartesian_joint_posture_stiffness < 0.0)
            or np.any(self.cartesian_joint_posture_damping < 0.0)
        ):
            raise ValueError(
                "Cartesian, nullspace, and joint-posture gains must be "
                "nonnegative"
            )
        if (
            not np.all(np.isfinite(self.cartesian_torque_limit))
            or np.any(self.cartesian_torque_limit <= 0.0)
            or np.any(self.cartesian_torque_limit > 10.0)
        ):
            raise ValueError(
                "Cartesian absolute torque limits must be in (0, 10] N·m"
            )
        if (
            not np.all(np.isfinite(self.cartesian_model_scale))
            or np.any(self.cartesian_model_scale < 0.0)
            or np.any(self.cartesian_model_scale > 1.0)
        ):
            raise ValueError(
                "Cartesian model scales must be in [0, 1]"
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
            tuple(
                ROBOT_JOINT_LIMIT_PRESET_RAD[
                    self.robot_model
                ][f"joint{index}"]
            )
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
        equipped_urdf_path = resolve_tool_urdf_path(
            urdf_path,
            self.robot_model,
            self.tool_configuration,
            self.gravity_urdf_path,
        )
        self.gravity_model = None
        if self.mit_gravity_compensation_enabled:
            self.gravity_model = UrdfScrewModel(
                equipped_urdf_path,
                self.base_frame,
                self.tip_link,
                self.joint_count,
                self.gravity_vector,
            )
            self.get_logger().info(
                "URDF inverse dynamics ready: "
                f"{equipped_urdf_path}; "
                f"modeled_mass={self.gravity_model.modeled_mass:.3f} kg; "
                f"gravity={self.gravity_vector.tolist()}; "
                f"joints={self.gravity_model.movable_joint_names}"
            )
        self.ik_solver = create_screw_solver(
            equipped_urdf_path,
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
        self.control_engine = self._build_control_engine()
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
        self.last_mit_tick_time = None
        self.admittance_enabled = False
        self.admittance_previous_control_mode = "joint"
        self.last_admittance_tick_time = None
        self.feedback_previous_position = None
        self.feedback_previous_velocity = np.zeros(self.joint_count)
        self.feedback_previous_time = None
        self.latest_external_wrench = np.zeros(6)
        self.latest_external_wrench_received_at = -math.inf

        self.keyboard_sub = self.create_subscription(
            Int32MultiArray,
            self.keyboard_topic,
            self.keyboard_callback,
            qos_profile_sensor_data,
        )
        self.dynamics_state_publisher = self.create_publisher(
            JointState, self.dynamics_state_topic, 20
        )
        self.control_sample_publisher = self.create_publisher(
            String, self.control_sample_topic, 100
        )
        self.control_event_publisher = self.create_publisher(
            String, self.control_event_topic, 20
        )
        self.external_torque_sub = self.create_subscription(
            JointState,
            self.external_torque_topic,
            self.external_torque_callback,
            20,
        )
        self.connect_arm()
        self.mit_trajectory.reset(self.jog.target_joints)
        if start_in_impedance:
            self.toggle_impedance()
        self.timer = self.create_timer(
            1.0 / self.control_rate, self.control_tick
        )
        self.dynamics_timer = self.create_timer(
            1.0 / self.mit_command_rate, self.mit_tick
        )

        self.get_logger().info(
            f"{self.robot_model} ready: P=joint/IK; "
            f"joint 1-{self.joint_count}+A/D; "
            "IK W/S+A/D+Z/X; I="
            f"{self.impedance_backend} impedance via MIT; O=admittance; "
            "SPACE=home; E=E-stop"
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

    def _build_control_engine(self):
        """Create pure controller adapters from the validated node settings."""
        controllers = []
        joint_count = int(getattr(
            self, "joint_count", len(self.jog.target_joints)
        ))
        joint_settings = ("mit_kp", "mit_kd", "mit_feedforward")
        if all(hasattr(self, name) for name in joint_settings):
            controllers.append(JointMitController(
                joint_count,
                self.mit_kp,
                self.mit_kd,
                self.mit_feedforward,
                getattr(
                    self,
                    "mit_gravity_torque_limit",
                    [10.0] * joint_count,
                ),
                dynamics_model=getattr(self, "gravity_model", None),
                model_scale=getattr(self, "mit_gravity_scale", 1.0),
            ))

        model = getattr(self, "gravity_model", None)
        if model is None:
            model = getattr(getattr(self, "ik_solver", None), "model", None)
        cartesian_settings = (
            "cartesian_stiffness",
            "cartesian_damping",
            "cartesian_torque_limit",
        )
        if model is not None and all(
            hasattr(self, name) for name in cartesian_settings
        ):
            controllers.append(CartesianImpedanceController(
                model,
                self.cartesian_stiffness,
                self.cartesian_damping,
                self.cartesian_torque_limit,
                nullspace_stiffness=getattr(
                    self, "cartesian_nullspace_stiffness", None
                ),
                nullspace_damping=getattr(
                    self, "cartesian_nullspace_damping", None
                ),
                nullspace_enabled=(
                    getattr(self, "robot_model", "piper_l") == "nero"
                ),
                joint_posture_stiffness=getattr(
                    self, "cartesian_joint_posture_stiffness", None
                ),
                joint_posture_damping=getattr(
                    self, "cartesian_joint_posture_damping", None
                ),
                model_scale=getattr(
                    self,
                    "cartesian_model_scale",
                    np.ones(joint_count),
                ),
            ))
        if (
            hasattr(self, "admittance_controller")
            and hasattr(self, "ik_engine")
        ):
            controllers.append(CartesianAdmittanceController(
                model,
                self.admittance_controller,
                self.ik_engine,
                getattr(self, "mit_max_joint_step", 0.05),
                joint_count=joint_count,
            ))
        return ControlEngine(controllers)

    def _get_control_engine(self, controller_name):
        engine = getattr(self, "control_engine", None)
        if engine is None or controller_name not in engine.available:
            engine = self._build_control_engine()
            self.control_engine = engine
        if controller_name not in engine.available:
            raise RuntimeError(
                f"controller adapter {controller_name!r} is unavailable"
            )
        return engine

    def _next_control_input(self, feedback=None, external_wrench=None):
        """Capture one shared sample for a controller adapter."""
        now = time.monotonic()
        nominal = 1.0 / getattr(self, "mit_command_rate", 100.0)
        previous = getattr(self, "last_mit_tick_time", None)
        period = (
            nominal
            if previous is None
            else float(np.clip(now - previous, 0.25 * nominal, 2.0 * nominal))
        )
        self.last_mit_tick_time = now
        trajectory = getattr(self, "mit_trajectory", None)
        if trajectory is None:
            reference_position = np.asarray(
                self.jog.target_joints, dtype=float
            )
            reference_velocity = np.zeros(self.joint_count)
            reference_acceleration = np.zeros(self.joint_count)
        else:
            (
                reference_position,
                reference_velocity,
                reference_acceleration,
            ) = trajectory.step(self.jog.target_joints, period)

        position = feedback.position if feedback is not None else None
        velocity = feedback.velocity if feedback is not None else None
        effort = feedback.torque if feedback is not None else None
        if position is None:
            try:
                position = extract_joint_angles(
                    self.arm.get_joint_angles(), self.joint_count
                )
            except Exception:
                position = None
        if velocity is None:
            velocity = self.read_joint_velocities()
        position_valid = position is not None
        velocity_valid = velocity is not None
        effort_valid = effort is not None
        if position is None:
            position = reference_position
        if velocity is None:
            velocity = np.zeros(self.joint_count)
        if effort is None:
            effort = np.zeros(self.joint_count)
        state = ControlState(
            position,
            velocity,
            effort,
            position_valid=position_valid,
            velocity_valid=velocity_valid,
            effort_valid=effort_valid,
        )
        reference = ControlReference(
            reference_position,
            reference_velocity,
            reference_acceleration,
            np.zeros(6) if external_wrench is None else external_wrench,
        )
        return ControlInput(now, period, state, reference)

    def _publish_control_result(self, sample, result, interaction_mode):
        publisher = getattr(self, "control_sample_publisher", None)
        if publisher is None:
            return
        message = String()
        message.data = json.dumps(
            control_sample(
                sample,
                result,
                robot_model=getattr(self, "robot_model", "unknown"),
                interaction_mode=interaction_mode,
            ),
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        publisher.publish(message)

    def _publish_control_event(self, event, **fields):
        publisher = getattr(self, "control_event_publisher", None)
        if publisher is None:
            return
        message = String()
        message.data = json.dumps(
            {
                "timestamp": time.monotonic(),
                "robot_model": getattr(self, "robot_model", "unknown"),
                "event": str(event),
                **fields,
            },
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        publisher.publish(message)

    def _send_control_result(self, result):
        command = result.command
        if isinstance(command, MitCommand):
            for index in range(command.position.size):
                self.arm.move_mit(
                    joint_index=index + 1,
                    p_des=float(command.position[index]),
                    v_des=float(command.velocity[index]),
                    kp=float(command.kp[index]),
                    kd=float(command.kd[index]),
                    t_ff=float(command.feedforward[index]),
                )
            return
        if isinstance(command, PositionCommand):
            self.jog.sync_target(command.position)
            self.send_target("Cartesian admittance")
            return
        raise TypeError(
            f"unsupported control command {type(command).__name__}"
        )

    def connect_arm(self):
        self.device_firmware_info = {}

        if not self.execute_motion:
            config = create_agx_arm_config(
                robot=self.profile["arm_model"],
                firmeware_version=self.firmware,
                interface="socketcan",
                channel=self.can_interface,
            )
            self.arm = AgxArmFactory.create_arm(config)
            self.arm.set_joint_limits_enabled(True)
            self.get_logger().warning(
                f"dry-run mode: {self.robot_model} commands are disabled"
            )
            self.arm_ready = True
            return

        try:
            connection = connect_arm_two_stage(
                robot_model=self.robot_model,
                arm_model=self.profile["arm_model"],
                firmware_profiles=self.profile["firmwares"],
                can_interface=self.can_interface,
                probe_timeout=self.firmware_probe_timeout,
                probe_poll_period=self.firmware_probe_poll_period,
                reconnect_delay=self.firmware_reconnect_delay,
                report=self.get_logger().info,
            )
            self.arm = connection.arm
            self.device_firmware_info = connection.firmware_info
            detected_name = connection.firmware_profile
            if (
                self.requested_firmware_name != "auto"
                and detected_name != self.firmware_name
            ):
                self.get_logger().warning(
                    f"configured firmware {self.firmware_name} differs from "
                    f"detected {detected_name}; using detected profile"
                )
            self.firmware_name = detected_name
            self.firmware = self.profile["firmwares"][detected_name]
            self.arm.set_joint_limits_enabled(True)
            self.arm_connected = True
            self.get_logger().info(
                f"connected {self.robot_model} on {self.can_interface} "
                f"with detected firmware profile {self.firmware_name}; "
                f"saved device data: {self.device_firmware_info}"
            )
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
                    "startup safety check: no electronic emergency stop "
                    "reported"
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
        if self.admittance_enabled:
            permitted = {
                KEY_ESTOP,
                KEY_IMPEDANCE_TOGGLE,
                KEY_ADMITTANCE_TOGGLE,
            }
            state_keys = [
                value if index in permitted else 0
                for index, value in enumerate(state_keys)
            ]
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

        if (
            update.impedance_toggle_requested
            and update.admittance_toggle_requested
        ):
            self.get_logger().warning(
                "I and O were pressed together; interaction mode unchanged"
            )
            return

        if update.admittance_toggle_requested:
            self.toggle_admittance()
            return

        if update.impedance_toggle_requested:
            self.toggle_impedance()
            return

        if self.admittance_enabled:
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
        if self.admittance_enabled:
            self.get_logger().warning(
                "P is locked while admittance is active; press O to exit"
            )
            return
        if self.control_mode == "joint":
            joints = self.current_or_target_joints()
            if self.impedance_enabled:
                self.jog.sync_target(joints, clamp_to_limits=False)
                trajectory = getattr(self, "mit_trajectory", None)
                if trajectory is not None:
                    trajectory.reset(joints)
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
            self.jog.sync_target(
                joints, clamp_to_limits=not self.impedance_enabled
            )
            if self.impedance_enabled:
                trajectory = getattr(self, "mit_trajectory", None)
                if trajectory is not None:
                    trajectory.reset(joints)
            self.control_mode = "joint"
            self.get_logger().info(
                f"mode=JOINT: 1-{self.joint_count} select, A/D jog"
            )

    def toggle_impedance(self):
        """Switch between planned motion and the selected MIT backend."""
        entering = not self.impedance_enabled
        if entering and getattr(self, "admittance_enabled", False):
            self._exit_admittance("switching to impedance")
            if self.admittance_enabled:
                self.get_logger().error(
                    "cannot enter impedance: admittance did not exit"
                )
                return
        backend = getattr(self, "impedance_backend", "joint")
        if entering and self.execute_motion:
            if not self.arm_ready:
                self.get_logger().error(
                    "cannot enter MIT: arm is not ready"
                )
                return
            joints = None
            try:
                joints = extract_joint_angles(
                    self.arm.get_joint_angles(), self.joint_count
                )
            except Exception:
                pass
            velocities = self.read_joint_velocities()
            if joints is None or velocities is None:
                self.get_logger().error(
                    "cannot enter MIT: complete q/dq feedback is required"
                )
                return
            if self.gravity_model is None:
                self.get_logger().error(
                    "cannot enter MIT: URDF inverse dynamics is unavailable"
                )
                return
            try:
                support = np.asarray(
                    self.gravity_model.inverse_dynamics(
                        joints,
                        velocities,
                        np.zeros(self.joint_count),
                    ),
                    dtype=float,
                )
            except Exception as exc:
                self.get_logger().error(
                    f"cannot enter MIT: inverse dynamics failed: {exc}"
                )
                return
            if support.shape != (self.joint_count,) or not np.all(
                np.isfinite(support)
            ):
                self.get_logger().error(
                    "cannot enter MIT: inverse dynamics torque is invalid"
                )
                return
            if backend == "cartesian":
                try:
                    state = ControlState(
                        joints,
                        velocities,
                        np.zeros(self.joint_count),
                        effort_valid=False,
                    )
                    preflight = ControlInput(
                        time.monotonic(),
                        1.0 / self.mit_command_rate,
                        state,
                        ControlReference.hold(joints),
                    )
                    engine = self._get_control_engine(
                        "cartesian_impedance"
                    )
                    engine.reset("cartesian_impedance", state)
                    engine.step("cartesian_impedance", preflight)
                except Exception as exc:
                    self.get_logger().error(
                        "cannot enter Cartesian MIT: formula preflight "
                        f"failed: {exc}"
                    )
                    return
        else:
            joints = self.current_or_target_joints()
        self.jog.sync_target(joints, clamp_to_limits=False)
        self.impedance_enabled = not self.impedance_enabled
        self._check_interaction_mode_invariant()
        if self.impedance_enabled:
            self.last_mit_tick_time = None
            self.mit_trajectory.reset(joints)
            selected = (
                "cartesian_impedance"
                if backend == "cartesian"
                else "joint_impedance"
            )
            if (
                getattr(self, "control_engine", None) is not None
                or all(
                    hasattr(self, name)
                    for name in ("mit_kp", "mit_kd", "mit_feedforward")
                )
            ):
                state = ControlState(
                    joints,
                    np.zeros(self.joint_count),
                    np.zeros(self.joint_count),
                    velocity_valid=False,
                    effort_valid=False,
                )
                self._get_control_engine(selected).reset(selected, state)
            if backend == "cartesian":
                pose = self.gravity_model.forward_kinematics(joints)
                self.ik_target_position = np.asarray(
                    pose[:3, 3], dtype=float
                )
                self.ik_target_rotation = np.asarray(
                    pose[:3, :3], dtype=float
                )
                self.ik_valid_history.clear()
                self.remember_ik_valid(
                    self.ik_target_position,
                    self.ik_target_rotation,
                    joints,
                )
            self.get_logger().warning(
                f"backend={backend} impedance via MIT + "
                f"control={self.control_mode.upper()}: "
                + (
                    "holding current tool pose"
                    if backend == "cartesian"
                    else "holding current joints"
                )
            )
            self._publish_control_event(
                "controller_enabled", controller=selected
            )
            self.mit_tick()
            return
        if self.execute_motion and not prepare_planned_joint_mode(
            self.arm, self.position_mode_timeout
        ):
            self.get_logger().error(
                "failed to restore planned joint mode after MIT exit"
            )
            return
        self.get_logger().info("backend=planned position control")
        self._publish_control_event(
            "controller_disabled", controller=f"{backend}_impedance"
        )
        self.send_target("MIT exit hold")

    def _check_interaction_mode_invariant(self):
        if (
            getattr(self, "impedance_enabled", False)
            and getattr(self, "admittance_enabled", False)
        ):
            raise RuntimeError(
                "impedance and admittance cannot be active together"
            )

    def toggle_admittance(self):
        """Toggle Piper-L Cartesian admittance in planned joint mode."""
        if getattr(self, "admittance_enabled", False):
            self._exit_admittance("O toggle")
            return
        if self.robot_model != "piper_l":
            self.get_logger().error(
                "Cartesian admittance is currently implemented for "
                "piper_l only"
            )
            return
        if self.impedance_enabled:
            self.toggle_impedance()
            if self.impedance_enabled:
                self.get_logger().error(
                    "cannot enter admittance: impedance did not exit"
                )
                return
        if self.execute_motion and not self.arm_ready:
            self.get_logger().error(
                "cannot enter admittance: arm is not ready"
            )
            return
        model = getattr(self, "gravity_model", None)
        if model is None:
            self.get_logger().error(
                "cannot enter admittance: URDF screw model is unavailable"
            )
            return
        feedback = self.read_motor_feedback() if self.execute_motion else None
        if self.execute_motion and feedback is None:
            self.get_logger().error(
                "cannot enter admittance: complete q/dq/torque feedback "
                "is required"
            )
            return
        if (
            self.execute_motion
            and time.monotonic() - self.latest_external_wrench_received_at
            > self.admittance_wrench_timeout
        ):
            self.get_logger().error(
                "cannot enter admittance: momentum-observer wrench is stale; "
                "check /arm_external_joint_torque"
            )
            return
        joints = (
            feedback.position
            if feedback is not None
            else self.current_or_target_joints()
        )
        if self.execute_motion and not prepare_planned_joint_mode(
            self.arm, self.position_mode_timeout
        ):
            self.get_logger().error(
                "cannot enter admittance: planned joint mode timed out"
            )
            return
        pose = model.forward_kinematics(joints)
        self.admittance_previous_control_mode = self.control_mode
        self.control_mode = "ik"
        self.jog.sync_target(joints, clamp_to_limits=False)
        self.mit_trajectory.reset(joints)
        self.ik_target_position = np.asarray(pose[:3, 3], dtype=float)
        self.ik_target_rotation = np.asarray(pose[:3, :3], dtype=float)
        self.ik_valid_history.clear()
        self.remember_ik_valid(
            self.ik_target_position, self.ik_target_rotation, joints
        )
        joint_count = int(getattr(
            self, "joint_count", len(self.jog.target_joints)
        ))
        state = ControlState(
            joints,
            feedback.velocity
            if feedback is not None else np.zeros(joint_count),
            feedback.torque
            if feedback is not None else np.zeros(joint_count),
            velocity_valid=feedback is not None,
            effort_valid=feedback is not None,
        )
        if hasattr(self, "ik_engine"):
            engine = self._get_control_engine("cartesian_admittance")
            engine.reset("cartesian_admittance", state)
        else:
            # Retain the legacy formula seam for isolated state-machine tests.
            self.admittance_controller.reset(pose)
        self.latest_external_wrench = np.zeros(6)
        self.latest_external_wrench_received_at = time.monotonic()
        self.last_admittance_tick_time = None
        self.admittance_enabled = True
        self._check_interaction_mode_invariant()
        self.get_logger().warning(
            "backend=planned joint + control=Cartesian admittance; "
            "I is interlocked; press O to exit"
        )
        self._publish_control_event(
            "controller_enabled", controller="cartesian_admittance"
        )

    def _exit_admittance(self, reason):
        """Leave admittance while holding the latest measured position."""
        if not getattr(self, "admittance_enabled", False):
            return
        self.admittance_enabled = False
        self.last_admittance_tick_time = None
        joints = self.current_or_target_joints()
        self.jog.sync_target(joints, clamp_to_limits=False)
        model = getattr(self, "gravity_model", None)
        if model is not None:
            pose = model.forward_kinematics(joints)
            self.ik_target_position = np.asarray(pose[:3, 3], dtype=float)
            self.ik_target_rotation = np.asarray(pose[:3, :3], dtype=float)
        self.control_mode = self.admittance_previous_control_mode
        self._check_interaction_mode_invariant()
        self.send_target(f"admittance exit hold ({reason})")
        self._publish_control_event(
            "controller_disabled",
            controller="cartesian_admittance",
            reason=str(reason),
        )
        self.get_logger().info("Cartesian admittance exited")

    def external_torque_callback(self, message):
        """Convert observer residual torque to a base-frame wrench."""
        if self.robot_model != "piper_l":
            return
        model = getattr(self, "gravity_model", None)
        if model is None:
            return
        try:
            joints = np.asarray(message.position, dtype=float)
            external_torque = np.asarray(message.effort, dtype=float)
            if (
                joints.shape != (self.joint_count,)
                or external_torque.shape != (self.joint_count,)
                or not np.all(np.isfinite(joints))
                or not np.all(np.isfinite(external_torque))
            ):
                raise ValueError("external torque sample is incomplete")
            jacobian, _ = geometric_jacobian(model, joints)
            wrench = estimate_cartesian_wrench(
                jacobian,
                external_torque,
                self.admittance_wrench_dls_damping,
            )
        except Exception as error:
            self.get_logger().warning(
                f"external torque sample rejected: {error}",
                throttle_duration_sec=1.0,
            )
            return
        self.latest_external_wrench = wrench
        self.latest_external_wrench_received_at = time.monotonic()

    def read_motor_feedback(self):
        """Read one complete cached q/dq/motor-torque sample."""
        if not hasattr(self.arm, "get_motor_states"):
            return None
        try:
            positions = extract_joint_angles(
                self.arm.get_joint_angles(), self.joint_count
            )
        except Exception:
            return None
        if positions is None:
            return None
        velocities = []
        torques = []
        for joint_index in range(1, self.joint_count + 1):
            try:
                state = self.arm.get_motor_states(joint_index)
                message = getattr(state, "msg", state)
                velocity = float(getattr(message, "velocity"))
                torque = float(getattr(message, "torque"))
            except Exception:
                return None
            if not np.isfinite(velocity) or not np.isfinite(torque):
                return None
            velocities.append(velocity)
            torques.append(torque)
        position = np.asarray(positions, dtype=float)
        velocity = self._select_feedback_velocity(
            position, np.asarray(velocities, dtype=float)
        )
        return MotorFeedback(
            position=position,
            velocity=velocity,
            torque=np.asarray(torques, dtype=float),
        )

    def _select_feedback_velocity(self, positions, sdk_velocities):
        """Use finite differences for Nero firmware with zero SDK speed."""
        estimate = (
            getattr(self, "robot_model", "") == "nero"
            and getattr(self, "firmware_name", "") in ("v111", "v112")
            and getattr(self, "nero_velocity_estimation_enabled", True)
        )
        if not estimate:
            return np.asarray(sdk_velocities, dtype=float).copy()

        position = np.asarray(positions, dtype=float)
        now = time.monotonic()
        previous_position = getattr(
            self, "feedback_previous_position", None
        )
        previous_time = getattr(self, "feedback_previous_time", None)
        previous_velocity = np.asarray(
            getattr(
                self,
                "feedback_previous_velocity",
                np.zeros(position.size),
            ),
            dtype=float,
        )
        if (
            previous_position is None
            or previous_time is None
            or previous_velocity.shape != position.shape
            or now <= previous_time
        ):
            velocity = np.zeros(position.size, dtype=float)
        else:
            velocity = estimate_joint_velocity(
                previous_position,
                position,
                previous_velocity,
                now - previous_time,
                getattr(self, "velocity_filter_time_constant", 0.03),
            )
        self.feedback_previous_position = position.copy()
        self.feedback_previous_velocity = velocity.copy()
        self.feedback_previous_time = now
        return velocity

    def read_joint_velocities(self):
        """Read cached motor velocity for MIT dynamics and torque limiting."""
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

    def _cartesian_nullspace_options(
        self, reference_position, reference_velocity
    ):
        """Return Nero-only redundant-joint impedance arguments."""
        if getattr(self, "robot_model", "piper_l") != "nero":
            return {}
        return {
            "nullspace_reference": reference_position,
            "nullspace_reference_velocity": reference_velocity,
            "nullspace_stiffness": self.cartesian_nullspace_stiffness,
            "nullspace_damping": self.cartesian_nullspace_damping,
        }

    def _publish_dynamics_state(self, feedback):
        """Publish measured q/dq/motor torque from the shared 100 Hz sample."""
        publisher = getattr(self, "dynamics_state_publisher", None)
        if publisher is None:
            return
        position = np.asarray(feedback.position, dtype=float)
        velocity = np.asarray(feedback.velocity, dtype=float)
        torque = np.asarray(feedback.torque, dtype=float)
        if any(
            values.shape != (self.joint_count,)
            or not np.all(np.isfinite(values))
            for values in (position, velocity, torque)
        ):
            self.get_logger().warning(
                "dynamics state is incomplete; sample not published",
                throttle_duration_sec=1.0,
            )
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        model = getattr(self, "gravity_model", None)
        message.name = (
            list(model.movable_joint_names)
            if model is not None
            else [f"joint{index}" for index in range(1, self.joint_count + 1)]
        )
        message.position = position.tolist()
        message.velocity = velocity.tolist()
        message.effort = torque.tolist()
        publisher.publish(message)

    def mit_tick(self):
        """Publish one shared sample, then run admittance or MIT impedance."""
        if self.emergency_stopped:
            return
        if not self.execute_motion:
            self.get_logger().info(
                "dry-run interaction backend active",
                throttle_duration_sec=1.0,
            )
            return
        if not self.arm_ready:
            return
        feedback = self.read_motor_feedback()
        if feedback is not None:
            self._publish_dynamics_state(feedback)
        if getattr(self, "admittance_enabled", False):
            if feedback is None:
                self.get_logger().warning(
                    "admittance requires complete q/dq/torque feedback; "
                    "holding",
                    throttle_duration_sec=1.0,
                )
                return
            self._admittance_tick(feedback)
            return
        if not self.impedance_enabled:
            return
        try:
            if getattr(self, "impedance_backend", "joint") == "cartesian":
                self._cartesian_mit_tick(feedback)
                return
            sample = self._next_control_input(feedback)
            if not sample.state.velocity_valid:
                self.get_logger().warning(
                    "motor velocity unavailable; torque limit assumes zero "
                    "velocity",
                    throttle_duration_sec=1.0,
                )
            if not sample.state.position_valid:
                self.get_logger().warning(
                    "joint feedback unavailable; combined torque limit is "
                    "inactive",
                    throttle_duration_sec=1.0,
                )
            result = self._get_control_engine("joint_impedance").step(
                "joint_impedance", sample
            )
            limit = np.asarray(
                getattr(
                    self,
                    "mit_gravity_torque_limit",
                    [10.0] * self.joint_count,
                ),
                dtype=float,
            )
            if np.any(np.abs(result.command.estimated_torque) > limit + 1e-9):
                self.get_logger().warning(
                    "PD torque alone prevents the configured combined limit; "
                    "t_ff is already maximally counteracting it",
                    throttle_duration_sec=2.0,
                )
            self.get_logger().info(
                "estimated combined MIT torque=%s N·m"
                % np.round(result.command.estimated_torque, 3).tolist(),
                throttle_duration_sec=2.0,
            )
            self._send_control_result(result)
            self._publish_control_result(sample, result, "impedance")
        except Exception as exc:
            self.get_logger().error(f"MIT command failed: {exc}")
            self.trigger_emergency_stop()

    def _cartesian_mit_tick(self, feedback=None):
        """Evaluate ``J.T F + C*dq + g`` and send it only through t_ff."""
        sample = self._next_control_input(feedback)
        result = self._get_control_engine("cartesian_impedance").step(
            "cartesian_impedance", sample
        )
        raw = result.raw
        command_torque = result.command.feedforward
        self.last_cartesian_impedance_command = raw
        if result.signals["torque_clipped"]:
            self.get_logger().warning(
                "Cartesian MIT absolute torque limit active: raw=%s, "
                "sent=%s N·m"
                % (
                    np.round(
                        result.signals["raw_command_torque"], 3
                    ).tolist(),
                    np.round(command_torque, 3).tolist(),
                ),
                throttle_duration_sec=1.0,
            )
        self.get_logger().info(
            "Cartesian MIT error=%s wrench=%s task=%s null=%s posture=%s "
            "model=%s total=%s sent=%s N·m"
            % (
                np.round(raw.pose_error, 4).tolist(),
                np.round(raw.commanded_wrench, 3).tolist(),
                np.round(raw.task_torque, 3).tolist(),
                np.round(raw.nullspace_torque, 3).tolist(),
                np.round(
                    result.signals["joint_posture_torque"], 3
                ).tolist(),
                np.round(raw.model_torque, 3).tolist(),
                np.round(
                    result.signals["raw_command_torque"], 3
                ).tolist(),
                np.round(command_torque, 3).tolist(),
            ),
            throttle_duration_sec=2.0,
        )
        self._send_control_result(result)
        self._publish_control_result(sample, result, "impedance")

    def _admittance_tick(self, feedback):
        """Advance the Piper-L virtual dynamics and send one planned target."""
        now = time.monotonic()
        nominal_period = 1.0 / self.mit_command_rate
        previous = self.last_admittance_tick_time
        period = (
            nominal_period
            if previous is None
            else float(np.clip(
                now - previous,
                0.25 * nominal_period,
                2.0 * nominal_period,
            ))
        )
        self.last_admittance_tick_time = now
        wrench_age = now - self.latest_external_wrench_received_at
        wrench = (
            self.latest_external_wrench
            if wrench_age <= self.admittance_wrench_timeout
            else np.zeros(6)
        )
        control_state = ControlState(
            feedback.position, feedback.velocity, feedback.torque
        )
        sample = ControlInput(
            now,
            period,
            control_state,
            ControlReference.hold(feedback.position, wrench),
        )
        try:
            result = self._get_control_engine(
                "cartesian_admittance"
            ).step(
                "cartesian_admittance", sample
            )
        except IkFailure as error:
            self.get_logger().warning(
                f"admittance IK failed; exiting to hold: {error}"
            )
            self._exit_admittance("IK failure")
            return
        except ControlSafetyError as error:
            self.get_logger().warning(
                f"admittance safety rejection; exiting to hold: {error}"
            )
            self._exit_admittance("IK joint-step bound")
            return
        state = result.raw
        joints = result.command.position
        self.ik_target_position = state.desired_pose[:3, 3].copy()
        self.ik_target_rotation = state.desired_pose[:3, :3].copy()
        self.remember_ik_valid(
            self.ik_target_position,
            self.ik_target_rotation,
            joints,
        )
        self._send_control_result(result)
        self._publish_control_result(sample, result, "admittance")
        self.get_logger().info(
            "admittance wrench=%s offset=%s"
            % (
                np.round(state.applied_wrench, 3).tolist(),
                np.round(state.offset, 4).tolist(),
            ),
            throttle_duration_sec=1.0,
        )

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
        self._publish_control_event("emergency_stop")
        if self.execute_motion and self.arm_connected:
            try:
                self.arm.electronic_emergency_stop()
            except Exception as exc:
                self.get_logger().error(
                    f"emergency stop command failed: {exc}"
                )

    def destroy_node(self):
        if self.arm is not None and self.arm_connected:
            try:
                self.get_logger().info(
                    "Ctrl-C shutdown: disconnecting without motion or "
                    "enable/disable commands"
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
