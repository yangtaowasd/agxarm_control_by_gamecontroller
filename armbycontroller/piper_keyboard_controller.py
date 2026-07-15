#!/usr/bin/env python3

import math
import time
from collections.abc import Iterable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray

from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config
from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD


KEY_W = 0
KEY_S = 1
KEY_A = 2
KEY_D = 3
KEY_SPACE = 4
KEY_UP = 5
KEY_DOWN = 6
KEY_LEFT = 7
KEY_RIGHT = 8
KEY_PAGEUP = 9
KEY_PAGEDOWN = 10
KEY_RESET_CORRECTION = 11
KEY_COUNT = 12


ARM_MODELS = {
    "piper": ArmModel.PIPER,
    "piper_h": ArmModel.PIPER_H,
    "piper_l": ArmModel.PIPER_L,
    "piper_x": ArmModel.PIPER_X,
}

PIPER_FIRMWARES = {
    "default": PiperFW.DEFAULT,
    "v183": PiperFW.V183,
    "v188": PiperFW.V188,
    "v189": PiperFW.V189,
}


def clamp(value, low, high):
    return min(max(value, low), high)


def angle_diff(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def vector_norm(vector):
    return math.sqrt(sum(float(value) * float(value) for value in vector))


def normalize_vector(vector):
    norm = vector_norm(vector)
    if norm < 1e-12:
        return None
    return [float(value) / norm for value in vector]


def cross_vector(left, right):
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def tool_z_axis_from_pose(pose):
    roll, pitch, yaw = [float(value) for value in pose[3:6]]
    return [
        math.cos(yaw) * math.sin(pitch) * math.cos(roll)
        + math.sin(yaw) * math.sin(roll),
        math.sin(yaw) * math.sin(pitch) * math.cos(roll)
        - math.cos(yaw) * math.sin(roll),
        math.cos(pitch) * math.cos(roll),
    ]


def unwrap_msg(ret):
    if ret is None:
        return None
    return ret.msg if hasattr(ret, "msg") else ret


def extract_joint_angles(ret, joint_count=6):
    msg = unwrap_msg(ret)
    if msg is None:
        return None

    if isinstance(msg, (list, tuple)) or (
        isinstance(msg, Iterable) and not isinstance(msg, (str, bytes, dict))
    ):
        values = [float(value) for value in msg]
        return values[:joint_count] if len(values) >= joint_count else None

    if isinstance(msg, dict):
        for key in ("joint_angles", "angles", "joint_angle", "angle", "data", "msg"):
            value = msg.get(key)
            if isinstance(value, (list, tuple)) and len(value) >= joint_count:
                return [float(item) for item in value[:joint_count]]

    for attr in ("joint_angles", "angles", "joint_angle", "angle", "data", "msg"):
        if hasattr(msg, attr):
            value = getattr(msg, attr)
            if isinstance(value, (list, tuple)) and len(value) >= joint_count:
                return [float(item) for item in value[:joint_count]]

    return None


class PiperKeyboardController(Node):
    def __init__(self):
        super().__init__("piper_keyboard_controller")

        self.declare_parameter("keyboard_topic", "/keyboard_state")
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("arm_model", "piper")
        self.declare_parameter("firmware", "default")
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("step_rad", 0.01)
        self.declare_parameter("arrow_mode", "orientation")
        self.declare_parameter("orientation_mapping", "square")
        self.declare_parameter("orientation_square_limit", 1.0)
        self.declare_parameter("orientation_square_step", 0.02)
        self.declare_parameter("orientation_square_max_tilt_deg", 80.0)
        self.declare_parameter("joint4_limit_deg", 127.0)
        self.declare_parameter("joint5_limit_deg", 89.5)
        self.declare_parameter("joint6_limit_deg", 170.0)
        self.declare_parameter("orientation_ik_damping", 0.2)
        self.declare_parameter("orientation_ik_max_iterations", 40)
        self.declare_parameter("orientation_ik_max_joint_step", 0.02)
        self.declare_parameter("orientation_ik_fd_eps", 0.001)
        self.declare_parameter("orientation_ik_line_search_steps", 6)
        self.declare_parameter("orientation_ik_tolerance", 0.002)
        self.declare_parameter("orientation_ik_refine_iterations", 10)
        self.declare_parameter("orientation_ik_refine_damping", 0.05)
        self.declare_parameter("orientation_ik_coordinate_iterations", 8)
        self.declare_parameter("orientation_j5_limit_deg", 80.0)
        self.declare_parameter("orientation_yaw_joint_step_scale", 2.0)
        self.declare_parameter("orientation_singularity_yaw_boost", 2.0)
        self.declare_parameter("avoid_j5_zero_deg", 0.0)
        self.declare_parameter("speed_percent", 40)
        self.declare_parameter("j4_j6_speed_percent", 80)
        self.declare_parameter("j3_correction_step_rad", 0.01)
        self.declare_parameter("j3_correction_limit_deg", 10.0)
        self.declare_parameter("keyboard_timeout", 0.3)
        self.declare_parameter("enable_timeout", 5.0)
        self.declare_parameter("clear_errors_on_enable", True)
        self.declare_parameter("shutdown_j5_deg", 30.0)
        self.declare_parameter("shutdown_home_tolerance", 0.0001)
        self.declare_parameter("shutdown_home_poll_interval", 0.05)
        self.declare_parameter("move_home_on_start", True)
        self.declare_parameter("execute_motion", True)

        self.keyboard_topic = self.get_parameter("keyboard_topic").value
        self.can_interface = self.get_parameter("can_interface").value
        self.arm_model_name = self.get_parameter("arm_model").value
        self.firmware_name = self.get_parameter("firmware").value
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.step_rad = float(self.get_parameter("step_rad").value)
        self.arrow_mode = self.get_parameter("arrow_mode").value
        self.orientation_mapping = self.get_parameter("orientation_mapping").value
        self.orientation_square_limit = float(
            self.get_parameter("orientation_square_limit").value
        )
        self.orientation_square_step = float(
            self.get_parameter("orientation_square_step").value
        )
        self.orientation_square_max_tilt_rad = math.radians(
            float(self.get_parameter("orientation_square_max_tilt_deg").value)
        )
        self.joint4_limit_rad = math.radians(
            abs(float(self.get_parameter("joint4_limit_deg").value))
        )
        self.joint5_limit_rad = math.radians(
            abs(float(self.get_parameter("joint5_limit_deg").value))
        )
        self.joint6_limit_rad = math.radians(
            abs(float(self.get_parameter("joint6_limit_deg").value))
        )
        self.orientation_ik_damping = float(self.get_parameter("orientation_ik_damping").value)
        self.orientation_ik_max_iterations = int(
            self.get_parameter("orientation_ik_max_iterations").value
        )
        self.orientation_ik_max_joint_step = float(
            self.get_parameter("orientation_ik_max_joint_step").value
        )
        self.orientation_ik_fd_eps = float(
            self.get_parameter("orientation_ik_fd_eps").value
        )
        self.orientation_ik_line_search_steps = int(
            self.get_parameter("orientation_ik_line_search_steps").value
        )
        self.orientation_ik_tolerance = float(
            self.get_parameter("orientation_ik_tolerance").value
        )
        self.orientation_ik_refine_iterations = int(
            self.get_parameter("orientation_ik_refine_iterations").value
        )
        self.orientation_ik_refine_damping = float(
            self.get_parameter("orientation_ik_refine_damping").value
        )
        self.orientation_ik_coordinate_iterations = int(
            self.get_parameter("orientation_ik_coordinate_iterations").value
        )
        self.orientation_j5_limit_rad = math.radians(
            abs(float(self.get_parameter("orientation_j5_limit_deg").value))
        )
        self.orientation_yaw_joint_step_scale = float(
            self.get_parameter("orientation_yaw_joint_step_scale").value
        )
        self.orientation_singularity_yaw_boost = float(
            self.get_parameter("orientation_singularity_yaw_boost").value
        )
        self.avoid_j5_zero_rad = math.radians(
            float(self.get_parameter("avoid_j5_zero_deg").value)
        )
        self.speed_percent = int(self.get_parameter("speed_percent").value)
        self.j4_j6_speed_percent = int(
            self.get_parameter("j4_j6_speed_percent").value
        )
        self.j3_correction_step_rad = float(
            self.get_parameter("j3_correction_step_rad").value
        )
        self.j3_correction_limit_rad = math.radians(
            abs(float(self.get_parameter("j3_correction_limit_deg").value))
        )
        self.keyboard_timeout = float(self.get_parameter("keyboard_timeout").value)
        self.enable_timeout = float(self.get_parameter("enable_timeout").value)
        self.clear_errors_on_enable = bool(self.get_parameter("clear_errors_on_enable").value)
        self.shutdown_j5_deg = float(self.get_parameter("shutdown_j5_deg").value)
        self.shutdown_home_tolerance = float(
            self.get_parameter("shutdown_home_tolerance").value
        )
        self.shutdown_home_poll_interval = float(
            self.get_parameter("shutdown_home_poll_interval").value
        )
        self.move_home_on_start = bool(self.get_parameter("move_home_on_start").value)
        self.execute_motion = bool(self.get_parameter("execute_motion").value)

        if self.control_rate <= 0.0:
            raise ValueError("control_rate must be > 0")

        if self.step_rad <= 0.0:
            raise ValueError("step_rad must be > 0")

        if self.arrow_mode not in ("orientation", "joint"):
            raise ValueError("arrow_mode must be 'orientation' or 'joint'")

        if self.orientation_mapping not in ("square", "euler"):
            raise ValueError("orientation_mapping must be 'square' or 'euler'")

        if self.orientation_square_limit <= 0.0:
            raise ValueError("orientation_square_limit must be > 0")

        if self.orientation_square_step <= 0.0:
            raise ValueError("orientation_square_step must be > 0")

        if not 0.0 < self.orientation_square_max_tilt_rad <= math.pi / 2.0:
            raise ValueError("orientation_square_max_tilt_deg must be in (0, 90]")

        if min(
            self.joint4_limit_rad,
            self.joint5_limit_rad,
            self.joint6_limit_rad,
        ) <= 0.0:
            raise ValueError("joint4_limit_deg through joint6_limit_deg must be > 0")

        if self.orientation_ik_damping <= 0.0:
            raise ValueError("orientation_ik_damping must be > 0")

        if self.orientation_ik_max_iterations < 1:
            raise ValueError("orientation_ik_max_iterations must be >= 1")

        if self.orientation_ik_max_joint_step <= 0.0:
            raise ValueError("orientation_ik_max_joint_step must be > 0")

        if self.orientation_ik_fd_eps <= 0.0:
            raise ValueError("orientation_ik_fd_eps must be > 0")

        if self.orientation_ik_line_search_steps < 1:
            raise ValueError("orientation_ik_line_search_steps must be >= 1")

        if self.orientation_ik_tolerance <= 0.0:
            raise ValueError("orientation_ik_tolerance must be > 0")

        if self.orientation_ik_refine_iterations < 0:
            raise ValueError("orientation_ik_refine_iterations must be >= 0")

        if self.orientation_ik_refine_damping <= 0.0:
            raise ValueError("orientation_ik_refine_damping must be > 0")

        if self.orientation_ik_coordinate_iterations < 0:
            raise ValueError("orientation_ik_coordinate_iterations must be >= 0")

        if self.orientation_j5_limit_rad <= 0.0:
            raise ValueError("orientation_j5_limit_deg must be > 0")

        if self.orientation_yaw_joint_step_scale <= 0.0:
            raise ValueError("orientation_yaw_joint_step_scale must be > 0")

        if self.orientation_singularity_yaw_boost < 1.0:
            raise ValueError("orientation_singularity_yaw_boost must be >= 1")

        if self.shutdown_home_tolerance <= 0.0:
            raise ValueError("shutdown_home_tolerance must be > 0")

        if self.shutdown_home_poll_interval <= 0.0:
            raise ValueError("shutdown_home_poll_interval must be > 0")

        if self.avoid_j5_zero_rad < 0.0:
            raise ValueError("avoid_j5_zero_deg must be >= 0")

        if not 0 <= self.speed_percent <= 100:
            raise ValueError("speed_percent must be in [0, 100]")

        if not 0 <= self.j4_j6_speed_percent <= 100:
            raise ValueError("j4_j6_speed_percent must be in [0, 100]")

        if self.speed_percent == 0 and self.j4_j6_speed_percent > 0:
            raise ValueError("speed_percent must be > 0 when j4/j6 motion is enabled")

        if self.j3_correction_step_rad <= 0.0:
            raise ValueError("j3_correction_step_rad must be > 0")

        if self.j3_correction_limit_rad < 0.0:
            raise ValueError("j3_correction_limit_deg must be >= 0")

        self.arm_model = ARM_MODELS.get(self.arm_model_name)
        if self.arm_model is None:
            raise ValueError(f"unsupported arm_model: {self.arm_model_name}")

        self.firmware = PIPER_FIRMWARES.get(self.firmware_name)
        if self.firmware is None:
            raise ValueError(f"unsupported firmware: {self.firmware_name}")

        limit_key = self.arm_model_name
        if limit_key not in ROBOT_JOINT_LIMIT_PRESET_RAD:
            limit_key = "piper"

        self.joint_limits = [
            tuple(ROBOT_JOINT_LIMIT_PRESET_RAD[limit_key][f"joint{i}"])
            for i in range(1, 7)
        ]
        self.joint_limits[3] = (-self.joint4_limit_rad, self.joint4_limit_rad)
        self.joint_limits[4] = (-self.joint5_limit_rad, self.joint5_limit_rad)
        self.joint_limits[5] = (-self.joint6_limit_rad, self.joint6_limit_rad)

        self.target_joints = [0.0] * 6
        self.wrist_j6_offset = 0.0
        self.j3_correction_rad = 0.0
        self.orientation_home_pitch = 0.0
        self.orientation_home_yaw = 0.0
        self.orientation_home_axis = [1.0, 0.0, 0.0]
        self.orientation_horizontal_axis = [0.0, 1.0, 0.0]
        self.orientation_down_axis = [0.0, 0.0, -1.0]
        self.target_orientation_axis = self.orientation_home_axis[:]
        self.orientation_square_x = 0.0
        self.orientation_square_y = 0.0
        self.target_orientation_pitch = 0.0
        self.target_orientation_yaw = 0.0
        self.key_state = [0] * KEY_COUNT
        self.last_active_keys = ""
        self.last_keyboard_time = 0.0
        self.arm = None
        self.arm_ready = False

        self.keyboard_sub = self.create_subscription(
            Int32MultiArray,
            self.keyboard_topic,
            self.keyboard_callback,
            qos_profile_sensor_data,
        )

        self.connect_arm()

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_tick,
        )

        self.get_logger().info("piper_keyboard_controller started")
        self.get_logger().info(
            "keys: A/D=j1, W/S=j2+j3, "
            f"arrows={self.arrow_mode}/{self.orientation_mapping} "
            "IK on j4/j5/j6, PAGEUP/PAGEDOWN=-/+j6, "
            "_=reset j3 correction, SPACE=home"
        )

    def connect_arm(self):
        custom_joint_limits = {
            f"joint{index + 1}": [float(low), float(high)]
            for index, (low, high) in enumerate(self.joint_limits)
        }
        cfg = create_agx_arm_config(
            robot=self.arm_model,
            firmeware_version=self.firmware,
            channel=self.can_interface,
            joint_limits=custom_joint_limits,
        )
        self.arm = AgxArmFactory.create_arm(cfg)
        self.arm.set_joint_limits_enabled(True)

        if not self.execute_motion:
            self.get_logger().warning(
                "execute_motion is False; target will update but arm will not move"
            )
            return

        try:
            self.arm.connect()
            self.get_logger().info("Piper connected, waiting 0.5s before enable")
            time.sleep(0.5)
            if not self.enable_arm():
                self.arm_ready = False
                self.get_logger().error("Piper enable failed; commands will be skipped")
                return

            self.get_logger().info("Piper enabled, waiting 0.2s before home")
            time.sleep(0.2)
            self.arm.set_speed_percent(self.speed_percent)
            self.arm_ready = True
            self.get_logger().info(
                f"connected Piper on {self.can_interface}, "
                f"speed={self.speed_percent}%, "
                f"j4/j6 keyboard speed={self.j4_j6_speed_percent}%"
            )

            if self.move_home_on_start:
                self.move_to_start_home()

            self.sync_orientation_target_from_fk()
        except Exception as exc:
            self.arm_ready = False
            self.get_logger().error(f"connect Piper failed: {exc}")

    def enable_arm(self):
        deadline = time.monotonic() + self.enable_timeout
        last_result = None

        while time.monotonic() < deadline:
            try:
                if self.clear_errors_on_enable:
                    self.arm.clear_joint_error()

                last_result = self.arm.enable()
                joints_enable = self.arm.get_joints_enable_status_list()
                self.get_logger().info(
                    f"enable result={last_result}, joints_enable={joints_enable}"
                )

                if last_result or all(joints_enable):
                    return True

                for joint_index, enabled in enumerate(joints_enable, start=1):
                    if enabled:
                        continue

                    joint_result = self.arm.enable(joint_index)
                    self.get_logger().warning(
                        f"joint {joint_index} enable retry result={joint_result}"
                    )

                joints_enable = self.arm.get_joints_enable_status_list()
                self.get_logger().info(f"joints_enable after retry={joints_enable}")
                if all(joints_enable):
                    return True
            except Exception as exc:
                self.get_logger().warning(
                    f"enable attempt failed: {exc}", throttle_duration_sec=1.0
                )

            time.sleep(0.2)

        self.get_logger().error(f"enable timeout, last_result={last_result}")
        return False

    def keyboard_callback(self, msg):
        data = list(msg.data)
        if len(data) < KEY_COUNT:
            self.get_logger().warning(
                f"keyboard_state length {len(data)} < {KEY_COUNT}; rebuild keyboard node",
                throttle_duration_sec=1.0,
            )
            return

        self.key_state = [1 if value else 0 for value in data[:KEY_COUNT]]
        self.last_keyboard_time = time.monotonic()

        active_keys = self.format_active_keys()
        if active_keys and active_keys != self.last_active_keys:
            self.get_logger().info(f"active keys: {active_keys}")
        self.last_active_keys = active_keys

    def format_active_keys(self):
        names = [
            "W",
            "S",
            "A",
            "D",
            "SPACE",
            "UP",
            "DOWN",
            "LEFT",
            "RIGHT",
            "PAGEUP",
            "PAGEDOWN",
            "_",
        ]
        return "+".join(name for name, pressed in zip(names, self.key_state) if pressed)

    def control_tick(self):
        if self.last_keyboard_time == 0.0:
            return

        if time.monotonic() - self.last_keyboard_time > self.keyboard_timeout:
            self.key_state = [0] * KEY_COUNT
            return

        if self.key_state[KEY_SPACE]:
            self.set_work_home_target()
            self.sync_orientation_target_from_fk()
            self.send_joint_target("space work home")
            return

        if self.key_state[KEY_RESET_CORRECTION]:
            if self.reset_j3_correction():
                self.send_joint_target("reset j3 correction")
            return

        joint_changed = self.apply_joint_step()
        if joint_changed:
            self.send_joint_target("joint keyboard")

    def apply_joint_step(self):
        old_target = self.target_joints.copy()

        j1_axis = self.key_state[KEY_D] - self.key_state[KEY_A]
        lift_axis = self.key_state[KEY_W] - self.key_state[KEY_S]
        wrist_pitch_axis = self.key_state[KEY_UP] - self.key_state[KEY_DOWN]
        wrist_yaw_axis = self.key_state[KEY_RIGHT] - self.key_state[KEY_LEFT]
        j6_axis = self.key_state[KEY_PAGEDOWN] - self.key_state[KEY_PAGEUP]

        self.target_joints[0] += j1_axis * self.step_rad

        self.target_joints[1] += lift_axis * self.step_rad
        self.target_joints[2] -= lift_axis * self.step_rad

        if self.arrow_mode == "orientation" and (wrist_pitch_axis or wrist_yaw_axis):
            self.apply_orientation_step(wrist_pitch_axis, wrist_yaw_axis)
        elif self.arrow_mode == "joint":
            self.apply_wrist_joint_step(wrist_pitch_axis, wrist_yaw_axis)

        self.target_joints[5] += (
            j6_axis * self.step_rad * self.j4_j6_speed_scale()
        )

        self.clamp_joint_target()
        if j6_axis and self.arrow_mode == "orientation":
            self.wrist_j6_offset = self.target_joints[5] + self.target_joints[3]
            self.sync_orientation_target_from_fk()
        return any(abs(a - b) > 1e-9 for a, b in zip(old_target, self.target_joints))

    def move_to_start_home(self):
        self.target_joints = [0.0] * 6
        self.send_joint_target("mechanical home")
        self.set_work_home_target()
        if any(abs(value) > 1e-9 for value in self.target_joints):
            self.send_joint_target("work home")

    def set_work_home_target(self):
        self.target_joints = [0.0] * 6
        self.wrist_j6_offset = 0.0
        self.j3_correction_rad = 0.0
        if self.arrow_mode != "orientation":
            return

        _, j5 = self.clamp_parallel_wrist(0.0, 0.0)
        self.set_parallel_wrist(self.target_joints, 0.0, j5)

    def apply_orientation_step(self, pitch_axis, yaw_axis):
        pitch_blocked = False
        if self.orientation_mapping == "square":
            pitch_blocked = self.apply_square_orientation_step(
                pitch_axis,
                yaw_axis,
            )
        else:
            self.target_orientation_pitch -= pitch_axis * self.step_rad
            self.target_orientation_yaw -= yaw_axis * self.step_rad

        self.clamp_orientation_target()

        solved = self.solve_wrist_orientation_ik(
            self.target_joints,
            self.target_orientation_pitch,
            self.target_orientation_yaw,
        )
        self.target_joints[3:6] = solved[3:6]

        if pitch_blocked and self.j5_at_orientation_limit():
            if self.apply_j3_correction(pitch_axis):
                self.update_square_orientation_axis()
                solved = self.solve_wrist_orientation_ik(
                    self.target_joints,
                    self.target_orientation_pitch,
                    self.target_orientation_yaw,
                )
                self.target_joints[3:6] = solved[3:6]

    def apply_square_orientation_step(self, pitch_axis, yaw_axis):
        requested_y = (
            self.orientation_square_y
            - pitch_axis * self.orientation_square_step
        )
        self.orientation_square_x = clamp(
            self.orientation_square_x
            - yaw_axis
            * self.orientation_square_step
            * self.j4_j6_speed_scale(),
            -self.orientation_square_limit,
            self.orientation_square_limit,
        )
        self.orientation_square_y = clamp(
            requested_y,
            -self.orientation_square_limit,
            self.orientation_square_limit,
        )

        self.update_square_orientation_axis()
        return bool(
            pitch_axis
            and abs(requested_y - self.orientation_square_y) > 1e-12
        )

    def update_square_orientation_axis(self):
        correction_scale = (
            self.j3_correction_rad / self.orientation_square_max_tilt_rad
        )

        square_x = self.orientation_square_x / self.orientation_square_limit
        square_y = (
            self.orientation_square_y / self.orientation_square_limit
            + correction_scale
        )
        polar_scale = max(abs(square_x), abs(square_y))
        if polar_scale < 1e-12:
            self.target_orientation_axis = self.orientation_home_axis[:]
            return

        tangent_axis = normalize_vector(
            [
                square_x * self.orientation_horizontal_axis[index]
                + square_y * self.orientation_down_axis[index]
                for index in range(3)
            ]
        )
        if tangent_axis is None:
            return

        polar_angle = polar_scale * self.orientation_square_max_tilt_rad
        self.target_orientation_axis = [
            math.cos(polar_angle) * self.orientation_home_axis[index]
            + math.sin(polar_angle) * tangent_axis[index]
            for index in range(3)
        ]

    def j4_j6_speed_scale(self):
        if self.speed_percent <= 0:
            return 0.0
        return self.j4_j6_speed_percent / self.speed_percent

    def j5_at_orientation_limit(self):
        limit = min(
            self.orientation_j5_limit_rad,
            abs(self.joint_limits[4][0]),
            abs(self.joint_limits[4][1]),
        )
        tolerance = max(self.orientation_ik_tolerance * 2.0, math.radians(0.25))
        return abs(self.target_joints[4]) >= limit - tolerance

    def apply_j3_correction(self, pitch_axis):
        old_correction = self.j3_correction_rad
        desired_correction = clamp(
            old_correction - pitch_axis * self.j3_correction_step_rad,
            -self.j3_correction_limit_rad,
            self.j3_correction_limit_rad,
        )
        base_j3 = self.target_joints[2] - old_correction
        j3_low, j3_high = self.joint_limits[2]
        new_correction = clamp(
            desired_correction,
            j3_low - base_j3,
            j3_high - base_j3,
        )
        if abs(new_correction - old_correction) < 1e-12:
            return False

        self.j3_correction_rad = new_correction
        self.target_joints[2] = base_j3 + new_correction
        self.get_logger().info(
            f"j3 correction={math.degrees(new_correction):.2f} deg",
            throttle_duration_sec=0.5,
        )
        return True

    def reset_j3_correction(self):
        if abs(self.j3_correction_rad) < 1e-12:
            return False

        self.target_joints[2] -= self.j3_correction_rad
        self.j3_correction_rad = 0.0
        self.clamp_joint_target()
        if self.orientation_mapping == "square":
            self.update_square_orientation_axis()
            solved = self.solve_wrist_orientation_ik(
                self.target_joints,
                self.target_orientation_pitch,
                self.target_orientation_yaw,
            )
            self.target_joints[3:6] = solved[3:6]
        return True

    def clamp_orientation_target(self):
        self.target_orientation_pitch = clamp(
            self.target_orientation_pitch,
            -math.pi / 2.0 + 1e-3,
            math.pi / 2.0 - 1e-3,
        )
        self.target_orientation_yaw = angle_diff(self.target_orientation_yaw, 0.0)

    def sync_orientation_target_from_fk(self):
        pose = self.fk_pose(self.target_joints)
        if pose is None:
            self.target_orientation_pitch = 0.0
            self.target_orientation_yaw = 0.0
            return

        self.orientation_home_pitch = float(pose[4])
        self.orientation_home_yaw = float(pose[5])
        home_axis = normalize_vector(tool_z_axis_from_pose(pose))
        if home_axis is None:
            home_axis = [1.0, 0.0, 0.0]

        horizontal_axis = normalize_vector([-home_axis[1], home_axis[0], 0.0])
        if horizontal_axis is None:
            horizontal_axis = [0.0, 1.0, 0.0]

        down_axis = normalize_vector(cross_vector(horizontal_axis, home_axis))
        if down_axis is None:
            down_axis = [0.0, 0.0, -1.0]

        self.orientation_home_axis = home_axis
        self.orientation_horizontal_axis = horizontal_axis
        self.orientation_down_axis = down_axis
        self.target_orientation_axis = home_axis[:]
        self.orientation_square_x = 0.0
        self.orientation_square_y = 0.0
        self.target_orientation_pitch = self.orientation_home_pitch
        self.target_orientation_yaw = self.orientation_home_yaw
        self.clamp_orientation_target()

    def fk_pose(self, joints):
        if self.arm is None:
            return None

        try:
            pose = self.arm.fk([float(value) for value in joints])
        except Exception as exc:
            self.get_logger().warning(f"fk failed: {exc}", throttle_duration_sec=1.0)
            return None

        if pose is None or len(pose) < 6:
            return None

        return [float(value) for value in pose[:6]]

    def solve_wrist_orientation_ik(self, seed_joints, target_pitch, target_yaw, j5_side=None):
        if self.orientation_mapping == "square":
            target = [float(value) for value in self.target_orientation_axis]
        else:
            target = [float(target_pitch), float(target_yaw)]
        seed = [float(value) for value in seed_joints]
        seed_parallel_yaw = seed[3]
        seed_j5 = seed[4]

        best_result = None
        best_score = None
        best_residual = None

        for yaw_seed_index, candidate_yaw in enumerate(
            self.parallel_yaw_seed_values(seed_parallel_yaw)
        ):
            for candidate_j5 in self.j5_seed_values(seed_j5, j5_side):
                candidate = seed[:]
                parallel_yaw, j5 = self.clamp_parallel_wrist(
                    candidate_yaw,
                    candidate_j5,
                    j5_side,
                )
                self.set_parallel_wrist(candidate, parallel_yaw, j5)
                solved, residual = self.solve_wrist_orientation_ik_from_seed(
                    candidate,
                    parallel_yaw,
                    j5,
                    target,
                    j5_side,
                )
                if residual is None:
                    continue

                residual_norm = vector_norm(residual)
                continuity_penalty = 0.0001 * (
                    abs(angle_diff(solved[3], seed[3]))
                    + abs(angle_diff(solved[4], seed_j5))
                )
                score = residual_norm + continuity_penalty
                if best_score is None or score < best_score:
                    best_result = solved
                    best_score = score
                    best_residual = residual

            if (
                yaw_seed_index == 0
                and best_residual is not None
                and vector_norm(best_residual) <= self.orientation_ik_tolerance
            ):
                break

        if best_result is None:
            return seed

        if best_residual is not None:
            residual_norm = vector_norm(best_residual)
            if residual_norm > max(0.02, self.orientation_ik_tolerance * 5.0):
                residual_text = ", ".join(f"{value:.4f}" for value in best_residual)
                self.get_logger().warning(
                    f"orientation IK residual too large: [{residual_text}]",
                    throttle_duration_sec=1.0,
                )

        return best_result

    def parallel_yaw_seed_values(self, seed_parallel_yaw):
        return [
            seed_parallel_yaw,
            0.0,
            math.pi / 2.0,
            -math.pi / 2.0,
        ]

    def j5_seed_values(self, seed_j5, j5_side=None):
        seed_step = max(self.orientation_ik_max_joint_step, math.radians(2.0))
        seed_abs_j5 = min(
            self.orientation_j5_limit_rad,
            max(abs(seed_j5), seed_step),
        )
        values = [
            seed_j5,
            seed_j5 + seed_step,
            seed_j5 - seed_step,
            seed_abs_j5,
            -seed_abs_j5,
        ]
        if j5_side is None:
            return values
        if j5_side > 0.0:
            return [max(0.0, value) for value in values]
        return [min(0.0, value) for value in values]

    def solve_wrist_orientation_ik_from_seed(self, result, parallel_yaw, j5, target, j5_side=None):
        result, parallel_yaw, j5 = self.run_orientation_ik_iterations(
            result,
            parallel_yaw,
            j5,
            target,
            self.orientation_ik_max_iterations,
            self.orientation_ik_damping,
            self.orientation_ik_tolerance,
            True,
            j5_side,
        )

        result, parallel_yaw, j5 = self.run_orientation_ik_iterations(
            result,
            parallel_yaw,
            j5,
            target,
            self.orientation_ik_refine_iterations,
            self.orientation_ik_refine_damping,
            self.orientation_ik_tolerance * 0.25,
            False,
            j5_side,
        )

        result, parallel_yaw, j5 = self.run_orientation_coordinate_search(
            result,
            parallel_yaw,
            j5,
            target,
            j5_side,
        )

        solved = [float(value) for value in result]
        final_orientation = self.orientation_from_fk(solved)
        if final_orientation is None:
            return solved, None

        return solved, self.orientation_error(target, final_orientation)

    def run_orientation_ik_iterations(
        self,
        result,
        parallel_yaw,
        j5,
        target,
        iteration_count,
        damping,
        tolerance,
        allow_yaw_boost,
        j5_side=None,
    ):
        eps = self.orientation_ik_fd_eps

        for _ in range(iteration_count):
            current = self.orientation_from_fk(result)
            if current is None:
                return result, parallel_yaw, j5

            error = self.orientation_error(target, current)
            if vector_norm(error) < tolerance:
                break

            jacobian = [[0.0, 0.0] for _ in error]
            for column in range(2):
                plus = result[:]
                minus = result[:]
                if column == 0:
                    self.set_parallel_wrist(plus, parallel_yaw + eps, j5)
                    self.set_parallel_wrist(minus, parallel_yaw - eps, j5)
                else:
                    self.set_parallel_wrist(plus, parallel_yaw, j5 + eps)
                    self.set_parallel_wrist(minus, parallel_yaw, j5 - eps)

                plus_orientation = self.orientation_from_fk(plus)
                minus_orientation = self.orientation_from_fk(minus)
                if plus_orientation is None or minus_orientation is None:
                    return result, parallel_yaw, j5

                derivative = self.orientation_error(
                    plus_orientation,
                    minus_orientation,
                )
                for row, value in enumerate(derivative):
                    jacobian[row][column] = value / (2.0 * eps)

            delta = self.damped_least_squares(jacobian, error, damping)
            yaw_limit = self.orientation_ik_max_joint_step
            if allow_yaw_boost:
                yaw_limit *= self.orientation_yaw_joint_step_scale
                yaw_limit *= self.singularity_yaw_boost(j5)

            j5_limit = self.orientation_ik_max_joint_step
            delta[0] = clamp(delta[0], -yaw_limit, yaw_limit)
            delta[1] = clamp(delta[1], -j5_limit, j5_limit)

            if math.hypot(delta[0], delta[1]) < 1e-8:
                break

            accepted = False
            best_norm = vector_norm(error)
            best_result = result
            best_parallel_yaw = parallel_yaw
            best_j5 = j5

            for search_index in range(self.orientation_ik_line_search_steps):
                scale = 0.5 ** search_index
                candidate_parallel_yaw, candidate_j5 = self.clamp_parallel_wrist(
                    parallel_yaw + delta[0] * scale,
                    j5 + delta[1] * scale,
                    j5_side,
                )
                candidate = result[:]
                self.set_parallel_wrist(candidate, candidate_parallel_yaw, candidate_j5)
                candidate_orientation = self.orientation_from_fk(candidate)
                if candidate_orientation is None:
                    continue

                candidate_error = self.orientation_error(target, candidate_orientation)
                candidate_norm = vector_norm(candidate_error)
                if candidate_norm + 1e-9 < best_norm:
                    accepted = True
                    best_norm = candidate_norm
                    best_result = candidate
                    best_parallel_yaw = candidate_parallel_yaw
                    best_j5 = candidate_j5
                    break

            if not accepted:
                break

            result = best_result
            parallel_yaw = best_parallel_yaw
            j5 = best_j5

        return result, parallel_yaw, j5

    def run_orientation_coordinate_search(self, result, parallel_yaw, j5, target, j5_side=None):
        if self.orientation_ik_coordinate_iterations == 0:
            return result, parallel_yaw, j5

        current = self.orientation_from_fk(result)
        if current is None:
            return result, parallel_yaw, j5

        current_error = self.orientation_error(target, current)
        best_norm = vector_norm(current_error)
        yaw_step = self.orientation_ik_max_joint_step
        yaw_step *= self.orientation_yaw_joint_step_scale
        yaw_step *= self.singularity_yaw_boost(j5)
        j5_step = self.orientation_ik_max_joint_step

        for _ in range(self.orientation_ik_coordinate_iterations):
            improved = False
            candidates = (
                (yaw_step, 0.0),
                (-yaw_step, 0.0),
                (0.0, j5_step),
                (0.0, -j5_step),
                (yaw_step, j5_step),
                (yaw_step, -j5_step),
                (-yaw_step, j5_step),
                (-yaw_step, -j5_step),
            )

            for yaw_delta, j5_delta in candidates:
                candidate_parallel_yaw, candidate_j5 = self.clamp_parallel_wrist(
                    parallel_yaw + yaw_delta,
                    j5 + j5_delta,
                    j5_side,
                )
                candidate = result[:]
                self.set_parallel_wrist(candidate, candidate_parallel_yaw, candidate_j5)
                candidate_orientation = self.orientation_from_fk(candidate)
                if candidate_orientation is None:
                    continue

                candidate_error = self.orientation_error(target, candidate_orientation)
                candidate_norm = vector_norm(candidate_error)
                if candidate_norm + 1e-9 < best_norm:
                    result = candidate
                    parallel_yaw = candidate_parallel_yaw
                    j5 = candidate_j5
                    best_norm = candidate_norm
                    improved = True
                    break

            if improved:
                yaw_step *= 1.2
                continue

            yaw_step *= 0.5
            j5_step *= 0.5
            if max(yaw_step, j5_step) < self.orientation_ik_tolerance * 0.25:
                break

        return result, parallel_yaw, j5

    def singularity_yaw_boost(self, j5):
        singularity_window = max(2.0 * self.avoid_j5_zero_rad, math.radians(10.0))
        distance = abs(float(j5))
        if distance >= singularity_window:
            return 1.0

        closeness = 1.0 - distance / singularity_window
        return 1.0 + (self.orientation_singularity_yaw_boost - 1.0) * closeness

    def set_parallel_wrist(self, joints, parallel_yaw, j5):
        joints[3] = float(parallel_yaw)
        joints[4] = float(j5)
        joints[5] = float(-parallel_yaw + self.wrist_j6_offset)

    def clamp_parallel_wrist(self, parallel_yaw, j5, j5_side=None):
        j4_low, j4_high = self.joint_limits[3]
        j5_low, j5_high = self.joint_limits[4]
        j6_low, j6_high = self.joint_limits[5]
        yaw_low = max(j4_low, self.wrist_j6_offset - j6_high)
        yaw_high = min(j4_high, self.wrist_j6_offset - j6_low)
        j5_low = max(j5_low, -self.orientation_j5_limit_rad)
        j5_high = min(j5_high, self.orientation_j5_limit_rad)
        if j5_side is not None:
            if j5_side > 0.0:
                j5_low = max(j5_low, 0.0)
            elif j5_side < 0.0:
                j5_high = min(j5_high, 0.0)

        parallel_yaw = clamp(float(parallel_yaw), yaw_low, yaw_high)
        j5 = clamp(float(j5), j5_low, j5_high)

        if self.avoid_j5_zero_rad > 0.0 and abs(j5) < self.avoid_j5_zero_rad:
            if j5 > 0.0 and self.avoid_j5_zero_rad <= j5_high:
                j5 = self.avoid_j5_zero_rad
            elif j5 < 0.0 and -self.avoid_j5_zero_rad >= j5_low:
                j5 = -self.avoid_j5_zero_rad
            else:
                if self.avoid_j5_zero_rad <= j5_high:
                    j5 = self.avoid_j5_zero_rad
                elif -self.avoid_j5_zero_rad >= j5_low:
                    j5 = -self.avoid_j5_zero_rad

        return parallel_yaw, j5

    def damped_least_squares(self, jacobian, error, damping):
        a00 = sum(row[0] * row[0] for row in jacobian) + damping * damping
        a01 = sum(row[0] * row[1] for row in jacobian)
        a11 = sum(row[1] * row[1] for row in jacobian) + damping * damping
        b0 = sum(row[0] * value for row, value in zip(jacobian, error))
        b1 = sum(row[1] * value for row, value in zip(jacobian, error))
        det = a00 * a11 - a01 * a01

        if abs(det) < 1e-12:
            return [0.0, 0.0]

        return [
            (a11 * b0 - a01 * b1) / det,
            (-a01 * b0 + a00 * b1) / det,
        ]

    def orientation_from_fk(self, joints):
        pose = self.fk_pose([float(value) for value in joints])
        if pose is None:
            return None
        if self.orientation_mapping == "square":
            return tool_z_axis_from_pose(pose)
        return [float(pose[4]), float(pose[5])]

    def orientation_error(self, target, current):
        if len(target) == 3 and len(current) == 3:
            return [
                float(target_value) - float(current_value)
                for target_value, current_value in zip(target, current)
            ]
        return [
            angle_diff(float(target[0]), float(current[0])),
            angle_diff(float(target[1]), float(current[1])),
        ]

    def apply_wrist_joint_step(self, pitch_axis, yaw_axis):
        yaw_step = yaw_axis * self.step_rad * self.j4_j6_speed_scale()
        self.target_joints[3] += yaw_step
        self.target_joints[4] -= pitch_axis * self.step_rad
        self.target_joints[5] -= yaw_step

    def clamp_joint_target(self):
        for index, (low, high) in enumerate(self.joint_limits):
            self.target_joints[index] = clamp(self.target_joints[index], low, high)

    def read_joint_angles(self):
        if self.arm is None:
            return None

        try:
            return extract_joint_angles(self.arm.get_joint_angles(), 6)
        except Exception as exc:
            self.get_logger().warning(f"get_joint_angles failed: {exc}", throttle_duration_sec=1.0)
            return None

    def send_joint_target(self, reason):
        target = [float(value) for value in self.target_joints]
        self.get_logger().info(
            f"{reason} joint target: {[round(value, 4) for value in target]}",
            throttle_duration_sec=0.5,
        )

        if not self.execute_motion:
            return

        if not self.arm_ready:
            self.get_logger().warning("arm is not ready, skip move_j", throttle_duration_sec=1.0)
            return

        try:
            result = self.arm.move_j(target)
            self.get_logger().info(f"move_j sent, result={result}", throttle_duration_sec=0.5)
        except Exception as exc:
            self.get_logger().error(f"move_j failed: {exc}")

    def destroy_node(self):
        if self.arm is not None and self.arm_ready:
            try:
                self.move_home_before_disconnect()
                self.arm.disconnect()
            except Exception as exc:
                self.get_logger().error(f"disconnect Piper failed: {exc}")
        super().destroy_node()

    def move_home_before_disconnect(self):
        if not self.execute_motion:
            return

        target = [0.0] * 6
        target[4] = math.radians(self.shutdown_j5_deg)
        self.target_joints = target.copy()

        self.get_logger().info(
            "shutdown home target before disconnect: "
            f"{[round(value, 4) for value in target]}, "
            f"tolerance={self.shutdown_home_tolerance}"
        )
        self.arm.move_j(target)

        while True:
            current = self.read_joint_angles()
            if current is None:
                self.get_logger().warning(
                    "waiting shutdown home: cannot read joint angles; disconnect is blocked",
                    throttle_duration_sec=1.0,
                )
                time.sleep(self.shutdown_home_poll_interval)
                continue

            errors = [
                abs(angle_diff(target_value, current_value))
                for target_value, current_value in zip(target, current)
            ]
            max_error = max(errors)
            if max_error <= self.shutdown_home_tolerance:
                self.get_logger().info(
                    "shutdown home reached: "
                    f"max_error={max_error:.6f}, "
                    f"joints={[round(value, 6) for value in current]}"
                )
                return

            self.get_logger().info(
                "waiting shutdown home before disconnect: "
                f"max_error={max_error:.6f}, "
                f"joints={[round(value, 6) for value in current]}",
                throttle_duration_sec=1.0,
            )
            time.sleep(self.shutdown_home_poll_interval)


def main(args=None):
    rclpy.init(args=args)
    node = PiperKeyboardController()

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
