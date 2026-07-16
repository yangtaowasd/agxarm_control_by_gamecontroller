#!/usr/bin/env python3
"""ROS 2 keyboard controller for NERO joint jogging and Cartesian IK."""

import time
from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Sequence

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD

from armbycontroller.agx_ik import AgxIkEngine
from armbycontroller.agx_ik import create_tracik_solver
from armbycontroller.agx_ik import IkFailure
from armbycontroller.agx_ik import quaternion_to_rotation_matrix
from armbycontroller.agx_ik import resolve_urdf_path
from armbycontroller.nero_keyboard_teleop import make_pointing_quaternion


JOINT_COUNT = 7
KEY_JOINT_1, KEY_JOINT_7 = 0, 6
KEY_DECREASE, KEY_INCREASE = 7, 8
KEY_HOME, KEY_ESTOP, KEY_MODE_TOGGLE = 9, 10, 11
KEY_FORWARD, KEY_BACKWARD, KEY_UP, KEY_DOWN = 12, 13, 14, 15
KEY_COUNT = 16


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


class NeroJointJogState:
    """Track selection, key edges, and a limit-safe seven-joint target."""

    def __init__(self, joint_limits, step_rad, initial_joints=None):
        if len(joint_limits) != JOINT_COUNT or step_rad <= 0.0:
            raise ValueError("NERO requires 7 limits and step_rad > 0")
        self.joint_limits = [tuple(map(float, limit)) for limit in joint_limits]
        if any(low >= high for low, high in self.joint_limits):
            raise ValueError("each joint limit must satisfy low < high")
        self.step_rad = float(step_rad)
        self.selected_joint = 0
        self.target_joints = [0.0] * JOINT_COUNT
        self.previous_keys = [0] * KEY_COUNT
        if initial_joints is not None:
            self.sync_target(initial_joints)

    def sync_target(self, joints: Sequence[float]):
        if len(joints) != JOINT_COUNT:
            raise ValueError("NERO joint target must contain 7 values")
        self.target_joints = [
            clamp(float(value), low, high)
            for value, (low, high) in zip(joints, self.joint_limits)
        ]

    def update(self, keys: Sequence[int]):
        if len(keys) < KEY_COUNT:
            raise ValueError(f"keyboard state must contain {KEY_COUNT} values")
        pressed = [bool(value) for value in keys[:KEY_COUNT]]
        rising = [
            current and not previous
            for current, previous in zip(pressed, self.previous_keys)
        ]
        old_selection = self.selected_joint
        for index in range(KEY_JOINT_1, KEY_JOINT_7 + 1):
            if rising[index]:
                self.selected_joint = index

        home = rising[KEY_HOME]
        estop = rising[KEY_ESTOP]
        toggle = rising[KEY_MODE_TOGGLE]
        changed = False
        if home:
            target = [clamp(0.0, low, high) for low, high in self.joint_limits]
            changed, self.target_joints = target != self.target_joints, target
        elif not estop and not toggle:
            direction = pressed[KEY_INCREASE] - pressed[KEY_DECREASE]
            if direction:
                low, high = self.joint_limits[self.selected_joint]
                old = self.target_joints[self.selected_joint]
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
        )


NERO_FIRMWARES = {
    "default": NeroFW.DEFAULT,
    "v111": NeroFW.V111,
    "v112": NeroFW.V112,
    "v120": NeroFW.V120,
}


def extract_joint_angles(result):
    if result is None:
        return None
    value = result.msg if hasattr(result, "msg") else result
    if isinstance(value, Iterable) and not isinstance(value, (str, bytes, dict)):
        joints = [float(item) for item in value]
        return joints[:JOINT_COUNT] if len(joints) >= JOINT_COUNT else None
    if isinstance(value, dict):
        for key in ("joint_angles", "angles", "data", "msg"):
            joints = value.get(key)
            if isinstance(joints, (list, tuple)) and len(joints) >= JOINT_COUNT:
                return [float(item) for item in joints[:JOINT_COUNT]]
    return None


class NeroJointKeyboardController(Node):
    def __init__(self):
        super().__init__("nero_joint_keyboard_controller")

        self.declare_parameter("keyboard_topic", "/nero_keyboard_state")
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("firmware", "default")
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("step_rad", 0.005)
        self.declare_parameter("speed_percent", 20)
        self.declare_parameter("joint_max_acceleration", 1.0)
        self.declare_parameter("keyboard_timeout", 0.3)
        self.declare_parameter("enable_timeout", 5.0)
        self.declare_parameter("feedback_timeout", 3.0)
        self.declare_parameter("move_home_on_start", True)
        self.declare_parameter("startup_home_timeout", 30.0)
        self.declare_parameter("startup_home_tolerance", 0.01)
        self.declare_parameter("clear_errors_on_enable", True)
        self.declare_parameter("execute_motion", True)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tip_link", "link7")
        self.declare_parameter("cartesian_step", 0.005)
        self.declare_parameter("ik_timeout", 0.01)
        self.declare_parameter("ik_tolerance", 1e-5)
        self.declare_parameter("fk_position_tolerance", 1e-4)
        self.declare_parameter("fk_rotation_tolerance", 1e-3)
        self.declare_parameter("pointing_roll_samples", 8)
        self.declare_parameter("pointing_direction", [0.0, 0.0, -1.0])
        self.declare_parameter("roll_reference", [1.0, 0.0, 0.0])
        self.declare_parameter("robot_min_reach", 0.1447354)
        self.declare_parameter("robot_max_reach", 0.7374482)
        self.declare_parameter("workspace_inner_margin", 0.05)
        self.declare_parameter("workspace_outer_margin", 0.10)
        self.declare_parameter("ik_recovery_pause", 2.0)

        self.keyboard_topic = str(self.get_parameter("keyboard_topic").value)
        self.can_interface = str(self.get_parameter("can_interface").value)
        self.firmware_name = str(self.get_parameter("firmware").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.step_rad = float(self.get_parameter("step_rad").value)
        self.speed_percent = int(self.get_parameter("speed_percent").value)
        self.joint_max_acceleration = float(
            self.get_parameter("joint_max_acceleration").value
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
        self.execute_motion = bool(self.get_parameter("execute_motion").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tip_link = str(self.get_parameter("tip_link").value)
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
        self.workspace_min_radius = (
            float(self.get_parameter("robot_min_reach").value)
            + float(self.get_parameter("workspace_inner_margin").value)
        )
        self.workspace_max_radius = (
            float(self.get_parameter("robot_max_reach").value)
            - float(self.get_parameter("workspace_outer_margin").value)
        )
        self.ik_recovery_pause = float(
            self.get_parameter("ik_recovery_pause").value
        )

        if self.control_rate <= 0.0:
            raise ValueError("control_rate must be > 0")
        if not 1 <= self.speed_percent <= 100:
            raise ValueError("speed_percent must be in [1, 100]")
        if self.joint_max_acceleration <= 0.0:
            raise ValueError("joint_max_acceleration must be > 0")
        if self.cartesian_step <= 0.0:
            raise ValueError("cartesian_step must be > 0")
        if self.workspace_min_radius >= self.workspace_max_radius:
            raise ValueError("NERO workspace margins leave no usable region")
        if not 1 <= self.pointing_roll_samples <= 72:
            raise ValueError("pointing_roll_samples must be in [1, 72]")
        if self.keyboard_timeout <= 0.0:
            raise ValueError("keyboard_timeout must be > 0")
        if self.startup_home_timeout <= 0.0:
            raise ValueError("startup_home_timeout must be > 0")
        if self.startup_home_tolerance <= 0.0:
            raise ValueError("startup_home_tolerance must be > 0")

        self.firmware = NERO_FIRMWARES.get(self.firmware_name)
        if self.firmware is None:
            raise ValueError(
                f"unsupported NERO firmware {self.firmware_name!r}; "
                f"choose one of {sorted(NERO_FIRMWARES)}"
            )

        self.joint_limits = [
            tuple(ROBOT_JOINT_LIMIT_PRESET_RAD["nero"][f"joint{index}"])
            for index in range(1, JOINT_COUNT + 1)
        ]
        self.jog = NeroJointJogState(self.joint_limits, self.step_rad)
        urdf_path = resolve_urdf_path(
            str(self.get_parameter("urdf_path").value), "nero"
        )
        self.ik_solver = create_tracik_solver(
            urdf_path,
            self.base_frame,
            self.tip_link,
            JOINT_COUNT,
            self.ik_timeout,
            self.ik_tolerance,
        )
        self.ik_engine = AgxIkEngine(
            self.ik_solver,
            JOINT_COUNT,
            self.workspace_min_radius,
            self.workspace_max_radius,
            self.fk_position_tolerance,
            self.fk_rotation_tolerance,
            self.pointing_roll_samples,
        )
        pointing_quaternion = make_pointing_quaternion(
            self.get_parameter("pointing_direction").value,
            self.get_parameter("roll_reference").value,
        )
        self.ik_target_rotation = quaternion_to_rotation_matrix(
            *pointing_quaternion
        )
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

        self.keyboard_sub = self.create_subscription(
            Int32MultiArray,
            self.keyboard_topic,
            self.keyboard_callback,
            qos_profile_sensor_data,
        )
        self.connect_arm()
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_tick)

        self.get_logger().info(
            "NERO ready: P=joint/IK; joint 1-7+A/D; "
            "IK W/S+A/D+Z/X; SPACE=home; E=E-stop"
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
            robot=ArmModel.NERO,
            firmeware_version=self.firmware,
            channel=self.can_interface,
        )
        self.arm = AgxArmFactory.create_arm(config)
        self.arm.set_joint_limits_enabled(True)

        if not self.execute_motion:
            self.get_logger().warning("dry-run mode: NERO hardware commands are disabled")
            self.arm_ready = True
            return

        try:
            self.arm.connect()
            self.arm_connected = True
            self.get_logger().info(f"connected NERO on {self.can_interface}")
            if not self.enable_arm():
                self.get_logger().error("NERO enable timed out; motion is disabled")
                return
            limits_ok = self.arm.set_joint_acc_limits(
                joint_index=255,
                max_joint_acc=self.joint_max_acceleration,
                timeout=1.0,
            )
            if not limits_ok:
                self.get_logger().error(
                    "failed to set/read back NERO joint acceleration limits"
                )
                return
            self.arm.set_speed_percent(self.speed_percent)
            joints = self.wait_for_joint_feedback()
            if joints is None:
                self.get_logger().error(
                    "no complete 7-joint feedback; motion is disabled for safety"
                )
                return
            self.jog.sync_target(joints)
            self.get_logger().info(
                "synced current NERO joints: "
                f"{[round(value, 4) for value in self.jog.target_joints]}"
            )
            if self.move_home_on_start and not self.move_home_and_wait():
                self.get_logger().error(
                    "NERO startup home was not reached; keyboard motion is disabled"
                )
                return
            self.arm_ready = True
        except Exception as exc:
            self.arm_ready = False
            self.get_logger().error(f"failed to initialize NERO: {exc}")

    def enable_arm(self):
        deadline = time.monotonic() + self.enable_timeout
        while time.monotonic() < deadline:
            try:
                if self.clear_errors_on_enable:
                    self.arm.clear_joint_error()
                if self.arm.enable():
                    return True
                states = self.arm.get_joints_enable_status_list()
                if len(states) >= JOINT_COUNT and all(states[:JOINT_COUNT]):
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
                joints = extract_joint_angles(self.arm.get_joint_angles())
                if joints is not None:
                    return joints
            except Exception as exc:
                self.get_logger().warning(
                    f"joint feedback failed: {exc}", throttle_duration_sec=1.0
                )
            time.sleep(0.05)
        return None

    def move_home_and_wait(self):
        home = [0.0] * JOINT_COUNT
        self.get_logger().info(
            "commanding NERO startup home [0, 0, 0, 0, 0, 0, 0]"
        )
        self.arm.move_j(home)
        deadline = time.monotonic() + self.startup_home_timeout

        while time.monotonic() < deadline:
            joints = extract_joint_angles(self.arm.get_joint_angles())
            if joints is not None:
                max_error = max(abs(value) for value in joints)
                if max_error <= self.startup_home_tolerance:
                    self.jog.sync_target(home)
                    self.get_logger().info(
                        f"NERO startup home reached; max error={max_error:.6f} rad"
                    )
                    return True
                self.get_logger().info(
                    f"waiting for NERO startup home; max error={max_error:.4f} rad",
                    throttle_duration_sec=1.0,
                )
            time.sleep(0.05)
        return False

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
            self.get_logger().info(f"selected NERO joint {update.selected_joint + 1}")

        if update.estop_requested:
            self.trigger_emergency_stop()
            return
        if self.emergency_stopped:
            return

        if update.mode_toggle_requested:
            self.toggle_control_mode()
            return

        if update.home_requested:
            self.ik_target_position = None
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
            position, _ = self.ik_solver.fk(joints)
            self.ik_target_position = np.asarray(position, dtype=float)
            self.ik_valid_history.clear()
            self.remember_ik_valid(self.ik_target_position, joints)
            self.control_mode = "ik"
            self.get_logger().info(
                "mode=IK: fixed pointing direction; W/S=X, A/D=Y, Z/X=Z"
            )
        else:
            joints = self.current_or_target_joints()
            self.jog.sync_target(joints)
            self.control_mode = "joint"
            self.get_logger().info("mode=JOINT: 1-7 select, A/D jog")

    def current_or_target_joints(self):
        if self.execute_motion and self.arm_ready:
            try:
                joints = extract_joint_angles(self.arm.get_joint_angles())
                if joints is not None:
                    return np.asarray(joints, dtype=float)
            except Exception as exc:
                self.get_logger().warning(
                    f"joint feedback unavailable; using last target: {exc}",
                    throttle_duration_sec=1.0,
                )
        return np.asarray(self.jog.target_joints, dtype=float)

    def remember_ik_valid(self, position, joints):
        self.ik_valid_history.append(
            (np.asarray(position, dtype=float).copy(),
             np.asarray(joints, dtype=float).copy())
        )

    def recover_ik(self, reason):
        self.ik_recovery_until = time.monotonic() + self.ik_recovery_pause
        if self.ik_valid_history:
            position, joints = self.ik_valid_history[-1]
            self.ik_target_position = position.copy()
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
                keys[KEY_UP] - keys[KEY_DOWN],
            ],
            dtype=float,
        )
        if not np.any(direction):
            return
        if self.ik_target_position is None:
            joints = self.current_or_target_joints()
            self.ik_target_position = np.asarray(
                self.ik_solver.fk(joints)[0], dtype=float
            )
        candidate = self.ik_target_position + direction * self.cartesian_step
        seed = np.asarray(self.jog.target_joints, dtype=float)
        try:
            result = self.ik_engine.solve(
                candidate, self.ik_target_rotation, seed
            )
        except IkFailure as error:
            self.recover_ik(str(error))
            return
        self.ik_target_position = candidate
        self.jog.sync_target(result.joints)
        self.remember_ik_valid(candidate, result.joints)
        self.send_target("IK Cartesian jog")

    def send_target(self, reason):
        target = [float(value) for value in self.jog.target_joints]
        self.get_logger().info(
            f"{reason}: {[round(value, 4) for value in target]}",
            throttle_duration_sec=0.25,
        )
        if not self.execute_motion:
            return
        if not self.arm_ready:
            self.get_logger().warning(
                "NERO is not ready; command skipped", throttle_duration_sec=1.0
            )
            return
        try:
            self.arm.move_j(target)
        except Exception as exc:
            self.get_logger().error(f"NERO move_j failed: {exc}")

    def trigger_emergency_stop(self):
        self.emergency_stopped = True
        self.arm_ready = False
        self.get_logger().error("NERO ELECTRONIC EMERGENCY STOP requested")
        if self.execute_motion and self.arm_connected:
            try:
                self.arm.electronic_emergency_stop()
            except Exception as exc:
                self.get_logger().error(f"NERO emergency stop command failed: {exc}")

    def destroy_node(self):
        if self.arm is not None and self.arm_connected:
            try:
                self.get_logger().info(
                    "Ctrl-C shutdown: disconnecting without motion or enable/disable commands"
                )
                self.arm.disconnect()
            except Exception as exc:
                self.get_logger().error(f"failed to disconnect NERO: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NeroJointKeyboardController()
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
