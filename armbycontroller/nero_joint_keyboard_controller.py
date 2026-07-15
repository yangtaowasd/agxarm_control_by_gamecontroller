#!/usr/bin/env python3
"""ROS 2 keyboard controller for independently jogging NERO's seven joints."""

import time
from collections.abc import Iterable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD

from armbycontroller.nero_joint_logic import (
    JOINT_COUNT,
    KEY_COUNT,
    NeroJointJogState,
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
        self.declare_parameter("keyboard_timeout", 0.3)
        self.declare_parameter("enable_timeout", 5.0)
        self.declare_parameter("feedback_timeout", 3.0)
        self.declare_parameter("move_home_on_start", True)
        self.declare_parameter("startup_home_timeout", 30.0)
        self.declare_parameter("startup_home_tolerance", 0.01)
        self.declare_parameter("clear_errors_on_enable", True)
        self.declare_parameter("execute_motion", True)

        self.keyboard_topic = str(self.get_parameter("keyboard_topic").value)
        self.can_interface = str(self.get_parameter("can_interface").value)
        self.firmware_name = str(self.get_parameter("firmware").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.step_rad = float(self.get_parameter("step_rad").value)
        self.speed_percent = int(self.get_parameter("speed_percent").value)
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

        if self.control_rate <= 0.0:
            raise ValueError("control_rate must be > 0")
        if not 1 <= self.speed_percent <= 100:
            raise ValueError("speed_percent must be in [1, 100]")
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
            "NERO joint keyboard ready: 1-7=select, A/D=-/+, SPACE=home, E=E-stop"
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

        update = self.jog.update(keys)
        if update.selection_changed:
            self.get_logger().info(f"selected NERO joint {update.selected_joint + 1}")

        if update.estop_requested:
            self.trigger_emergency_stop()
            return
        if self.emergency_stopped or not update.target_changed:
            return

        reason = "home" if update.home_requested else f"joint {update.selected_joint + 1} jog"
        self.send_target(reason)

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
