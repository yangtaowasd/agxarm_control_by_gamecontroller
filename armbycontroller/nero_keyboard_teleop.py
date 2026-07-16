#!/usr/bin/env python3
"""Increment Nero Cartesian position from a terminal keyboard."""

import json
import select
import sys
import termios
import time
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from armbycontroller.agx_ik import rotation_matrix_to_quaternion


KEY_DIRECTIONS = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "z": (0.0, 0.0, 1.0),
    "x": (0.0, 0.0, -1.0),
}


def make_pointing_quaternion(direction, roll_reference):
    """Point link7 local +Z along direction with a stable roll reference."""
    tool_z = np.asarray(direction, dtype=float)
    reference = np.asarray(roll_reference, dtype=float)
    if tool_z.shape != (3,) or not np.all(np.isfinite(tool_z)):
        raise ValueError("pointing_direction must contain three finite values")
    if reference.shape != (3,) or not np.all(np.isfinite(reference)):
        raise ValueError("roll_reference must contain three finite values")
    direction_norm = float(np.linalg.norm(tool_z))
    if direction_norm < 1e-12:
        raise ValueError("pointing_direction must be non-zero")
    tool_z /= direction_norm

    tool_x = reference - np.dot(reference, tool_z) * tool_z
    if np.linalg.norm(tool_x) < 1e-12:
        # Choose the global axis least parallel to the requested direction.
        fallback = np.zeros(3)
        fallback[int(np.argmin(np.abs(tool_z)))] = 1.0
        tool_x = fallback - np.dot(fallback, tool_z) * tool_z
    tool_x /= np.linalg.norm(tool_x)
    tool_y = np.cross(tool_z, tool_x)
    rotation = np.column_stack((tool_x, tool_y, tool_z))
    return rotation_matrix_to_quaternion(rotation)


class NeroKeyboardTeleop(Node):
    """Publish position increments with a fixed tool pointing direction."""

    def __init__(self):
        super().__init__("nero_keyboard_teleop")
        self.declare_parameter("step", 0.01)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("topic_prefix", "/nero")
        self.declare_parameter("pointing_direction", [0.0, 0.0, -1.0])
        self.declare_parameter("roll_reference", [1.0, 0.0, 0.0])
        self.step = float(self.get_parameter("step").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        prefix = str(self.get_parameter("topic_prefix").value).strip("/")
        self.topic_prefix = f"/{prefix}" if prefix else ""
        if self.step <= 0.0:
            raise ValueError("step must be greater than zero")
        self.target_orientation = make_pointing_quaternion(
            self.get_parameter("pointing_direction").value,
            self.get_parameter("roll_reference").value,
        )

        self.publisher = self.create_publisher(
            PoseStamped, f"{self.topic_prefix}/target_pose", 10
        )
        self.create_subscription(
            PoseStamped,
            f"{self.topic_prefix}/current_pose",
            self.pose_callback,
            10,
        )
        self.create_subscription(
            String, f"{self.topic_prefix}/ik_status", self.status_callback, 10
        )
        self.current_position = None
        self.target_position = None
        self.recovery_until = -1.0
        self.reset_after_recovery = False

    def pose_callback(self, message):
        """Capture the latest achieved position."""
        position = message.pose.position
        self.current_position = [position.x, position.y, position.z]
        if self.target_position is None:
            self.target_position = self.current_position.copy()
            self.get_logger().info(
                "current pose received; fixed pointing direction is active"
            )

    def status_callback(self, message):
        """Pause keyboard input and reset accumulation after IK recovery."""
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if status.get("state") != "recovering":
            return
        pause = max(0.0, float(status.get("pause_seconds", 2.0)))
        self.recovery_until = time.monotonic() + pause
        self.reset_after_recovery = True
        self.get_logger().warning(status.get("message", "IK recovery"))

    def handle_key(self, key):
        """Apply one key increment and publish the resulting target pose."""
        key = key.lower()
        remaining = self.recovery_until - time.monotonic()
        if remaining > 0.0:
            self.get_logger().warning(
                f"keyboard paused for IK recovery: {remaining:.1f} s"
            )
            return
        if self.reset_after_recovery:
            if self.current_position is None:
                return
            self.target_position = self.current_position.copy()
            self.reset_after_recovery = False
            self.get_logger().info(
                "recovery complete; target reset to current position"
            )
        if key == "r":
            if self.current_position is not None:
                self.target_position = self.current_position.copy()
                self.get_logger().info("target reset to current position")
            return
        direction = KEY_DIRECTIONS.get(key)
        if direction is None:
            return
        if self.target_position is None:
            self.get_logger().warning(
                f"waiting for {self.topic_prefix}/current_pose"
            )
            return

        for index, value in enumerate(direction):
            self.target_position[index] += value * self.step
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.base_frame
        target.pose.position.x = self.target_position[0]
        target.pose.position.y = self.target_position[1]
        target.pose.position.z = self.target_position[2]
        target.pose.orientation.x = float(self.target_orientation[0])
        target.pose.orientation.y = float(self.target_orientation[1])
        target.pose.orientation.z = float(self.target_orientation[2])
        target.pose.orientation.w = float(self.target_orientation[3])
        self.publisher.publish(target)


def main(args=None):
    """Run raw-terminal keyboard control while servicing ROS callbacks."""
    rclpy.init(args=args)
    node = NeroKeyboardTeleop()
    if not sys.stdin.isatty():
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(
            "keyboard teleop must run in an interactive terminal"
        )

    print("W/S: +/-X  A/D: +/-Y  Z/X: +/-Z  R: reset  Q: quit")
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                key = sys.stdin.read(1)
                if key.lower() == "q":
                    break
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
