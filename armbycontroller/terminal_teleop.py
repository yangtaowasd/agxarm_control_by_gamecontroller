#!/usr/bin/env python3
"""Publish Cartesian position increments from a terminal keyboard."""

import json
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from armbycontroller.ik.core import increment_tool_orientation
from armbycontroller.ik.core import quaternion_to_rotation_matrix
from armbycontroller.ik.core import rotation_matrix_to_quaternion


KEY_DIRECTIONS = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "z": (0.0, 0.0, 1.0),
    "x": (0.0, 0.0, -1.0),
}
KEY_ORIENTATIONS = {
    "\x1b[A": (-1.0, 0.0, 0.0),
    "\x1b[B": (1.0, 0.0, 0.0),
    "\x1b[D": (0.0, 1.0, 0.0),
    "\x1b[C": (0.0, -1.0, 0.0),
    "\x1b[5~": (0.0, 0.0, 1.0),
    "\x1b[6~": (0.0, 0.0, -1.0),
}


def read_terminal_key(stream):
    """Read one character or one complete ANSI special-key sequence."""
    key = stream.read(1)
    if key != "\x1b":
        return key
    while select.select([stream], [], [], 0.005)[0]:
        key += stream.read(1)
    return key


class TerminalTeleop(Node):
    """Publish incremental Cartesian position and orientation targets."""

    def __init__(self):
        super().__init__("terminal_teleop")
        self.declare_parameter("step", 0.01)
        self.declare_parameter("orientation_step_rad", 0.02)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("topic_prefix", "/nero")
        self.step = float(self.get_parameter("step").value)
        self.orientation_step_rad = float(
            self.get_parameter("orientation_step_rad").value
        )
        self.base_frame = str(self.get_parameter("base_frame").value)
        prefix = str(self.get_parameter("topic_prefix").value).strip("/")
        self.topic_prefix = f"/{prefix}" if prefix else ""
        if self.step <= 0.0 or self.orientation_step_rad <= 0.0:
            raise ValueError("position and orientation steps must be positive")

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
        self.current_orientation = None
        self.target_position = None
        self.target_orientation = None
        self.recovery_until = -1.0
        self.reset_after_recovery = False

    def pose_callback(self, message):
        """Capture the latest achieved position."""
        position = message.pose.position
        self.current_position = [position.x, position.y, position.z]
        orientation = message.pose.orientation
        self.current_orientation = quaternion_to_rotation_matrix(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        if self.target_position is None:
            self.target_position = self.current_position.copy()
            self.target_orientation = self.current_orientation.copy()
            self.get_logger().info(
                "current full pose received; position and orientation "
                "are active"
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
        if len(key) == 1:
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
            self.target_orientation = self.current_orientation.copy()
            self.reset_after_recovery = False
            self.get_logger().info(
                "recovery complete; target reset to current position"
            )
        if key == "r":
            if self.current_position is not None:
                self.target_position = self.current_position.copy()
                self.target_orientation = self.current_orientation.copy()
                self.get_logger().info("target reset to current pose")
            return
        direction = KEY_DIRECTIONS.get(key)
        orientation_step = KEY_ORIENTATIONS.get(key)
        if direction is None and orientation_step is None:
            return
        if self.target_position is None or self.target_orientation is None:
            self.get_logger().warning(
                f"waiting for {self.topic_prefix}/current_pose"
            )
            return

        if direction is not None:
            for index, value in enumerate(direction):
                self.target_position[index] += value * self.step
        if orientation_step is not None:
            pitch, yaw, roll = (
                value * self.orientation_step_rad
                for value in orientation_step
            )
            self.target_orientation = increment_tool_orientation(
                self.target_orientation, pitch, yaw, roll
            )
        quaternion = rotation_matrix_to_quaternion(self.target_orientation)
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.base_frame
        target.pose.position.x = self.target_position[0]
        target.pose.position.y = self.target_position[1]
        target.pose.position.z = self.target_position[2]
        target.pose.orientation.x = float(quaternion[0])
        target.pose.orientation.y = float(quaternion[1])
        target.pose.orientation.z = float(quaternion[2])
        target.pose.orientation.w = float(quaternion[3])
        self.publisher.publish(target)


def main(args=None):
    """Run raw-terminal keyboard control while servicing ROS callbacks."""
    rclpy.init(args=args)
    node = TerminalTeleop()
    if not sys.stdin.isatty():
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(
            "keyboard teleop must run in an interactive terminal"
        )

    print(
        "W/S:A/D:Z/X = XYZ; arrows = point; PgUp/PgDn = tilt; "
        "R = reset; Q = quit"
    )
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                key = read_terminal_key(sys.stdin)
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
