#!/usr/bin/env python3
"""Bridge Motion Link phone orientation and ROS2 arm pose/status topics."""

import json
import math
import threading
import time
from urllib.parse import quote
from urllib.parse import urlparse
from urllib.request import urlopen

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import websocket

from armbycontroller.ik_core import quaternion_to_rotation_matrix
from armbycontroller.ik_core import rotation_matrix_to_quaternion
from armbycontroller.lie import rotation_exp
from armbycontroller.lie import rotation_from_vector
from armbycontroller.lie import rotation_vector


def phone_rotation(orientation):
    """Convert Motion Link alpha/beta/gamma degrees to a rotation matrix."""
    try:
        alpha = math.radians(float(orientation["alpha"]))
        beta = math.radians(float(orientation["beta"]))
        gamma = math.radians(float(orientation["gamma"]))
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("phone orientation is incomplete") from error
    if not all(math.isfinite(value) for value in (alpha, beta, gamma)):
        raise ValueError("phone orientation must be finite")
    return (
        rotation_exp(np.asarray([0.0, 0.0, 1.0]), alpha)
        @ rotation_exp(np.asarray([1.0, 0.0, 0.0]), beta)
        @ rotation_exp(np.asarray([0.0, 1.0, 0.0]), gamma)
    )


def relative_target_rotation(
    robot_reference, phone_reference, phone_current, maximum_angle
):
    """Map bounded phone-relative rotation onto the reference tool pose."""
    maximum_angle = float(maximum_angle)
    if not math.isfinite(maximum_angle) or maximum_angle <= 0.0:
        raise ValueError("maximum_angle must be positive and finite")
    phone_delta = phone_reference.T @ phone_current
    delta_vector = rotation_vector(phone_delta)
    angle = float(np.linalg.norm(delta_vector))
    if angle > maximum_angle:
        delta_vector *= maximum_angle / angle
    return robot_reference @ rotation_from_vector(delta_vector)


def websocket_url(server_url, session):
    """Build the authenticated Motion Link robot WebSocket URL."""
    parsed = urlparse(str(server_url).rstrip("/"))
    if parsed.scheme not in ("http", "https") or not parsed.netloc:
        raise ValueError("server_url must be an HTTP or HTTPS origin")
    scheme = "wss" if parsed.scheme == "https" else "ws"
    return (
        f"{scheme}://{parsed.netloc}/ws"
        f"?session={quote(str(session), safe='')}&role=robot"
    )


class MotionLinkBridge(Node):
    """Use phone orientation as a bounded tool-orientation command."""

    def __init__(self):
        super().__init__("motion_link_bridge")
        self.declare_parameter("server_url", "http://127.0.0.1:8080")
        self.declare_parameter("robot_model", "nero")
        self.declare_parameter("topic_prefix", "/nero")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("enable_commands", False)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("maximum_rotation_rad", 0.6)
        self.declare_parameter("sample_timeout", 0.25)
        self.declare_parameter("end_effector", "gripper")

        self.server_url = str(self.get_parameter("server_url").value).rstrip(
            "/"
        )
        self.robot_model = str(
            self.get_parameter("robot_model").value
        ).lower()
        if self.robot_model not in ("nero", "piper_l"):
            raise ValueError("robot_model must be nero or piper_l")
        prefix = str(self.get_parameter("topic_prefix").value).strip("/")
        self.topic_prefix = f"/{prefix}" if prefix else ""
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.enable_commands = bool(
            self.get_parameter("enable_commands").value
        )
        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.maximum_rotation_rad = float(
            self.get_parameter("maximum_rotation_rad").value
        )
        self.sample_timeout = float(
            self.get_parameter("sample_timeout").value
        )
        self.end_effector = str(
            self.get_parameter("end_effector").value
        ).strip().lower()
        if self.end_effector not in ("none", "gripper", "revo2"):
            raise ValueError(
                "end_effector must be none, gripper or revo2"
            )
        if (
            not math.isfinite(self.publish_rate_hz)
            or self.publish_rate_hz <= 0.0
            or not math.isfinite(self.sample_timeout)
            or self.sample_timeout <= 0.0
            or not math.isfinite(self.maximum_rotation_rad)
            or self.maximum_rotation_rad <= 0.0
        ):
            raise ValueError("bridge rate, timeout and angle must be positive")

        self.target_publisher = self.create_publisher(
            PoseStamped, f"{self.topic_prefix}/target_pose", 10
        )
        self.create_subscription(
            PoseStamped,
            f"{self.topic_prefix}/current_pose",
            self._pose_callback,
            10,
        )
        self.create_subscription(
            JointState, "/joint_states", self._joint_callback, 10
        )
        self.create_subscription(
            String,
            f"{self.topic_prefix}/ik_status",
            self._ik_status_callback,
            10,
        )

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._socket = None
        self._latest_sample = None
        self._latest_sample_at = -math.inf
        self._last_sequence = None
        self._current_position = None
        self._current_rotation = None
        self._phone_reference = None
        self._robot_reference_position = None
        self._robot_reference_rotation = None
        self._joint_degrees = []
        self._joint_state_at = -math.inf
        self._ik_error = ""
        self._connected = False
        self._last_connect_error = ""

        self.create_timer(1.0 / self.publish_rate_hz, self._publish_target)
        self._thread = threading.Thread(
            target=self._connection_loop,
            name="motion-link-bridge",
            daemon=True,
        )
        self._thread.start()
        command_state = "enabled" if self.enable_commands else "disabled"
        self.get_logger().info(
            f"Motion Link bridge started; pose commands {command_state}"
        )

    def _pose_callback(self, message):
        position = message.pose.position
        orientation = message.pose.orientation
        try:
            rotation = quaternion_to_rotation_matrix(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        except ValueError:
            return
        with self._lock:
            self._current_position = np.array(
                [position.x, position.y, position.z], dtype=float
            )
            self._current_rotation = rotation

    def _joint_callback(self, message):
        expected = 7 if self.robot_model == "nero" else 6
        values = np.asarray(message.position[:expected], dtype=float)
        if values.shape != (expected,) or not np.all(np.isfinite(values)):
            return
        with self._lock:
            self._joint_degrees = np.degrees(values).tolist()
            self._joint_state_at = time.monotonic()

    def _ik_status_callback(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        state = str(status.get("state", ""))
        with self._lock:
            self._ik_error = (
                str(status.get("message", ""))[:160]
                if state == "recovering"
                else ""
            )
            if state == "recovering":
                self._clear_reference_locked()

    def _clear_reference_locked(self):
        self._phone_reference = None
        self._robot_reference_position = None
        self._robot_reference_rotation = None

    def _accept_sensor_message(self, message):
        if message.get("type") != "sensor":
            return
        sample = message.get("sample")
        if not isinstance(sample, dict):
            return
        try:
            rotation = phone_rotation(sample.get("orientation", {}))
        except ValueError:
            return
        sequence = sample.get("sequence")
        with self._lock:
            self._latest_sample = rotation
            self._latest_sample_at = time.monotonic()
            self._last_sequence = sequence

    def _publish_target(self):
        if not self.enable_commands:
            return
        now = time.monotonic()
        with self._lock:
            if (
                self._latest_sample is None
                or self._current_position is None
                or now - self._latest_sample_at > self.sample_timeout
            ):
                return
            if self._phone_reference is None:
                self._phone_reference = self._latest_sample.copy()
                self._robot_reference_position = self._current_position.copy()
                self._robot_reference_rotation = self._current_rotation.copy()
                self.get_logger().info(
                    "phone zero captured; holding the current tool position"
                )
                return
            target_position = self._robot_reference_position.copy()
            target_rotation = relative_target_rotation(
                self._robot_reference_rotation,
                self._phone_reference,
                self._latest_sample,
                self.maximum_rotation_rad,
            )

        quaternion = rotation_matrix_to_quaternion(target_rotation)
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.base_frame
        target.pose.position.x = float(target_position[0])
        target.pose.position.y = float(target_position[1])
        target.pose.position.z = float(target_position[2])
        target.pose.orientation.x = float(quaternion[0])
        target.pose.orientation.y = float(quaternion[1])
        target.pose.orientation.z = float(quaternion[2])
        target.pose.orientation.w = float(quaternion[3])
        self.target_publisher.publish(target)

    def _bootstrap_session(self):
        with urlopen(
            f"{self.server_url}/api/bootstrap", timeout=2.0
        ) as response:
            payload = json.loads(response.read().decode("utf-8"))
        session = payload.get("session")
        if not session:
            raise RuntimeError("Motion Link bootstrap omitted its session")
        return str(session)

    def _robot_status(self):
        now = time.monotonic()
        with self._lock:
            joints = list(self._joint_degrees)
            recent = now - self._joint_state_at <= 1.0
            errors = [self._ik_error] if self._ik_error else []
        return {
            "type": "robot-status",
            "robot": {
                "connected": recent,
                "manufacturer": "AgileX",
                "model": (
                    "NERO" if self.robot_model == "nero" else "PIPER"
                ),
                "endEffector": self.end_effector,
                "gripper": self.end_effector == "gripper",
                "mode": "PHONE" if self.enable_commands else "MONITOR",
                "joints": joints if recent else [],
                "errors": errors,
            },
        }

    def _connection_loop(self):
        while not self._stop.is_set():
            connection = None
            try:
                session = self._bootstrap_session()
                connection = websocket.create_connection(
                    websocket_url(self.server_url, session),
                    timeout=1.0,
                    http_proxy_host=None,
                )
                connection.settimeout(0.05)
                with self._lock:
                    self._socket = connection
                    self._connected = True
                    self._last_connect_error = ""
                self.get_logger().info(
                    f"connected to Motion Link at {self.server_url}"
                )
                last_status_at = -math.inf
                while not self._stop.is_set():
                    now = time.monotonic()
                    if now - last_status_at >= 0.1:
                        connection.send(json.dumps(self._robot_status()))
                        last_status_at = now
                    try:
                        raw_message = connection.recv()
                    except websocket.WebSocketTimeoutException:
                        continue
                    if not raw_message:
                        raise RuntimeError("Motion Link closed the connection")
                    try:
                        message = json.loads(raw_message)
                    except (TypeError, ValueError):
                        continue
                    self._accept_sensor_message(message)
            except Exception as error:
                detail = str(error)
                with self._lock:
                    self._connected = False
                    self._socket = None
                    self._clear_reference_locked()
                    changed = detail != self._last_connect_error
                    self._last_connect_error = detail
                if changed and not self._stop.is_set():
                    self.get_logger().warning(
                        f"Motion Link unavailable: {detail}"
                    )
                self._stop.wait(1.0)
            finally:
                if connection is not None:
                    connection.close()
                with self._lock:
                    self._connected = False
                    self._socket = None

    def destroy_node(self):
        """Stop the WebSocket worker before destroying ROS resources."""
        self._stop.set()
        with self._lock:
            connection = self._socket
        if connection is not None:
            connection.close()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    """Run the Motion Link bridge."""
    rclpy.init(args=args)
    node = MotionLinkBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
