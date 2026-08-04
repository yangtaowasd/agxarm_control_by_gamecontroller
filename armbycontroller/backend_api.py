#!/usr/bin/env python3
"""Expose a local HTTP/SSE backend API backed by ROS 2 topics."""

import hmac
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
import json
import math
import threading
import time
from urllib.parse import urlparse
import uuid

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

from armbycontroller.backend_protocol import API_VERSION
from armbycontroller.backend_protocol import extract_named_arm_joint_state
from armbycontroller.backend_protocol import MAX_BODY_BYTES
from armbycontroller.backend_protocol import sanitize_action
from armbycontroller.backend_protocol import sanitize_pose_command
from armbycontroller.backend_protocol import validate_bind_address
from armbycontroller.control_protocol import ACTION_KEYS
from armbycontroller.control_protocol import KEY_COUNT
from armbycontroller.control_protocol import sanitize_controller_keys
from armbycontroller.model_profiles import get_arm_profile


class BackendState:
    """Thread-safe latest-value store shared by ROS and HTTP threads."""

    def __init__(
        self,
        robot_model,
        commands_enabled,
        simulation_mode=True,
        execute_motion=False,
    ):
        self._condition = threading.Condition()
        self._version = 0
        self._state = {
            "apiVersion": API_VERSION,
            "robotModel": robot_model,
            "commandsEnabled": bool(commands_enabled),
            "runtimeMode": (
                "simulation" if simulation_mode else "hardware"
            ),
            "simulationMode": bool(simulation_mode),
            "executeMotion": bool(execute_motion),
            "jointState": None,
            "currentPose": None,
            "ikStatus": None,
            "updatedAt": None,
        }
        self._joint_received_at = -math.inf

    def update(self, field, value, joint_feedback=False):
        """Store a new field and wake event-stream subscribers."""
        with self._condition:
            self._state[field] = value
            self._state["updatedAt"] = time.time()
            if joint_feedback:
                self._joint_received_at = time.monotonic()
            self._version += 1
            self._condition.notify_all()

    def snapshot(self):
        """Return an isolated JSON-compatible state snapshot and version."""
        with self._condition:
            state = json.loads(json.dumps(self._state))
            state["connected"] = (
                time.monotonic() - self._joint_received_at <= 1.0
            )
            return self._version, state

    def wait_after(self, version, timeout):
        """Wait for a newer state version, then return a snapshot."""
        with self._condition:
            if self._version <= version:
                self._condition.wait(timeout)
        return self.snapshot()


class _BackendHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class BackendRequestHandler(BaseHTTPRequestHandler):
    """Serve the versioned robot API without exposing ROS details."""

    server_version = "ArmByControllerBackend/1"

    @property
    def node(self):
        return self.server.node

    def log_message(self, format_string, *args):
        self.node.get_logger().debug(format_string % args)

    def _headers(self, content_type="application/json; charset=utf-8"):
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Type", content_type)
        self.send_header("X-Content-Type-Options", "nosniff")

    def _json(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self._headers()
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _error(self, status, code, message):
        self._json(
            status,
            {"ok": False, "error": {"code": code, "message": message}},
        )

    def _authorized(self):
        token = self.node.api_token
        if not token:
            return True
        authorization = self.headers.get("Authorization", "")
        supplied = (
            authorization[7:]
            if authorization.startswith("Bearer ")
            else self.headers.get("X-Armby-Token", "")
        )
        return hmac.compare_digest(str(supplied), token)

    def _require_authorization(self):
        if self._authorized():
            return True
        self._error(401, "unauthorized", "valid API token required")
        return False

    def _read_json(self):
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError as error:
            raise ValueError("invalid Content-Length") from error
        if content_length <= 0 or content_length > MAX_BODY_BYTES:
            raise ValueError("JSON body must be between 1 and 16384 bytes")
        try:
            return json.loads(self.rfile.read(content_length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ValueError("request body must be valid JSON") from error

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/v1/health":
            _, state = self.node.backend_state.snapshot()
            self._json(
                200,
                {
                    "ok": True,
                    "apiVersion": API_VERSION,
                    "robotModel": self.node.robot_model,
                    "connected": state["connected"],
                    "commandsEnabled": self.node.enable_commands,
                    "runtimeMode": self.node.runtime_mode,
                    "simulationMode": self.node.simulation_mode,
                    "executeMotion": self.node.execute_motion,
                },
            )
            return
        if not self._require_authorization():
            return
        if path == "/api/v1/state":
            _, state = self.node.backend_state.snapshot()
            self._json(200, {"ok": True, "state": state})
            return
        if path == "/api/v1/events":
            self._serve_events()
            return
        self._error(404, "not_found", "endpoint not found")

    def do_POST(self):
        if not self._require_authorization():
            return
        if not self.node.enable_commands:
            self._error(403, "commands_disabled", "command API is disabled")
            return
        path = urlparse(self.path).path
        try:
            payload = self._read_json()
            if path == "/api/v1/commands/keys":
                keys = sanitize_controller_keys(payload.get("keys"))
                self.node.publish_keys(keys)
            elif path == "/api/v1/commands/pose":
                pose = sanitize_pose_command(payload, self.node.base_frame)
                self.node.publish_pose(pose)
            elif path == "/api/v1/commands/action":
                action = sanitize_action(payload)
                self.node.publish_action(action)
            else:
                self._error(404, "not_found", "endpoint not found")
                return
        except ValueError as error:
            self._error(400, "invalid_request", str(error))
            return
        self._json(
            202,
            {
                "ok": True,
                "accepted": True,
                "requestId": str(uuid.uuid4()),
            },
        )

    def _serve_events(self):
        self.send_response(200)
        self._headers("text/event-stream; charset=utf-8")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        version = -1
        try:
            while not self.node.stop_event.is_set():
                next_version, state = self.node.backend_state.wait_after(
                    version, 10.0
                )
                if next_version == version:
                    self.wfile.write(b": keepalive\n\n")
                else:
                    body = json.dumps(
                        state, ensure_ascii=False, separators=(",", ":")
                    ).encode("utf-8")
                    self.wfile.write(b"event: state\ndata: " + body + b"\n\n")
                    version = next_version
                self.wfile.flush()
                # Coalesce the controller's joint and pose publications into
                # a backend-friendly stream of at most 20 snapshots/second.
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass


class BackendApiNode(Node):
    """Translate backend HTTP requests into the stable arm ROS protocol."""

    def __init__(self):
        super().__init__("backend_api")
        self.declare_parameter("robot_model", "nero")
        self.declare_parameter("api_host", "127.0.0.1")
        self.declare_parameter("api_port", 8765)
        self.declare_parameter("api_token", "")
        self.declare_parameter("enable_commands", False)
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("execute_motion", False)
        self.declare_parameter("base_frame", "base_link")

        self.robot_model = str(
            self.get_parameter("robot_model").value
        ).lower()
        self.model_profile = get_arm_profile(self.robot_model)
        self.api_host = str(self.get_parameter("api_host").value).strip()
        self.api_port = int(self.get_parameter("api_port").value)
        self.api_token = str(self.get_parameter("api_token").value)
        self.enable_commands = bool(
            self.get_parameter("enable_commands").value
        )
        self.simulation_mode = bool(
            self.get_parameter("simulation_mode").value
        )
        self.execute_motion = bool(
            self.get_parameter("execute_motion").value
        )
        self.runtime_mode = (
            "simulation" if self.simulation_mode else "hardware"
        )
        self.base_frame = str(self.get_parameter("base_frame").value)
        if not 1 <= self.api_port <= 65535:
            raise ValueError("api_port must be in [1, 65535]")
        validate_bind_address(self.api_host, self.api_token)

        prefix = self.model_profile.topic_prefix
        self.keyboard_publisher = self.create_publisher(
            Int32MultiArray, "/arm_keyboard_state", 10
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped, f"{prefix}/target_pose", 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._joint_callback, 10
        )
        self.create_subscription(
            PoseStamped, f"{prefix}/current_pose", self._pose_callback, 10
        )
        self.create_subscription(
            String, f"{prefix}/ik_status", self._ik_callback, 10
        )

        self.backend_state = BackendState(
            self.robot_model,
            self.enable_commands,
            simulation_mode=self.simulation_mode,
            execute_motion=self.execute_motion,
        )
        self.stop_event = threading.Event()
        self._action_timers = set()
        self._timer_lock = threading.Lock()
        self.http_server = _BackendHttpServer(
            (self.api_host, self.api_port), BackendRequestHandler
        )
        self.http_server.node = self
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name="armby-backend-api",
            daemon=True,
        )
        self.http_thread.start()
        self.get_logger().info(
            f"backend API listening on http://{self.api_host}:"
            f"{self.api_port}/api/v1"
        )

    def _joint_callback(self, message):
        state = extract_named_arm_joint_state(message, self.robot_model)
        if state is None:
            return
        value = {
            "names": state["names"],
            "positionRad": state["positions"],
            "velocityRadS": state["velocities"],
        }
        self.backend_state.update("jointState", value, joint_feedback=True)

    def _pose_callback(self, message):
        pose = message.pose
        value = {
            "frameId": message.header.frame_id or self.base_frame,
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            },
        }
        self.backend_state.update("currentPose", value)

    def _ik_callback(self, message):
        try:
            value = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            value = {"state": "unknown", "message": str(message.data)}
        self.backend_state.update("ikStatus", value)

    def publish_keys(self, keys):
        message = Int32MultiArray()
        message.data = list(keys)
        self.keyboard_publisher.publish(message)

    def publish_pose(self, pose):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = pose["frameId"]
        message.pose.position.x = pose["position"]["x"]
        message.pose.position.y = pose["position"]["y"]
        message.pose.position.z = pose["position"]["z"]
        message.pose.orientation.x = pose["orientation"]["x"]
        message.pose.orientation.y = pose["orientation"]["y"]
        message.pose.orientation.z = pose["orientation"]["z"]
        message.pose.orientation.w = pose["orientation"]["w"]
        self.pose_publisher.publish(message)

    def publish_action(self, action):
        if action == "release":
            self.publish_keys([0] * KEY_COUNT)
            return
        keys = [0] * KEY_COUNT
        keys[ACTION_KEYS[action]] = 1
        self.publish_keys(keys)
        holder = {}

        def release():
            self.publish_keys([0] * KEY_COUNT)
            with self._timer_lock:
                self._action_timers.discard(holder["timer"])

        timer = threading.Timer(0.05, release)
        holder["timer"] = timer
        with self._timer_lock:
            self._action_timers.add(timer)
        timer.start()

    def destroy_node(self):
        self.stop_event.set()
        self.http_server.shutdown()
        self.http_server.server_close()
        if self.http_thread.is_alive():
            self.http_thread.join(timeout=2.0)
        with self._timer_lock:
            timers = list(self._action_timers)
            self._action_timers.clear()
        for timer in timers:
            timer.cancel()
        if self.enable_commands:
            try:
                self.publish_keys([0] * KEY_COUNT)
            except Exception:
                # The ROS context can already be invalid during global
                # shutdown, when no subscriber can receive the release.
                pass
        return super().destroy_node()


def main(args=None):
    """Run the backend API gateway."""
    rclpy.init(args=args)
    node = BackendApiNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
