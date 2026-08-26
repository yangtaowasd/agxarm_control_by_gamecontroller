"""ROS publishers and stable schemas for controller telemetry."""

import json
import time

import numpy as np
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from armbycontroller.api import interaction_state_payload
from armbycontroller.control import control_sample


class RosTelemetry:
    """Own all controller publishers and their serialized message schemas."""

    def __init__(
        self,
        node,
        *,
        dynamics_state_topic,
        control_sample_topic,
        control_event_topic,
        interaction_state_topic,
    ):
        self._node = node
        self.dynamics_state_publisher = node.create_publisher(
            JointState, dynamics_state_topic, 20
        )
        self.control_sample_publisher = node.create_publisher(
            String, control_sample_topic, 100
        )
        self.control_event_publisher = node.create_publisher(
            String, control_event_topic, 20
        )
        interaction_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.interaction_state_publisher = node.create_publisher(
            String, interaction_state_topic, interaction_qos
        )

    @classmethod
    def from_controller_publishers(cls, node):
        """Adapt legacy publisher attributes used by isolated node tests."""
        telemetry = object.__new__(cls)
        telemetry._node = node
        for name in (
            "dynamics_state_publisher",
            "control_sample_publisher",
            "control_event_publisher",
            "interaction_state_publisher",
        ):
            setattr(telemetry, name, getattr(node, name, None))
        return telemetry

    @staticmethod
    def _json_message(payload):
        message = String()
        message.data = json.dumps(
            payload,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        return message

    @staticmethod
    def _resolved_service_name(node, attribute):
        name = str(getattr(node, attribute, ""))
        if not name:
            return ""
        resolver = getattr(node, "resolve_service_name", None)
        return str(resolver(name)) if callable(resolver) else name

    def publish_control_result(self, sample, result, interaction_mode):
        """Publish one stable control-sample JSON message."""
        if self.control_sample_publisher is None:
            return
        payload = control_sample(
            sample,
            result,
            robot_model=getattr(self._node, "robot_model", "unknown"),
            interaction_mode=interaction_mode,
        )
        self.control_sample_publisher.publish(self._json_message(payload))

    def publish_control_event(self, event, **fields):
        """Publish one timestamped discrete controller event."""
        if self.control_event_publisher is None:
            return
        payload = {
            "timestamp": time.monotonic(),
            "robot_model": getattr(self._node, "robot_model", "unknown"),
            "event": str(event),
            **fields,
        }
        self.control_event_publisher.publish(self._json_message(payload))

    def publish_interaction_state(self, reason):
        """Publish one latched UI-facing interaction-state snapshot."""
        if self.interaction_state_publisher is None:
            return
        node = self._node
        payload = interaction_state_payload(
            node._current_interaction_mode(),
            timestamp=time.monotonic(),
            robot_model=getattr(node, "robot_model", "unknown"),
            control_mode=getattr(node, "control_mode", "joint"),
            impedance_backend=getattr(node, "impedance_backend", "unknown"),
            admittance_mode=getattr(node, "admittance_mode", "unknown"),
            arm_ready=bool(getattr(node, "arm_ready", False)),
            arm_connected=bool(getattr(node, "arm_connected", False)),
            emergency_stopped=bool(
                getattr(node, "emergency_stopped", False)
            ),
            interaction_transitioning=bool(
                getattr(node, "interaction_transitioning", False)
            ),
            interaction_transition_target=str(
                getattr(node, "interaction_transition_target", "")
            ),
            interaction_fault_reason=str(
                getattr(node, "interaction_fault_reason", "")
            ),
            execute_motion=bool(getattr(node, "execute_motion", False)),
            mode_services={
                "normal": self._resolved_service_name(
                    node, "normal_mode_service_name"
                ),
                "impedance": self._resolved_service_name(
                    node, "impedance_mode_service_name"
                ),
                "admittance": self._resolved_service_name(
                    node, "admittance_mode_service_name"
                ),
            },
            reason=str(reason),
        )
        self.interaction_state_publisher.publish(self._json_message(payload))

    def publish_dynamics_state(self, feedback):
        """Publish measured q/dq/motor torque from one shared sample."""
        if self.dynamics_state_publisher is None:
            return
        node = self._node
        position = np.asarray(feedback.position, dtype=float)
        velocity = np.asarray(feedback.velocity, dtype=float)
        torque = np.asarray(feedback.torque, dtype=float)
        joint_count = int(node.joint_count)
        if any(
            values.shape != (joint_count,) or not np.all(np.isfinite(values))
            for values in (position, velocity, torque)
        ):
            node.get_logger().warning(
                "dynamics state is incomplete; sample not published",
                throttle_duration_sec=1.0,
            )
            return
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        model = getattr(node, "gravity_model", None)
        message.name = (
            list(model.movable_joint_names)
            if model is not None
            else [f"joint{index}" for index in range(1, joint_count + 1)]
        )
        message.position = position.tolist()
        message.velocity = velocity.tolist()
        message.effort = torque.tolist()
        self.dynamics_state_publisher.publish(message)
