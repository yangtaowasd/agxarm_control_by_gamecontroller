#!/usr/bin/env python3
"""ROS 2 adapter for passive generalized-momentum torque observation."""

import math

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from armbycontroller.ik.core import resolve_urdf_path
from armbycontroller.ik.core import resolve_tool_urdf_path
from armbycontroller.modeling.momentum_observer import (
    GeneralizedMomentumObserver,
)
from armbycontroller.modeling.screw_model import project_gravity_vector
from armbycontroller.modeling.screw_model import UrdfScrewModel


MODEL_PROFILES = {
    "piper_l": (6, "link6"),
    "nero": (7, "link7"),
}


class MomentumObserverNode(Node):
    """Estimate external torque from shared motor state in another process."""

    def __init__(self):
        super().__init__("arm_momentum_observer")
        scalar_or_array = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("robot_model", "nero")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tip_link", "")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("gravity_urdf_path", "")
        self.declare_parameter("nero_mount", "")
        self.declare_parameter("tool_configuration", "auto")
        self.declare_parameter("gravity_vector", [0.0, 0.0, -9.80665])
        self.declare_parameter(
            "dynamics_state_topic", "/arm_dynamics_state"
        )
        self.declare_parameter(
            "external_torque_topic", "/arm_external_joint_torque"
        )
        self.declare_parameter("momentum_observer_rate", 100.0)
        self.declare_parameter(
            "momentum_observer_gain", [10.0], scalar_or_array
        )
        self.declare_parameter("momentum_observer_max_period", 0.05)

        self.robot_model = str(
            self.get_parameter("robot_model").value
        ).lower()
        if self.robot_model not in MODEL_PROFILES:
            raise ValueError("robot_model must be nero or piper_l")
        self.joint_count, default_tip = MODEL_PROFILES[
            self.robot_model
        ]
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tip_link = (
            str(self.get_parameter("tip_link").value) or default_tip
        )
        self.rate = float(
            self.get_parameter("momentum_observer_rate").value
        )
        self.maximum_period = float(
            self.get_parameter("momentum_observer_max_period").value
        )
        if (
            not math.isfinite(self.rate)
            or self.rate <= 0.0
            or not math.isfinite(self.maximum_period)
            or self.maximum_period < 1.0 / self.rate
        ):
            raise ValueError(
                "observer rate must be positive and maximum period must cover "
                "at least one observer sample"
            )
        gain = np.asarray(
            self.get_parameter("momentum_observer_gain").value, dtype=float
        )
        if gain.ndim == 0 or gain.size == 1:
            gain = np.full(self.joint_count, float(gain.reshape(-1)[0]))
        nero_mount = str(
            self.get_parameter("nero_mount").value
        ).strip().lower()
        gravity = np.asarray(
            project_gravity_vector(nero_mount)
            if self.robot_model == "nero" and nero_mount
            else self.get_parameter("gravity_vector").value,
            dtype=float,
        )
        bare_urdf = resolve_urdf_path(
            str(self.get_parameter("urdf_path").value), self.robot_model
        )
        configured = str(
            self.get_parameter("gravity_urdf_path").value
        )
        model_path = resolve_tool_urdf_path(
            bare_urdf,
            self.robot_model,
            str(self.get_parameter("tool_configuration").value),
            configured,
        )
        self.model = UrdfScrewModel(
            model_path,
            self.base_frame,
            self.tip_link,
            self.joint_count,
            gravity,
        )
        self.observer = GeneralizedMomentumObserver(
            self.joint_count, gain, self.maximum_period
        )
        self.previous_sample_time = None
        self.previous_command = None
        self.previous_beta = None
        input_topic = str(
            self.get_parameter("dynamics_state_topic").value
        )
        output_topic = str(
            self.get_parameter("external_torque_topic").value
        )
        self.publisher = self.create_publisher(
            JointState, output_topic, 20
        )
        self.subscription = self.create_subscription(
            JointState,
            input_topic,
            self._state_callback,
            20,
        )
        self.get_logger().info(
            "passive momentum observer ready: model=%s rate=%.1f Hz "
            "gain=%s 1/s input=%s output=%s"
            % (
                model_path,
                self.rate,
                np.round(self.observer.gain, 3).tolist(),
                input_topic,
                output_topic,
            )
        )

    def _sample(self, message):
        arrays = tuple(
            np.asarray(values, dtype=float)
            for values in (
                message.position, message.velocity, message.effort
            )
        )
        if any(
            values.shape != (self.joint_count,)
            or not np.all(np.isfinite(values))
            for values in arrays
        ):
            raise ValueError(
                f"dynamics state must contain {self.joint_count} finite "
                "positions, velocities, and efforts"
            )
        return arrays

    @staticmethod
    def _sample_time(message):
        stamp = message.header.stamp
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    def _reset_stream(
        self, momentum, beta, command, sample_time
    ):
        self.observer.reset(momentum)
        self.previous_command = command.copy()
        self.previous_beta = beta.copy()
        self.previous_sample_time = sample_time

    def _state_callback(self, message):
        try:
            position, velocity, command = self._sample(message)
            sample_time = self._sample_time(message)
            if not math.isfinite(sample_time) or sample_time <= 0.0:
                raise ValueError("dynamics state timestamp must be positive")
            momentum, beta = self.model.momentum_observer_terms(
                position, velocity
            )
            if self.previous_sample_time is None:
                self._reset_stream(
                    momentum, beta, command, sample_time
                )
                return
            period = sample_time - self.previous_sample_time
            if period <= 0.0 or period > self.maximum_period:
                self._reset_stream(
                    momentum, beta, command, sample_time
                )
                self.get_logger().warning(
                    "momentum observer stream gap; state reset",
                    throttle_duration_sec=1.0,
                )
                return
            observation = self.observer.update(
                momentum,
                self.previous_command,
                self.previous_beta,
                period,
            )
            self.previous_command = command.copy()
            self.previous_beta = beta.copy()
            self.previous_sample_time = sample_time

            output = JointState()
            output.header.stamp = message.header.stamp
            output.name = list(self.model.movable_joint_names)
            output.position = position.tolist()
            output.velocity = velocity.tolist()
            output.effort = observation.external_torque.tolist()
            self.publisher.publish(output)
            self.get_logger().info(
                "estimated external joint torque=%s N·m"
                % np.round(observation.external_torque, 3).tolist(),
                throttle_duration_sec=1.0,
            )
        except Exception as error:
            self.get_logger().warning(
                f"momentum observer sample rejected: {error}",
                throttle_duration_sec=1.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = MomentumObserverNode()
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
