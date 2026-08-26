#!/usr/bin/env python3
"""ROS 2 adapter that records standardized control samples as experiments."""

import json
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from armbycontroller.experiment import ExperimentRun
from armbycontroller.experiment import JsonlExperimentSink


class ExperimentRecorderNode(Node):
    """Manage one reproducible experiment run at a time."""

    def __init__(self):
        super().__init__("arm_experiment_recorder")
        self.declare_parameter("sample_topic", "arm_control_sample")
        self.declare_parameter("event_topic", "arm_control_event")
        self.declare_parameter(
            "output_directory",
            "~/.ros/agxarm_control_by_gamecontroller/experiments",
        )
        self.declare_parameter("experiment_name", "manual_control")
        self.declare_parameter("robot_model", "unknown")
        self.declare_parameter("start_on_launch", False)
        self.declare_parameter("flush_every", 1)

        self.sample_topic = str(self.get_parameter("sample_topic").value)
        self.event_topic = str(self.get_parameter("event_topic").value)
        self.output_directory = Path(
            str(self.get_parameter("output_directory").value)
        ).expanduser()
        self.experiment_name = str(
            self.get_parameter("experiment_name").value
        ).strip()
        self.robot_model = str(self.get_parameter("robot_model").value)
        self.flush_every = int(self.get_parameter("flush_every").value)
        if not self.experiment_name:
            raise ValueError("experiment_name must not be empty")
        if self.flush_every < 1:
            raise ValueError("flush_every must be at least one")

        self.run = None
        self.status_publisher = self.create_publisher(
            String, "~/status", 10
        )
        self.create_subscription(String, self.sample_topic, self._sample, 100)
        self.create_subscription(String, self.event_topic, self._event, 20)
        self.create_service(SetBool, "~/recording", self._recording)
        if bool(self.get_parameter("start_on_launch").value):
            self.start_recording()
        else:
            self._publish_status("idle")

    def _publish_status(self, status, **fields):
        message = String()
        message.data = json.dumps(
            {
                "status": status,
                "run_id": self.run.run_id if self.run else None,
                "run_directory": (
                    str(self.run.sink.run_directory) if self.run else None
                ),
                **fields,
            },
            sort_keys=True,
        )
        self.status_publisher.publish(message)

    def start_recording(self):
        if self.run is not None and self.run.active:
            raise RuntimeError("an experiment run is already active")
        run_id = ExperimentRun.new_run_id(self.experiment_name)
        sink = JsonlExperimentSink(
            self.output_directory, run_id, self.flush_every
        )
        run = ExperimentRun(
            self.experiment_name,
            sink,
            metadata={
                "robot_model": self.robot_model,
                "sample_topic": self.sample_topic,
                "event_topic": self.event_topic,
            },
            run_id=run_id,
        )
        run.start()
        self.run = run
        self._publish_status("recording")
        self.get_logger().info(
            f"experiment recording started: {self.run.sink.run_directory}"
        )

    def stop_recording(self, outcome="completed", *, publish_status=True):
        if self.run is None or not self.run.active:
            raise RuntimeError("no experiment run is active")
        summary = self.run.close(outcome=outcome)
        directory = str(self.run.sink.run_directory)
        if publish_status and rclpy.ok():
            self._publish_status("closed", summary=summary)
        if rclpy.ok():
            self.get_logger().info(
                f"experiment recording closed: {directory}"
            )
        return directory

    def _recording(self, request, response):
        try:
            if request.data:
                self.start_recording()
                response.message = str(self.run.sink.run_directory)
            else:
                response.message = self.stop_recording()
            response.success = True
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def _decode(self, message, kind):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError("payload is not an object")
            return value
        except (json.JSONDecodeError, ValueError) as error:
            self.get_logger().warning(f"rejected {kind}: {error}")
            return None

    def _sample(self, message):
        if self.run is None or not self.run.active:
            return
        value = self._decode(message, "control sample")
        if value is not None:
            try:
                self.run.record_sample(value)
            except ValueError as error:
                self.get_logger().warning(f"rejected control sample: {error}")

    def _event(self, message):
        if self.run is None or not self.run.active:
            return
        value = self._decode(message, "control event")
        if value is not None:
            event = value.pop("event", "control_event")
            try:
                self.run.record_event(event, **value)
            except ValueError as error:
                self.get_logger().warning(f"rejected control event: {error}")

    def destroy_node(self):
        if self.run is not None and self.run.active:
            try:
                self.stop_recording(
                    outcome="node_shutdown", publish_status=False
                )
            except Exception as error:
                if rclpy.ok():
                    self.get_logger().error(
                        f"failed to close experiment recording: {error}"
                    )
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ExperimentRecorderNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
