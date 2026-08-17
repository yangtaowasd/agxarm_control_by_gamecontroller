#!/usr/bin/env python3
"""Solve AGX arm poses with PoE screw IK and send limited joint goals."""

from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from armbycontroller.hardware import connect_arm_two_stage
from armbycontroller.ik.core import AgxIkEngine
from armbycontroller.ik.core import create_screw_solver
from armbycontroller.ik.core import IkFailure
from armbycontroller.ik.core import prepare_planned_joint_mode
from armbycontroller.ik.core import quaternion_to_rotation_matrix
from armbycontroller.ik.core import resolve_firmware_name
from armbycontroller.ik.core import resolve_urdf_path
from armbycontroller.ik.core import rotation_matrix_to_quaternion
from armbycontroller.ik.core import set_joint_acceleration_limits
from armbycontroller.model_profiles import get_arm_profile


class PoseController(Node):
    """Use screw-theory IK/FK for Nero or Piper-L joint-space control."""

    def __init__(self):
        super().__init__("pose_controller")

        self.declare_parameter("robot_model", "nero")
        declared_model = str(
            self.get_parameter("robot_model").value
        ).lower()
        default_profile = get_arm_profile(declared_model)
        self.declare_parameter("topic_prefix", default_profile.topic_prefix)
        self.declare_parameter("target_pose_topic", "")
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("firmware", "auto")
        self.declare_parameter("firmware_probe_timeout", 5.0)
        self.declare_parameter("firmware_probe_poll_period", 0.1)
        self.declare_parameter("firmware_reconnect_delay", 0.5)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tip_link", default_profile.tip_link)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("simulation_mode", False)
        self.declare_parameter(
            "initial_joint_positions",
            list(default_profile.initial_joint_positions),
        )
        self.declare_parameter("state_period", 0.05)
        self.declare_parameter("execute_motion", False)
        self.declare_parameter("auto_enable", True)
        self.declare_parameter("disable_on_shutdown", True)
        self.declare_parameter("speed_percent", 20)
        self.declare_parameter("enable_timeout", 5.0)
        self.declare_parameter("command_period", 0.1)
        self.declare_parameter("joint_max_acceleration", 1.0)
        self.declare_parameter("joint_acc_timeout", 2.0)
        self.declare_parameter("position_mode_timeout", 2.0)
        self.declare_parameter("joint_max_velocity", 1.0)
        self.declare_parameter("ik_timeout", 0.01)
        self.declare_parameter("ik_tolerance", 1e-5)
        self.declare_parameter("fk_position_tolerance", 1e-4)
        self.declare_parameter("fk_rotation_tolerance", 1e-3)
        self.declare_parameter("pointing_axis_only", True)
        self.declare_parameter("pointing_roll_samples", 8)
        self.declare_parameter("valid_history_size", 10)
        self.declare_parameter("recovery_pause", 2.0)
        self.declare_parameter("workspace_limit_enabled", True)
        self.declare_parameter("robot_min_reach", default_profile.min_reach)
        self.declare_parameter("robot_max_reach", default_profile.max_reach)
        self.declare_parameter("workspace_inner_margin", 0.05)
        self.declare_parameter("workspace_outer_margin", 0.10)

        self.robot_model = str(
            self.get_parameter("robot_model").value
        ).lower()
        self.model_profile = get_arm_profile(self.robot_model)
        self.joint_count = self.model_profile.joint_count
        prefix = str(self.get_parameter("topic_prefix").value).strip("/")
        self.topic_prefix = f"/{prefix}" if prefix else ""
        configured_target_topic = str(
            self.get_parameter("target_pose_topic").value
        ).strip()
        self.target_pose_topic = (
            configured_target_topic
            if configured_target_topic
            else f"{self.topic_prefix}/target_pose"
        )
        self.can_interface = str(self.get_parameter("can_interface").value)
        self.requested_firmware = str(
            self.get_parameter("firmware").value
        ).lower()
        self.firmware = resolve_firmware_name(
            self.robot_model, self.requested_firmware
        )
        self.firmware_probe_timeout = float(
            self.get_parameter("firmware_probe_timeout").value
        )
        self.firmware_probe_poll_period = float(
            self.get_parameter("firmware_probe_poll_period").value
        )
        self.firmware_reconnect_delay = float(
            self.get_parameter("firmware_reconnect_delay").value
        )
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tip_link = str(self.get_parameter("tip_link").value)
        self.urdf_path = resolve_urdf_path(
            str(self.get_parameter("urdf_path").value), self.robot_model
        )
        self.simulation_mode = bool(
            self.get_parameter("simulation_mode").value
        )
        self.simulated_joints = np.asarray(
            self.get_parameter("initial_joint_positions").value,
            dtype=float,
        )
        self.state_period = float(self.get_parameter("state_period").value)
        self.execute_motion = bool(self.get_parameter("execute_motion").value)
        self.auto_enable = bool(self.get_parameter("auto_enable").value)
        self.disable_on_shutdown = bool(
            self.get_parameter("disable_on_shutdown").value
        )
        self.speed_percent = int(self.get_parameter("speed_percent").value)
        self.enable_timeout = float(self.get_parameter("enable_timeout").value)
        self.command_period = float(self.get_parameter("command_period").value)
        self.joint_max_acceleration = float(
            self.get_parameter("joint_max_acceleration").value
        )
        self.joint_acc_timeout = float(
            self.get_parameter("joint_acc_timeout").value
        )
        self.position_mode_timeout = float(
            self.get_parameter("position_mode_timeout").value
        )
        self.joint_max_velocity = float(
            self.get_parameter("joint_max_velocity").value
        )
        self.ik_timeout = float(self.get_parameter("ik_timeout").value)
        self.ik_tolerance = float(self.get_parameter("ik_tolerance").value)
        self.fk_position_tolerance = float(
            self.get_parameter("fk_position_tolerance").value
        )
        self.fk_rotation_tolerance = float(
            self.get_parameter("fk_rotation_tolerance").value
        )
        self.pointing_axis_only = bool(
            self.get_parameter("pointing_axis_only").value
        )
        self.pointing_roll_samples = int(
            self.get_parameter("pointing_roll_samples").value
        )
        self.valid_history_size = int(
            self.get_parameter("valid_history_size").value
        )
        self.recovery_pause = float(
            self.get_parameter("recovery_pause").value
        )
        self.workspace_limit_enabled = bool(
            self.get_parameter("workspace_limit_enabled").value
        )
        self.robot_min_reach = float(
            self.get_parameter("robot_min_reach").value
        )
        self.robot_max_reach = float(
            self.get_parameter("robot_max_reach").value
        )
        self.workspace_inner_margin = float(
            self.get_parameter("workspace_inner_margin").value
        )
        self.workspace_outer_margin = float(
            self.get_parameter("workspace_outer_margin").value
        )
        self.workspace_min_radius = (
            self.robot_min_reach + self.workspace_inner_margin
        )
        self.workspace_max_radius = (
            self.robot_max_reach - self.workspace_outer_margin
        )
        self._validate_parameters()

        self.robot = None
        self.device_firmware_info = {}
        self.ik_solver = None
        self.ik_engine = None
        self.simulated_target_joints = self.simulated_joints.copy()
        self.simulated_velocities = np.zeros(self.joint_count, dtype=float)
        self.valid_history = deque(maxlen=self.valid_history_size)
        self.history_initialized = False
        self.recovery_until = -math.inf
        self.enabled_by_node = False
        self.last_command_time = -math.inf
        try:
            self._create_solver()
            if not self.simulation_mode:
                self._connect_robot()
        except Exception:
            self.close()
            raise

        self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self.target_pose_callback,
            10,
        )
        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self.current_pose_pub = self.create_publisher(
            PoseStamped, f"{self.topic_prefix}/current_pose", 10
        )
        self.ik_status_pub = self.create_publisher(
            String, f"{self.topic_prefix}/ik_status", 10
        )
        self.create_timer(self.state_period, self.publish_state)
        if self.simulation_mode:
            self.get_logger().info(
                "simulation mode: CAN disabled; publishing RViz joint states"
            )
        elif self.execute_motion:
            self.get_logger().info(
                "screw IK/FK ready; joint acceleration limits verified"
            )
        else:
            self.get_logger().warning(
                "execute_motion=false: IK/FK runs, but move_j is not sent"
            )
        self.get_logger().info(
            f"pose command topic: {self.target_pose_topic}"
        )
        if self.workspace_limit_enabled:
            self.get_logger().info(
                "safe radial workspace: %.4f to %.4f m from base_link"
                % (self.workspace_min_radius, self.workspace_max_radius)
            )

    def _validate_parameters(self):
        if not self.urdf_path.is_file():
            raise ValueError(f"URDF does not exist: {self.urdf_path}")
        if not 1 <= self.speed_percent <= 100:
            raise ValueError("speed_percent must be in [1, 100]")
        positive = {
            "enable_timeout": self.enable_timeout,
            "firmware_probe_timeout": self.firmware_probe_timeout,
            "command_period": self.command_period,
            "joint_max_acceleration": self.joint_max_acceleration,
            "joint_max_velocity": self.joint_max_velocity,
            "ik_timeout": self.ik_timeout,
            "ik_tolerance": self.ik_tolerance,
            "fk_position_tolerance": self.fk_position_tolerance,
            "fk_rotation_tolerance": self.fk_rotation_tolerance,
            "state_period": self.state_period,
            "recovery_pause": self.recovery_pause,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{name} must be finite and greater than zero"
                )
        if (
            not math.isfinite(self.firmware_probe_poll_period)
            or self.firmware_probe_poll_period < 0.0
        ):
            raise ValueError(
                "firmware_probe_poll_period must be finite and nonnegative"
            )
        if (
            not math.isfinite(self.firmware_reconnect_delay)
            or self.firmware_reconnect_delay < 0.0
        ):
            raise ValueError(
                "firmware_reconnect_delay must be finite and nonnegative"
            )
        if (
            self.simulated_joints.shape != (self.joint_count,)
            or not np.all(np.isfinite(self.simulated_joints))
        ):
            raise ValueError(
                "initial_joint_positions must match the robot joint count"
            )
        if not 2 <= self.valid_history_size <= 100:
            raise ValueError("valid_history_size must be in [2, 100]")
        if not 1 <= self.pointing_roll_samples <= 72:
            raise ValueError("pointing_roll_samples must be in [1, 72]")
        workspace_values = (
            self.robot_min_reach,
            self.robot_max_reach,
            self.workspace_inner_margin,
            self.workspace_outer_margin,
        )
        if not all(math.isfinite(value) for value in workspace_values):
            raise ValueError("workspace reach values must be finite")
        if (
            self.robot_min_reach < 0.0
            or self.robot_max_reach <= self.robot_min_reach
            or self.workspace_inner_margin < 0.0
            or self.workspace_outer_margin < 0.0
            or self.workspace_min_radius >= self.workspace_max_radius
        ):
            raise ValueError("workspace reach values and margins are invalid")

    def _create_solver(self):
        self.ik_solver = create_screw_solver(
            self.urdf_path,
            self.base_frame,
            self.tip_link,
            self.joint_count,
            self.ik_timeout,
            self.ik_tolerance,
        )
        workspace_min = self.workspace_min_radius
        workspace_max = self.workspace_max_radius
        if not self.workspace_limit_enabled:
            workspace_min, workspace_max = 0.0, math.inf
        self.ik_engine = AgxIkEngine(
            self.ik_solver,
            self.joint_count,
            workspace_min,
            workspace_max,
            self.fk_position_tolerance,
            self.fk_rotation_tolerance,
            self.pointing_roll_samples,
            self.pointing_axis_only,
        )

    def _connect_robot(self):
        from pyAgxArm import ArmModel, NeroFW, PiperFW

        if self.robot_model == "nero":
            arm_model = ArmModel.NERO
            firmware_map = {
                "default": NeroFW.DEFAULT,
                "v111": NeroFW.V111,
                "v112": NeroFW.V112,
                "v120": NeroFW.V120,
            }
        else:
            arm_model = ArmModel.PIPER_L
            firmware_map = {
                "default": PiperFW.DEFAULT,
                "v183": PiperFW.V183,
                "v188": PiperFW.V188,
                "v189": PiperFW.V189,
            }
        if self.firmware not in firmware_map:
            raise ValueError(
                f"unsupported {self.robot_model} firmware: {self.firmware}"
            )
        connection = connect_arm_two_stage(
            robot_model=self.robot_model,
            arm_model=arm_model,
            firmware_profiles=firmware_map,
            can_interface=self.can_interface,
            probe_timeout=self.firmware_probe_timeout,
            probe_poll_period=self.firmware_probe_poll_period,
            reconnect_delay=self.firmware_reconnect_delay,
            report=self.get_logger().info,
        )
        self.robot = connection.arm
        self.device_firmware_info = connection.firmware_info
        detected_firmware = connection.firmware_profile
        if (
            self.requested_firmware != "auto"
            and detected_firmware != self.firmware
        ):
            self.get_logger().warning(
                f"configured firmware {self.firmware} differs from detected "
                f"{detected_firmware}; using detected profile"
            )
        self.firmware = detected_firmware
        self.get_logger().info(
            f"connected to {self.robot_model} on {self.can_interface} "
            f"with detected profile {self.firmware}; saved device data: "
            f"{self.device_firmware_info}"
        )
        if not self.execute_motion:
            return

        if self.auto_enable:
            self._enable_robot()
        limits_ok, failed_joint = set_joint_acceleration_limits(
            self.robot,
            self.joint_count,
            self.joint_max_acceleration,
            self.joint_acc_timeout,
        )
        if not limits_ok:
            raise RuntimeError(
                "failed to set/read back acceleration limit for joint "
                f"{failed_joint}"
            )
        self.robot.set_speed_percent(self.speed_percent)
        if not prepare_planned_joint_mode(
            self.robot, self.position_mode_timeout
        ):
            raise RuntimeError("CAN_CTRL/MOVE_J mode timed out")

    def _enable_robot(self):
        deadline = time.monotonic() + self.enable_timeout
        while not self.robot.enable():
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"timed out while enabling {self.robot_model}"
                )
            time.sleep(0.05)
        self.enabled_by_node = True

    def _current_joint_angles(self):
        if self.simulation_mode:
            return self.simulated_joints.copy()
        feedback = self.robot.get_joint_angles()
        if feedback is None:
            return None
        joints = np.asarray(feedback.msg, dtype=float)
        if (
            joints.shape != (self.joint_count,)
            or not np.all(np.isfinite(joints))
        ):
            return None
        return joints

    def _advance_simulation(self):
        """Advance simulated joints with velocity and acceleration limits."""
        error = self.simulated_target_joints - self.simulated_joints
        acceleration = self.joint_max_acceleration
        braking_velocity = np.sqrt(2.0 * acceleration * np.abs(error))
        desired_velocity = np.sign(error) * np.minimum(
            self.joint_max_velocity, braking_velocity
        )
        max_velocity_change = acceleration * self.state_period
        velocity_change = np.clip(
            desired_velocity - self.simulated_velocities,
            -max_velocity_change,
            max_velocity_change,
        )
        self.simulated_velocities += velocity_change
        step = self.simulated_velocities * self.state_period
        arrived = (
            np.abs(error) < 1e-9
        ) | (
            (error * step > 0.0) & (np.abs(step) >= np.abs(error))
        )
        self.simulated_joints += step
        self.simulated_joints[arrived] = self.simulated_target_joints[arrived]
        self.simulated_velocities[arrived] = 0.0

    def publish_state(self):
        """Publish joint and FK pose feedback for RViz and keyboard control."""
        if self.simulation_mode:
            self._advance_simulation()
        joints = self._current_joint_angles()
        if joints is None:
            return
        stamp = self.get_clock().now().to_msg()
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = [
            f"joint{index}" for index in range(1, self.joint_count + 1)
        ]
        joint_state.position = joints.tolist()
        self.joint_state_pub.publish(joint_state)

        position, rotation = self.ik_solver.fk(joints)
        if not self.history_initialized:
            self._remember_valid(position, rotation, joints)
            self.history_initialized = True
        quaternion = rotation_matrix_to_quaternion(rotation)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        self.current_pose_pub.publish(pose)

    def _remember_valid(self, position, rotation, joints):
        """Store one independently copied IK/FK-verified state."""
        self.valid_history.append(
            (
                np.asarray(position, dtype=float).copy(),
                np.asarray(rotation, dtype=float).copy(),
                np.asarray(joints, dtype=float).copy(),
            )
        )

    def _publish_ik_status(self, state, message, position=None):
        payload = {
            "state": state,
            "message": message,
            "pause_seconds": (
                self.recovery_pause if state == "recovering" else 0.0
            ),
        }
        if position is not None:
            payload["position"] = np.asarray(position, dtype=float).tolist()
        status = String()
        status.data = json.dumps(payload, ensure_ascii=False)
        self.ik_status_pub.publish(status)

    def _recover_from_ik_failure(self, reason):
        """Return to the latest valid state and pause input briefly."""
        self.recovery_until = time.monotonic() + self.recovery_pause
        if not self.valid_history:
            message = f"IK stuck: {reason}; no valid history is available"
            self.get_logger().error(message)
            self._publish_ik_status("recovering", message)
            return

        position, _, joints = self.valid_history[-1]
        if self.simulation_mode:
            self.simulated_target_joints = joints.copy()
        elif self.execute_motion:
            try:
                self.robot.move_j(joints.tolist())
            except (TypeError, ValueError, RuntimeError) as error:
                self.get_logger().error(f"recovery move_j failed: {error}")
        message = (
            f"IK stuck: {reason}; returning to last valid position "
            f"{position.tolist()}, pausing {self.recovery_pause:.1f} s"
        )
        self.get_logger().warning(message)
        self._publish_ik_status("recovering", message, position)

    def target_pose_callback(self, message):
        """Solve IK, verify the solution with FK, then send a joint goal."""
        remaining = self.recovery_until - time.monotonic()
        if remaining > 0.0:
            self.get_logger().warning(
                f"IK recovery pause active: {remaining:.1f} s remaining"
            )
            return
        if (
            message.header.frame_id
            and message.header.frame_id != self.base_frame
        ):
            self.get_logger().error(
                f"target frame '{message.header.frame_id}' is not "
                f"'{self.base_frame}'"
            )
            return
        target_position = np.asarray(
            [message.pose.position.x, message.pose.position.y,
             message.pose.position.z],
            dtype=float,
        )
        if not np.all(np.isfinite(target_position)):
            self.get_logger().error(
                "target position contains a non-finite value"
            )
            return
        try:
            target_rotation = quaternion_to_rotation_matrix(
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            )
        except ValueError as error:
            self.get_logger().error(f"invalid target orientation: {error}")
            return

        if self.simulation_mode:
            seed = self.simulated_target_joints.copy()
        else:
            seed = self._current_joint_angles()
        if seed is None:
            self.get_logger().error(
                f"no valid {self.robot_model} joint feedback; IK not run"
            )
            return
        try:
            result = self.ik_engine.solve(
                target_position, target_rotation, seed
            )
        except IkFailure as error:
            self._recover_from_ik_failure(str(error))
            return
        solution = result.joints
        position_error = result.position_error
        rotation_error = result.orientation_error

        now = time.monotonic()
        if now - self.last_command_time < self.command_period:
            self.get_logger().warning(
                "target ignored: command rate is too high"
            )
            return
        self.last_command_time = now
        joints = solution.tolist()
        self._remember_valid(target_position, target_rotation, solution)
        self._publish_ik_status(
            "ok",
            f"valid target stored ({len(self.valid_history)}/"
            f"{self.valid_history.maxlen})",
            target_position,
        )
        if self.simulation_mode:
            self.simulated_target_joints = solution
            self.get_logger().info(
                "simulation target accepted; FK errors: %.6g m, %.6g rad"
                % (position_error, rotation_error)
            )
            return
        if not self.execute_motion:
            self.get_logger().info(f"dry-run joint target: {joints}")
            return
        try:
            # The arm's joint planner applies limits verified at startup.
            self.robot.move_j(joints)
            self.get_logger().info(
                "move_j sent; FK errors: %.6g m, %.6g rad"
                % (position_error, rotation_error)
            )
        except (TypeError, ValueError, RuntimeError) as error:
            self.get_logger().error(f"joint target rejected: {error}")

    def close(self):
        """Release the arm connection owned by this node."""
        if self.robot is None:
            return
        if self.enabled_by_node and self.disable_on_shutdown:
            try:
                self.robot.disable()
            except Exception as error:
                self.get_logger().error(
                    f"failed to disable {self.robot_model}: {error}"
                )
        try:
            self.robot.disconnect()
        except Exception as error:
            self.get_logger().error(
                f"failed to disconnect {self.robot_model}: {error}"
            )
        finally:
            self.robot = None

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None):
    """Run the shared AGX pose controller node."""
    rclpy.init(args=args)
    node = None
    try:
        node = PoseController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
