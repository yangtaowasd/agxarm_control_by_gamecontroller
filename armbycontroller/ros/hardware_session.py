"""AGX hardware connection, feedback, command, and shutdown lifecycle."""

import math
import time

import numpy as np

from pyAgxArm import AgxArmFactory
from pyAgxArm import create_agx_arm_config

from armbycontroller.cartesian import geometric_jacobian
from armbycontroller.cartesian import wrench_from_joint_torque
from armbycontroller.hardware import connect_arm_two_stage
from armbycontroller.hardware import estimate_joint_velocity
from armbycontroller.hardware import extract_joint_angles
from armbycontroller.hardware import MotorFeedback
from armbycontroller.ik.core import prepare_planned_joint_mode
from armbycontroller.ik.core import set_joint_acceleration_limits


class HardwareSessionMixin:
    """Own two-stage connection, measured feedback, and SDK commands."""

    def connect_arm(self):
        self.device_firmware_info = {}
        enabled = False

        if not self.execute_motion:
            config = create_agx_arm_config(
                robot=self.profile["arm_model"],
                firmeware_version=self.firmware,
                interface="socketcan",
                channel=self.can_interface,
            )
            self.arm = AgxArmFactory.create_arm(config)
            self.arm.set_joint_limits_enabled(True)
            self.get_logger().warning(
                f"dry-run mode: {self.robot_model} commands are disabled"
            )
            self.arm_ready = True
            return

        try:
            connection = connect_arm_two_stage(
                robot_model=self.robot_model,
                arm_model=self.profile["arm_model"],
                firmware_profiles=self.profile["firmwares"],
                can_interface=self.can_interface,
                probe_timeout=self.firmware_probe_timeout,
                probe_poll_period=self.firmware_probe_poll_period,
                reconnect_delay=self.firmware_reconnect_delay,
                report=self.get_logger().info,
            )
            self.arm = connection.arm
            self.firmware_probe_arm = getattr(connection, "probe_arm", None)
            self.device_firmware_info = connection.firmware_info
            detected_name = connection.firmware_profile
            if (
                self.requested_firmware_name != "auto"
                and detected_name != self.firmware_name
            ):
                self.get_logger().warning(
                    f"configured firmware {self.firmware_name} differs from "
                    f"detected {detected_name}; using detected profile"
                )
            self.firmware_name = detected_name
            self.firmware = self.profile["firmwares"][detected_name]
            self.arm.set_joint_limits_enabled(True)
            self.arm_connected = True
            self.get_logger().info(
                f"connected {self.robot_model} on {self.can_interface} "
                f"with detected firmware profile {self.firmware_name}; "
                f"saved device data: {self.device_firmware_info}"
            )
            if not self.prepare_startup_safety():
                return
            if not self.enable_arm():
                self.get_logger().error("enable timed out; motion is disabled")
                self._disable_after_initialization_failure("enable timeout")
                return
            enabled = True
            limits_ok, failed_joint = set_joint_acceleration_limits(
                self.arm,
                self.joint_count,
                self.joint_max_acceleration,
                self.joint_acc_timeout,
            )
            if not limits_ok:
                self.get_logger().error(
                    "failed to set/read back acceleration limit for joint "
                    f"{failed_joint}"
                )
                self._disable_after_initialization_failure(
                    "joint acceleration limit verification failed"
                )
                return
            self.get_logger().info(
                "joint acceleration limits verified individually: "
                f"{self.joint_max_acceleration:.3f} rad/s²"
            )
            self.arm.set_speed_percent(self.speed_percent)
            if not prepare_planned_joint_mode(
                self.arm, self.position_mode_timeout
            ):
                self.get_logger().error(
                    "CAN_CTRL/MOVE_J mode timed out; motion is disabled"
                )
                self._disable_after_initialization_failure(
                    "planned joint mode was not confirmed"
                )
                return
            self.get_logger().info(
                "planned joint mode confirmed: CAN_CTRL/MOVE_J"
            )
            joints = self.wait_for_joint_feedback()
            if joints is None:
                self.get_logger().error(
                    f"no complete {self.joint_count}-joint feedback; "
                    "motion is disabled for safety"
                )
                self._disable_after_initialization_failure(
                    "complete startup feedback is unavailable"
                )
                return
            self.jog.sync_target(joints, clamp_to_limits=False)
            self.get_logger().info(
                f"synced current {self.robot_model} joints: "
                f"{[round(value, 4) for value in self.jog.target_joints]}"
            )
            outside = [
                index
                for index, (value, (low, high)) in enumerate(
                    zip(joints, self.joint_limits), start=1
                )
                if value < low or value > high
            ]
            if outside:
                self.get_logger().warning(
                    "feedback outside configured limits on joints "
                    f"{outside}; use joint mode to jog each axis inward. "
                    "The first jog command will not jump to the limit."
                )
            if self.move_home_on_start and not self.move_home_and_wait():
                self.get_logger().error(
                    "startup home was not reached; keyboard motion is disabled"
                )
                self._disable_after_initialization_failure(
                    "startup home was not reached"
                )
                return
            self.arm_ready = True
        except Exception as exc:
            self.arm_ready = False
            self.get_logger().error(
                f"failed to initialize {self.robot_model}: {exc}"
            )
            if enabled:
                self._disable_after_initialization_failure(
                    f"initialization exception: {exc}"
                )

    def _disable_after_initialization_failure(self, reason):
        """Fail closed after a formal arm instance may have been enabled."""
        self.arm_ready = False
        if (
            not self.execute_motion
            or self.arm is None
            or getattr(self, "emergency_stopped", False)
        ):
            return
        try:
            disabled = self.arm.disable()
            if disabled is False:
                raise RuntimeError("arm.disable() returned false")
            self.get_logger().warning(
                f"arm disabled after initialization failure: {reason}"
            )
        except Exception as error:
            self.get_logger().error(
                f"disable failed after initialization failure: {error}"
            )
            self.trigger_emergency_stop()

    def arm_reports_emergency_stop(self):
        status = self.arm.get_arm_status()
        if status is None or not hasattr(status, "msg"):
            return False
        return "EMERGENCY_STOP" in str(
            getattr(status.msg, "arm_status", "")
        )

    def prepare_startup_safety(self):
        """Apply the explicit reset policy before enabling any motor."""
        if self.reset_emergency_stop_on_start:
            if not self.reset_emergency_stop():
                self.get_logger().error(
                    "emergency-stop reset timed out; motion is disabled"
                )
                return False
            return True
        if self.arm_reports_emergency_stop():
            self.get_logger().error(
                "arm reports EMERGENCY_STOP; motion is disabled. "
                "Inspect the arm and restart with "
                "reset_emergency_stop_on_start:=true to explicitly reset "
                "the controller."
            )
            return False
        self.get_logger().info(
            "startup safety check: no electronic emergency stop reported"
        )
        return True

    def reset_emergency_stop(self):
        self.get_logger().warning(
            "explicitly resetting the electronic emergency stop"
        )
        self.arm.reset()
        deadline = time.monotonic() + self.emergency_reset_timeout
        while time.monotonic() < deadline:
            if not self.arm_reports_emergency_stop():
                self.get_logger().info("electronic emergency stop reset")
                return True
            time.sleep(0.05)
        return False

    def enable_arm(self):
        deadline = time.monotonic() + self.enable_timeout
        while time.monotonic() < deadline:
            try:
                if self.clear_errors_on_enable:
                    self.arm.clear_joint_error()
                if self.arm.enable():
                    return True
                states = self.arm.get_joints_enable_status_list()
                if (len(states) >= self.joint_count and
                        all(states[:self.joint_count])):
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
                joints = extract_joint_angles(
                    self.arm.get_joint_angles(), self.joint_count
                )
                if joints is not None:
                    return joints
            except Exception as exc:
                self.get_logger().warning(
                    f"joint feedback failed: {exc}", throttle_duration_sec=1.0
                )
            time.sleep(0.05)
        return None

    def move_home_and_wait(self):
        try:
            joints = extract_joint_angles(
                self.arm.get_joint_angles(), self.joint_count
            )
        except Exception as error:
            self.get_logger().error(
                f"cannot read feedback before startup zeroing: {error}"
            )
            return False
        if joints is None:
            self.get_logger().error(
                "cannot start sequential zeroing without joint feedback"
            )
            return False

        target = list(joints)
        outside = [
            index
            for index, (value, (low, high)) in enumerate(
                zip(joints, self.joint_limits), start=1
            )
            if value < low or value > high
        ]
        if outside:
            self.get_logger().error(
                "startup zeroing refused because feedback is outside soft "
                f"limits on joints {outside}"
            )
            return False
        motion_started = False
        try:
            for index in range(self.joint_count):
                target[index] = 0.0
                self.get_logger().info(
                    f"forcing startup zero joint {index + 1}/"
                    f"{self.joint_count}: "
                    f"{[round(value, 4) for value in target]}"
                )
                motion_started = True
                self.arm.move_j(list(target))
                deadline = time.monotonic() + self.startup_home_timeout

                while time.monotonic() < deadline:
                    feedback = extract_joint_angles(
                        self.arm.get_joint_angles(), self.joint_count
                    )
                    if (feedback is not None and
                            abs(feedback[index]) <=
                            self.startup_home_tolerance):
                        self.get_logger().info(
                            f"joint {index + 1} zero reached: "
                            f"error={abs(feedback[index]):.6f} rad"
                        )
                        break
                    time.sleep(0.05)
                else:
                    self.get_logger().error(
                        f"joint {index + 1} sequential zero timed out"
                    )
                    self.trigger_emergency_stop()
                    return False

            home = [0.0] * self.joint_count
            self.jog.sync_target(home)
            self.get_logger().info("sequential startup zero complete")
            return True
        except Exception as error:
            self.get_logger().error(f"startup zeroing failed: {error}")
            if motion_started:
                self.trigger_emergency_stop()
            return False

    def external_torque_callback(self, message):
        """Convert observer residual torque to a base-frame wrench."""
        model = getattr(self, "gravity_model", None)
        if model is None:
            return
        try:
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            source_time = (
                0.0
                if stamp is None
                else float(stamp.sec) + 1e-9 * float(stamp.nanosec)
            )
            if source_time > 0.0:
                previous_source_time = float(getattr(
                    self, "latest_external_wrench_source_time", -math.inf
                ))
                if source_time <= previous_source_time:
                    raise ValueError("external torque timestamp is not newer")
                clock = getattr(self, "get_clock", None)
                if callable(clock):
                    ros_now = float(clock().now().nanoseconds) * 1e-9
                    source_age = ros_now - source_time
                    if (
                        source_age > self.admittance_wrench_timeout
                        or source_age < -0.05
                    ):
                        raise ValueError(
                            "external torque timestamp is outside the "
                            "freshness window"
                        )
            joints = np.asarray(message.position, dtype=float)
            external_torque = np.asarray(message.effort, dtype=float)
            if (
                joints.shape != (self.joint_count,)
                or external_torque.shape != (self.joint_count,)
                or not np.all(np.isfinite(joints))
                or not np.all(np.isfinite(external_torque))
            ):
                raise ValueError("external torque sample is incomplete")
            jacobian, _ = geometric_jacobian(model, joints)
            wrench = wrench_from_joint_torque(
                jacobian,
                external_torque,
                self.admittance_wrench_dls_damping,
            )
        except Exception as error:
            self.get_logger().warning(
                f"external torque sample rejected: {error}",
                throttle_duration_sec=1.0,
            )
            return
        self.latest_external_wrench = wrench
        self.latest_external_wrench_received_at = time.monotonic()
        if source_time > 0.0:
            self.latest_external_wrench_source_time = source_time

    def _external_wrench_is_fresh(self, now=None):
        """Distinguish a recent valid sample from a synthetic zero wrench."""
        checked_at = time.monotonic() if now is None else float(now)
        received_at = float(getattr(
            self, "latest_external_wrench_received_at", -math.inf
        ))
        age = checked_at - received_at
        return bool(
            math.isfinite(age)
            and 0.0 <= age <= self.admittance_wrench_timeout
        )

    def read_motor_feedback(self):
        """Read one complete cached q/dq/motor-torque sample."""
        if not hasattr(self.arm, "get_motor_states"):
            return None
        try:
            positions = extract_joint_angles(
                self.arm.get_joint_angles(), self.joint_count
            )
        except Exception:
            return None
        if positions is None:
            return None
        velocities = []
        torques = []
        for joint_index in range(1, self.joint_count + 1):
            try:
                state = self.arm.get_motor_states(joint_index)
                message = getattr(state, "msg", state)
                velocity = float(getattr(message, "velocity"))
                torque = float(getattr(message, "torque"))
            except Exception:
                return None
            if not np.isfinite(velocity) or not np.isfinite(torque):
                return None
            velocities.append(velocity)
            torques.append(torque)
        position = np.asarray(positions, dtype=float)
        velocity = self._select_feedback_velocity(
            position, np.asarray(velocities, dtype=float)
        )
        return MotorFeedback(
            position=position,
            velocity=velocity,
            torque=np.asarray(torques, dtype=float),
        )

    def _select_feedback_velocity(self, positions, sdk_velocities):
        """Use finite differences for Nero firmware with zero SDK speed."""
        estimate = (
            getattr(self, "robot_model", "") == "nero"
            and getattr(self, "firmware_name", "") in ("v111", "v112")
            and getattr(self, "nero_velocity_estimation_enabled", True)
        )
        if not estimate:
            return np.asarray(sdk_velocities, dtype=float).copy()

        position = np.asarray(positions, dtype=float)
        now = time.monotonic()
        previous_position = getattr(
            self, "feedback_previous_position", None
        )
        previous_time = getattr(self, "feedback_previous_time", None)
        previous_velocity = np.asarray(
            getattr(
                self,
                "feedback_previous_velocity",
                np.zeros(position.size),
            ),
            dtype=float,
        )
        if (
            previous_position is None
            or previous_time is None
            or previous_velocity.shape != position.shape
            or now <= previous_time
        ):
            velocity = np.zeros(position.size, dtype=float)
        else:
            velocity = estimate_joint_velocity(
                previous_position,
                position,
                previous_velocity,
                now - previous_time,
                getattr(self, "velocity_filter_time_constant", 0.03),
            )
        self.feedback_previous_position = position.copy()
        self.feedback_previous_velocity = velocity.copy()
        self.feedback_previous_time = now
        return velocity

    def read_joint_velocities(self):
        """Read cached motor velocity for MIT dynamics and torque limiting."""
        if not hasattr(self.arm, "get_motor_states"):
            return None
        velocities = []
        for joint_index in range(1, self.joint_count + 1):
            try:
                state = self.arm.get_motor_states(joint_index)
                message = getattr(state, "msg", state)
                velocity = float(getattr(message, "velocity"))
            except Exception:
                return None
            if not np.isfinite(velocity):
                return None
            velocities.append(velocity)
        return np.asarray(velocities, dtype=float)

    def send_target(self, reason):
        target = [float(value) for value in self.jog.target_joints]
        self.get_logger().info(
            f"{reason}: {[round(value, 4) for value in target]}",
            throttle_duration_sec=0.25,
        )
        if not self.execute_motion:
            return
        if (
            self.impedance_enabled
            or getattr(self, "admittance_enabled", False)
            or getattr(self, "hybrid_enabled", False)
        ):
            return
        if not self.arm_ready:
            self.get_logger().warning(
                "arm is not ready; command skipped", throttle_duration_sec=1.0
            )
            return
        try:
            self.arm.move_j(target)
        except Exception as exc:
            self.get_logger().error(f"move_j failed: {exc}")

    def trigger_emergency_stop(self):
        self.emergency_stopped = True
        self.arm_ready = False
        self.get_logger().error("ELECTRONIC EMERGENCY STOP requested")
        self._publish_control_event("emergency_stop")
        self._publish_interaction_state("emergency_stop")
        if self.execute_motion and getattr(self, "arm_connected", False):
            try:
                self.arm.electronic_emergency_stop()
            except Exception as exc:
                self.get_logger().error(
                    f"emergency stop command failed: {exc}"
                )

    def destroy_node(self):
        if self.arm is not None and self.arm_connected:
            if getattr(self, "disable_arm_on_shutdown", False):
                try:
                    self.get_logger().info(
                        "shutdown: disabling arm before disconnect"
                    )
                    self.arm.disable()
                except Exception as exc:
                    self.get_logger().error(
                        f"failed to disable {self.robot_model}: {exc}"
                    )
            else:
                self.get_logger().info(
                    "shutdown: leaving arm enabled for external takeover"
                )
            try:
                self.arm.disconnect()
            except Exception as exc:
                self.get_logger().error(
                    f"failed to disconnect {self.robot_model}: {exc}"
                )
            probe_arm = getattr(self, "firmware_probe_arm", None)
            if probe_arm is not None and probe_arm is not self.arm:
                try:
                    probe_arm.disconnect()
                except Exception as exc:
                    self.get_logger().error(
                        f"failed to disconnect {self.robot_model} firmware "
                        f"probe: {exc}"
                    )
                self.firmware_probe_arm = None
        super().destroy_node()
