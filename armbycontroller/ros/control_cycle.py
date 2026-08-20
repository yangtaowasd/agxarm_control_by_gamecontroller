"""Periodic control dispatch, MIT execution, and Cartesian targets."""

import time

import numpy as np

from armbycontroller.control import ControlInput
from armbycontroller.control import ControlReference
from armbycontroller.control import ControlSafetyError
from armbycontroller.control import ControlState
from armbycontroller.control import MitCommand
from armbycontroller.control import PositionCommand
from armbycontroller.hardware import extract_joint_angles
from armbycontroller.ik.core import IkFailure
from armbycontroller.ik.core import increment_tool_orientation
from armbycontroller.teleop import KEY_ADMITTANCE_TOGGLE
from armbycontroller.teleop import KEY_ARROW_DOWN
from armbycontroller.teleop import KEY_ARROW_LEFT
from armbycontroller.teleop import KEY_ARROW_RIGHT
from armbycontroller.teleop import KEY_ARROW_UP
from armbycontroller.teleop import KEY_BACKWARD
from armbycontroller.teleop import KEY_COUNT
from armbycontroller.teleop import KEY_DECREASE
from armbycontroller.teleop import KEY_ESTOP
from armbycontroller.teleop import KEY_FORWARD
from armbycontroller.teleop import KEY_HYBRID_TOGGLE
from armbycontroller.teleop import KEY_IMPEDANCE_TOGGLE
from armbycontroller.teleop import KEY_INCREASE
from armbycontroller.teleop import KEY_ROLL_LEFT
from armbycontroller.teleop import KEY_ROLL_RIGHT
from armbycontroller.teleop import KEY_Z_DOWN
from armbycontroller.teleop import KEY_Z_UP


class ControlCycleMixin:
    """Own periodic dispatch and exactly one command output per cycle."""

    def _send_control_result(self, result):
        command = result.command
        if isinstance(command, MitCommand):
            for index in range(command.position.size):
                self.arm.move_mit(
                    joint_index=index + 1,
                    p_des=float(command.position[index]),
                    v_des=float(command.velocity[index]),
                    kp=float(command.kp[index]),
                    kd=float(command.kd[index]),
                    t_ff=float(command.feedforward[index]),
                )
            return
        if isinstance(command, PositionCommand):
            self.jog.sync_target(command.position)
            self.send_target("Cartesian admittance")
            return
        raise TypeError(
            f"unsupported control command {type(command).__name__}"
        )

    def control_tick(self):
        keys = self.key_state
        if time.monotonic() - self.last_keyboard_time > self.keyboard_timeout:
            keys = [0] * KEY_COUNT

        state_keys = list(keys)
        if (
            getattr(self, "admittance_enabled", False)
            or getattr(self, "hybrid_enabled", False)
        ):
            permitted = {
                KEY_ESTOP,
                KEY_IMPEDANCE_TOGGLE,
                KEY_ADMITTANCE_TOGGLE,
                KEY_HYBRID_TOGGLE,
            }
            state_keys = [
                value if index in permitted else 0
                for index, value in enumerate(state_keys)
            ]
        if self.control_mode == "ik":
            # A/D belong to Cartesian Y in IK mode, not joint jogging.
            state_keys[KEY_DECREASE] = 0
            state_keys[KEY_INCREASE] = 0
        update = self.jog.update(state_keys)
        if update.selection_changed:
            self.get_logger().info(
                f"selected joint {update.selected_joint + 1}"
            )

        if update.estop_requested:
            self.trigger_emergency_stop()
            return
        if self.emergency_stopped:
            return
        if getattr(self, "interaction_transitioning", False):
            return

        interaction_requests = sum((
            update.impedance_toggle_requested,
            update.admittance_toggle_requested,
            update.hybrid_toggle_requested,
        ))
        if interaction_requests > 1:
            self.get_logger().warning(
                "I, O, and H are mutually exclusive; interaction mode "
                "unchanged"
            )
            return

        if update.hybrid_toggle_requested:
            self.toggle_hybrid()
            return

        if update.admittance_toggle_requested:
            self.toggle_admittance()
            return

        if update.impedance_toggle_requested:
            self.toggle_impedance()
            return

        if (
            getattr(self, "admittance_enabled", False)
            or getattr(self, "hybrid_enabled", False)
        ):
            return

        if update.mode_toggle_requested:
            self.toggle_control_mode()
            return

        if update.home_requested:
            self.ik_target_position = None
            self.ik_target_rotation = None
            if update.target_changed:
                self.send_target("home")
            return

        if self.control_mode == "ik":
            self.apply_cartesian_step(keys)
            return
        if not update.target_changed:
            return

        self.send_target(f"joint {update.selected_joint + 1} jog")

    def _cartesian_nullspace_options(
        self, reference_position, reference_velocity
    ):
        """Return Nero-only redundant-joint impedance arguments."""
        if getattr(self, "robot_model", "piper_l") != "nero":
            return {}
        return {
            "nullspace_reference": reference_position,
            "nullspace_reference_velocity": reference_velocity,
            "nullspace_stiffness": self.cartesian_nullspace_stiffness,
            "nullspace_damping": self.cartesian_nullspace_damping,
        }

    def _publish_dynamics_state(self, feedback):
        self._telemetry().publish_dynamics_state(feedback)

    def _validate_interaction_velocity(self, feedback, mode):
        """Apply one shared measured-speed policy to every MIT mode."""
        guard = getattr(self, "interaction_velocity_guard", None)
        if guard is None:
            return True
        try:
            guard.validate(feedback.velocity)
        except (ControlSafetyError, ValueError) as error:
            safety = getattr(self, "interaction_safety", None)
            stop = getattr(
                safety, "measured_velocity_stop_limit", guard.sustained_limits
            )
            hard = getattr(
                safety, "measured_velocity_hard_limit", guard.hard_limits
            )
            self.get_logger().error(
                f"{mode} measured joint velocity safety limit exceeded: "
                f"dq={np.round(feedback.velocity, 3).tolist()}, "
                f"stop={np.asarray(stop).tolist()}, "
                f"hard={np.asarray(hard).tolist()} rad/s; {error}"
            )
            self.trigger_emergency_stop()
            return False
        return True

    def mit_tick(self):
        """Publish one sample, then run the one active interaction mode."""
        if self.emergency_stopped:
            return
        if getattr(self, "interaction_transitioning", False):
            return
        if not self.execute_motion:
            self.get_logger().info(
                "dry-run interaction backend active",
                throttle_duration_sec=1.0,
            )
            return
        if not self.arm_ready:
            return
        feedback = self.read_motor_feedback()
        if feedback is not None:
            self._publish_dynamics_state(feedback)
        active_mode = None
        if getattr(self, "hybrid_enabled", False):
            active_mode = "hybrid"
        elif getattr(self, "admittance_enabled", False):
            active_mode = "admittance"
        elif self.impedance_enabled:
            active_mode = "impedance"
        if (
            active_mode is not None
            and getattr(self, "interaction_velocity_guard", None) is not None
        ):
            if feedback is None:
                self.get_logger().warning(
                    f"{active_mode} requires complete q/dq/torque feedback; "
                    "holding",
                    throttle_duration_sec=1.0,
                )
                return
            if not self._validate_interaction_velocity(
                feedback, active_mode
            ):
                return
        if getattr(self, "hybrid_enabled", False):
            if feedback is None:
                self.get_logger().warning(
                    "hybrid control requires complete q/dq/torque feedback; "
                    "holding",
                    throttle_duration_sec=1.0,
                )
                return
            self._hybrid_tick(feedback)
            return
        if getattr(self, "admittance_enabled", False):
            if feedback is None:
                self.get_logger().warning(
                    "admittance requires complete q/dq/torque feedback; "
                    "holding",
                    throttle_duration_sec=1.0,
                )
                return
            self._admittance_tick(feedback)
            return
        if not self.impedance_enabled:
            return
        try:
            if getattr(self, "impedance_backend", "joint") == "cartesian":
                self._cartesian_mit_tick(feedback)
                return
            sample = self._next_control_input(feedback)
            if not sample.state.velocity_valid:
                self.get_logger().warning(
                    "motor velocity unavailable; torque limit assumes zero "
                    "velocity",
                    throttle_duration_sec=1.0,
                )
            if not sample.state.position_valid:
                self.get_logger().warning(
                    "joint feedback unavailable; combined torque limit is "
                    "inactive",
                    throttle_duration_sec=1.0,
                )
            result = self._get_control_engine("joint_impedance").step(
                "joint_impedance", sample
            )
            limit = np.asarray(
                getattr(
                    self,
                    "mit_gravity_torque_limit",
                    [8.0] * self.joint_count,
                ),
                dtype=float,
            )
            if np.any(np.abs(result.command.estimated_torque) > limit + 1e-9):
                self.get_logger().warning(
                    "PD torque alone prevents the configured combined limit; "
                    "t_ff is already maximally counteracting it",
                    throttle_duration_sec=2.0,
                )
            self.get_logger().info(
                "estimated combined MIT torque=%s N·m"
                % np.round(result.command.estimated_torque, 3).tolist(),
                throttle_duration_sec=2.0,
            )
            self._send_control_result(result)
            self._publish_control_result(sample, result, "impedance")
        except Exception as exc:
            self.get_logger().error(f"MIT command failed: {exc}")
            self.trigger_emergency_stop()

    def _cartesian_mit_tick(self, feedback=None):
        """Evaluate ``J.T F + C*dq + g`` and send it only through t_ff."""
        sample = self._next_control_input(feedback)
        result = self._get_control_engine("cartesian_impedance").step(
            "cartesian_impedance", sample
        )
        raw = result.raw
        command_torque = result.command.feedforward
        self.last_cartesian_impedance_command = raw
        if result.signals["torque_clipped"]:
            self.get_logger().warning(
                "Cartesian MIT absolute torque limit active: raw=%s, "
                "sent=%s N·m"
                % (
                    np.round(
                        result.signals["raw_command_torque"], 3
                    ).tolist(),
                    np.round(command_torque, 3).tolist(),
                ),
                throttle_duration_sec=1.0,
            )
        self.get_logger().info(
            "Cartesian MIT error=%s wrench=%s task=%s null=%s posture=%s "
            "model=%s total=%s sent=%s N·m"
            % (
                np.round(raw.pose_error, 4).tolist(),
                np.round(raw.commanded_wrench, 3).tolist(),
                np.round(raw.task_torque, 3).tolist(),
                np.round(raw.nullspace_torque, 3).tolist(),
                np.round(
                    result.signals["joint_posture_torque"], 3
                ).tolist(),
                np.round(raw.model_torque, 3).tolist(),
                np.round(
                    result.signals["raw_command_torque"], 3
                ).tolist(),
                np.round(command_torque, 3).tolist(),
            ),
            throttle_duration_sec=2.0,
        )
        self._send_control_result(result)
        self._publish_control_result(sample, result, "impedance")

    def _hybrid_tick(self, feedback):
        """Run complementary Twist admittance and Cartesian impedance."""
        now = time.monotonic()
        nominal_period = 1.0 / self.mit_command_rate
        previous = self.last_hybrid_tick_time
        period = (
            nominal_period
            if previous is None
            else float(np.clip(
                now - previous,
                0.25 * nominal_period,
                2.0 * nominal_period,
            ))
        )
        self.last_hybrid_tick_time = now
        if not self._external_wrench_is_fresh(now):
            self.get_logger().error(
                "hybrid external wrench timed out; leaving interaction mode"
            )
            self._exit_hybrid("external wrench timeout", feedback)
            return
        wrench = self.latest_external_wrench
        control_state = ControlState(
            feedback.position, feedback.velocity, feedback.torque
        )
        sample = ControlInput(
            now,
            period,
            control_state,
            ControlReference.hold(feedback.position, wrench),
        )
        try:
            result = self._get_control_engine("hybrid_cartesian").step(
                "hybrid_cartesian", sample
            )
        except (ControlSafetyError, ValueError) as error:
            if isinstance(error, ControlSafetyError) and error.reason in (
                "measured_velocity_limit",
                "measured_velocity_hard_limit",
            ):
                hard_stop = error.reason == "measured_velocity_hard_limit"
                limit = (
                    self.admittance_measured_joint_velocity_hard_limit
                    if hard_stop
                    else self.admittance_measured_joint_velocity_stop_limit
                )
                limit_kind = (
                    "hard limit" if hard_stop else "sustained limit"
                )
                self.get_logger().error(
                    f"hybrid measured joint velocity {limit_kind} exceeded: "
                    f"dq={np.round(feedback.velocity, 3).tolist()}, "
                    "limit=%s rad/s"
                    % np.asarray(limit, dtype=float).tolist()
                )
                self.trigger_emergency_stop()
                return
            self.get_logger().warning(
                f"hybrid safety rejection; exiting to hold: {error}"
            )
            self._exit_hybrid("velocity IK or MIT safety rejection")
            return
        joints = result.command.position
        desired_pose = np.asarray(result.signals["desired_pose"], dtype=float)
        self.ik_target_position = desired_pose[:3, 3].copy()
        self.ik_target_rotation = desired_pose[:3, :3].copy()
        self.remember_ik_valid(
            self.ik_target_position,
            self.ik_target_rotation,
            joints,
        )
        try:
            self._send_control_result(result)
        except Exception as error:
            self.get_logger().error(f"hybrid MIT command failed: {error}")
            self.trigger_emergency_stop()
            return
        self._publish_control_result(sample, result, "hybrid")
        if result.signals.get("torque_saturated", False):
            self.get_logger().warning(
                "hybrid MIT torque envelope active: requested=%s, "
                "sent=%s, reason=%s"
                % (
                    np.round(
                        result.signals["torque_total_requested"], 3
                    ).tolist(),
                    np.round(
                        result.signals["torque_total_estimated"], 3
                    ).tolist(),
                    result.signals["torque_saturation_reason"],
                ),
                throttle_duration_sec=1.0,
            )
        self.get_logger().info(
            "hybrid axes=%s adm_twist=%s imp_wrench=%s model=%s "
            "total=%s N.m"
            % (
                self.hybrid_admittance_axes,
                np.round(result.signals["admittance_twist"], 4).tolist(),
                np.round(result.signals["commanded_wrench"], 3).tolist(),
                np.round(
                    result.signals["torque_model_requested"], 3
                ).tolist(),
                np.round(result.command.estimated_torque, 3).tolist(),
            ),
            throttle_duration_sec=1.0,
        )

    def _admittance_tick(self, feedback):
        """Advance admittance and send one bounded low-gain MIT command."""
        now = time.monotonic()
        nominal_period = 1.0 / self.mit_command_rate
        previous = self.last_admittance_tick_time
        period = (
            nominal_period
            if previous is None
            else float(np.clip(
                now - previous,
                0.25 * nominal_period,
                2.0 * nominal_period,
            ))
        )
        self.last_admittance_tick_time = now
        if not self._external_wrench_is_fresh(now):
            self.get_logger().error(
                "admittance external wrench timed out; leaving interaction "
                "mode"
            )
            self._exit_admittance("external wrench timeout", feedback)
            return
        wrench = self.latest_external_wrench
        control_state = ControlState(
            feedback.position, feedback.velocity, feedback.torque
        )
        sample = ControlInput(
            now,
            period,
            control_state,
            ControlReference.hold(feedback.position, wrench),
        )
        try:
            result = self._get_control_engine(
                "cartesian_admittance"
            ).step(
                "cartesian_admittance", sample
            )
        except (ControlSafetyError, ValueError) as error:
            if isinstance(error, ControlSafetyError) and error.reason in (
                "measured_velocity_limit",
                "measured_velocity_hard_limit",
            ):
                hard_stop = error.reason == "measured_velocity_hard_limit"
                limit = (
                    self.admittance_measured_joint_velocity_hard_limit
                    if hard_stop
                    else self.admittance_measured_joint_velocity_stop_limit
                )
                limit_kind = (
                    "hard limit" if hard_stop else "sustained limit"
                )
                self.get_logger().error(
                    f"admittance measured joint velocity {limit_kind} "
                    "exceeded: "
                    f"dq={np.round(feedback.velocity, 3).tolist()}, "
                    "limit=%s rad/s"
                    % np.asarray(limit, dtype=float).tolist()
                )
                self.trigger_emergency_stop()
                return
            self.get_logger().warning(
                f"admittance safety rejection; exiting to hold: {error}"
            )
            self._exit_admittance("velocity IK or MIT safety rejection")
            return
        state = result.raw
        joints = result.command.position
        desired_pose = np.asarray(result.signals["desired_pose"], dtype=float)
        self.ik_target_position = desired_pose[:3, 3].copy()
        self.ik_target_rotation = desired_pose[:3, :3].copy()
        self.remember_ik_valid(
            self.ik_target_position,
            self.ik_target_rotation,
            joints,
        )
        try:
            self._send_control_result(result)
        except Exception as error:
            self.get_logger().error(
                f"admittance MIT command failed: {error}"
            )
            self.trigger_emergency_stop()
            return
        selected_mode = getattr(
            self,
            "admittance_mode",
            getattr(self.admittance_controller, "mode", "resistive"),
        )
        self._publish_control_result(
            sample, result, f"admittance_{selected_mode}"
        )
        if result.signals.get("torque_saturated", False):
            self.get_logger().warning(
                "admittance MIT torque envelope active: requested=%s, "
                "sent=%s, reason=%s"
                % (
                    np.round(
                        result.signals["torque_total_requested"], 3
                    ).tolist(),
                    np.round(
                        result.signals["torque_total_estimated"], 3
                    ).tolist(),
                    result.signals["torque_saturation_reason"],
                ),
                throttle_duration_sec=1.0,
            )
        self.get_logger().info(
            "admittance mode=%s wrench=%s resistance=%s offset=%s "
            "model=%s t_ff=%s total=%s N·m"
            % (
                selected_mode,
                np.round(state.applied_wrench, 3).tolist(),
                np.round(
                    getattr(state, "resisting_wrench", np.zeros(6)), 3
                ).tolist(),
                np.round(state.offset, 4).tolist(),
                np.round(
                    result.signals.get(
                        "torque_model_requested",
                        np.zeros(result.command.position.size),
                    ),
                    3,
                ).tolist(),
                np.round(result.command.feedforward, 3).tolist(),
                np.round(result.command.estimated_torque, 3).tolist(),
            ),
            throttle_duration_sec=1.0,
        )

    def current_or_target_joints(self):
        if self.execute_motion and self.arm_ready:
            try:
                joints = extract_joint_angles(
                    self.arm.get_joint_angles(), self.joint_count
                )
                if joints is not None:
                    return np.asarray(joints, dtype=float)
            except Exception as exc:
                self.get_logger().warning(
                    f"joint feedback unavailable; using last target: {exc}",
                    throttle_duration_sec=1.0,
                )
        return np.asarray(self.jog.target_joints, dtype=float)

    def remember_ik_valid(self, position, rotation, joints):
        self.ik_valid_history.append(
            (np.asarray(position, dtype=float).copy(),
             np.asarray(rotation, dtype=float).copy(),
             np.asarray(joints, dtype=float).copy())
        )

    def recover_ik(self, reason):
        self.ik_recovery_until = time.monotonic() + self.ik_recovery_pause
        if self.ik_valid_history:
            position, rotation, joints = self.ik_valid_history[-1]
            self.ik_target_position = position.copy()
            self.ik_target_rotation = rotation.copy()
            self.jog.sync_target(joints)
            self.send_target("IK recovery to last valid target")
        self.get_logger().warning(
            f"IK stuck: {reason}; retained last valid target and paused "
            f"{self.ik_recovery_pause:.1f} s"
        )

    def apply_cartesian_step(self, keys):
        remaining = self.ik_recovery_until - time.monotonic()
        if remaining > 0.0:
            return
        direction = np.asarray(
            [
                keys[KEY_FORWARD] - keys[KEY_BACKWARD],
                keys[KEY_DECREASE] - keys[KEY_INCREASE],
                keys[KEY_Z_UP] - keys[KEY_Z_DOWN],
            ],
            dtype=float,
        )
        pitch = (
            keys[KEY_ARROW_DOWN] - keys[KEY_ARROW_UP]
        ) * self.orientation_step_per_tick
        yaw = (
            keys[KEY_ARROW_LEFT] - keys[KEY_ARROW_RIGHT]
        ) * self.orientation_step_per_tick
        roll = (
            keys[KEY_ROLL_LEFT] - keys[KEY_ROLL_RIGHT]
        ) * self.orientation_step_per_tick
        if not np.any(direction) and pitch == yaw == roll == 0.0:
            return
        if self.ik_target_position is None or self.ik_target_rotation is None:
            joints = self.current_or_target_joints()
            position, rotation = self.ik_solver.fk(joints)
            self.ik_target_position = np.asarray(position, dtype=float)
            self.ik_target_rotation = np.asarray(rotation, dtype=float)
        candidate = (
            self.ik_target_position + direction * self.cartesian_step_per_tick
        )
        candidate_rotation = increment_tool_orientation(
            self.ik_target_rotation, pitch, yaw, roll
        )
        seed = np.asarray(self.jog.target_joints, dtype=float)
        try:
            result = self.ik_engine.solve(
                candidate, candidate_rotation, seed
            )
        except IkFailure as error:
            self.recover_ik(str(error))
            return
        if (self.impedance_enabled and
                float(np.max(np.abs(result.joints - seed))) >
                self.mit_max_joint_step):
            self.recover_ik("MIT IK joint step exceeds configured limit")
            return
        self.ik_target_position = candidate
        self.ik_target_rotation = candidate_rotation
        self.jog.sync_target(result.joints)
        self.remember_ik_valid(candidate, candidate_rotation, result.joints)
        self.send_target("IK pose jog")
