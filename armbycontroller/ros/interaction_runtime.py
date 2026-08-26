"""Interaction-mode services, interlock transitions, and ownership."""

import json
import time

import numpy as np

from armbycontroller.api import InteractionModeRequestResult
from armbycontroller.control import ControlInput
from armbycontroller.control import ControlReference
from armbycontroller.control import ControlState
from armbycontroller.control import InteractionModeLifecycle
from armbycontroller.ik.core import prepare_planned_joint_mode


class InteractionRuntimeMixin:
    """Own public mode requests and normal-mediated mode transitions."""

    def _public_mode_service(self, target, response):
        """Adapt one standard Trigger call to the transport-neutral API."""
        source = "ros_service"
        try:
            result = self.public_interaction_mode_interface.request(
                target, source=source
            )
        except Exception as error:
            try:
                active = self._current_interaction_mode()
            except Exception:
                active = "normal"
            result = InteractionModeRequestResult(
                False,
                str(target),
                active,
                False,
                f"interaction mode request failed: {error}",
            )
        response.success = result.success
        response.message = json.dumps(
            result.to_payload(),
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        self._publish_interaction_state(
            f"{source} requested {target}"
        )
        return response

    def _normal_mode_service_callback(self, request, response):
        del request
        return self._public_mode_service("normal", response)

    def _impedance_mode_service_callback(self, request, response):
        del request
        return self._public_mode_service("impedance", response)

    def _admittance_mode_service_callback(self, request, response):
        del request
        return self._public_mode_service("admittance", response)

    def _enter_public_impedance(self, reason):
        """Enter impedance from normal for a public set-mode request."""
        del reason
        self.toggle_impedance()
        return self._current_interaction_mode() == "impedance"

    def _enter_public_admittance(self, reason):
        """Enter admittance from normal for a public set-mode request."""
        del reason
        self.toggle_admittance()
        return self._current_interaction_mode() == "admittance"

    def toggle_control_mode(self):
        if (
            getattr(self, "admittance_enabled", False)
            or getattr(self, "hybrid_enabled", False)
        ):
            active = (
                "hybrid"
                if getattr(self, "hybrid_enabled", False)
                else "admittance"
            )
            key = "H" if active == "hybrid" else "O"
            self.get_logger().warning(
                f"P is locked while {active} is active; press {key} to exit"
            )
            return
        if self.control_mode == "joint":
            joints = self.current_or_target_joints()
            if self.impedance_enabled:
                self.jog.sync_target(joints, clamp_to_limits=False)
                trajectory = getattr(self, "mit_trajectory", None)
                if trajectory is not None:
                    trajectory.reset(joints)
            position, rotation = self.ik_solver.fk(joints)
            self.ik_target_position = np.asarray(position, dtype=float)
            self.ik_target_rotation = np.asarray(rotation, dtype=float)
            self.ik_valid_history.clear()
            self.remember_ik_valid(
                self.ik_target_position, self.ik_target_rotation, joints
            )
            self.control_mode = "ik"
            self.get_logger().info(
                "mode=IK + backend="
                f"{'MIT' if self.impedance_enabled else 'planned'}: "
                "XYZ=W/S+A/D+Z/X; arrows=pointing; PgUp/PgDn=tilt"
            )
        else:
            joints = self.current_or_target_joints()
            self.jog.sync_target(
                joints, clamp_to_limits=not self.impedance_enabled
            )
            if self.impedance_enabled:
                trajectory = getattr(self, "mit_trajectory", None)
                if trajectory is not None:
                    trajectory.reset(joints)
            self.control_mode = "joint"
            self.get_logger().info(
                f"mode=JOINT: 1-{self.joint_count} select, A/D jog"
            )
        self._publish_interaction_state("control_mode_changed")

    def _start_impedance_if_requested(self, requested):
        """Enter startup impedance only after feedback is proven live."""
        if not requested:
            return
        feedback = None
        if self.execute_motion:
            if not self.arm_ready:
                self.toggle_impedance()
                return
            feedback = self.wait_for_motor_feedback()
            if feedback is None:
                self.get_logger().error(
                    "cannot start in MIT impedance: live q/dq/torque "
                    "feedback did not arrive before feedback_timeout; "
                    "remaining in normal mode"
                )
                return
        self.toggle_impedance(feedback=feedback)

    def toggle_impedance(self, feedback=None):
        """Switch between planned motion and the selected MIT backend."""
        transition = self._plan_interaction_mode("impedance")
        if transition.target == "normal":
            self._enter_normal_interaction_mode("I toggle")
            return
        if (
            transition.path[0] == "normal"
            and not self._enter_normal_interaction_mode(
                "switching to impedance"
            )
        ):
            self.get_logger().error(
                "cannot enter impedance: normal mode was not reached"
            )
            return
        backend = getattr(self, "impedance_backend", "joint")
        if self.execute_motion:
            if not self.arm_ready:
                self.get_logger().error(
                    "cannot enter MIT: arm is not ready"
                )
                return
            if feedback is None:
                feedback = self.read_motor_feedback()
            if feedback is None:
                self.get_logger().error(
                    "cannot enter MIT: complete q/dq/torque feedback is "
                    "required"
                )
                return
            joints = feedback.position
            velocities = feedback.velocity
            if self.gravity_model is None:
                self.get_logger().error(
                    "cannot enter MIT: URDF inverse dynamics is unavailable"
                )
                return
            try:
                support = np.asarray(
                    self.gravity_model.inverse_dynamics(
                        joints,
                        velocities,
                        np.zeros(self.joint_count),
                    ),
                    dtype=float,
                )
            except Exception as exc:
                self.get_logger().error(
                    f"cannot enter MIT: inverse dynamics failed: {exc}"
                )
                return
            if support.shape != (self.joint_count,) or not np.all(
                np.isfinite(support)
            ):
                self.get_logger().error(
                    "cannot enter MIT: inverse dynamics torque is invalid"
                )
                return
            if backend == "cartesian":
                try:
                    state = ControlState(
                        joints,
                        velocities,
                        np.zeros(self.joint_count),
                        effort_valid=False,
                    )
                    preflight = ControlInput(
                        time.monotonic(),
                        1.0 / self.mit_command_rate,
                        state,
                        ControlReference.hold(joints),
                    )
                    engine = self._get_control_engine(
                        "cartesian_impedance"
                    )
                    engine.reset("cartesian_impedance", state)
                    engine.step("cartesian_impedance", preflight)
                except Exception as exc:
                    self.get_logger().error(
                        "cannot enter Cartesian MIT: formula preflight "
                        f"failed: {exc}"
                    )
                    return
        else:
            joints = self.current_or_target_joints()
        self.jog.sync_target(joints, clamp_to_limits=False)
        self._commit_interaction_mode("impedance")
        self._check_interaction_mode_invariant()
        self.last_mit_tick_time = None
        self.mit_trajectory.reset(joints)
        selected = (
            "cartesian_impedance"
            if backend == "cartesian"
            else "joint_impedance"
        )
        if (
            getattr(self, "control_engine", None) is not None
            or all(
                hasattr(self, name)
                for name in ("mit_kp", "mit_kd", "mit_feedforward")
            )
        ):
            state = ControlState(
                joints,
                feedback.velocity
                if feedback is not None else np.zeros(self.joint_count),
                feedback.torque
                if feedback is not None else np.zeros(self.joint_count),
                velocity_valid=feedback is not None,
                effort_valid=feedback is not None,
            )
            self._get_control_engine(selected).reset(selected, state)
        if backend == "cartesian":
            pose = self.gravity_model.forward_kinematics(joints)
            self.ik_target_position = np.asarray(
                pose[:3, 3], dtype=float
            )
            self.ik_target_rotation = np.asarray(
                pose[:3, :3], dtype=float
            )
            self.ik_valid_history.clear()
            self.remember_ik_valid(
                self.ik_target_position,
                self.ik_target_rotation,
                joints,
            )
        self.get_logger().warning(
            f"backend={backend} impedance via MIT + "
            f"control={self.control_mode.upper()}: "
            + (
                "holding current tool pose"
                if backend == "cartesian"
                else "holding current joints"
            )
        )
        self._publish_control_event(
            "controller_enabled", controller=selected
        )
        self.mit_tick()

    def _exit_impedance(self, reason, feedback=None):
        """Restore planned control before committing the normal mode."""
        if not getattr(self, "impedance_enabled", False):
            return True
        self._begin_interaction_transition("normal", reason)
        joints = self._interaction_exit_joints("impedance", feedback)
        if joints is None:
            return False
        self.jog.sync_target(joints, clamp_to_limits=False)
        try:
            restored = not self.execute_motion or prepare_planned_joint_mode(
                self.arm, self.position_mode_timeout
            )
        except Exception as error:
            return self._fail_interaction_transition(
                f"planned joint mode failed during impedance exit: {error}"
            )
        if not restored:
            return self._fail_interaction_transition(
                "planned joint mode was not confirmed after impedance exit"
            )
        if not self.send_planned_hold(f"MIT exit hold ({reason})"):
            return self._fail_interaction_transition(
                "planned hold command failed after impedance exit"
            )
        self._commit_interaction_mode("normal")
        self._complete_interaction_transition()
        self._check_interaction_mode_invariant()
        backend = getattr(self, "impedance_backend", "joint")
        self.get_logger().info("backend=planned position control")
        self._publish_control_event(
            "controller_disabled",
            controller=f"{backend}_impedance",
            reason=str(reason),
        )
        self._publish_interaction_state("mode_exit_complete")
        return True

    def _enter_normal_interaction_mode(self, reason):
        """Exit the active interaction controller and hold in planned mode."""
        self._check_interaction_mode_invariant()
        if getattr(self, "hybrid_enabled", False):
            restored = self._exit_hybrid(reason)
            if restored:
                self._check_interaction_mode_invariant()
            return restored
        if getattr(self, "admittance_enabled", False):
            restored = self._exit_admittance(reason)
            if restored:
                self._check_interaction_mode_invariant()
            return restored
        if getattr(self, "impedance_enabled", False):
            restored = self._exit_impedance(reason)
            if restored:
                self._check_interaction_mode_invariant()
            return restored
        return True

    def _begin_interaction_transition(self, target, reason):
        """Suppress control output while hardware mode ownership changes."""
        self.interaction_transitioning = True
        self.interaction_transition_target = str(target)
        self._publish_control_event(
            "mode_transition_started", target=str(target), reason=str(reason)
        )
        self._publish_interaction_state("mode_transition_started")

    def _complete_interaction_transition(self):
        self.interaction_transitioning = False
        self.interaction_transition_target = ""
        self.interaction_fault_reason = ""

    def _fail_interaction_transition(self, reason):
        """Latch an ambiguous transition as a fault and stop the hardware."""
        self.interaction_transitioning = False
        self.interaction_fault_reason = str(reason)
        self.get_logger().error(str(reason))
        self._publish_control_event(
            "mode_transition_failed",
            target=getattr(self, "interaction_transition_target", ""),
            reason=str(reason),
        )
        self.trigger_emergency_stop()
        return False

    def _interaction_exit_joints(self, mode, feedback=None):
        """Require fresh feedback before handing MIT back to MOVE_J."""
        if not self.execute_motion:
            return self.current_or_target_joints()
        sample = (
            feedback
            if feedback is not None
            else self.read_motor_feedback()
        )
        if sample is None:
            self._fail_interaction_transition(
                f"cannot exit {mode}: fresh q/dq/torque feedback is "
                "unavailable"
            )
            return None
        arrays = tuple(
            np.asarray(getattr(sample, name), dtype=float)
            for name in ("position", "velocity", "torque")
        )
        if any(
            values.shape != (self.joint_count,)
            or not np.all(np.isfinite(values))
            for values in arrays
        ):
            self._fail_interaction_transition(
                f"cannot exit {mode}: feedback sample is incomplete"
            )
            return None
        return arrays[0].copy()

    def _check_interaction_mode_invariant(self):
        lifecycle = getattr(self, "interaction_lifecycle", None)
        if lifecycle is None:
            lifecycle = InteractionModeLifecycle()
            self.interaction_lifecycle = lifecycle
        return lifecycle.synchronize(
            getattr(self, "impedance_enabled", False),
            getattr(self, "admittance_enabled", False),
            getattr(self, "hybrid_enabled", False),
        )

    def _plan_interaction_mode(self, target):
        """Return the required normal-mediated path to a target mode."""
        self._check_interaction_mode_invariant()
        return self.interaction_lifecycle.plan(target)

    def _commit_interaction_mode(self, mode):
        """Commit one completed mode transition and synchronize flags."""
        lifecycle = getattr(self, "interaction_lifecycle", None)
        if lifecycle is None:
            lifecycle = InteractionModeLifecycle()
            lifecycle.synchronize(
                getattr(self, "impedance_enabled", False),
                getattr(self, "admittance_enabled", False),
                getattr(self, "hybrid_enabled", False),
            )
            self.interaction_lifecycle = lifecycle
        active = lifecycle.commit(mode)
        self.impedance_enabled = active == "impedance"
        self.admittance_enabled = active == "admittance"
        self.hybrid_enabled = active == "hybrid"
        velocity_guard = getattr(self, "interaction_velocity_guard", None)
        if velocity_guard is not None:
            velocity_guard.reset()
        self.interaction_fault_reason = ""
        self._publish_interaction_state("mode_committed")
        return active

    def toggle_admittance(self):
        """Toggle Cartesian velocity admittance over low-gain MIT."""
        transition = self._plan_interaction_mode("admittance")
        if transition.target == "normal":
            self._enter_normal_interaction_mode("O toggle")
            return
        if (
            transition.path[0] == "normal"
            and not self._enter_normal_interaction_mode(
                "switching to admittance"
            )
        ):
            self.get_logger().error(
                "cannot enter admittance: normal mode was not reached"
            )
            return
        if self.execute_motion and not self.arm_ready:
            self.get_logger().error(
                "cannot enter admittance: arm is not ready"
            )
            return
        model = getattr(self, "gravity_model", None)
        if model is None:
            self.get_logger().error(
                "cannot enter admittance: URDF screw model is unavailable"
            )
            return
        feedback = self.read_motor_feedback() if self.execute_motion else None
        if self.execute_motion and feedback is None:
            self.get_logger().error(
                "cannot enter admittance: complete q/dq/torque feedback "
                "is required"
            )
            return
        if self.execute_motion and not self._external_wrench_is_fresh():
            self.get_logger().error(
                "cannot enter admittance: momentum-observer wrench is stale; "
                f"check {self.external_torque_topic}"
            )
            return
        joints = (
            feedback.position
            if feedback is not None
            else self.current_or_target_joints()
        )
        pose = model.forward_kinematics(joints)
        joint_count = int(getattr(
            self, "joint_count", len(self.jog.target_joints)
        ))
        state = ControlState(
            joints,
            feedback.velocity
            if feedback is not None else np.zeros(joint_count),
            feedback.torque
            if feedback is not None else np.zeros(joint_count),
            velocity_valid=feedback is not None,
            effort_valid=feedback is not None,
        )
        try:
            engine = self._get_control_engine("cartesian_admittance")
            engine.reset("cartesian_admittance", state)
            if feedback is not None:
                preflight = ControlInput(
                    time.monotonic(),
                    1.0 / self.mit_command_rate,
                    state,
                    ControlReference.hold(joints, np.zeros(6)),
                )
                engine.step("cartesian_admittance", preflight)
                engine.reset("cartesian_admittance", state)
        except Exception as error:
            self.get_logger().error(
                "cannot enter admittance MIT: formula preflight failed: "
                f"{error}"
            )
            return
        self.admittance_previous_control_mode = self.control_mode
        self.control_mode = "ik"
        self.jog.sync_target(joints, clamp_to_limits=False)
        self.mit_trajectory.reset(joints)
        self.ik_target_position = np.asarray(pose[:3, 3], dtype=float)
        self.ik_target_rotation = np.asarray(pose[:3, :3], dtype=float)
        self.ik_valid_history.clear()
        self.remember_ik_valid(
            self.ik_target_position, self.ik_target_rotation, joints
        )
        self.last_admittance_tick_time = None
        self._commit_interaction_mode("admittance")
        self._check_interaction_mode_invariant()
        selected_mode = getattr(
            self,
            "admittance_mode",
            getattr(self.admittance_controller, "mode", "resistive"),
        )
        self.get_logger().warning(
            "backend=low-gain MIT + control=Cartesian admittance "
            f"({selected_mode}); "
            "I is interlocked; press O to exit"
        )
        self._publish_control_event(
            "controller_enabled",
            controller="cartesian_admittance",
            admittance_mode=selected_mode,
        )
        if feedback is not None:
            self._admittance_tick(feedback)

    def _exit_admittance(self, reason, feedback=None):
        """Restore planned control before committing the normal mode."""
        if not getattr(self, "admittance_enabled", False):
            return True
        self._begin_interaction_transition("normal", reason)
        joints = self._interaction_exit_joints("admittance", feedback)
        if joints is None:
            return False
        self.jog.sync_target(joints, clamp_to_limits=False)
        try:
            restored = not self.execute_motion or prepare_planned_joint_mode(
                self.arm, self.position_mode_timeout
            )
        except Exception as error:
            return self._fail_interaction_transition(
                f"planned joint mode failed during admittance exit: {error}"
            )
        if not restored:
            return self._fail_interaction_transition(
                "planned joint mode was not confirmed after admittance exit"
            )
        if not self.send_planned_hold(f"admittance exit hold ({reason})"):
            return self._fail_interaction_transition(
                "planned hold command failed after admittance exit"
            )
        self._commit_interaction_mode("normal")
        self._complete_interaction_transition()
        self.last_admittance_tick_time = None
        model = getattr(self, "gravity_model", None)
        if model is not None:
            pose = model.forward_kinematics(joints)
            self.ik_target_position = np.asarray(pose[:3, 3], dtype=float)
            self.ik_target_rotation = np.asarray(pose[:3, :3], dtype=float)
        self.control_mode = getattr(
            self, "admittance_previous_control_mode", "joint"
        )
        self._check_interaction_mode_invariant()
        selected_mode = getattr(
            self,
            "admittance_mode",
            getattr(self.admittance_controller, "mode", "resistive"),
        )
        self._publish_control_event(
            "controller_disabled",
            controller="cartesian_admittance",
            admittance_mode=selected_mode,
            reason=str(reason),
        )
        self.get_logger().info(
            "Cartesian admittance MIT exited; backend=planned position"
        )
        self._publish_interaction_state("mode_exit_complete")
        return True

    def toggle_hybrid(self):
        """Toggle complementary Cartesian impedance/admittance control."""
        transition = self._plan_interaction_mode("hybrid")
        if transition.target == "normal":
            self._enter_normal_interaction_mode("H toggle")
            return
        if (
            transition.path[0] == "normal"
            and not self._enter_normal_interaction_mode(
                "switching to hybrid"
            )
        ):
            self.get_logger().error(
                "cannot enter hybrid: normal mode was not reached"
            )
            return
        if self.execute_motion and not self.arm_ready:
            self.get_logger().error(
                "cannot enter hybrid: arm is not ready"
            )
            return
        model = getattr(self, "gravity_model", None)
        if model is None:
            self.get_logger().error(
                "cannot enter hybrid: URDF screw model is unavailable"
            )
            return
        feedback = self.read_motor_feedback() if self.execute_motion else None
        if self.execute_motion and feedback is None:
            self.get_logger().error(
                "cannot enter hybrid: complete q/dq/torque feedback is "
                "required"
            )
            return
        if self.execute_motion and not self._external_wrench_is_fresh():
            self.get_logger().error(
                "cannot enter hybrid: momentum-observer wrench is stale; "
                f"check {self.external_torque_topic}"
            )
            return
        joints = (
            feedback.position
            if feedback is not None
            else self.current_or_target_joints()
        )
        pose = model.forward_kinematics(joints)
        joint_count = int(getattr(
            self, "joint_count", len(self.jog.target_joints)
        ))
        state = ControlState(
            joints,
            feedback.velocity
            if feedback is not None else np.zeros(joint_count),
            feedback.torque
            if feedback is not None else np.zeros(joint_count),
            velocity_valid=feedback is not None,
            effort_valid=feedback is not None,
        )
        try:
            engine = self._get_control_engine("hybrid_cartesian")
            engine.reset("hybrid_cartesian", state)
            if feedback is not None:
                preflight = ControlInput(
                    time.monotonic(),
                    1.0 / self.mit_command_rate,
                    state,
                    ControlReference.hold(joints, np.zeros(6)),
                )
                engine.step("hybrid_cartesian", preflight)
                engine.reset("hybrid_cartesian", state)
        except Exception as error:
            self.get_logger().error(
                "cannot enter hybrid MIT: formula preflight failed: "
                f"{error}"
            )
            return
        self.hybrid_previous_control_mode = self.control_mode
        self.control_mode = "ik"
        self.jog.sync_target(joints, clamp_to_limits=False)
        self.mit_trajectory.reset(joints)
        self.ik_target_position = np.asarray(pose[:3, 3], dtype=float)
        self.ik_target_rotation = np.asarray(pose[:3, :3], dtype=float)
        self.ik_valid_history.clear()
        self.remember_ik_valid(
            self.ik_target_position, self.ik_target_rotation, joints
        )
        self.last_hybrid_tick_time = None
        self._commit_interaction_mode("hybrid")
        self._check_interaction_mode_invariant()
        self.get_logger().warning(
            "backend=low-gain MIT + control=Cartesian hybrid; "
            f"admittance axes={self.hybrid_admittance_axes}; "
            "I and O are interlocked; press H to exit"
        )
        self._publish_control_event(
            "controller_enabled",
            controller="hybrid_cartesian",
            admittance_axes=self.hybrid_admittance_axes,
        )
        if feedback is not None:
            self._hybrid_tick(feedback)

    def _exit_hybrid(self, reason, feedback=None):
        """Restore planned control before committing the normal mode."""
        if not getattr(self, "hybrid_enabled", False):
            return True
        self._begin_interaction_transition("normal", reason)
        joints = self._interaction_exit_joints("hybrid", feedback)
        if joints is None:
            return False
        self.jog.sync_target(joints, clamp_to_limits=False)
        try:
            restored = not self.execute_motion or prepare_planned_joint_mode(
                self.arm, self.position_mode_timeout
            )
        except Exception as error:
            return self._fail_interaction_transition(
                f"planned joint mode failed during hybrid exit: {error}"
            )
        if not restored:
            return self._fail_interaction_transition(
                "planned joint mode was not confirmed after hybrid exit"
            )
        if not self.send_planned_hold(f"hybrid exit hold ({reason})"):
            return self._fail_interaction_transition(
                "planned hold command failed after hybrid exit"
            )
        self._commit_interaction_mode("normal")
        self._complete_interaction_transition()
        self.last_hybrid_tick_time = None
        model = getattr(self, "gravity_model", None)
        if model is not None:
            pose = model.forward_kinematics(joints)
            self.ik_target_position = np.asarray(pose[:3, 3], dtype=float)
            self.ik_target_rotation = np.asarray(pose[:3, :3], dtype=float)
        self.control_mode = getattr(
            self, "hybrid_previous_control_mode", "joint"
        )
        self._check_interaction_mode_invariant()
        self._publish_control_event(
            "controller_disabled",
            controller="hybrid_cartesian",
            admittance_axes=self.hybrid_admittance_axes,
            reason=str(reason),
        )
        self.get_logger().info(
            "Cartesian hybrid MIT exited; backend=planned position"
        )
        self._publish_interaction_state("mode_exit_complete")
        return True
