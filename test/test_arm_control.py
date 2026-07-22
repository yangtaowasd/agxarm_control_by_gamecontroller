"""Tests for shared arm IK, keyboard state, and hand compatibility."""

from collections import deque
import math
from types import SimpleNamespace

import numpy as np
import pytest

from armbycontroller.ik_core import AgxIkEngine
from armbycontroller.ik_core import IkFailure
from armbycontroller.ik_core import increment_tool_orientation
from armbycontroller.ik_core import make_pointing_quaternion
from armbycontroller.ik_core import pointing_error_angle
from armbycontroller.ik_core import prepare_planned_joint_mode
from armbycontroller.ik_core import quaternion_to_rotation_matrix
from armbycontroller.ik_core import radial_workspace_check
from armbycontroller.ik_core import rotation_error_angle
from armbycontroller.ik_core import rotation_matrix_to_quaternion
from armbycontroller.ik_core import resolve_firmware_name
from armbycontroller.ik_core import set_joint_acceleration_limits
from armbycontroller.ik_core import solve_pointing_ik
from armbycontroller.keyboard_controller import ArmJointJogState
from armbycontroller.keyboard_controller import ArmKeyboardController
from armbycontroller.keyboard_controller import blend_handover_kp
from armbycontroller.keyboard_controller import bounded_velocity_damping
from armbycontroller.keyboard_controller import expand_joint_values
from armbycontroller.keyboard_controller import default_mit_gains
from armbycontroller.keyboard_controller import default_mit_feedforward
from armbycontroller.keyboard_controller import KEY_COUNT
from armbycontroller.keyboard_controller import KEY_DECREASE
from armbycontroller.keyboard_controller import KEY_ESTOP
from armbycontroller.keyboard_controller import KEY_HOME
from armbycontroller.keyboard_controller import KEY_INCREASE
from armbycontroller.keyboard_controller import KEY_IMPEDANCE_TOGGLE
from armbycontroller.keyboard_controller import KEY_MODE_TOGGLE
from armbycontroller.pose_controller import PoseController


def test_quaternion_conversion_normalizes_input():
    """A non-unit quaternion should produce the expected rotation."""
    rotation = quaternion_to_rotation_matrix(0.0, 0.0, 2.0, 2.0)

    expected = np.array([[0.0, -1.0, 0.0],
                         [1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0]])
    assert rotation == pytest.approx(expected)
    assert rotation_error_angle(rotation, expected) < 1e-7
    quaternion = rotation_matrix_to_quaternion(rotation)
    converted = quaternion_to_rotation_matrix(*quaternion)
    assert converted == pytest.approx(rotation)


@pytest.mark.parametrize(
    "quaternion",
    [(0.0, 0.0, 0.0, 0.0),
     (math.nan, 0.0, 0.0, 1.0),
     (0.0, math.inf, 0.0, 1.0)],
)
def test_quaternion_conversion_rejects_invalid_input(quaternion):
    """Invalid orientations must not reach the IK solver."""
    with pytest.raises(ValueError):
        quaternion_to_rotation_matrix(*quaternion)


def test_simulation_respects_joint_acceleration_limit():
    """The RViz simulation must ramp velocity instead of jumping."""
    controller = object.__new__(PoseController)
    controller.simulated_joints = np.zeros(7)
    controller.simulated_target_joints = np.ones(7)
    controller.simulated_velocities = np.zeros(7)
    controller.joint_max_acceleration = 1.0
    controller.joint_max_velocity = 1.0
    controller.state_period = 0.1

    controller._advance_simulation()
    assert controller.simulated_velocities == pytest.approx(np.full(7, 0.1))
    assert controller.simulated_joints == pytest.approx(np.full(7, 0.01))

    controller._advance_simulation()
    assert controller.simulated_velocities == pytest.approx(np.full(7, 0.2))
    assert controller.simulated_joints == pytest.approx(np.full(7, 0.03))


def test_valid_history_keeps_only_latest_ten_steps():
    """Recovery history must remain bounded and retain the latest state."""
    controller = object.__new__(PoseController)
    controller.valid_history = deque(maxlen=10)
    rotation = np.eye(3)
    for index in range(12):
        values = np.full(7, float(index))
        controller._remember_valid(values[:3], rotation, values)

    assert len(controller.valid_history) == 10
    assert controller.valid_history[0][2] == pytest.approx(np.full(7, 2.0))
    assert controller.valid_history[-1][2] == pytest.approx(np.full(7, 11.0))


def test_radial_workspace_uses_requested_safety_margins():
    """Workspace permits only min+5 cm through max-10 cm."""
    minimum = 0.1447354 + 0.05
    maximum = 0.7374482 - 0.10

    assert radial_workspace_check([minimum, 0.0, 0.0], minimum, maximum)[0]
    assert radial_workspace_check([maximum, 0.0, 0.0], minimum, maximum)[0]
    assert not radial_workspace_check(
        [minimum - 0.001, 0.0, 0.0], minimum, maximum
    )[0]
    assert not radial_workspace_check(
        [maximum + 0.001, 0.0, 0.0], minimum, maximum
    )[0]


def test_pointing_orientation_maps_tool_z_to_requested_direction():
    """The command orientation points link7 +Z down in base_link."""
    quaternion = make_pointing_quaternion(
        [0.0, 0.0, -2.0], [1.0, 0.0, 0.0]
    )
    rotation = quaternion_to_rotation_matrix(*quaternion)

    assert rotation[:, 2] == pytest.approx([0.0, 0.0, -1.0])
    assert rotation.T @ rotation == pytest.approx(np.eye(3))


def test_pointing_orientation_handles_parallel_roll_reference():
    """A parallel reference must select a stable fallback axis."""
    quaternion = make_pointing_quaternion(
        [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]
    )
    rotation = quaternion_to_rotation_matrix(*quaternion)

    assert rotation[:, 2] == pytest.approx([1.0, 0.0, 0.0])


def test_orientation_arrows_point_up_and_left_in_base_frame():
    # Tool +Z initially points base +X.
    initial = np.array([[0.0, 0.0, 1.0],
                        [0.0, 1.0, 0.0],
                        [-1.0, 0.0, 0.0]])
    step = 0.1

    upward = increment_tool_orientation(initial, -step, 0.0, 0.0)
    leftward = increment_tool_orientation(initial, 0.0, step, 0.0)
    rolled = increment_tool_orientation(initial, 0.0, 0.0, step)

    assert upward[:, 2][2] > initial[:, 2][2]
    assert leftward[:, 2][1] > initial[:, 2][1]
    assert rolled[:, 2] == pytest.approx(initial[:, 2])
    assert np.linalg.norm(rolled[:, 0] - initial[:, 0]) > 0.05


def test_pointing_error_ignores_rotation_about_tool_axis():
    target = np.eye(3)
    rolled = np.array([[0.0, -1.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0]])

    assert pointing_error_angle(rolled, target) == pytest.approx(0.0)
    assert rotation_error_angle(rolled, target) == pytest.approx(math.pi / 2)


def test_pointing_ik_selects_roll_solution_nearest_seed():
    class FakeSolver:
        def ik(self, position, rotation, seed_jnt_values):
            del position, seed_jnt_values
            angle = math.atan2(rotation[1, 0], rotation[0, 0])
            return np.array([angle, 0.0])

    solution = solve_pointing_ik(
        FakeSolver(), np.zeros(3), np.eye(3), np.array([1.4, 0.0]), 4
    )

    assert solution == pytest.approx([math.pi / 2, 0.0])


def test_shared_engine_solves_and_fk_verifies_pointing_target():
    class ExactSolver:
        def ik(self, position, rotation, seed_jnt_values):
            del rotation, seed_jnt_values
            return np.asarray(position[:2], dtype=float)

        def fk(self, joints):
            return np.array([joints[0], joints[1], 0.2]), np.eye(3)

    engine = AgxIkEngine(
        ExactSolver(), 2, 0.1, 1.0, 1e-9, 1e-9, roll_samples=1
    )
    result = engine.solve(
        np.array([0.3, 0.1, 0.2]), np.eye(3), np.zeros(2)
    )

    assert result.joints == pytest.approx([0.3, 0.1])
    assert result.position_error == pytest.approx(0.0)
    assert result.orientation_error == pytest.approx(0.0)


def test_shared_engine_rejects_workspace_before_calling_solver():
    class UnusedSolver:
        def ik(self, *args, **kwargs):
            raise AssertionError("IK must not run outside workspace")

    engine = AgxIkEngine(
        UnusedSolver(), 2, 0.2, 0.5, 1e-4, 1e-3
    )

    with pytest.raises(IkFailure, match="workspace radius"):
        engine.solve(np.array([0.6, 0.0, 0.0]), np.eye(3), np.zeros(2))


LIMITS = [(-1.0, 1.0)] * 7


def test_auto_firmware_matches_verified_hardware_versions():
    assert resolve_firmware_name("nero", "auto") == "v112"
    assert resolve_firmware_name("piper_l", "auto") == "v188"
    assert resolve_firmware_name("piper_l", "v189") == "v189"


def keys(*pressed):
    state = [0] * KEY_COUNT
    for index in pressed:
        state[index] = 1
    return state


def test_selects_joint_and_only_jogs_that_joint():
    jog = ArmJointJogState(LIMITS, 0.1)
    update = jog.update(keys(3, KEY_INCREASE))
    assert update.selected_joint == 3
    assert update.selection_changed and update.target_changed
    assert jog.target_joints == pytest.approx(
        [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
    )


def test_holding_joint_key_clamps_at_limit():
    jog = ArmJointJogState(LIMITS, 0.6)
    jog.update(keys(KEY_INCREASE))
    jog.update(keys(KEY_INCREASE))
    update = jog.update(keys(KEY_INCREASE))
    assert jog.target_joints[0] == 1.0
    assert not update.target_changed


def test_opposite_joint_keys_cancel():
    jog = ArmJointJogState(LIMITS, 0.1)
    assert not jog.update(keys(KEY_DECREASE, KEY_INCREASE)).target_changed


def test_home_is_edge_triggered():
    jog = ArmJointJogState(LIMITS, 0.1, [0.2] * 7)
    first = jog.update(keys(KEY_HOME))
    second = jog.update(keys(KEY_HOME))
    assert first.home_requested and first.target_changed
    assert not second.home_requested
    assert jog.target_joints == [0.0] * 7


def test_estop_and_mode_toggle_are_edge_triggered_and_suppress_jog():
    jog = ArmJointJogState(LIMITS, 0.1)
    first = jog.update(keys(KEY_ESTOP, KEY_INCREASE))
    second = jog.update(keys(KEY_ESTOP))
    assert first.estop_requested and not first.target_changed
    assert not second.estop_requested

    jog.update(keys())
    first = jog.update(keys(KEY_MODE_TOGGLE, KEY_INCREASE))
    second = jog.update(keys(KEY_MODE_TOGGLE))
    assert first.mode_toggle_requested and not first.target_changed
    assert not second.mode_toggle_requested


def test_sync_target_clamps_target_by_default():
    jog = ArmJointJogState(LIMITS, math.radians(1.0))
    jog.sync_target([-2.0, -0.5, 0.0, 0.5, 2.0, 0.1, -0.1])
    assert jog.target_joints == pytest.approx(
        [-1.0, -0.5, 0.0, 0.5, 1.0, 0.1, -0.1]
    )


def test_out_of_limit_feedback_recovers_one_step_without_jump():
    jog = ArmJointJogState(LIMITS, 0.1)
    jog.sync_target([1.2, -1.2, 0.0, 0.0, 0.0, 0.0, 0.0],
                    clamp_to_limits=False)

    blocked = jog.update(keys(KEY_INCREASE))
    jog.update(keys())
    inward = jog.update(keys(KEY_DECREASE))

    assert not blocked.target_changed
    assert inward.target_changed
    assert jog.target_joints[0] == pytest.approx(1.1)
    assert jog.target_joints[1] == pytest.approx(-1.2)


def test_piper_l_uses_same_keys_but_ignores_joint_seven():
    jog = ArmJointJogState([(-1.0, 1.0)] * 6, 0.1)

    ignored = jog.update(keys(6, KEY_INCREASE))
    jog.update(keys())
    selected = jog.update(keys(5, KEY_INCREASE))

    assert ignored.selected_joint == 0
    assert jog.target_joints[0] == pytest.approx(0.1)
    assert selected.selected_joint == 5
    assert jog.target_joints[5] == pytest.approx(0.1)


def test_impedance_toggle_is_edge_triggered_and_suppresses_jog():
    jog = ArmJointJogState(LIMITS, 0.1)
    first = jog.update(keys(KEY_IMPEDANCE_TOGGLE, KEY_INCREASE))
    second = jog.update(keys(KEY_IMPEDANCE_TOGGLE))

    assert first.impedance_toggle_requested
    assert not first.target_changed
    assert not second.impedance_toggle_requested


def test_mit_gains_expand_from_scalar_and_validate_joint_count():
    assert expand_joint_values([0.8], 6, "kd") == [0.8] * 6
    with pytest.raises(ValueError, match="1 or 6"):
        expand_joint_values([1.0, 2.0], 6, "kp")


def test_piper_mit_gains_are_independent_per_joint():
    kp, kd = default_mit_gains("piper_l")

    assert kp == [0.3, 0.5, 0.5, 0.5, 1.0, 0.3]
    assert kd == [0.01] * 6
    assert default_mit_feedforward("piper_l") == [
        0.0, 3.0, -3.5, 0.0, -1.0, 0.0
    ]
    assert len(set(kp)) > 1


def test_velocity_damping_is_soft_at_low_speed_and_bounded_at_high_speed():
    velocities = np.array([1e-6, 0.3, 100.0])
    effective_kd = bounded_velocity_damping(
        velocities,
        damping_min=[0.2] * 3,
        damping_max=[0.6] * 3,
        transition_velocity=[0.3] * 3,
        torque_limit=[1.0] * 3,
    )
    damping_torque = effective_kd * velocities

    assert effective_kd[0] == pytest.approx(0.2, rel=1e-6)
    assert damping_torque[1] > damping_torque[0]
    assert damping_torque[2] == pytest.approx(1.0)
    assert np.all(damping_torque <= 1.0)


def test_mit_handover_starts_stiff_and_smoothly_reaches_soft_kp():
    assert blend_handover_kp([0.4], [10.0], 0.0, 0.5) == [10.0]
    assert blend_handover_kp([0.4], [10.0], 0.25, 0.5) == pytest.approx([5.2])
    assert blend_handover_kp([0.4], [10.0], 0.5, 0.5) == pytest.approx([0.4])


def test_mit_tick_sends_one_impedance_command_per_joint():
    class FakeArm:
        def __init__(self):
            self.commands = []

        def move_mit(self, **command):
            self.commands.append(command)

        def get_motor_states(self, joint_index):
            del joint_index
            return SimpleNamespace(msg=SimpleNamespace(velocity=0.3))

    controller = object.__new__(ArmKeyboardController)
    controller.impedance_enabled = True
    controller.emergency_stopped = False
    controller.execute_motion = True
    controller.arm_ready = True
    controller.arm = FakeArm()
    controller.joint_count = 6
    controller.jog = ArmJointJogState([(-1.0, 1.0)] * 6, 0.1)
    controller.mit_kp = [10.0] * 6
    controller.mit_handover_kp = [10.0] * 6
    controller.mit_handover_started = float("-inf")
    controller.mit_handover_duration = 0.5
    controller.mit_kd = [0.8] * 6
    controller.mit_kd_max = [1.0] * 6
    controller.mit_damping_transition_velocity = [0.3] * 6
    controller.mit_damping_torque_limit = [1.0] * 6
    controller.mit_feedforward = [0.0] * 6

    controller.mit_tick()

    assert [item["joint_index"] for item in controller.arm.commands] == [
        1, 2, 3, 4, 5, 6
    ]
    assert all(item["kp"] == 10.0 for item in controller.arm.commands)
    assert all(0.8 < item["kd"] < 1.0 for item in controller.arm.commands)


def test_ik_joint_target_is_consumed_only_by_mit_backend(monkeypatch):
    class FakeArm:
        def __init__(self):
            self.mit_commands = []
            self.move_j_commands = []

        def move_mit(self, **command):
            self.mit_commands.append(command)

        def get_motor_states(self, joint_index):
            del joint_index
            return SimpleNamespace(msg=SimpleNamespace(velocity=0.0))

        def move_j(self, target):
            self.move_j_commands.append(target)

    class FakeLogger:
        def info(self, message, **kwargs):
            del message, kwargs

        def warning(self, message, **kwargs):
            del message, kwargs

        def error(self, message, **kwargs):
            del message, kwargs

    controller = object.__new__(ArmKeyboardController)
    controller.impedance_enabled = True
    controller.emergency_stopped = False
    controller.execute_motion = True
    controller.arm_ready = True
    controller.arm = FakeArm()
    controller.joint_count = 6
    controller.jog = ArmJointJogState([(-1.0, 1.0)] * 6, 0.1)
    controller.jog.sync_target([0.1, -0.2, 0.3, 0.0, 0.2, -0.1])
    controller.mit_kp = [1.0] * 6
    controller.mit_handover_kp = [10.0] * 6
    controller.mit_handover_started = float("-inf")
    controller.mit_handover_duration = 0.5
    controller.mit_kd = [0.2] * 6
    controller.mit_kd_max = [0.6] * 6
    controller.mit_damping_transition_velocity = [0.3] * 6
    controller.mit_damping_torque_limit = [1.0] * 6
    controller.mit_feedforward = [0.0] * 6
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )

    controller.send_target("IK pose jog")
    controller.mit_tick()

    assert controller.arm.move_j_commands == []
    assert [command["p_des"] for command in controller.arm.mit_commands] == (
        pytest.approx(controller.jog.target_joints)
    )


def test_startup_reset_clears_latched_emergency_stop(monkeypatch):
    class FakeArm:
        def __init__(self):
            self.reset_calls = 0
            self.emergency_stopped = True

        def reset(self):
            self.reset_calls += 1
            self.emergency_stopped = False

        def get_arm_status(self):
            state = "EMERGENCY_STOP" if self.emergency_stopped else "NORMAL"
            return SimpleNamespace(msg=SimpleNamespace(arm_status=state))

    class FakeLogger:
        def info(self, message):
            del message

        def warning(self, message):
            del message

    controller = object.__new__(ArmKeyboardController)
    controller.arm = FakeArm()
    controller.emergency_reset_timeout = 0.1
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )

    assert controller.reset_emergency_stop()
    assert controller.arm.reset_calls == 1


def test_startup_home_zeros_joints_strictly_in_order(monkeypatch):
    class FakeArm:
        def __init__(self):
            self.joints = [1.0, 2.0, 3.0]
            self.commands = []
            self.limit_changes = []

        def get_joint_angles(self):
            return list(self.joints)

        def move_j(self, target):
            self.commands.append(list(target))
            self.joints = list(target)

        def get_joint_limits_enabled(self):
            return True

        def set_joint_limits_enabled(self, enabled):
            self.limit_changes.append(enabled)

    class FakeLogger:
        def info(self, message, **kwargs):
            del message, kwargs

        def error(self, message, **kwargs):
            del message, kwargs

    controller = object.__new__(ArmKeyboardController)
    controller.arm = FakeArm()
    controller.joint_count = 3
    controller.startup_home_timeout = 0.1
    controller.startup_home_tolerance = 0.01
    controller.jog = ArmJointJogState([(-4.0, 4.0)] * 3, 0.1)
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )

    assert controller.move_home_and_wait()
    assert controller.arm.commands == [
        [0.0, 2.0, 3.0],
        [0.0, 0.0, 3.0],
        [0.0, 0.0, 0.0],
    ]
    assert controller.arm.limit_changes == [False, True]


def test_acceleration_limits_are_set_per_joint_not_with_255():
    class FakeArm:
        def __init__(self):
            self.calls = []

        def set_joint_acc_limits(self, **kwargs):
            self.calls.append(kwargs)
            return kwargs["joint_index"] != 255

    arm = FakeArm()
    result = set_joint_acceleration_limits(
        arm, 6, max_joint_acceleration=1.0, timeout=2.0
    )

    assert result == (True, None)
    assert [call["joint_index"] for call in arm.calls] == [
        1, 2, 3, 4, 5, 6
    ]
    assert all(call["timeout"] == 2.0 for call in arm.calls)


def test_position_mode_is_confirmed_before_startup_motion():
    class FakeArm:
        OPTIONS = SimpleNamespace(
            MOTION_MODE=SimpleNamespace(J="j")
        )

        def __init__(self):
            self.requested_modes = []

        def set_motion_mode(self, mode):
            self.requested_modes.append(mode)

        def get_arm_status(self):
            return SimpleNamespace(msg=SimpleNamespace(
                ctrl_mode="CAN_CTRL", mode_feedback="MOVE_J"
            ))

    arm = FakeArm()

    assert prepare_planned_joint_mode(arm, timeout=0.1, poll_period=0.0)
    assert arm.requested_modes == ["j"]


def test_explicit_emergency_stop_reset_waits_for_normal_status():
    class Status:
        def __init__(self, value):
            self.msg = type("Message", (), {"arm_status": value})()

    class FakeArm:
        def __init__(self):
            self.reset_called = False
            self.statuses = [Status("EMERGENCY_STOP"), Status("NORMAL")]

        def reset(self):
            self.reset_called = True

        def get_arm_status(self):
            return self.statuses.pop(0)

    class FakeLogger:
        def warning(self, message):
            del message

        def info(self, message):
            del message

    controller = object.__new__(ArmKeyboardController)
    controller.arm = FakeArm()
    controller.emergency_reset_timeout = 1.0
    controller.get_logger = lambda: FakeLogger()

    assert controller.reset_emergency_stop()
    assert controller.arm.reset_called


def test_revo2_normalized_mode_compatibility():
    from pyAgxArm.protocols.can_protocol.drivers.effector.revo2_touch import (
        default,
    )
    from armbycontroller.hand_controller import resolve_normalized_unit_mode

    mode = resolve_normalized_unit_mode(default.driver.Driver)
    assert mode is not None
    assert str(mode).endswith("Normalized")
