"""Tests for the Nero pytracik controller's pure math helpers."""

from collections import deque
import math

import numpy as np
import pytest

from armbycontroller.agx_ik import AgxIkEngine
from armbycontroller.agx_ik import IkFailure
from armbycontroller.agx_ik import pointing_error_angle
from armbycontroller.agx_ik import quaternion_to_rotation_matrix
from armbycontroller.agx_ik import radial_workspace_check
from armbycontroller.agx_ik import rotation_error_angle
from armbycontroller.agx_ik import rotation_matrix_to_quaternion
from armbycontroller.agx_ik import solve_pointing_ik
from armbycontroller.nero_ik_controller import NeroIkController
from armbycontroller.nero_joint_keyboard_controller import KEY_COUNT
from armbycontroller.nero_joint_keyboard_controller import KEY_DECREASE
from armbycontroller.nero_joint_keyboard_controller import KEY_ESTOP
from armbycontroller.nero_joint_keyboard_controller import KEY_HOME
from armbycontroller.nero_joint_keyboard_controller import KEY_INCREASE
from armbycontroller.nero_joint_keyboard_controller import KEY_MODE_TOGGLE
from armbycontroller.nero_joint_keyboard_controller import NeroJointJogState
from armbycontroller.nero_keyboard_teleop import make_pointing_quaternion


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
    controller = object.__new__(NeroIkController)
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
    controller = object.__new__(NeroIkController)
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


def keys(*pressed):
    state = [0] * KEY_COUNT
    for index in pressed:
        state[index] = 1
    return state


def test_selects_joint_and_only_jogs_that_joint():
    jog = NeroJointJogState(LIMITS, 0.1)
    update = jog.update(keys(3, KEY_INCREASE))
    assert update.selected_joint == 3
    assert update.selection_changed and update.target_changed
    assert jog.target_joints == pytest.approx(
        [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
    )


def test_holding_joint_key_clamps_at_limit():
    jog = NeroJointJogState(LIMITS, 0.6)
    jog.update(keys(KEY_INCREASE))
    jog.update(keys(KEY_INCREASE))
    update = jog.update(keys(KEY_INCREASE))
    assert jog.target_joints[0] == 1.0
    assert not update.target_changed


def test_opposite_joint_keys_cancel():
    jog = NeroJointJogState(LIMITS, 0.1)
    assert not jog.update(keys(KEY_DECREASE, KEY_INCREASE)).target_changed


def test_home_is_edge_triggered():
    jog = NeroJointJogState(LIMITS, 0.1, [0.2] * 7)
    first = jog.update(keys(KEY_HOME))
    second = jog.update(keys(KEY_HOME))
    assert first.home_requested and first.target_changed
    assert not second.home_requested
    assert jog.target_joints == [0.0] * 7


def test_estop_and_mode_toggle_are_edge_triggered_and_suppress_jog():
    jog = NeroJointJogState(LIMITS, 0.1)
    first = jog.update(keys(KEY_ESTOP, KEY_INCREASE))
    second = jog.update(keys(KEY_ESTOP))
    assert first.estop_requested and not first.target_changed
    assert not second.estop_requested

    jog.update(keys())
    first = jog.update(keys(KEY_MODE_TOGGLE, KEY_INCREASE))
    second = jog.update(keys(KEY_MODE_TOGGLE))
    assert first.mode_toggle_requested and not first.target_changed
    assert not second.mode_toggle_requested


def test_sync_target_clamps_joint_feedback():
    jog = NeroJointJogState(LIMITS, math.radians(1.0))
    jog.sync_target([-2.0, -0.5, 0.0, 0.5, 2.0, 0.1, -0.1])
    assert jog.target_joints == pytest.approx(
        [-1.0, -0.5, 0.0, 0.5, 1.0, 0.1, -0.1]
    )


def test_revo2_normalized_mode_compatibility():
    from pyAgxArm.protocols.can_protocol.drivers.effector.revo2_touch import (
        default,
    )
    from armbycontroller.revo2_hand_test import resolve_normalized_unit_mode

    mode = resolve_normalized_unit_mode(default.driver.Driver)
    assert mode is not None
    assert str(mode).endswith("Normalized")
