import math

import pytest

from armbycontroller.nero_joint_logic import (
    KEY_COUNT,
    KEY_DECREASE,
    KEY_ESTOP,
    KEY_HOME,
    KEY_INCREASE,
    NeroJointJogState,
)


LIMITS = [(-1.0, 1.0)] * 7


def keys(*pressed):
    state = [0] * KEY_COUNT
    for index in pressed:
        state[index] = 1
    return state


def test_selects_joint_and_only_jogs_that_joint():
    jog = NeroJointJogState(LIMITS, step_rad=0.1)

    update = jog.update(keys(3, KEY_INCREASE))

    assert update.selected_joint == 3
    assert update.selection_changed
    assert update.target_changed
    assert jog.target_joints == pytest.approx([0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0])


def test_holding_key_jogs_repeatedly_and_clamps_at_limit():
    jog = NeroJointJogState(LIMITS, step_rad=0.6)

    jog.update(keys(KEY_INCREASE))
    jog.update(keys(KEY_INCREASE))
    update = jog.update(keys(KEY_INCREASE))

    assert jog.target_joints[0] == 1.0
    assert not update.target_changed


def test_opposite_direction_keys_cancel_each_other():
    jog = NeroJointJogState(LIMITS, step_rad=0.1)

    update = jog.update(keys(KEY_DECREASE, KEY_INCREASE))

    assert not update.target_changed
    assert jog.target_joints == [0.0] * 7


def test_home_is_edge_triggered_and_returns_all_joints_to_zero():
    jog = NeroJointJogState(LIMITS, step_rad=0.1, initial_joints=[0.2] * 7)

    first = jog.update(keys(KEY_HOME))
    second = jog.update(keys(KEY_HOME))

    assert first.home_requested
    assert first.target_changed
    assert not second.home_requested
    assert jog.target_joints == [0.0] * 7


def test_estop_is_edge_triggered_and_suppresses_jog_on_same_tick():
    jog = NeroJointJogState(LIMITS, step_rad=0.1)

    first = jog.update(keys(KEY_ESTOP, KEY_INCREASE))
    second = jog.update(keys(KEY_ESTOP))

    assert first.estop_requested
    assert not first.target_changed
    assert not second.estop_requested


def test_sync_target_clamps_feedback_to_configured_limits():
    jog = NeroJointJogState(LIMITS, step_rad=math.radians(1.0))

    jog.sync_target([-2.0, -0.5, 0.0, 0.5, 2.0, 0.1, -0.1])

    assert jog.target_joints == pytest.approx([-1.0, -0.5, 0.0, 0.5, 1.0, 0.1, -0.1])
