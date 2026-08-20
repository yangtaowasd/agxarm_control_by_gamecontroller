"""Safety contracts for bounded observer-based friction assistance."""

import numpy as np
import pytest

from armbycontroller.observers import ObserverFrictionAssist


def assist():
    return ObserverFrictionAssist(
        3,
        gain=[0.85, 0.85, 0.85],
        torque_limit=[0.3, 0.2, 0.1],
        velocity_threshold=[0.08, 0.08, 0.08],
        restoring_torque_threshold=[0.03, 0.03, 0.03],
    )


def test_assist_cancels_only_bounded_opposing_low_speed_residual():
    torque = assist().evaluate(
        restoring_torque=[0.2, -0.2, 0.2],
        measured_velocity=[0.0, 0.0, 0.0],
        external_torque=[-0.1, 0.5, 0.5],
        observation_valid=True,
    )

    assert torque == pytest.approx([0.085, -0.2, 0.0])


def test_assist_exits_when_moving_or_observation_is_invalid():
    moving = assist().evaluate(
        [0.2, 0.2, 0.2],
        [0.08, 0.09, 0.2],
        [-1.0, -1.0, -1.0],
        observation_valid=True,
    )
    stale = assist().evaluate(
        [0.2, 0.2, 0.2],
        np.zeros(3),
        [-1.0, -1.0, -1.0],
        observation_valid=False,
    )

    assert moving == pytest.approx(np.zeros(3))
    assert stale == pytest.approx(np.zeros(3))


def test_assist_never_creates_motion_without_restoring_torque():
    torque = assist().evaluate(
        [0.0, 0.02, -0.02],
        np.zeros(3),
        [-1.0, -1.0, 1.0],
        observation_valid=True,
    )

    assert torque == pytest.approx(np.zeros(3))


@pytest.mark.parametrize("gain", [1.0, 1.01])
def test_assist_rejects_full_or_over_compensation(gain):
    with pytest.raises(ValueError, match=r"gain values must be in \[0, 1\)"):
        ObserverFrictionAssist(1, gain, 0.1, 0.08, 0.03)
