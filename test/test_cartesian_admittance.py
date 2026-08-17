"""Formula and state contracts for Cartesian admittance control."""

import numpy as np
import pytest

from armbycontroller.admittance import ResistiveAdmittance
from armbycontroller.admittance import ZeroForceAdmittance
from armbycontroller.admittance import create_cartesian_admittance


COMMON_SETTINGS = {
    "virtual_mass": [1.0] * 6,
    "wrench_deadband": [0.0] * 6,
    "wrench_limit": [100.0] * 6,
    "offset_limit": [1.0] * 6,
    "velocity_limit": [10.0] * 6,
    "wrench_filter_hz": 0.0,
}
ZERO_FORCE_SETTINGS = {
    "damping": [4.0] * 6,
    "holding_stiffness": [0.1] * 6,
    "friction": [0.25] * 6,
    "stiction_velocity": [0.01] * 6,
}


def test_admittance_constant_force_converges_to_spring_equilibrium():
    controller = ResistiveAdmittance(
        damping=[4.0] * 6,
        stiffness=[10.0] * 6,
        **COMMON_SETTINGS,
    )
    controller.reset(np.eye(4))
    state = None
    for _ in range(1000):
        state = controller.step([0.0, 0.0, 0.0, 1.0, 0.0, 0.0], 0.01)

    assert state.offset[:3] == pytest.approx(np.zeros(3), abs=1e-9)
    assert state.offset[3] == pytest.approx(0.1, abs=1e-5)
    assert state.desired_pose[0, 3] == pytest.approx(0.1, abs=1e-5)
    assert state.mode == "resistive"


def test_soft_zero_force_moves_under_contact_then_sticks_after_release():
    controller = ZeroForceAdmittance(
        **ZERO_FORCE_SETTINGS,
        **COMMON_SETTINGS,
    )
    controller.reset(np.eye(4))
    for _ in range(100):
        controller.step([0.0, 0.0, 0.0, 1.0, 0.0, 0.0], 0.01)
    released_at = controller.offset[3]
    state = None
    for _ in range(500):
        state = controller.step(np.zeros(6), 0.01)

    assert state.mode == "zero_force"
    assert state.velocity == pytest.approx(np.zeros(6), abs=1e-9)
    assert state.offset[3] > released_at
    assert state.offset[3] > 0.05


def test_zero_force_mode_rejects_small_static_bias_without_drift():
    controller = ZeroForceAdmittance(
        **ZERO_FORCE_SETTINGS,
        **COMMON_SETTINGS,
    )
    controller.reset(np.eye(4))
    for _ in range(5000):
        state = controller.step(
            [0.0, 0.0, 0.0, 0.2, 0.0, 0.0], 0.01
        )

    assert state.offset == pytest.approx(np.zeros(6), abs=1e-12)
    assert state.velocity == pytest.approx(np.zeros(6), abs=1e-12)
    assert state.resisting_wrench[3] == pytest.approx(0.2)


def test_nero_soft_zero_force_rejects_bias_but_yields_to_hand_contact():
    controller = ZeroForceAdmittance(
        virtual_mass=[0.2, 0.2, 0.2, 2.0, 2.0, 2.0],
        damping=[2.0, 2.0, 2.0, 18.0, 18.0, 18.0],
        holding_stiffness=[0.25, 0.25, 0.25, 5.0, 5.0, 5.0],
        friction=[0.03, 0.03, 0.03, 0.35, 0.35, 0.35],
        stiction_velocity=[0.015, 0.015, 0.015, 0.005, 0.005, 0.005],
        wrench_deadband=[0.05, 0.05, 0.05, 0.25, 0.25, 0.25],
        wrench_limit=[1.5, 1.5, 1.5, 6.0, 6.0, 6.0],
        offset_limit=[0.25, 0.25, 0.25, 0.08, 0.08, 0.08],
        velocity_limit=[0.35, 0.35, 0.35, 0.10, 0.10, 0.10],
        wrench_filter_hz=0.0,
    )
    controller.reset(np.eye(4))
    for _ in range(2000):
        quiet = controller.step(
            [0.0, 0.0, 0.0, 0.55, 0.0, 0.0], 0.01
        )
    for _ in range(100):
        contact = controller.step(
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0], 0.01
        )

    assert quiet.offset == pytest.approx(np.zeros(6), abs=1e-12)
    assert contact.offset[3] > 0.0
    assert contact.resisting_wrench[3] >= 0.35


def test_resistive_mode_returns_to_anchor_after_force_is_released():
    controller = ResistiveAdmittance(
        damping=[4.0] * 6,
        stiffness=[10.0] * 6,
        **COMMON_SETTINGS,
    )
    controller.reset(np.eye(4))
    for _ in range(100):
        controller.step([0.0, 0.0, 0.0, 1.0, 0.0, 0.0], 0.01)
    for _ in range(500):
        state = controller.step(np.zeros(6), 0.01)

    assert state.offset == pytest.approx(np.zeros(6), abs=1e-4)


def test_factory_selects_explicit_modes_and_zero_force_ignores_stiffness():
    zero_force = create_cartesian_admittance(
        "zero_force",
        zero_force_damping=[4.0] * 6,
        zero_force_holding_stiffness=[0.1] * 6,
        zero_force_friction=[0.25] * 6,
        zero_force_stiction_velocity=[0.01] * 6,
        resistive_damping=[4.0] * 6,
        resistive_stiffness=[10.0] * 6,
        **COMMON_SETTINGS,
    )
    resistive = create_cartesian_admittance(
        "resistive",
        zero_force_damping=[4.0] * 6,
        zero_force_holding_stiffness=[0.1] * 6,
        zero_force_friction=[0.25] * 6,
        zero_force_stiction_velocity=[0.01] * 6,
        resistive_damping=[4.0] * 6,
        resistive_stiffness=[10.0] * 6,
        **COMMON_SETTINGS,
    )

    assert isinstance(zero_force, ZeroForceAdmittance)
    assert zero_force.holding_stiffness == pytest.approx([0.1] * 6)
    assert isinstance(resistive, ResistiveAdmittance)
    with pytest.raises(ValueError, match="admittance_mode"):
        create_cartesian_admittance(
            "unknown",
            zero_force_damping=[4.0] * 6,
            zero_force_holding_stiffness=[0.1] * 6,
            zero_force_friction=[0.25] * 6,
            zero_force_stiction_velocity=[0.01] * 6,
            resistive_damping=[4.0] * 6,
            resistive_stiffness=[10.0] * 6,
            **COMMON_SETTINGS,
        )


def test_admittance_uses_rotation_vector_and_enforces_motion_bounds():
    controller = ResistiveAdmittance(
        virtual_mass=[0.1] * 6,
        damping=[0.01] * 6,
        stiffness=[0.01] * 6,
        wrench_deadband=[0.0] * 6,
        wrench_limit=[1.0] * 6,
        offset_limit=[0.2, 0.2, 0.2, 0.01, 0.01, 0.01],
        velocity_limit=[0.3, 0.3, 0.3, 0.02, 0.02, 0.02],
        wrench_filter_hz=0.0,
    )
    controller.reset(np.eye(4))
    state = None
    for _ in range(200):
        state = controller.step([1.0, 0.0, 0.0, 1.0, 0.0, 0.0], 0.01)

    assert state.offset == pytest.approx(
        [0.2, 0.0, 0.0, 0.01, 0.0, 0.0]
    )
    assert state.velocity[0] == 0.0
    assert state.velocity[3] == 0.0
    assert state.desired_pose[1, 1] == pytest.approx(np.cos(0.2))
    assert state.desired_pose[2, 1] == pytest.approx(np.sin(0.2))


def test_admittance_requires_reset_and_rejects_bad_period():
    controller = ResistiveAdmittance(
        virtual_mass=[1.0] * 6,
        damping=[1.0] * 6,
        stiffness=[1.0] * 6,
        wrench_deadband=[0.0] * 6,
        wrench_limit=[1.0] * 6,
        offset_limit=[1.0] * 6,
        velocity_limit=[1.0] * 6,
    )
    with pytest.raises(RuntimeError, match="reset"):
        controller.step(np.zeros(6), 0.01)
    controller.reset(np.eye(4))
    with pytest.raises(ValueError, match="period"):
        controller.step(np.zeros(6), 0.0)
