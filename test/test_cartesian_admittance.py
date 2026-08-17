"""Formula and state contracts for Cartesian admittance control."""

import numpy as np
import pytest

from armbycontroller.impedance.admittance import CartesianAdmittance
from armbycontroller.impedance.admittance import estimate_cartesian_wrench


def test_joint_torque_maps_to_base_frame_tool_wrench():
    jacobian = np.diag([1.0, 2.0, 4.0, 0.5, 1.0, 2.0])
    expected = np.asarray([0.2, -0.4, 0.6, 1.0, -2.0, 3.0])
    joint_torque = jacobian.T @ expected

    actual = estimate_cartesian_wrench(
        jacobian, joint_torque, damping=1e-8
    )

    assert actual == pytest.approx(expected, abs=1e-12)


def test_admittance_constant_force_converges_to_spring_equilibrium():
    controller = CartesianAdmittance(
        virtual_mass=[1.0] * 6,
        damping=[4.0] * 6,
        stiffness=[10.0] * 6,
        wrench_deadband=[0.0] * 6,
        wrench_limit=[100.0] * 6,
        offset_limit=[1.0] * 6,
        velocity_limit=[10.0] * 6,
        wrench_filter_hz=0.0,
    )
    controller.reset(np.eye(4))
    state = None
    for _ in range(1000):
        state = controller.step([0.0, 0.0, 0.0, 1.0, 0.0, 0.0], 0.01)

    assert state.offset[:3] == pytest.approx(np.zeros(3), abs=1e-9)
    assert state.offset[3] == pytest.approx(0.1, abs=1e-5)
    assert state.desired_pose[0, 3] == pytest.approx(0.1, abs=1e-5)


def test_admittance_uses_rotation_vector_and_enforces_motion_bounds():
    controller = CartesianAdmittance(
        virtual_mass=[0.1] * 6,
        damping=[0.0] * 6,
        stiffness=[0.0] * 6,
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
    controller = CartesianAdmittance(
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
