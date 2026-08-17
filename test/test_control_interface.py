"""Contracts shared by every controller adapter and experiment consumer."""

from types import SimpleNamespace

import numpy as np
import pytest

from armbycontroller.admittance.controller import CartesianAdmittanceController
from armbycontroller.control import ControlEngine
from armbycontroller.control import ControlInput
from armbycontroller.control import ControlReference
from armbycontroller.control import ControlSafetyError
from armbycontroller.control import ControlState
from armbycontroller.control import MitCommand
from armbycontroller.control import PositionCommand
from armbycontroller.control import control_sample
from armbycontroller.impedance.controllers import CartesianImpedanceController
from armbycontroller.impedance.controllers import JointMitController


class IdentityModel:
    joint_count = 6

    def forward_kinematics(self, joints):
        pose = np.eye(4)
        pose[:3, 3] = np.asarray(joints)[3:6]
        return pose

    def space_jacobian(self, joints):
        del joints
        return np.eye(6)

    def inverse_dynamics(self, position, velocity, acceleration):
        del position, velocity, acceleration
        return np.arange(1.0, 7.0)


def sample(position=None, target=None, wrench=None):
    position = np.zeros(6) if position is None else np.asarray(position)
    target = np.zeros(6) if target is None else np.asarray(target)
    return ControlInput(
        12.5,
        0.01,
        ControlState(position, np.zeros(6), np.zeros(6)),
        ControlReference(
            target, np.zeros(6), np.zeros(6),
            np.zeros(6) if wrench is None else wrench,
        ),
    )


def test_engine_runs_joint_mit_through_one_interface():
    controller = JointMitController(
        6,
        kp=np.ones(6) * 2.0,
        kd=np.ones(6) * 0.2,
        feedforward=np.zeros(6),
        torque_limit=np.ones(6) * 5.0,
        dynamics_model=IdentityModel(),
    )
    engine = ControlEngine([controller])
    control_input = sample(target=np.ones(6))

    result = engine.step("joint_impedance", control_input)

    assert isinstance(result.command, MitCommand)
    assert result.command.position == pytest.approx(np.ones(6))
    assert result.command.estimated_torque == pytest.approx(
        [3.0, 4.0, 5.0, 5.0, 5.0, 5.0]
    )
    assert result.command.feedforward == pytest.approx(
        [1.0, 2.0, 3.0, 3.0, 3.0, 3.0]
    )


def test_cartesian_adapter_produces_zero_gain_mit_torque():
    controller = CartesianImpedanceController(
        IdentityModel(),
        stiffness=[0.0, 0.0, 0.0, 10.0, 0.0, 0.0],
        damping=np.zeros(6),
        torque_limit=np.ones(6) * 8.0,
    )
    result = controller.step(sample(target=[0, 0, 0, 2, 0, 0]))

    assert isinstance(result.command, MitCommand)
    assert result.command.kp == pytest.approx(np.zeros(6))
    assert result.command.kd == pytest.approx(np.zeros(6))
    # 20 N.m task request plus model support is clipped by the adapter.
    assert result.command.feedforward == pytest.approx([1, 2, 3, 8, 5, 6])
    assert result.signals["torque_clipped"]


def test_cartesian_adapter_applies_configured_model_scale():
    controller = CartesianImpedanceController(
        IdentityModel(),
        stiffness=np.zeros(6),
        damping=np.zeros(6),
        torque_limit=np.ones(6) * 8.0,
        model_scale=[1.0, 0.5, 0.0, 1.0, 1.0, 1.0],
    )

    result = controller.step(sample())

    assert result.signals["model_torque"] == pytest.approx(
        [1.0, 1.0, 0.0, 4.0, 5.0, 6.0]
    )


def test_cartesian_adapter_adds_unprojected_joint_posture_impedance():
    controller = CartesianImpedanceController(
        IdentityModel(),
        stiffness=np.zeros(6),
        damping=np.zeros(6),
        torque_limit=np.ones(6) * 8.0,
        joint_posture_stiffness=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        joint_posture_damping=[0.0, 0.0, 0.0, 0.2, 0.0, 0.0],
    )
    control_input = ControlInput(
        12.5,
        0.01,
        ControlState(
            [0.0, 0.0, 0.0, 0.4, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.5, 0.0, 0.0],
            np.zeros(6),
        ),
        ControlReference.hold(np.zeros(6)),
    )

    result = controller.step(control_input)

    assert result.signals["task_torque"] == pytest.approx(np.zeros(6))
    assert result.signals["joint_posture_torque"] == pytest.approx(
        [0.0, 0.0, 0.0, -0.5, 0.0, 0.0]
    )
    assert result.signals["raw_command_torque"] == pytest.approx(
        [1.0, 2.0, 3.0, 3.5, 5.0, 6.0]
    )
    assert result.command.feedforward == pytest.approx(
        [1.0, 2.0, 3.0, 3.5, 5.0, 6.0]
    )
    assert result.command.kp == pytest.approx(np.zeros(6))
    assert result.command.kd == pytest.approx(np.zeros(6))


def test_cartesian_adapter_limits_joint_posture_with_total_torque():
    controller = CartesianImpedanceController(
        IdentityModel(),
        stiffness=np.zeros(6),
        damping=np.zeros(6),
        torque_limit=np.ones(6) * 8.0,
        joint_posture_stiffness=[0.0, 0.0, 0.0, 100.0, 0.0, 0.0],
    )

    result = controller.step(
        sample(position=[0.0, 0.0, 0.0, 0.4, 0.0, 0.0])
    )

    assert result.signals["joint_posture_torque"][3] == pytest.approx(-40.0)
    assert result.signals["raw_command_torque"][3] == pytest.approx(-36.0)
    assert result.command.feedforward[3] == pytest.approx(-8.0)
    assert result.signals["torque_clipped"]


def test_cartesian_adapter_reuses_unchanged_reference_kinematics():
    class CountingModel(IdentityModel):
        def __init__(self):
            self.fk_calls = 0
            self.jacobian_calls = 0

        def forward_kinematics(self, joints):
            self.fk_calls += 1
            return super().forward_kinematics(joints)

        def space_jacobian(self, joints):
            self.jacobian_calls += 1
            return super().space_jacobian(joints)

    model = CountingModel()
    controller = CartesianImpedanceController(
        model,
        stiffness=np.ones(6),
        damping=np.ones(6),
        torque_limit=np.ones(6) * 8.0,
    )
    held = sample(target=[0, 0, 0, 0.2, 0, 0])

    first = controller.step(held)
    second = controller.step(held)

    assert second.command.feedforward == pytest.approx(
        first.command.feedforward
    )
    assert model.fk_calls == 3
    assert model.jacobian_calls == 3

    controller.step(sample(target=[0, 0, 0, 0.3, 0, 0]))

    assert model.fk_calls == 5
    assert model.jacobian_calls == 5


def test_admittance_adapter_produces_checked_position_command():
    class FakeAdmittance:
        def reset(self, pose):
            self.pose = pose

        def step(self, wrench, period):
            pose = np.eye(4)
            pose[0, 3] = float(wrench[3]) * period
            return SimpleNamespace(
                mode="zero_force",
                offset=np.asarray([0, 0, 0, pose[0, 3], 0, 0]),
                velocity=np.zeros(6),
                acceleration=np.zeros(6),
                applied_wrench=np.asarray(wrench),
                resisting_wrench=np.asarray(wrench) * 0.25,
                desired_pose=pose,
                desired_twist=np.zeros(6),
            )

    class FakeIk:
        def solve(self, position, rotation, seed):
            del rotation
            joints = np.asarray(seed).copy()
            joints[0] += position[0]
            return SimpleNamespace(joints=joints)

    controller = CartesianAdmittanceController(
        IdentityModel(), FakeAdmittance(), FakeIk(), max_joint_step=0.05
    )
    initial = sample().state
    controller.reset(initial)
    result = controller.step(sample(wrench=[0, 0, 0, 2, 0, 0]))

    assert isinstance(result.command, PositionCommand)
    assert result.command.position[0] == pytest.approx(0.02)
    assert result.signals["applied_wrench"] == pytest.approx(
        [0, 0, 0, 2, 0, 0]
    )
    assert result.signals["admittance_mode"] == "zero_force"
    assert result.signals["resisting_wrench"] == pytest.approx(
        [0, 0, 0, 0.5, 0, 0]
    )


def test_admittance_adapter_rejects_large_ik_step():
    class FakeAdmittance:
        def step(self, wrench, period):
            del wrench, period
            return SimpleNamespace(desired_pose=np.eye(4))

    class FakeIk:
        def solve(self, position, rotation, seed):
            del position, rotation, seed
            return SimpleNamespace(joints=np.ones(6))

    controller = CartesianAdmittanceController(
        IdentityModel(), FakeAdmittance(), FakeIk(), max_joint_step=0.05
    )
    with pytest.raises(ControlSafetyError, match="joint step"):
        controller.step(sample())


def test_control_sample_is_stable_json_compatible_schema():
    control_input = sample(target=np.ones(6))
    result = JointMitController(
        6, np.ones(6), np.zeros(6), np.zeros(6), np.ones(6) * 10.0
    ).step(control_input)

    value = control_sample(
        control_input,
        result,
        robot_model="piper_l",
        interaction_mode="impedance",
    )

    assert value["schema_version"] == 1
    assert value["controller"] == "joint_impedance"
    assert value["command"]["mode"] == "mit"
    assert value["state"]["position"] == [0.0] * 6
    assert value["reference"]["position"] == [1.0] * 6
