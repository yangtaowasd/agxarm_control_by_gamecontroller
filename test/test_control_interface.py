"""Contracts shared by every controller adapter and experiment consumer."""

from types import SimpleNamespace

import numpy as np
import pytest

from armbycontroller.admittance.controller import (
    ADMITTANCE_MIT_TORQUE_LIMIT_MAX,
)
from armbycontroller.admittance.controller import CartesianAdmittanceController
from armbycontroller.control import ControlEngine
from armbycontroller.control import ControlCycleGuard
from armbycontroller.control import ControlInput
from armbycontroller.control import ControlReference
from armbycontroller.control import ControlSafetyError
from armbycontroller.control import ControlState
from armbycontroller.control import MitCommand
from armbycontroller.control import MitTorqueEnvelope
from armbycontroller.control import ModelCompensator
from armbycontroller.control import InteractionModeLifecycle
from armbycontroller.control import SustainedVelocityGuard
from armbycontroller.control import control_sample
from armbycontroller.impedance.controllers import CartesianImpedanceController
from armbycontroller.impedance.controllers import JointMitController
from armbycontroller.ik.screw import BoundedScrewVelocityIk


class IdentityModel:
    joint_count = 6
    joint_limits = np.asarray([[-1.0, 1.0]] * 6)

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


def test_shared_model_compensator_selects_gravity_bias_and_dynamics():
    class RecordingModel(IdentityModel):
        def __init__(self):
            self.inverse_calls = []

        def gravity_torque(self, position):
            self.gravity_position = np.asarray(position).copy()
            return np.ones(6) * 2.0

        def inverse_dynamics(self, position, velocity, acceleration):
            self.inverse_calls.append((
                np.asarray(position).copy(),
                np.asarray(velocity).copy(),
                np.asarray(acceleration).copy(),
            ))
            return np.ones(6) * 4.0

    model = RecordingModel()
    position = np.linspace(0.0, 0.5, 6)
    velocity = np.linspace(0.1, 0.6, 6)
    acceleration = np.linspace(0.2, 0.7, 6)

    gravity = ModelCompensator(
        model, 6, "gravity", scale=0.5
    ).evaluate(position)
    bias = ModelCompensator(model, 6, "bias").evaluate(
        position, velocity
    )
    dynamics = ModelCompensator(
        model, 6, "inverse_dynamics"
    ).evaluate(position, velocity, acceleration)

    assert gravity.raw_torque == pytest.approx([2.0] * 6)
    assert gravity.requested_torque == pytest.approx([1.0] * 6)
    assert bias.requested_torque == pytest.approx([4.0] * 6)
    assert model.inverse_calls[0][2] == pytest.approx(np.zeros(6))
    assert dynamics.requested_torque == pytest.approx([4.0] * 6)
    assert model.inverse_calls[1][2] == pytest.approx(acceleration)


def test_shared_mit_envelope_reports_one_torque_decomposition():
    envelope = MitTorqueEnvelope(
        kp=np.zeros(2), kd=np.zeros(2), torque_limit=[4.0, 4.0]
    )

    result = envelope.command(
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [5.0, -1.0],
    )
    signals = result.signals(
        model_torque=[2.0, -1.0],
        task_torque=[2.0, 0.0],
        auxiliary_torque=[1.0, 0.0],
    )

    assert result.command.feedforward == pytest.approx([4.0, -1.0])
    assert signals["torque_saturated"]
    assert signals["torque_saturation_reason"] == "total_limit"
    assert signals["torque_total_estimated"] == pytest.approx([4.0, -1.0])


def test_bounded_screw_velocity_ik_uses_model_jacobian_and_caps_speed():
    solver = BoundedScrewVelocityIk(
        IdentityModel(), velocity_limits=np.ones(6) * 0.5
    )

    velocity = solver.solve(np.ones(6), np.zeros(6), 0.01)

    assert velocity == pytest.approx([0.5] * 6)


def test_bounded_screw_velocity_ik_never_drives_farther_outside_limit():
    solver = BoundedScrewVelocityIk(
        IdentityModel(),
        velocity_limits=np.ones(6) * 0.5,
        joint_limit_margin=0.03,
    )
    position = np.asarray([1.02, 0.0, 0.0, 0.0, 0.0, 0.0])

    blocked_outward = solver.solve(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], position, 0.01
    )
    inward = solver.solve(
        [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0], position, 0.01
    )

    assert blocked_outward[0] == pytest.approx(0.0)
    assert inward[0] == pytest.approx(-0.5)


def test_control_cycle_guard_reports_measured_overspeed_reason():
    guard = ControlCycleGuard(6, velocity_limits=np.ones(6) * 0.5)

    with pytest.raises(ControlSafetyError) as raised:
        guard.validate(sample(velocity=[0.0, 0.51, 0.0, 0.0, 0.0, 0.0]))

    assert raised.value.reason == "measured_velocity_limit"


def test_sustained_velocity_guard_has_tracking_headroom_and_debounce():
    guard = SustainedVelocityGuard(
        sustained_limits=np.ones(2),
        hard_limits=np.ones(2) * 2.0,
        violation_cycles=3,
    )

    guard.validate([0.634, 0.0])
    guard.validate([1.1, 0.0])
    guard.validate([1.1, 0.0])
    with pytest.raises(ControlSafetyError) as raised:
        guard.validate([1.1, 0.0])

    assert raised.value.reason == "measured_velocity_limit"


def test_sustained_velocity_guard_hard_limit_stops_immediately():
    guard = SustainedVelocityGuard(
        sustained_limits=np.ones(2),
        hard_limits=np.ones(2) * 2.0,
        violation_cycles=3,
    )

    with pytest.raises(ControlSafetyError) as raised:
        guard.validate([0.0, 2.01])

    assert raised.value.reason == "measured_velocity_hard_limit"


def test_control_cycle_guard_allows_only_bounded_position_recovery_band():
    guard = ControlCycleGuard(
        6,
        joint_limits=np.asarray([[-1.0, 1.0]] * 6),
        position_tolerance=0.03,
    )

    guard.validate(sample(position=[1.02, 0.0, 0.0, 0.0, 0.0, 0.0]))
    with pytest.raises(ControlSafetyError) as raised:
        guard.validate(sample(
            position=[1.031, 0.0, 0.0, 0.0, 0.0, 0.0]
        ))

    assert raised.value.reason == "measured_position_limit"


def test_interaction_lifecycle_requires_normal_cross_transition():
    lifecycle = InteractionModeLifecycle("impedance")

    transition = lifecycle.plan("admittance")

    assert transition.path == ("normal", "admittance")
    with pytest.raises(RuntimeError, match="through normal"):
        lifecycle.commit("admittance")
    lifecycle.commit("normal")
    lifecycle.commit("admittance")
    assert lifecycle.active == "admittance"


def sample(position=None, target=None, wrench=None, velocity=None):
    position = np.zeros(6) if position is None else np.asarray(position)
    target = np.zeros(6) if target is None else np.asarray(target)
    return ControlInput(
        12.5,
        0.01,
        ControlState(
            position,
            np.zeros(6) if velocity is None else velocity,
            np.zeros(6),
        ),
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


def test_admittance_adapter_produces_reanchored_mit_command():
    class FakeAdmittance:
        def reset(self, pose):
            self.pose = pose

        def step(self, wrench, period):
            pose = np.eye(4)
            return SimpleNamespace(
                mode="zero_force",
                offset=np.zeros(6),
                velocity=np.asarray([0.2, 0, 0, 0, 0, 0]),
                acceleration=np.zeros(6),
                applied_wrench=np.asarray(wrench),
                resisting_wrench=np.asarray(wrench) * 0.25,
                desired_pose=pose,
                desired_twist=np.asarray([0.2, 0, 0, 0, 0, 0]),
            )

    model = IdentityModel()
    controller = CartesianAdmittanceController(
        model,
        FakeAdmittance(),
        BoundedScrewVelocityIk(
            model,
            velocity_limits=np.ones(6) * 0.5,
            damping=0.02,
        ),
        kp=np.ones(6) * 0.3,
        kd=np.ones(6) * 0.08,
        torque_limit=np.ones(6) * 8.0,
        model_scale=0.0,
    )
    initial = sample(position=[0.4, 0, 0, 0, 0, 0]).state
    controller.reset(initial)
    result = controller.step(sample(
        position=[0.4, 0, 0, 0, 0, 0],
        wrench=[0, 0, 0, 2, 0, 0],
    ))

    assert isinstance(result.command, MitCommand)
    assert result.command.position[0] == pytest.approx(0.402, abs=1e-6)
    assert result.command.velocity[0] == pytest.approx(0.2, abs=1e-4)
    assert result.command.kp == pytest.approx([0.3] * 6)
    assert result.command.kd == pytest.approx([0.08] * 6)
    assert result.signals["applied_wrench"] == pytest.approx(
        [0, 0, 0, 2, 0, 0]
    )
    assert result.signals["admittance_mode"] == "zero_force"
    assert result.signals["resisting_wrench"] == pytest.approx(
        [0, 0, 0, 0.5, 0, 0]
    )

    moved = controller.step(sample(
        position=[0.6, 0, 0, 0, 0, 0],
        wrench=[0, 0, 0, 2, 0, 0],
    ))
    assert moved.command.position[0] == pytest.approx(0.602, abs=1e-6)


def test_admittance_velocity_ik_enforces_predictive_joint_limit():
    class FakeAdmittance:
        def reset(self, pose):
            del pose

        def step(self, wrench, period):
            del wrench, period
            return SimpleNamespace(
                mode="zero_force",
                offset=np.zeros(6),
                velocity=np.ones(6),
                acceleration=np.zeros(6),
                applied_wrench=np.zeros(6),
                resisting_wrench=np.zeros(6),
                desired_pose=np.eye(4),
                desired_twist=np.ones(6),
            )

    model = IdentityModel()
    controller = CartesianAdmittanceController(
        model,
        FakeAdmittance(),
        BoundedScrewVelocityIk(
            model,
            velocity_limits=np.ones(6) * 0.5,
            damping=0.02,
            joint_limit_margin=0.0,
        ),
        kp=np.ones(6) * 0.3,
        kd=np.ones(6) * 0.08,
        torque_limit=np.ones(6) * 8.0,
        model_scale=0.0,
    )
    position = np.ones(6) * 0.999
    controller.reset(sample(position=position).state)

    result = controller.step(sample(position=position))

    assert result.command.velocity == pytest.approx([0.1] * 6, abs=1e-6)
    assert result.command.position == pytest.approx(np.ones(6), abs=1e-9)


def test_admittance_rejects_mit_feedback_that_cannot_be_torque_limited():
    class FakeAdmittance:
        def reset(self, pose):
            del pose

        def step(self, wrench, period):
            del wrench, period
            return SimpleNamespace(
                mode="zero_force",
                offset=np.zeros(6),
                velocity=np.zeros(6),
                acceleration=np.zeros(6),
                applied_wrench=np.zeros(6),
                resisting_wrench=np.zeros(6),
                desired_pose=np.eye(4),
                desired_twist=np.zeros(6),
            )

    model = IdentityModel()
    controller = CartesianAdmittanceController(
        model,
        FakeAdmittance(),
        BoundedScrewVelocityIk(
            model,
            velocity_limits=np.ones(6) * 0.5,
        ),
        kp=np.zeros(6),
        kd=np.ones(6),
        torque_limit=np.ones(6) * 0.1,
        model_scale=0.0,
    )

    with pytest.raises(ControlSafetyError, match="feedback alone"):
        controller.step(sample(velocity=np.ones(6) * 0.3))


def test_admittance_debounces_measured_joint_tracking_overspeed():
    class FakeAdmittance:
        def reset(self, pose):
            del pose

        def step(self, wrench, period):
            del wrench, period
            return SimpleNamespace(
                desired_twist=np.zeros(6),
                desired_pose=np.eye(4),
            )

    model = IdentityModel()
    controller = CartesianAdmittanceController(
        model,
        FakeAdmittance(),
        BoundedScrewVelocityIk(
            model,
            velocity_limits=np.ones(6) * 0.5,
        ),
        kp=np.zeros(6),
        kd=np.zeros(6),
        torque_limit=np.ones(6) * 8.0,
        model_scale=0.0,
    )

    controller.step(sample(velocity=[0.634, 0, 0, 0, 0, 0]))
    controller.step(sample(velocity=[1.1, 0, 0, 0, 0, 0]))
    controller.step(sample(velocity=[1.1, 0, 0, 0, 0, 0]))
    with pytest.raises(ControlSafetyError) as raised:
        controller.step(sample(velocity=[1.1, 0, 0, 0, 0, 0]))

    assert raised.value.reason == "measured_velocity_limit"


def test_admittance_rejects_torque_limit_above_eight_newton_metres():
    with pytest.raises(ValueError, match=r"\(0, 8\]"):
        CartesianAdmittanceController(
            IdentityModel(),
            SimpleNamespace(),
            BoundedScrewVelocityIk(
                IdentityModel(),
                velocity_limits=np.ones(6) * 0.5,
            ),
            kp=np.zeros(6),
            kd=np.zeros(6),
            torque_limit=np.ones(6) * (
                ADMITTANCE_MIT_TORQUE_LIMIT_MAX + 0.01
            ),
        )


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
