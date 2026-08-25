"""Contracts shared by every controller adapter and experiment consumer."""

from types import SimpleNamespace

import numpy as np
import pytest

from armbycontroller.api import interaction_state_payload
from armbycontroller.api import InteractionModeInterface
from armbycontroller.api import PUBLIC_INTERACTION_MODES
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
from armbycontroller.control import InteractionSafetyLimits
from armbycontroller.control import INTERACTION_TORQUE_LIMIT_MAX
from armbycontroller.control import SustainedVelocityGuard
from armbycontroller.control import control_sample
from armbycontroller.impedance.controllers import CartesianImpedanceController
from armbycontroller.impedance.controllers import JointMitController
from armbycontroller.ik.screw import BoundedScrewVelocityIk
from armbycontroller.modeling.gravity_schedule import ScheduledGravityModel
from armbycontroller.modeling.gravity_schedule import (
    SignedGravityCalibration,
)
from armbycontroller.modeling.gravity_schedule import (
    SmoothJointGravitySchedule,
)


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


def test_interaction_safety_limits_are_one_validated_source():
    limits = InteractionSafetyLimits(
        torque_limit=np.ones(6) * 8.0,
        torque_rate_limit=np.ones(6) * 20.0,
        reference_velocity_limit=np.ones(6) * 1.0,
        measured_velocity_stop_limit=np.ones(6) * 2.0,
        measured_velocity_hard_limit=np.ones(6) * 2.5,
        measured_velocity_violation_cycles=3,
        joint_limit_margin=0.03,
    )

    assert limits.joint_count == 6
    assert limits.torque_limit == pytest.approx([8.0] * 6)
    assert limits.reference_velocity_limit == pytest.approx([1.0] * 6)
    assert limits.velocity_guard().violation_cycles == 3
    assert INTERACTION_TORQUE_LIMIT_MAX == 8.0

    with pytest.raises(ValueError, match=r"\(0, 8\] N.m"):
        InteractionSafetyLimits(
            torque_limit=np.ones(6) * 8.01,
            torque_rate_limit=np.ones(6) * 20.0,
            reference_velocity_limit=np.ones(6) * 1.0,
            measured_velocity_stop_limit=np.ones(6) * 2.0,
            measured_velocity_hard_limit=np.ones(6) * 2.5,
            measured_velocity_violation_cycles=3,
            joint_limit_margin=0.03,
        )


@pytest.mark.parametrize(
    "controller_factory",
    [
        lambda: JointMitController(
            6,
            kp=np.zeros(6),
            kd=np.zeros(6),
            feedforward=np.zeros(6),
            torque_limit=np.ones(6) * 8.01,
        ),
        lambda: CartesianImpedanceController(
            IdentityModel(),
            stiffness=np.zeros(6),
            damping=np.zeros(6),
            torque_limit=np.ones(6) * 8.01,
        ),
    ],
)
def test_impedance_adapters_cannot_bypass_shared_torque_cap(
    controller_factory,
):
    with pytest.raises(ValueError, match=r"\(0, 8\] N.m"):
        controller_factory()


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


def test_horizontal_gravity_schedule_blends_signed_j2_j4_calibration():
    schedule = SmoothJointGravitySchedule(
        7,
        transition_angle=np.deg2rad(2.0),
        calibrations={
            1: SignedGravityCalibration(0.8, 1.2, -0.1, 0.1),
            3: SignedGravityCalibration(0.5, 1.5, -0.2, 0.2),
        },
    )
    raw = np.ones(7)
    negative = np.zeros(7)
    negative[[1, 3]] = np.deg2rad(-3.0)
    positive = -negative

    assert schedule.apply(negative, raw) == pytest.approx(
        [1.0, 0.7, 1.0, 0.3, 1.0, 1.0, 1.0]
    )
    assert schedule.apply(positive, raw) == pytest.approx(
        [1.0, 1.3, 1.0, 1.7, 1.0, 1.0, 1.0]
    )
    assert schedule.apply(np.zeros(7), raw) == pytest.approx(raw)


def test_scheduled_gravity_model_preserves_non_gravity_dynamics():
    class DynamicsModel:
        marker = "delegated"

        def gravity_torque(self, position):
            del position
            return np.arange(1.0, 8.0)

        def inverse_dynamics(self, position, velocity, acceleration):
            del position, velocity, acceleration
            return np.arange(1.0, 8.0) + 10.0

        def momentum_observer_terms(self, position, velocity):
            del position, velocity
            return np.ones(7) * 5.0, np.arange(1.0, 8.0) + 20.0

    schedule = SmoothJointGravitySchedule(
        7,
        transition_angle=np.deg2rad(2.0),
        calibrations={
            1: SignedGravityCalibration(0.5, 1.5, 0.0, 0.0),
            3: SignedGravityCalibration(1.0, 1.0, -0.2, 0.2),
        },
    )
    model = ScheduledGravityModel(DynamicsModel(), schedule)
    position = np.zeros(7)
    position[1] = np.deg2rad(3.0)
    position[3] = np.deg2rad(-3.0)

    gravity = model.gravity_torque(position)
    total = model.inverse_dynamics(position, np.zeros(7), np.zeros(7))
    momentum, beta = model.momentum_observer_terms(
        position, np.zeros(7)
    )

    assert gravity == pytest.approx([1.0, 3.0, 3.0, 3.8, 5.0, 6.0, 7.0])
    assert total - gravity == pytest.approx([10.0] * 7)
    assert momentum == pytest.approx([5.0] * 7)
    assert beta - gravity == pytest.approx([20.0] * 7)
    assert model.marker == "delegated"


def test_horizontal_gravity_schedule_rejects_unsafe_parameters():
    with pytest.raises(ValueError, match=r"\[0, 2\]"):
        SignedGravityCalibration(negative_scale=2.1)
    with pytest.raises(ValueError, match=r"\[-2, 2\]"):
        SignedGravityCalibration(positive_bias_nm=2.1)
    with pytest.raises(ValueError, match=r"\[0.5, 15\]"):
        SmoothJointGravitySchedule(7, np.deg2rad(0.1), {})


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


def test_shared_mit_envelope_limits_torque_change_per_cycle():
    envelope = MitTorqueEnvelope(
        kp=np.zeros(2),
        kd=np.zeros(2),
        torque_limit=[8.0, 8.0],
        torque_rate_limit=[10.0, 20.0],
    )
    envelope.reset([0.0, 0.0])

    result = envelope.command(
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [5.0, -5.0],
        period=0.01,
    )

    assert result.command.estimated_torque == pytest.approx([0.1, -0.2])
    assert result.command.feedforward == pytest.approx([0.1, -0.2])
    assert result.rate_limited
    assert result.saturation_reason == "torque_rate_limit"


def test_shared_mit_envelope_rejects_uncontrollable_torque_rate():
    envelope = MitTorqueEnvelope(
        kp=[10.0],
        kd=[0.0],
        torque_limit=[8.0],
        torque_rate_limit=[10.0],
    )
    envelope.reset([0.0])

    with pytest.raises(ControlSafetyError) as captured:
        envelope.command(
            [1.0], [0.0], [0.0], [0.0], [0.0], period=0.01
        )

    assert captured.value.reason == "torque_rate_infeasible"


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


def test_interaction_lifecycle_interlocks_three_interaction_controllers():
    lifecycle = InteractionModeLifecycle("impedance")

    assert lifecycle.plan("hybrid").path == ("normal", "hybrid")
    lifecycle.commit("normal")
    lifecycle.commit("hybrid")
    with pytest.raises(RuntimeError, match="cannot be active together"):
        lifecycle.synchronize(True, False, True)


def test_public_mode_interface_is_idempotent_and_excludes_hybrid():
    active = ["normal"]
    calls = []

    def enter(mode):
        def callback(reason):
            calls.append((mode, reason))
            active[0] = mode
            return True

        return callback

    interface = InteractionModeInterface(
        current_mode=lambda: active[0],
        enter_normal=enter("normal"),
        enter_impedance=enter("impedance"),
        enter_admittance=enter("admittance"),
    )

    changed = interface.request("impedance", source="ui")
    unchanged = interface.request("impedance", source="ui")

    assert changed.success
    assert changed.changed
    assert changed.active_mode == "impedance"
    assert changed.to_payload()["requested_mode"] == "impedance"
    assert unchanged.success
    assert not unchanged.changed
    assert calls == [("impedance", "ui requested impedance")]
    assert PUBLIC_INTERACTION_MODES == (
        "normal", "impedance", "admittance"
    )
    with pytest.raises(ValueError, match="public interaction mode"):
        interface.request("hybrid", source="ui")


def test_public_mode_interface_forces_normal_and_stops_on_failed_exit():
    active = ["impedance"]
    calls = []

    def enter_normal(reason):
        calls.append(("normal", reason))
        active[0] = "normal"
        return True

    def enter_admittance(reason):
        calls.append(("admittance", reason))
        active[0] = "admittance"
        return True

    interface = InteractionModeInterface(
        current_mode=lambda: active[0],
        enter_normal=enter_normal,
        enter_impedance=lambda reason: False,
        enter_admittance=enter_admittance,
    )

    result = interface.request("admittance", source="frontend")

    assert result.success
    assert result.active_mode == "admittance"
    assert [mode for mode, _ in calls] == ["normal", "admittance"]

    active[0] = "impedance"
    calls.clear()

    def fail_normal(reason):
        calls.append(("normal", reason))
        return False

    blocked = InteractionModeInterface(
        current_mode=lambda: active[0],
        enter_normal=fail_normal,
        enter_impedance=lambda reason: True,
        enter_admittance=enter_admittance,
    ).request("admittance", source="frontend")

    assert not blocked.success
    assert not blocked.changed
    assert blocked.active_mode == "impedance"
    assert [mode for mode, _ in calls] == ["normal"]


def test_public_mode_interface_normalizes_controller_exceptions():
    def fail_entry(reason):
        raise RuntimeError(f"preflight rejected: {reason}")

    interface = InteractionModeInterface(
        current_mode=lambda: "normal",
        enter_normal=lambda reason: True,
        enter_impedance=fail_entry,
        enter_admittance=lambda reason: True,
    )

    result = interface.request("impedance", source="frontend")

    assert not result.success
    assert not result.changed
    assert result.active_mode == "normal"
    assert "preflight rejected" in result.message


def test_public_state_payload_reports_hybrid_without_exposing_its_command():
    payload = interaction_state_payload(
        "hybrid",
        timestamp=12.5,
        robot_model="nero",
        arm_ready=True,
    )

    assert payload == {
        "schema_version": 1,
        "timestamp": 12.5,
        "robot_model": "nero",
        "interaction_mode": "hybrid",
        "available_modes": ["normal", "impedance", "admittance"],
        "arm_ready": True,
    }


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
            wrench is not None,
        ),
    )


def test_control_reference_four_argument_constructor_remains_compatible():
    reference = ControlReference(
        np.zeros(6), np.zeros(6), np.zeros(6), np.zeros(6)
    )

    assert reference.external_wrench_valid


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


def test_cartesian_position_integral_is_weak_bounded_and_one_cycle_delayed():
    class ZeroModel(IdentityModel):
        def inverse_dynamics(self, position, velocity, acceleration):
            del position, velocity, acceleration
            return np.zeros(6)

    controller = CartesianImpedanceController(
        ZeroModel(),
        stiffness=np.zeros(6),
        damping=np.zeros(6),
        torque_limit=np.ones(6) * 8.0,
        position_integral_gain=[0, 0, 0, 1000, 0, 0],
        position_integral_deadband=[0, 0, 0, 0.001, 0, 0],
        position_integral_max_force=0.05,
        position_integral_max_torque=0.1,
        position_integral_requires_external_wrench=True,
    )
    held = sample(target=[0, 0, 0, 0.01, 0, 0], wrench=np.zeros(6))

    first = controller.step(held)
    second = controller.step(held)
    third = controller.step(held)

    assert first.signals["position_integral_pause_reason"] == (
        "reference_changed"
    )
    assert second.signals["position_integral_limited"]
    assert second.signals["position_integral_next_wrench"] == pytest.approx(
        [0, 0, 0, 0.05, 0, 0]
    )
    assert third.signals["position_integral_wrench"] == pytest.approx(
        [0, 0, 0, 0.05, 0, 0]
    )
    assert third.command.feedforward == pytest.approx(
        [0, 0, 0, 0.05, 0, 0]
    )


def test_cartesian_position_integral_pauses_for_push_and_saturation():
    class ZeroModel(IdentityModel):
        def inverse_dynamics(self, position, velocity, acceleration):
            del position, velocity, acceleration
            return np.zeros(6)

    controller = CartesianImpedanceController(
        ZeroModel(),
        stiffness=[0, 0, 0, 10, 0, 0],
        damping=np.zeros(6),
        torque_limit=np.ones(6) * 8.0,
        maximum_force=0.05,
        position_integral_gain=[0, 0, 0, 2, 0, 0],
        position_integral_external_force_gate=1.0,
        position_integral_requires_external_wrench=True,
    )
    target = [0, 0, 0, 0.01, 0, 0]
    controller.step(sample(target=target, wrench=np.zeros(6)))

    unavailable = controller.step(sample(target=target))
    pushed = controller.step(sample(
        target=target, wrench=[0, 0, 0, 2.0, 0, 0]
    ))
    hysteresis = controller.step(sample(
        target=target, wrench=[0, 0, 0, 0.8, 0, 0]
    ))
    saturated = controller.step(sample(
        target=target, wrench=[0, 0, 0, 0.4, 0, 0]
    ))

    assert unavailable.signals["position_integral_pause_reason"] == (
        "external_wrench_unavailable"
    )
    assert pushed.signals["position_integral_pause_reason"] == (
        "external_force_gate"
    )
    assert hysteresis.signals["position_integral_pause_reason"] == (
        "external_wrench_hysteresis"
    )
    assert hysteresis.signals["position_integral_push_active"]
    assert saturated.signals["position_integral_pause_reason"] == (
        "wrench_saturated"
    )
    assert not saturated.signals["position_integral_push_active"]
    assert saturated.signals["position_integral_next_wrench"] == (
        pytest.approx(np.zeros(6))
    )

    controller.reset(sample(target=target).state)
    controller.step(sample(
        target=target, wrench=[0, 0, 0, 0.8, 0, 0]
    ))
    initial_hysteresis = controller.step(sample(
        target=target, wrench=[0, 0, 0, 0.8, 0, 0]
    ))
    assert initial_hysteresis.signals[
        "position_integral_pause_reason"
    ] == "external_wrench_hysteresis"


def test_cartesian_position_integral_rearms_after_wrench_becomes_unavailable():
    class ZeroModel(IdentityModel):
        def inverse_dynamics(self, position, velocity, acceleration):
            del position, velocity, acceleration
            return np.zeros(6)

    controller = CartesianImpedanceController(
        ZeroModel(),
        stiffness=np.zeros(6),
        damping=np.zeros(6),
        torque_limit=np.ones(6) * 8.0,
        position_integral_gain=[0, 0, 0, 2, 0, 0],
        position_integral_external_force_gate=1.0,
        position_integral_external_force_release=0.5,
        position_integral_requires_external_wrench=True,
    )
    target = [0, 0, 0, 0.01, 0, 0]
    controller.step(sample(target=target, wrench=np.zeros(6)))
    controller.step(sample(target=target, wrench=np.zeros(6)))

    unavailable = controller.step(sample(target=target))
    hysteresis = controller.step(sample(
        target=target, wrench=[0, 0, 0, 0.8, 0, 0]
    ))

    assert unavailable.signals["position_integral_pause_reason"] == (
        "external_wrench_unavailable"
    )
    assert unavailable.signals["position_integral_push_active"]
    assert hysteresis.signals["position_integral_pause_reason"] == (
        "external_wrench_hysteresis"
    )
    assert hysteresis.signals["position_integral_push_active"]


def test_cartesian_position_integral_resets_with_controller_and_reference():
    class ZeroModel(IdentityModel):
        def inverse_dynamics(self, position, velocity, acceleration):
            del position, velocity, acceleration
            return np.zeros(6)

    controller = CartesianImpedanceController(
        ZeroModel(),
        stiffness=np.zeros(6),
        damping=np.zeros(6),
        torque_limit=np.ones(6) * 8.0,
        position_integral_gain=[0, 0, 0, 2, 0, 0],
    )
    held = sample(target=[0, 0, 0, 0.01, 0, 0])
    controller.step(held)
    accumulated = controller.step(held)
    assert accumulated.signals["position_integral_next_wrench"][3] > 0.0

    changed = controller.step(sample(target=[0, 0, 0, 0.02, 0, 0]))
    assert changed.signals["position_integral_wrench"] == pytest.approx(
        np.zeros(6)
    )
    controller.step(held)
    controller.step(held)
    controller.reset(held.state)
    reset = controller.step(held)
    assert reset.signals["position_integral_wrench"] == pytest.approx(
        np.zeros(6)
    )


def test_cartesian_position_integral_uses_ten_second_saturation_decay():
    class ZeroModel(IdentityModel):
        def inverse_dynamics(self, position, velocity, acceleration):
            del position, velocity, acceleration
            return np.zeros(6)

    controller = CartesianImpedanceController(
        ZeroModel(),
        stiffness=np.zeros(6),
        damping=np.zeros(6),
        torque_limit=np.ones(6) * 8.0,
        maximum_force=0.01,
        position_integral_gain=[0, 0, 0, 1000, 0, 0],
        position_integral_max_force=0.1,
        position_integral_max_torque=0.1,
        position_integral_leak_rate=0.05,
        position_integral_saturation_leak_rate=0.1,
    )
    held = sample(target=[0, 0, 0, 0.01, 0, 0])
    controller.step(held)
    accumulated = controller.step(held)
    state_before_saturation = accumulated.signals[
        "position_integral_next_wrench"
    ][3]

    saturated = controller.step(held)

    assert saturated.signals["position_integral_pause_reason"] == (
        "wrench_saturated"
    )
    assert saturated.signals["position_integral_decay_rate"] == pytest.approx(
        0.1
    )
    assert saturated.signals["position_integral_next_wrench"][3] == (
        pytest.approx(state_before_saturation * np.exp(-0.1 * held.period))
    )


def test_cartesian_integral_survives_joint_only_reference_change():
    class RedundantModel:
        joint_limits = np.asarray([[-1.0, 1.0]] * 7)

        def forward_kinematics(self, joints):
            pose = np.eye(4)
            pose[:3, 3] = np.asarray(joints)[3:6]
            return pose

        def space_jacobian(self, joints):
            del joints
            return np.hstack((np.eye(6), np.zeros((6, 1))))

        def inverse_dynamics(self, position, velocity, acceleration):
            del position, velocity, acceleration
            return np.zeros(7)

    def redundant_sample(target):
        return ControlInput(
            12.5,
            0.01,
            ControlState(np.zeros(7), np.zeros(7), np.zeros(7)),
            ControlReference(
                np.asarray(target),
                np.zeros(7),
                np.zeros(7),
                np.zeros(6),
                False,
            ),
        )

    controller = CartesianImpedanceController(
        RedundantModel(),
        stiffness=np.zeros(6),
        damping=np.zeros(6),
        torque_limit=np.ones(7) * 8.0,
        position_integral_gain=[0, 0, 0, 2, 0, 0],
    )
    target = np.asarray([0, 0, 0, 0.01, 0, 0, 0], dtype=float)
    held = redundant_sample(target)
    controller.step(held)
    accumulated = controller.step(held)
    assert accumulated.signals["position_integral_next_wrench"][3] > 0.0

    target[6] = 0.2
    joint_only_change = controller.step(redundant_sample(target))

    assert joint_only_change.signals["position_integral_pause_reason"] == (
        "active"
    )
    assert joint_only_change.signals["position_integral_wrench"][3] > 0.0


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
        6, np.ones(6), np.zeros(6), np.zeros(6), np.ones(6) * 8.0
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
