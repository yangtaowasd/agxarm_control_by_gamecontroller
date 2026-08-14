"""Contracts for generalized-momentum external-torque observation."""

from pathlib import Path

import numpy as np
import pytest
from sensor_msgs.msg import JointState

from armbycontroller.momentum_observer import GeneralizedMomentumObserver
from armbycontroller.ros.momentum_observer_node import MomentumObserverNode
from armbycontroller.screw_model import UrdfScrewModel


def test_observer_ros_adapter_has_no_arm_sdk_or_can_access():
    source = (
        Path(__file__).resolve().parents[1]
        / "armbycontroller" / "ros" / "momentum_observer_node.py"
    ).read_text(encoding="utf-8")

    for forbidden in (
        "pyAgxArm",
        "get_joint_angles",
        "get_motor_states",
        "move_mit",
        "socketcan",
    ):
        assert forbidden not in source
    assert "message.header.stamp" in source
    assert "time.monotonic" not in source


def test_ros_adapter_uses_each_100_hz_timestamp_and_previous_command(
    monkeypatch,
):
    class FakeModel:
        movable_joint_names = ("joint1",)

        def momentum_observer_terms(self, position, velocity):
            return position.copy(), velocity.copy()

    class FakeObserver:
        def __init__(self):
            self.reset_momentum = None
            self.update_arguments = None

        def reset(self, momentum):
            self.reset_momentum = momentum.copy()

        def update(self, momentum, torque, bias, period):
            self.update_arguments = (
                momentum.copy(), torque.copy(), bias.copy(), period
            )
            return type("Observation", (), {
                "external_torque": np.asarray([0.25])
            })()

    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class FakeLogger:
        def info(self, message, **kwargs):
            del message, kwargs

        def warning(self, message, **kwargs):
            del message, kwargs

    def sample(seconds, position, velocity, torque):
        message = JointState()
        message.header.stamp.sec = int(seconds)
        message.header.stamp.nanosec = int(
            round((seconds - int(seconds)) * 1e9)
        )
        message.position = [position]
        message.velocity = [velocity]
        message.effort = [torque]
        return message

    node = object.__new__(MomentumObserverNode)
    node.joint_count = 1
    node.maximum_period = 0.05
    node.model = FakeModel()
    node.observer = FakeObserver()
    node.publisher = FakePublisher()
    node.previous_sample_time = None
    node.previous_command = None
    node.previous_beta = None
    monkeypatch.setattr(
        MomentumObserverNode, "get_logger", lambda self: FakeLogger()
    )

    node._state_callback(sample(10.00, 1.0, 0.5, 3.0))
    node._state_callback(sample(10.01, 1.1, 0.6, 9.0))

    momentum, torque, bias, period = node.observer.update_arguments
    assert momentum == pytest.approx([1.1])
    assert torque == pytest.approx([3.0])
    assert bias == pytest.approx([0.5])
    assert period == pytest.approx(0.01)
    assert len(node.publisher.messages) == 1
    output = node.publisher.messages[0]
    assert output.header.stamp == sample(
        10.01, 0.0, 0.0, 0.0
    ).header.stamp
    assert output.effort == pytest.approx([0.25])


@pytest.mark.parametrize(
    ("model_name", "joint_count", "tip_link", "accessory"),
    [
        ("piper_l", 6, "link6", "piper_l_with_gripper_description.xacro"),
        ("nero", 7, "link7", "nero_with_left_revo2_description.xacro"),
    ],
)
def test_spatial_momentum_matches_mass_matrix(
    model_name, joint_count, tip_link, accessory
):
    urdf = (
        Path(__file__).resolve().parents[1]
        / "agx_arm_urdf" / model_name / "urdf" / accessory
    )
    model = UrdfScrewModel(
        urdf, "base_link", tip_link, joint_count, [0.0, 0.0, -9.80665]
    )
    position = np.linspace(-0.2, 0.2, joint_count)
    velocity = np.linspace(0.3, -0.2, joint_count)

    momentum = model.generalized_momentum(position, velocity)

    assert momentum == pytest.approx(
        model.mass_matrix(position) @ velocity, abs=1e-10
    )


def test_kinetic_gradient_matches_coriolis_transpose_velocity(tmp_path):
    urdf = tmp_path / "two_link.urdf"
    urdf.write_text(
        """<robot name="two_link">
        <link name="base"/>
        <link name="one"><inertial><origin xyz="0.3 0 0"/>
          <mass value="2"/><inertia ixx=".1" ixy="0" ixz="0"
          iyy=".2" iyz="0" izz=".2"/></inertial></link>
        <link name="two"><inertial><origin xyz="0.2 0 0"/>
          <mass value="1"/><inertia ixx=".05" ixy="0" ixz="0"
          iyy=".1" iyz="0" izz=".1"/></inertial></link>
        <joint name="joint1" type="revolute"><parent link="base"/>
          <child link="one"/><axis xyz="0 0 1"/>
          <limit lower="-3" upper="3" effort="20" velocity="2"/></joint>
        <joint name="joint2" type="revolute"><parent link="one"/>
          <child link="two"/><origin xyz="0.6 0 0"/><axis xyz="0 0 1"/>
          <limit lower="-3" upper="3" effort="20" velocity="2"/></joint>
        </robot>""",
        encoding="utf-8",
    )
    model = UrdfScrewModel(urdf, "base", "two", 2)
    position = np.asarray([0.4, -0.7])
    velocity = np.asarray([0.6, -0.3])
    epsilon = 1e-6
    velocity_jacobian = np.column_stack([
        (
            model.coriolis_torque(
                position, velocity + epsilon * np.eye(2)[index]
            )
            - model.coriolis_torque(
                position, velocity - epsilon * np.eye(2)[index]
            )
        ) / (2.0 * epsilon)
        for index in range(2)
    ])
    coriolis_matrix = 0.5 * velocity_jacobian

    assert model.kinetic_energy_gradient(
        position, velocity
    ) == pytest.approx(coriolis_matrix.T @ velocity, abs=1e-8)
    momentum, beta = model.momentum_observer_terms(position, velocity)
    assert momentum == pytest.approx(model.mass_matrix(position) @ velocity)
    assert beta == pytest.approx(
        model.gravity_torque(position) - coriolis_matrix.T @ velocity,
        abs=1e-8,
    )


def test_observer_residual_converges_to_constant_external_torque():
    observer = GeneralizedMomentumObserver(2, gain=[10.0, 20.0])
    observer.reset([0.0, 0.0])
    external = np.asarray([1.5, -0.8])
    observation = None

    for index in range(1, 201):
        observation = observer.update(
            momentum=index * 0.01 * external,
            actuator_torque=np.zeros(2),
            beta=np.zeros(2),
            period=0.01,
        )

    assert observation.external_torque == pytest.approx(external, abs=1e-6)


def test_observer_resets_instead_of_integrating_across_a_stream_gap():
    observer = GeneralizedMomentumObserver(1, gain=10.0, maximum_period=0.05)
    observer.reset([0.0])
    observer.update([0.01], [0.0], [0.0], 0.01)

    observation = observer.update([5.0], [10.0], [2.0], 0.2)

    assert observation.reset
    assert observation.external_torque == pytest.approx([0.0])
    assert observation.predicted_momentum == pytest.approx([5.0])


def test_observer_rejects_discretely_unstable_gain_and_gap_combination():
    with pytest.raises(ValueError, match="stability limit"):
        GeneralizedMomentumObserver(1, gain=40.0, maximum_period=0.05)
