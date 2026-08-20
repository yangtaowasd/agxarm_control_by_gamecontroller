"""Contracts for complementary Twist-level impedance-admittance control."""

from types import SimpleNamespace

import numpy as np
import pytest

from armbycontroller.admittance import ResistiveAdmittance
from armbycontroller.control import ControlInput
from armbycontroller.control import ControlReference
from armbycontroller.control import ControlState
from armbycontroller.hybrid import HybridCartesianController
from armbycontroller.hybrid import task_axis_mask
from armbycontroller.hybrid import task_subspace_projector
from armbycontroller.ik.screw import BoundedScrewVelocityIk
from armbycontroller.ik.screw import ScrewIkFailure
from armbycontroller.modeling.lie import rotation_from_vector


class IdentityModel:
    """Six-axis model whose joints directly match project Twist ordering."""

    joint_count = 6
    joint_limits = np.asarray([[-1.0, 1.0]] * 6)

    def forward_kinematics(self, joints):
        pose = np.eye(4)
        pose[:3, 3] = np.asarray(joints, dtype=float)[3:]
        return pose

    def space_jacobian(self, joints):
        del joints
        return np.eye(6)

    def inverse_dynamics(self, position, velocity, acceleration):
        del position, velocity, acceleration
        return np.zeros(6)


def sample(position=None, velocity=None, wrench=None):
    position = np.zeros(6) if position is None else np.asarray(position)
    velocity = np.zeros(6) if velocity is None else np.asarray(velocity)
    wrench = np.zeros(6) if wrench is None else np.asarray(wrench)
    return ControlInput(
        1.0,
        0.01,
        ControlState(position, velocity, np.zeros(6)),
        ControlReference.hold(position, wrench),
    )


def test_project_axis_names_follow_angular_then_linear_order():
    assert task_axis_mask("z") == pytest.approx([0, 0, 0, 0, 0, 1])
    assert task_axis_mask("rz,x") == pytest.approx([0, 0, 1, 1, 0, 0])
    with pytest.raises(ValueError, match="unknown Cartesian axis"):
        task_axis_mask("fz")


def test_custom_compliance_frame_rotates_local_x_onto_base_y():
    rotation = rotation_from_vector([0.0, 0.0, np.pi / 2.0])

    projector = task_subspace_projector("x", rotation)

    expected = np.zeros((6, 6))
    expected[4, 4] = 1.0
    assert projector == pytest.approx(expected, abs=1e-12)
    assert projector @ projector == pytest.approx(projector, abs=1e-12)


def test_hybrid_z_admittance_is_complementary_to_impedance():
    model = IdentityModel()
    admittance = ResistiveAdmittance(
        virtual_mass=np.ones(6),
        damping=np.ones(6),
        stiffness=np.ones(6),
        wrench_deadband=np.zeros(6),
        wrench_limit=np.ones(6) * 10.0,
        offset_limit=np.ones(6),
        velocity_limit=np.ones(6),
        wrench_filter_hz=0.0,
    )
    controller = HybridCartesianController(
        model,
        admittance,
        BoundedScrewVelocityIk(
            model, velocity_limits=np.ones(6) * 0.5
        ),
        cartesian_stiffness=np.ones(6) * 10.0,
        cartesian_damping=np.ones(6),
        kp=np.zeros(6),
        kd=np.zeros(6),
        torque_limit=np.ones(6) * 8.0,
        admittance_axes="z",
        desired_wrench=np.zeros(6),
        model_scale=0.0,
    )
    controller.reset(sample().state)

    result = controller.step(sample(
        position=[0.0, 0.0, 0.0, 0.1, 0.0, 0.0],
        wrench=[3.0, 4.0, 5.0, 6.0, 7.0, 1.0],
    ))

    assert result.signals["admittance_axis_mask"] == pytest.approx(
        [0, 0, 0, 0, 0, 1]
    )
    assert result.signals["impedance_axis_mask"] == pytest.approx(
        [1, 1, 1, 1, 1, 0]
    )
    assert result.signals["admittance_twist"] == pytest.approx(
        [0, 0, 0, 0, 0, 0.01]
    )
    assert result.signals["commanded_wrench"] == pytest.approx(
        [0, 0, 0, -1.0, 0, 0]
    )
    assert result.command.velocity == pytest.approx(
        [0, 0, 0, 0, 0, 0.01], abs=1e-5
    )
    assert result.command.feedforward == pytest.approx(
        [0, 0, 0, -1.0, 0, 0]
    )


def test_hybrid_projects_desired_wrench_before_admittance():
    class RecordingAdmittance:
        mode = "recording"

        def reset(self, pose):
            self.pose = np.asarray(pose)

        def step(self, wrench, period):
            self.wrench = np.asarray(wrench)
            del period
            return SimpleNamespace(
                desired_twist=np.zeros(6),
                offset=np.zeros(6),
                velocity=np.zeros(6),
                acceleration=np.zeros(6),
                applied_wrench=self.wrench,
                resisting_wrench=np.zeros(6),
            )

    model = IdentityModel()
    admittance = RecordingAdmittance()
    controller = HybridCartesianController(
        model,
        admittance,
        BoundedScrewVelocityIk(model, np.ones(6) * 0.5),
        np.zeros(6),
        np.zeros(6),
        np.zeros(6),
        np.zeros(6),
        np.ones(6) * 8.0,
        admittance_axes="z",
        desired_wrench=[0, 0, 0, 0, 0, 2.0],
        model_scale=0.0,
    )
    controller.reset(sample().state)

    controller.step(sample(wrench=[4, 4, 4, 4, 4, 5]))

    assert admittance.wrench == pytest.approx([0, 0, 0, 0, 0, 3])


def test_hybrid_custom_frame_and_reconfiguration_are_bumpless():
    model = IdentityModel()
    admittance = ResistiveAdmittance(
        virtual_mass=np.ones(6),
        damping=np.ones(6),
        stiffness=np.ones(6),
        wrench_deadband=np.zeros(6),
        wrench_limit=np.ones(6) * 10.0,
        offset_limit=np.ones(6),
        velocity_limit=np.ones(6),
        wrench_filter_hz=0.0,
    )
    controller = HybridCartesianController(
        model,
        admittance,
        BoundedScrewVelocityIk(model, np.ones(6)),
        np.ones(6) * 10.0,
        np.ones(6),
        np.zeros(6),
        np.zeros(6),
        np.ones(6) * 8.0,
        admittance_axes="x",
        admittance_frame="custom",
        admittance_frame_rotation=[0.0, 0.0, np.pi / 2.0],
        model_scale=0.0,
    )
    controller.reset(sample().state)
    admittance.velocity = np.asarray([0, 0, 0, 0, 0.2, 0])
    current = np.eye(4)
    current[:3, 3] = [0.1, 0.2, 0.3]

    controller.reconfigure_admittance_subspace(
        current, axes="z", frame="base"
    )

    # Existing X impedance stays at zero; Y changes from compliant to rigid
    # and is captured at the measured value. Z is now compliant.
    assert controller.anchor_pose[:3, 3] == pytest.approx([0.0, 0.2, 0.3])
    assert admittance.anchor_pose == pytest.approx(current)
    assert admittance.velocity == pytest.approx(np.zeros(6))
    assert controller.admittance_selection[5, 5] == pytest.approx(1.0)


def test_velocity_ik_soft_scales_then_rejects_at_singularity_stop():
    class SingularModel(IdentityModel):
        def __init__(self, minimum):
            self.minimum = minimum

        def space_jacobian(self, joints):
            del joints
            return np.diag([1, 1, 1, 1, 1, self.minimum])

    soft = BoundedScrewVelocityIk(
        SingularModel(0.03),
        np.ones(6),
        damping=1e-6,
        singularity_slow_threshold=0.05,
        singularity_stop_threshold=0.01,
        singularity_damping=0.0,
    )
    velocity = soft.solve(np.asarray([1, 0, 0, 0, 0, 0]), np.zeros(6), 0.01)
    assert soft.last_minimum_singular_value == pytest.approx(0.03)
    assert soft.last_velocity_scale == pytest.approx(0.5)
    assert velocity[0] == pytest.approx(0.5, rel=1e-6)

    stopped = BoundedScrewVelocityIk(
        SingularModel(0.005),
        np.ones(6),
        singularity_slow_threshold=0.05,
        singularity_stop_threshold=0.01,
    )
    with pytest.raises(ScrewIkFailure, match="singularity stop"):
        stopped.solve(np.ones(6), np.zeros(6), 0.01)


def test_hybrid_rejects_torque_limit_above_eight_newton_metres():
    model = IdentityModel()
    admittance = ResistiveAdmittance(
        virtual_mass=np.ones(6),
        damping=np.ones(6),
        stiffness=np.ones(6),
        wrench_deadband=np.zeros(6),
        wrench_limit=np.ones(6),
        offset_limit=np.ones(6),
        velocity_limit=np.ones(6),
        wrench_filter_hz=0.0,
    )

    with pytest.raises(ValueError, match=r"\(0, 8\] N.m"):
        HybridCartesianController(
            model,
            admittance,
            BoundedScrewVelocityIk(model, np.ones(6) * 0.5),
            np.zeros(6),
            np.zeros(6),
            np.zeros(6),
            np.zeros(6),
            np.ones(6) * 8.01,
        )
