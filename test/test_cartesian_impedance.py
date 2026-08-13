"""Formula-level tests for Cartesian impedance in one base-frame convention."""

from pathlib import Path

import modern_robotics as mr
import numpy as np
import pytest

from armbycontroller.cartesian_impedance import cartesian_impedance_command
from armbycontroller.cartesian_impedance import cartesian_impedance_diagonals
from armbycontroller.cartesian_impedance import cartesian_pose_error
from armbycontroller.cartesian_impedance import equivalent_cartesian_impedance
from armbycontroller.cartesian_impedance import equivalent_joint_impedance
from armbycontroller.cartesian_impedance import geometric_jacobian
from armbycontroller.screw_model import UrdfScrewModel


class FixedModel:
    """Minimal PoE model seam with explicit pose and space Jacobian."""

    def __init__(self, pose=None, space_jacobian=None):
        self.pose = np.eye(4) if pose is None else np.asarray(pose, dtype=float)
        self.jacobian = (
            np.eye(6)
            if space_jacobian is None
            else np.asarray(space_jacobian, dtype=float)
        )

    def forward_kinematics(self, joint_positions):
        del joint_positions
        return self.pose.copy()

    def space_jacobian(self, joint_positions):
        del joint_positions
        return self.jacobian.copy()

    def mass_matrix(self, joint_positions):
        return np.eye(np.asarray(joint_positions).size)


class FixedDynamics:
    def __init__(self, support):
        self.support = np.asarray(support, dtype=float)
        self.received = None

    def inverse_dynamics(self, position, velocity, acceleration):
        self.received = (
            np.asarray(position, dtype=float).copy(),
            np.asarray(velocity, dtype=float).copy(),
            np.asarray(acceleration, dtype=float).copy(),
        )
        return self.support.copy()


def transform(rotation=None, translation=None):
    result = np.eye(4)
    if rotation is not None:
        result[:3, :3] = rotation
    if translation is not None:
        result[:3, 3] = translation
    return result


def test_base_z_rotation_stiffness_is_independent_from_xy_rotation():
    stiffness, damping = cartesian_impedance_diagonals(
        rotation_stiffness=0.4,
        base_z_rotation_stiffness=4.0,
        translation_stiffness=10.0,
        rotation_damping=0.08,
        translation_damping=0.8,
    )

    assert stiffness == pytest.approx([0.4, 0.4, 4.0, 10.0, 10.0, 10.0])
    assert damping == pytest.approx([0.08, 0.08, 0.08, 0.8, 0.8, 0.8])


def test_space_jacobian_is_converted_to_tool_origin_geometric_jacobian():
    pose = transform(translation=[0.4, -0.2, 0.3])
    space = np.arange(36, dtype=float).reshape(6, 6) / 10.0

    jacobian, returned_pose = geometric_jacobian(
        FixedModel(pose, space), np.zeros(6)
    )

    p_cross = np.asarray([
        [0.0, -0.3, -0.2],
        [0.3, 0.0, -0.4],
        [0.2, 0.4, 0.0],
    ])
    assert returned_pose == pytest.approx(pose)
    assert jacobian[:3] == pytest.approx(space[:3])
    assert jacobian[3:] == pytest.approx(
        space[3:] - p_cross @ space[:3]
    )


def test_pose_error_uses_base_frame_angular_then_linear_order():
    angle = 0.2
    target_rotation = mr.MatrixExp3(
        mr.VecToso3(np.asarray([0.0, 0.0, angle]))
    )

    error = cartesian_pose_error(
        np.eye(4),
        transform(target_rotation, [0.1, -0.2, 0.3]),
    )

    assert error == pytest.approx(
        [0.0, 0.0, angle, 0.1, -0.2, 0.3]
    )


def test_cartesian_impedance_is_full_jacobian_transpose_wrench_mapping():
    jacobian = np.asarray([
        [1.0, 0.2],
        [0.0, 1.0],
        [0.0, 0.0],
        [0.5, 0.0],
        [0.0, 0.4],
        [0.0, 0.0],
    ])
    stiffness = np.asarray([
        [2.0, 0.5, 0.0, 0.0, 0.0, 0.0],
        [0.5, 3.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 10.0, 2.0, 0.0],
        [0.0, 0.0, 0.0, 2.0, 20.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 30.0],
    ])
    damping = np.diag([0.2, 0.3, 0.4, 1.0, 1.5, 2.0])
    position = np.zeros(2)
    velocity = np.asarray([0.1, -0.2])
    target = transform(translation=[0.01, -0.02, 0.0])
    desired_twist = np.asarray([0.0, 0.1, 0.0, 0.02, 0.0, 0.0])

    result = cartesian_impedance_command(
        FixedModel(space_jacobian=jacobian),
        None,
        position,
        velocity,
        target,
        desired_twist,
        stiffness,
        damping,
    )

    expected_error = np.asarray([0.0, 0.0, 0.0, 0.01, -0.02, 0.0])
    expected_twist = jacobian @ velocity
    expected_wrench = (
        stiffness @ expected_error
        + damping @ (desired_twist - expected_twist)
    )
    assert result.pose_error == pytest.approx(expected_error)
    assert result.measured_twist == pytest.approx(expected_twist)
    assert result.commanded_wrench == pytest.approx(expected_wrench)
    assert result.task_torque == pytest.approx(jacobian.T @ expected_wrench)
    assert result.command_torque == pytest.approx(result.task_torque)


def test_seven_axis_nullspace_impedance_restores_only_redundant_motion():
    jacobian = np.hstack((np.eye(6), np.zeros((6, 1))))
    measured = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3])
    velocity = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2])

    result = cartesian_impedance_command(
        FixedModel(space_jacobian=jacobian),
        None,
        measured,
        velocity,
        np.eye(4),
        np.zeros(6),
        np.zeros(6),
        np.zeros(6),
        nullspace_reference=np.zeros(7),
        nullspace_reference_velocity=np.zeros(7),
        nullspace_stiffness=0.4,
        nullspace_damping=0.1,
    )

    assert result.task_torque == pytest.approx(np.zeros(7))
    assert result.nullspace_torque == pytest.approx(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.14]
    )
    assert jacobian @ result.nullspace_torque == pytest.approx(np.zeros(6))
    assert result.command_torque == pytest.approx(result.nullspace_torque)


def test_nero_revo2_nullspace_torque_has_no_dynamic_task_leakage():
    urdf = (
        Path(__file__).resolve().parents[1]
        / "agx_arm_urdf/nero/urdf/nero_with_left_revo2_description.xacro"
    )
    model = UrdfScrewModel(
        urdf, "base_link", "link7", 7, [0.0, 0.0, -9.80665]
    )
    measured = np.asarray([0.1, 0.35, -0.2, 0.45, 0.15, -0.25, 0.2])
    jacobian, pose = geometric_jacobian(model, measured)
    _, _, right_vectors = np.linalg.svd(jacobian)
    redundant_direction = right_vectors[-1]
    reference = measured + 0.2 * redundant_direction

    result = cartesian_impedance_command(
        model,
        None,
        measured,
        np.zeros(7),
        pose,
        np.zeros(6),
        np.zeros(6),
        np.zeros(6),
        nullspace_reference=reference,
        nullspace_reference_velocity=np.zeros(7),
        nullspace_stiffness=0.4,
        nullspace_damping=0.1,
    )

    dynamic_task_effect = jacobian @ np.linalg.solve(
        model.mass_matrix(measured), result.nullspace_torque
    )
    assert np.linalg.norm(result.nullspace_torque) > 0.1
    assert dynamic_task_effect == pytest.approx(np.zeros(6), abs=1e-9)


def test_nullspace_impedance_rejects_nonpositive_mass_matrix():
    class NonPhysicalMassModel(FixedModel):
        def mass_matrix(self, joint_positions):
            return -np.eye(np.asarray(joint_positions).size)

    with pytest.raises(ValueError, match="positive definite"):
        cartesian_impedance_command(
            NonPhysicalMassModel(),
            None,
            np.zeros(6),
            np.zeros(6),
            np.eye(4),
            np.zeros(6),
            np.zeros(6),
            np.zeros(6),
            nullspace_reference=np.zeros(6),
        )


def test_jacobian_transpose_preserves_instantaneous_power():
    rng = np.random.default_rng(7)
    jacobian = rng.normal(size=(6, 7))
    joint_velocity = rng.normal(size=7)
    wrench = rng.normal(size=6)

    twist = jacobian @ joint_velocity
    joint_torque = jacobian.T @ wrench

    assert joint_torque @ joint_velocity == pytest.approx(wrench @ twist)


def test_local_joint_impedance_equivalence_keeps_full_coupling():
    jacobian = np.asarray([
        [1.0, 1.0],
        [0.0, 1.0],
        [0.0, 0.0],
        [0.5, 0.0],
        [0.0, 0.5],
        [0.0, 0.0],
    ])
    cartesian_stiffness = np.diag([2.0, 3.0, 4.0, 10.0, 20.0, 30.0])
    cartesian_damping = np.diag([0.2, 0.3, 0.4, 1.0, 2.0, 3.0])

    joint_stiffness, joint_damping = equivalent_joint_impedance(
        jacobian, cartesian_stiffness, cartesian_damping
    )

    assert joint_stiffness == pytest.approx(
        jacobian.T @ cartesian_stiffness @ jacobian
    )
    assert joint_damping == pytest.approx(
        jacobian.T @ cartesian_damping @ jacobian
    )
    assert joint_stiffness[0, 1] != 0.0


def test_nonsingular_six_axis_joint_and_cartesian_gains_round_trip_exactly():
    jacobian = np.asarray([
        [1.0, 0.2, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.1, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.2, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.1, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.2],
        [0.1, 0.0, 0.0, 0.0, 0.0, 1.0],
    ])
    cartesian_stiffness = np.diag([2.0, 3.0, 4.0, 10.0, 20.0, 30.0])
    cartesian_damping = np.diag([0.2, 0.3, 0.4, 1.0, 2.0, 3.0])
    joint_stiffness, joint_damping = equivalent_joint_impedance(
        jacobian, cartesian_stiffness, cartesian_damping
    )

    recovered_stiffness, recovered_damping = (
        equivalent_cartesian_impedance(
            jacobian, joint_stiffness, joint_damping
        )
    )

    assert recovered_stiffness == pytest.approx(cartesian_stiffness)
    assert recovered_damping == pytest.approx(cartesian_damping)


@pytest.mark.parametrize(
    "jacobian",
    (
        np.eye(6, 7),
        np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 0.0]),
    ),
)
def test_exact_joint_to_cartesian_gain_conversion_rejects_nonunique_cases(
    jacobian,
):
    joint_count = jacobian.shape[1]
    with pytest.raises(ValueError, match="nonsingular 6x6"):
        equivalent_cartesian_impedance(
            jacobian, np.eye(joint_count), np.eye(joint_count)
        )


def test_urdf_support_uses_measured_state_and_zero_desired_acceleration():
    dynamics = FixedDynamics([1.0, -2.0])
    position = np.asarray([0.2, -0.3])
    velocity = np.asarray([0.1, -0.2])

    result = cartesian_impedance_command(
        FixedModel(space_jacobian=np.zeros((6, 2))),
        dynamics,
        position,
        velocity,
        np.eye(4),
        np.zeros(6),
        np.eye(6),
        np.eye(6),
    )

    assert dynamics.received[0] == pytest.approx(position)
    assert dynamics.received[1] == pytest.approx(velocity)
    assert dynamics.received[2] == pytest.approx([0.0, 0.0])
    assert result.model_torque == pytest.approx([1.0, -2.0])
    assert result.command_torque == pytest.approx([1.0, -2.0])


def test_command_torque_has_no_change_rate_limit_between_evaluations():
    stiffness = np.diag([0.0, 0.0, 0.0, 50.0, 0.0, 0.0])
    common = (
        FixedModel(),
        None,
        np.zeros(6),
        np.zeros(6),
    )

    first = cartesian_impedance_command(
        *common,
        np.eye(4),
        np.zeros(6),
        stiffness,
        np.zeros((6, 6)),
    )
    second = cartesian_impedance_command(
        *common,
        transform(translation=[2.0, 0.0, 0.0]),
        np.zeros(6),
        stiffness,
        np.zeros((6, 6)),
    )

    assert first.command_torque == pytest.approx(np.zeros(6))
    assert second.command_torque == pytest.approx(
        [0.0, 0.0, 0.0, 100.0, 0.0, 0.0]
    )
    assert second.command_torque - first.command_torque == pytest.approx(
        [0.0, 0.0, 0.0, 100.0, 0.0, 0.0]
    )


@pytest.mark.parametrize(
    "matrix",
    (
        np.diag([1.0, 1.0, 1.0, 1.0, 1.0, -0.1]),
        np.asarray([
            [1.0, 0.2, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ]),
    ),
)
def test_impedance_matrices_must_be_symmetric_positive_semidefinite(matrix):
    with pytest.raises(ValueError, match="symmetric positive semidefinite"):
        equivalent_joint_impedance(np.eye(6), matrix, np.eye(6))
