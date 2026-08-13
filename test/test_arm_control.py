"""Tests for shared arm IK, keyboard state, and hand compatibility."""

from collections import deque
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import modern_robotics as mr
import numpy as np
import pytest

from armbycontroller.ik_core import AgxIkEngine
from armbycontroller.ik_core import create_screw_solver
from armbycontroller.ik_core import IkFailure
from armbycontroller.ik_core import increment_tool_orientation
from armbycontroller.ik_core import make_pointing_quaternion
from armbycontroller.ik_core import pointing_error_angle
from armbycontroller.ik_core import prepare_planned_joint_mode
from armbycontroller.ik_core import quaternion_to_rotation_matrix
from armbycontroller.ik_core import radial_workspace_check
from armbycontroller.ik_core import rotation_error_angle
from armbycontroller.ik_core import rotation_matrix_to_quaternion
from armbycontroller.ik_core import resolve_firmware_name
from armbycontroller.ik_core import resolve_urdf_path
from armbycontroller.ik_core import set_joint_acceleration_limits
from armbycontroller.ik_core import solve_pointing_ik
from armbycontroller.keyboard_controller import ArmJointJogState
from armbycontroller.keyboard_controller import ArmKeyboardController
from armbycontroller.keyboard_controller import bounded_model_feedforward
from armbycontroller.keyboard_controller import expand_joint_values
from armbycontroller.keyboard_controller import default_mit_gains
from armbycontroller.keyboard_controller import default_mit_feedforward
from armbycontroller.keyboard_controller import KEY_COUNT
from armbycontroller.keyboard_controller import KEY_DECREASE
from armbycontroller.keyboard_controller import KEY_ESTOP
from armbycontroller.keyboard_controller import KEY_HOME
from armbycontroller.keyboard_controller import KEY_INCREASE
from armbycontroller.keyboard_controller import KEY_IMPEDANCE_TOGGLE
from armbycontroller.keyboard_controller import KEY_MODE_TOGGLE
from armbycontroller.keyboard_controller import JointTrajectoryState
from armbycontroller.keyboard_controller import limit_mit_combined_torque
from armbycontroller.lie import rotation_from_vector
from armbycontroller.lie import rotation_vector
from armbycontroller.lie import space_pose_error
from armbycontroller.lie import transform as screw_transform
from armbycontroller.motion_link_bridge import phone_rotation
from armbycontroller.motion_link_bridge import relative_target_rotation
from armbycontroller.motion_link_bridge import websocket_url
from armbycontroller.pose_controller import PoseController
from armbycontroller.screw_model import UrdfScrewModel
from armbycontroller.screw_model import project_gravity_vector


def test_keyboard_launch_exposes_cartesian_and_nero_nullspace_gains():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch" / "keyboard_control.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "keyboard_control_launch", launch_path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert module.COMMON[
        "cartesian_impedance_base_z_rotation_stiffness"
    ] == "4.0"
    assert module.COMMON[
        "cartesian_impedance_nullspace_stiffness"
    ] == "0.4"
    assert module.COMMON[
        "cartesian_impedance_nullspace_damping"
    ] == "0.1"


def _write_test_urdf(tmp_path, body):
    path = tmp_path / "test.urdf"
    path.write_text(body, encoding="utf-8")
    return path


def test_nero_mount_selection_sets_base_frame_gravity():
    assert project_gravity_vector("horizontal") == (0.0, 0.0, -9.80665)
    assert project_gravity_vector("side") == (-9.80665, 0.0, 0.0)
    with pytest.raises(ValueError, match="horizontal or side"):
        project_gravity_vector("select")


def test_motion_link_websocket_url_uses_robot_role():
    assert websocket_url(
        "http://127.0.0.1:8080", "a token/+"
    ) == (
        "ws://127.0.0.1:8080/ws?"
        "session=a%20token%2F%2B&role=robot"
    )


def test_phone_relative_rotation_is_bounded():
    reference = np.eye(3)
    phone_zero = phone_rotation(
        {"alpha": 0.0, "beta": 0.0, "gamma": 0.0}
    )
    phone_moved = phone_rotation(
        {"alpha": 90.0, "beta": 0.0, "gamma": 0.0}
    )

    target = relative_target_rotation(
        reference, phone_zero, phone_moved, 0.5
    )
    angle = np.linalg.norm(rotation_vector(target))

    assert angle == pytest.approx(0.5)
    assert target.T @ target == pytest.approx(np.eye(3), abs=1e-10)


def test_rotation_vector_exponential_and_logarithm_are_inverse():
    vector = np.asarray([0.2, -0.1, 0.3])

    assert rotation_vector(rotation_from_vector(vector)) == pytest.approx(
        vector
    )


def test_space_pose_error_is_full_base_frame_se3_logarithm():
    expected = np.asarray([0.2, -0.1, 0.3, 0.04, -0.03, 0.02])
    current = screw_transform(
        rotation_from_vector([0.1, 0.2, -0.1]), [0.3, -0.2, 0.4]
    )
    displacement = mr.MatrixExp6(mr.VecTose3(expected))
    desired = displacement @ current

    error = space_pose_error(current, desired)

    assert error == pytest.approx(expected)
    assert error[3:] != pytest.approx(
        desired[:3, 3] - current[:3, 3]
    )


def test_nero_urdf_resolves_from_validated_screw_package():
    path = resolve_urdf_path("", "nero")

    assert path.name == "nero_description.urdf"
    assert "nero_screw_dynamics" in str(path)


@pytest.mark.parametrize(
    ("model_name", "joint_count", "tip_link", "target"),
    [
        (
            "nero",
            7,
            "link7",
            [0.1, 0.2, -0.15, 0.25, 0.1, -0.1, 0.1],
        ),
        (
            "piper_l",
            6,
            "link6",
            [0.1, 0.2, -0.15, 0.25, 0.1, -0.1],
        ),
    ],
)
def test_screw_fk_and_ik_support_nero_and_piper(
    model_name, joint_count, tip_link, target
):
    urdf = (
        __import__("pathlib").Path(__file__).resolve().parents[1]
        / "agx_arm_urdf" / model_name / "urdf"
        / f"{model_name}_description.urdf"
    )
    model = UrdfScrewModel(
        urdf,
        base_link="base_link",
        tip_link=tip_link,
        joint_count=joint_count,
    )
    target_pose = model.forward_kinematics(target)

    assert model.urdf_forward_kinematics(target) == pytest.approx(
        target_pose, abs=1e-10
    )

    solver = create_screw_solver(
        urdf, "base_link", tip_link, joint_count, 0.02, 1e-5
    )
    solution = solver.ik(
        target_pose[:3, 3],
        target_pose[:3, :3],
        np.zeros(joint_count),
    )

    assert solution is not None
    assert solver.model.forward_kinematics(solution) == pytest.approx(
        target_pose, abs=1e-4
    )


def test_urdf_gravity_model_compensates_one_link_pendulum(tmp_path):
    urdf = _write_test_urdf(
        tmp_path,
        """<robot name="pendulum">
        <link name="base"/>
        <link name="tip"><inertial><origin xyz="1 0 0"/>
          <mass value="2"/><inertia ixx="1" ixy="0" ixz="0"
          iyy="1" iyz="0" izz="1"/></inertial></link>
        <joint name="joint1" type="revolute"><parent link="base"/>
          <child link="tip"/><axis xyz="0 1 0"/>
          <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
        </joint></robot>""",
    )
    model = UrdfScrewModel(urdf, "base", "tip", 1)

    assert model.movable_joint_names == ["joint1"]
    assert model.compensation([0.0]) == pytest.approx([-2.0 * 9.80665])
    assert model.compensation([math.pi / 2.0]) == pytest.approx([0.0], abs=1e-9)

    # Iyy + m*r^2 = 1 + 2*1^2 = 3 kg*m^2.
    assert model.mass_matrix([0.3]) == pytest.approx(np.array([[3.0]]))
    assert model.coriolis([0.3], [2.0]) == pytest.approx([0.0], abs=1e-9)
    expected = 3.0 * 2.0 + model.compensation([0.0])[0]
    assert model.inverse_dynamics([0.0], [0.0], [2.0]) == pytest.approx(
        [expected]
    )


def test_urdf_gravity_model_includes_all_downstream_masses(tmp_path):
    urdf = _write_test_urdf(
        tmp_path,
        """<robot name="two_link">
        <link name="base"/>
        <link name="middle"><inertial><origin xyz="1 0 0"/>
          <mass value="1"/><inertia ixx="1" ixy="0" ixz="0"
          iyy="1" iyz="0" izz="1"/></inertial></link>
        <link name="tip"><inertial><origin xyz="1 0 0"/>
          <mass value="1"/><inertia ixx="1" ixy="0" ixz="0"
          iyy="1" iyz="0" izz="1"/></inertial></link>
        <joint name="joint1" type="revolute"><parent link="base"/>
          <child link="middle"/><axis xyz="0 1 0"/></joint>
        <joint name="joint2" type="revolute"><origin xyz="1 0 0"/>
          <parent link="middle"/><child link="tip"/><axis xyz="0 1 0"/>
        </joint></robot>""",
    )
    model = UrdfScrewModel(urdf, "base", "tip", 2)

    expected = 9.80665
    assert model.compensation([0.0, 0.0]) == pytest.approx(
        [-3.0 * expected, -1.0 * expected]
    )
    mass_matrix = model.mass_matrix([0.2, -0.3])
    assert mass_matrix == pytest.approx(mass_matrix.T)
    assert np.all(np.linalg.eigvalsh(mass_matrix) > 0.0)


@pytest.mark.parametrize(
    ("model_name", "joint_count", "tip_link"),
    [("nero", 7, "link7"), ("piper_l", 6, "link6")],
)
def test_copied_arm_urdf_produces_finite_gravity_torque(
    model_name, joint_count, tip_link
):
    urdf = (
        __import__("pathlib").Path(__file__).resolve().parents[1]
        / "agx_arm_urdf" / model_name / "urdf"
        / f"{model_name}_description.urdf"
    )
    model = UrdfScrewModel(urdf, "base_link", tip_link, joint_count)
    torque = model.compensation(np.zeros(joint_count))

    assert torque.shape == (joint_count,)
    assert np.all(np.isfinite(torque))
    # Both bases rotate around gravity, so joint 1 needs no gravity torque.
    assert torque[0] == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize(
    ("model_name", "joint_count", "tip_link", "accessory_file", "added_mass"),
    [
        (
            "piper_l", 6, "link6",
            "piper_l_with_gripper_description.xacro", 0.54,
        ),
        (
            "nero", 7, "link7",
            "nero_with_left_revo2_description.xacro", 0.43771096,
        ),
    ],
)
def test_accessory_xacro_adds_payload_mass_to_arm_gravity_model(
    model_name, joint_count, tip_link, accessory_file, added_mass
):
    root = (
        __import__("pathlib").Path(__file__).resolve().parents[1]
        / "agx_arm_urdf" / model_name / "urdf"
    )
    bare = UrdfScrewModel(
        root / f"{model_name}_description.urdf",
        "base_link", tip_link, joint_count,
    )
    equipped = UrdfScrewModel(
        root / accessory_file, "base_link", tip_link, joint_count,
    )
    bare_torque = bare.compensation(np.zeros(joint_count))
    equipped_torque = equipped.compensation(np.zeros(joint_count))

    assert equipped.movable_joint_names == [
        f"joint{index}" for index in range(1, joint_count + 1)
    ]
    assert equipped.modeled_mass - bare.modeled_mass == pytest.approx(added_mass)
    assert not np.allclose(equipped_torque, bare_torque)


def test_quaternion_conversion_normalizes_input():
    """A non-unit quaternion should produce the expected rotation."""
    rotation = quaternion_to_rotation_matrix(0.0, 0.0, 2.0, 2.0)

    expected = np.array([[0.0, -1.0, 0.0],
                         [1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0]])
    assert rotation == pytest.approx(expected)
    assert rotation_error_angle(rotation, expected) < 1e-7
    quaternion = rotation_matrix_to_quaternion(rotation)
    converted = quaternion_to_rotation_matrix(*quaternion)
    assert converted == pytest.approx(rotation)


@pytest.mark.parametrize(
    "quaternion",
    [(0.0, 0.0, 0.0, 0.0),
     (math.nan, 0.0, 0.0, 1.0),
     (0.0, math.inf, 0.0, 1.0)],
)
def test_quaternion_conversion_rejects_invalid_input(quaternion):
    """Invalid orientations must not reach the IK solver."""
    with pytest.raises(ValueError):
        quaternion_to_rotation_matrix(*quaternion)


def test_simulation_respects_joint_acceleration_limit():
    """The RViz simulation must ramp velocity instead of jumping."""
    controller = object.__new__(PoseController)
    controller.simulated_joints = np.zeros(7)
    controller.simulated_target_joints = np.ones(7)
    controller.simulated_velocities = np.zeros(7)
    controller.joint_max_acceleration = 1.0
    controller.joint_max_velocity = 1.0
    controller.state_period = 0.1

    controller._advance_simulation()
    assert controller.simulated_velocities == pytest.approx(np.full(7, 0.1))
    assert controller.simulated_joints == pytest.approx(np.full(7, 0.01))

    controller._advance_simulation()
    assert controller.simulated_velocities == pytest.approx(np.full(7, 0.2))
    assert controller.simulated_joints == pytest.approx(np.full(7, 0.03))


def test_valid_history_keeps_only_latest_ten_steps():
    """Recovery history must remain bounded and retain the latest state."""
    controller = object.__new__(PoseController)
    controller.valid_history = deque(maxlen=10)
    rotation = np.eye(3)
    for index in range(12):
        values = np.full(7, float(index))
        controller._remember_valid(values[:3], rotation, values)

    assert len(controller.valid_history) == 10
    assert controller.valid_history[0][2] == pytest.approx(np.full(7, 2.0))
    assert controller.valid_history[-1][2] == pytest.approx(np.full(7, 11.0))


def test_radial_workspace_uses_requested_safety_margins():
    """Workspace permits only min+5 cm through max-10 cm."""
    minimum = 0.1447354 + 0.05
    maximum = 0.7374482 - 0.10

    assert radial_workspace_check([minimum, 0.0, 0.0], minimum, maximum)[0]
    assert radial_workspace_check([maximum, 0.0, 0.0], minimum, maximum)[0]
    assert not radial_workspace_check(
        [minimum - 0.001, 0.0, 0.0], minimum, maximum
    )[0]
    assert not radial_workspace_check(
        [maximum + 0.001, 0.0, 0.0], minimum, maximum
    )[0]


def test_pointing_orientation_maps_tool_z_to_requested_direction():
    """The command orientation points link7 +Z down in base_link."""
    quaternion = make_pointing_quaternion(
        [0.0, 0.0, -2.0], [1.0, 0.0, 0.0]
    )
    rotation = quaternion_to_rotation_matrix(*quaternion)

    assert rotation[:, 2] == pytest.approx([0.0, 0.0, -1.0])
    assert rotation.T @ rotation == pytest.approx(np.eye(3))


def test_pointing_orientation_handles_parallel_roll_reference():
    """A parallel reference must select a stable fallback axis."""
    quaternion = make_pointing_quaternion(
        [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]
    )
    rotation = quaternion_to_rotation_matrix(*quaternion)

    assert rotation[:, 2] == pytest.approx([1.0, 0.0, 0.0])


def test_orientation_arrows_point_up_and_left_in_base_frame():
    # Tool +Z initially points base +X.
    initial = np.array([[0.0, 0.0, 1.0],
                        [0.0, 1.0, 0.0],
                        [-1.0, 0.0, 0.0]])
    step = 0.1

    upward = increment_tool_orientation(initial, -step, 0.0, 0.0)
    leftward = increment_tool_orientation(initial, 0.0, step, 0.0)
    rolled = increment_tool_orientation(initial, 0.0, 0.0, step)

    assert upward[:, 2][2] > initial[:, 2][2]
    assert leftward[:, 2][1] > initial[:, 2][1]
    assert rolled[:, 2] == pytest.approx(initial[:, 2])
    assert np.linalg.norm(rolled[:, 0] - initial[:, 0]) > 0.05


def test_pointing_error_ignores_rotation_about_tool_axis():
    target = np.eye(3)
    rolled = np.array([[0.0, -1.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0]])

    assert pointing_error_angle(rolled, target) == pytest.approx(0.0)
    assert rotation_error_angle(rolled, target) == pytest.approx(math.pi / 2)


def test_pointing_ik_selects_roll_solution_nearest_seed():
    class FakeSolver:
        def ik(self, position, rotation, seed_jnt_values):
            del position, seed_jnt_values
            angle = math.atan2(rotation[1, 0], rotation[0, 0])
            return np.array([angle, 0.0])

    solution = solve_pointing_ik(
        FakeSolver(), np.zeros(3), np.eye(3), np.array([1.4, 0.0]), 4
    )

    assert solution == pytest.approx([math.pi / 2, 0.0])


def test_shared_engine_solves_and_fk_verifies_pointing_target():
    class ExactSolver:
        def ik(self, position, rotation, seed_jnt_values):
            del rotation, seed_jnt_values
            return np.asarray(position[:2], dtype=float)

        def fk(self, joints):
            return np.array([joints[0], joints[1], 0.2]), np.eye(3)

    engine = AgxIkEngine(
        ExactSolver(), 2, 0.1, 1.0, 1e-9, 1e-9, roll_samples=1
    )
    result = engine.solve(
        np.array([0.3, 0.1, 0.2]), np.eye(3), np.zeros(2)
    )

    assert result.joints == pytest.approx([0.3, 0.1])
    assert result.position_error == pytest.approx(0.0)
    assert result.orientation_error == pytest.approx(0.0)


def test_shared_engine_rejects_workspace_before_calling_solver():
    class UnusedSolver:
        def ik(self, *args, **kwargs):
            raise AssertionError("IK must not run outside workspace")

    engine = AgxIkEngine(
        UnusedSolver(), 2, 0.2, 0.5, 1e-4, 1e-3
    )

    with pytest.raises(IkFailure, match="workspace radius"):
        engine.solve(np.array([0.6, 0.0, 0.0]), np.eye(3), np.zeros(2))


LIMITS = [(-1.0, 1.0)] * 7


def test_auto_firmware_matches_verified_hardware_versions():
    assert resolve_firmware_name("nero", "auto") == "v112"
    assert resolve_firmware_name("piper_l", "auto") == "v188"
    assert resolve_firmware_name("piper_l", "v189") == "v189"


def keys(*pressed):
    state = [0] * KEY_COUNT
    for index in pressed:
        state[index] = 1
    return state


def test_selects_joint_and_only_jogs_that_joint():
    jog = ArmJointJogState(LIMITS, 0.1)
    update = jog.update(keys(3, KEY_INCREASE))
    assert update.selected_joint == 3
    assert update.selection_changed and update.target_changed
    assert jog.target_joints == pytest.approx(
        [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
    )


def test_holding_joint_key_clamps_at_limit():
    jog = ArmJointJogState(LIMITS, 0.6)
    jog.update(keys(KEY_INCREASE))
    jog.update(keys(KEY_INCREASE))
    update = jog.update(keys(KEY_INCREASE))
    assert jog.target_joints[0] == 1.0
    assert not update.target_changed


def test_opposite_joint_keys_cancel():
    jog = ArmJointJogState(LIMITS, 0.1)
    assert not jog.update(keys(KEY_DECREASE, KEY_INCREASE)).target_changed


def test_home_is_edge_triggered():
    jog = ArmJointJogState(LIMITS, 0.1, [0.2] * 7)
    first = jog.update(keys(KEY_HOME))
    second = jog.update(keys(KEY_HOME))
    assert first.home_requested and first.target_changed
    assert not second.home_requested
    assert jog.target_joints == [0.0] * 7


def test_estop_and_mode_toggle_are_edge_triggered_and_suppress_jog():
    jog = ArmJointJogState(LIMITS, 0.1)
    first = jog.update(keys(KEY_ESTOP, KEY_INCREASE))
    second = jog.update(keys(KEY_ESTOP))
    assert first.estop_requested and not first.target_changed
    assert not second.estop_requested

    jog.update(keys())
    first = jog.update(keys(KEY_MODE_TOGGLE, KEY_INCREASE))
    second = jog.update(keys(KEY_MODE_TOGGLE))
    assert first.mode_toggle_requested and not first.target_changed
    assert not second.mode_toggle_requested


def test_sync_target_clamps_target_by_default():
    jog = ArmJointJogState(LIMITS, math.radians(1.0))
    jog.sync_target([-2.0, -0.5, 0.0, 0.5, 2.0, 0.1, -0.1])
    assert jog.target_joints == pytest.approx(
        [-1.0, -0.5, 0.0, 0.5, 1.0, 0.1, -0.1]
    )


def test_out_of_limit_feedback_recovers_one_step_without_jump():
    jog = ArmJointJogState(LIMITS, 0.1)
    jog.sync_target([1.2, -1.2, 0.0, 0.0, 0.0, 0.0, 0.0],
                    clamp_to_limits=False)

    blocked = jog.update(keys(KEY_INCREASE))
    jog.update(keys())
    inward = jog.update(keys(KEY_DECREASE))

    assert not blocked.target_changed
    assert inward.target_changed
    assert jog.target_joints[0] == pytest.approx(1.1)
    assert jog.target_joints[1] == pytest.approx(-1.2)


def test_piper_l_uses_same_keys_but_ignores_joint_seven():
    jog = ArmJointJogState([(-1.0, 1.0)] * 6, 0.1)

    ignored = jog.update(keys(6, KEY_INCREASE))
    jog.update(keys())
    selected = jog.update(keys(5, KEY_INCREASE))

    assert ignored.selected_joint == 0
    assert jog.target_joints[0] == pytest.approx(0.1)
    assert selected.selected_joint == 5
    assert jog.target_joints[5] == pytest.approx(0.1)


def test_impedance_toggle_is_edge_triggered_and_suppresses_jog():
    jog = ArmJointJogState(LIMITS, 0.1)
    first = jog.update(keys(KEY_IMPEDANCE_TOGGLE, KEY_INCREASE))
    second = jog.update(keys(KEY_IMPEDANCE_TOGGLE))

    assert first.impedance_toggle_requested
    assert not first.target_changed
    assert not second.impedance_toggle_requested


def test_mit_gains_expand_from_scalar_and_validate_joint_count():
    assert expand_joint_values([0.8], 6, "kd") == [0.8] * 6
    with pytest.raises(ValueError, match="1 or 6"):
        expand_joint_values([1.0, 2.0], 6, "kp")


def test_piper_mit_gains_are_independent_per_joint():
    kp, kd = default_mit_gains("piper_l")

    assert kp == [0.3, 0.5, 0.5, 0.5, 1.0, 0.3]
    assert kd == [0.01] * 6
    assert default_mit_feedforward("piper_l") == [0.0] * 6
    assert len(set(kp)) > 1


def test_model_feedforward_is_immediate_and_limits_each_joint():
    gravity = [4.0, -10.0]
    residual = [0.2, -0.1]
    limits = [5.0, 5.0]

    assert bounded_model_feedforward(
        gravity, residual, 1.0, limits
    ) == pytest.approx([4.2, -5.0])


def test_combined_torque_limiter_uses_maximum_available_cancellation():
    feedforward, total = limit_mit_combined_torque(
        feedforward=[0.0],
        reference_position=[3.0],
        reference_velocity=[0.0],
        measured_position=[0.0],
        measured_velocity=[0.0],
        kp=[10.0],
        kd=[0.2],
        torque_limit=[10.0],
    )

    assert feedforward == pytest.approx([-10.0])
    assert total == pytest.approx([20.0])


def test_joint_trajectory_generates_bounded_q_dq_ddq():
    trajectory = JointTrajectoryState(
        2, max_velocity=[0.5] * 2,
        max_acceleration=[1.0] * 2, max_jerk=[5.0] * 2,
    )
    previous_acceleration = np.zeros(2)
    maximum_jerk = 0.0
    for _ in range(1000):
        position, velocity, acceleration = trajectory.step(
            [0.5, -0.5], 0.01
        )
        maximum_jerk = max(
            maximum_jerk,
            float(np.max(np.abs(acceleration - previous_acceleration)) / 0.01),
        )
        previous_acceleration = acceleration

    assert position == pytest.approx([0.5, -0.5], abs=1e-6)
    assert velocity == pytest.approx([0.0, 0.0], abs=1e-6)
    assert np.max(np.abs(acceleration)) <= 1.0
    assert maximum_jerk <= 5.0 + 1e-9


def test_mit_tick_sends_one_impedance_command_per_joint(monkeypatch):
    class FakeArm:
        def __init__(self):
            self.commands = []

        def move_mit(self, **command):
            self.commands.append(command)

        def get_motor_states(self, joint_index):
            del joint_index
            return SimpleNamespace(msg=SimpleNamespace(velocity=0.3))

        def get_joint_angles(self):
            return SimpleNamespace(msg=[0.0] * 6)

    class FakeLogger:
        def info(self, message, **kwargs):
            del message, kwargs

        def warning(self, message, **kwargs):
            del message, kwargs

    controller = object.__new__(ArmKeyboardController)
    controller.impedance_enabled = True
    controller.emergency_stopped = False
    controller.execute_motion = True
    controller.arm_ready = True
    controller.arm = FakeArm()
    controller.joint_count = 6
    controller.jog = ArmJointJogState([(-1.0, 1.0)] * 6, 0.1)
    controller.mit_kp = [10.0] * 6
    controller.mit_kd = [0.8] * 6
    controller.mit_feedforward = [0.0] * 6
    controller.mit_gravity_torque_limit = [10.0] * 6
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )

    controller.mit_tick()

    assert [item["joint_index"] for item in controller.arm.commands] == [
        1, 2, 3, 4, 5, 6
    ]
    assert all(item["kp"] == 10.0 for item in controller.arm.commands)
    assert all(item["kd"] == 0.8 for item in controller.arm.commands)


def test_cartesian_mit_tick_sends_strict_torque_through_zero_gain_tff(
    monkeypatch,
):
    class FakeArm:
        def __init__(self):
            self.commands = []

        def move_mit(self, **command):
            self.commands.append(command)

        def get_motor_states(self, joint_index):
            del joint_index
            return SimpleNamespace(msg=SimpleNamespace(velocity=0.0))

        def get_joint_angles(self):
            return SimpleNamespace(msg=[0.0] * 6)

    class FakeCartesianModel:
        def forward_kinematics(self, joints):
            pose = np.eye(4)
            pose[0, 3] = float(np.asarray(joints)[3])
            return pose

        def space_jacobian(self, joints):
            del joints
            return np.eye(6)

        def inverse_dynamics(self, positions, velocities, accelerations):
            del positions, velocities, accelerations
            return np.asarray([0.1, -0.2, 0.3, 0.4, 0.5, 0.6])

    class FakeLogger:
        def info(self, message, **kwargs):
            del message, kwargs

        def warning(self, message, **kwargs):
            del message, kwargs

        def error(self, message, **kwargs):
            del message, kwargs

    controller = object.__new__(ArmKeyboardController)
    controller.impedance_enabled = True
    controller.impedance_backend = "cartesian"
    controller.emergency_stopped = False
    controller.execute_motion = True
    controller.arm_ready = True
    controller.arm_connected = True
    controller.arm = FakeArm()
    controller.joint_count = 6
    controller.jog = ArmJointJogState([(-3.0, 3.0)] * 6, 0.1)
    controller.jog.sync_target([0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
    controller.mit_trajectory = None
    controller.mit_command_rate = 100.0
    controller.last_mit_tick_time = None
    controller.gravity_model = FakeCartesianModel()
    controller.cartesian_stiffness = [0.0, 0.0, 0.0, 10.0, 0.0, 0.0]
    controller.cartesian_damping = [0.0] * 6
    controller.cartesian_torque_limit = [8.0] * 6
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )

    controller.mit_tick()

    assert [command["joint_index"] for command in controller.arm.commands] == [
        1, 2, 3, 4, 5, 6
    ]
    assert [command["kp"] for command in controller.arm.commands] == [0.0] * 6
    assert [command["kd"] for command in controller.arm.commands] == [0.0] * 6
    assert [command["p_des"] for command in controller.arm.commands] == [0.0] * 6
    assert [command["v_des"] for command in controller.arm.commands] == [0.0] * 6
    assert [command["t_ff"] for command in controller.arm.commands] == (
        pytest.approx([0.1, -0.2, 0.3, 2.4, 0.5, 0.6])
    )


def test_nero_cartesian_mit_sends_complete_cycle_with_nullspace_torque(
    monkeypatch,
):
    class FakeArm:
        def __init__(self):
            self.commands = []

        def move_mit(self, **command):
            self.commands.append(command)

        def get_motor_states(self, joint_index):
            velocity = 0.2 if joint_index == 7 else 0.0
            return SimpleNamespace(msg=SimpleNamespace(velocity=velocity))

        def get_joint_angles(self):
            return SimpleNamespace(msg=[0.0] * 6 + [0.3])

    class FakeNeroModel:
        def forward_kinematics(self, joints):
            del joints
            return np.eye(4)

        def space_jacobian(self, joints):
            del joints
            return np.hstack((np.eye(6), np.zeros((6, 1))))

        def mass_matrix(self, joints):
            del joints
            return np.eye(7)

        def inverse_dynamics(self, positions, velocities, accelerations):
            del positions, velocities, accelerations
            return np.zeros(7)

    class FakeLogger:
        def info(self, message, **kwargs):
            del message, kwargs

        def warning(self, message, **kwargs):
            del message, kwargs

        def error(self, message, **kwargs):
            del message, kwargs

    controller = object.__new__(ArmKeyboardController)
    controller.robot_model = "nero"
    controller.impedance_enabled = True
    controller.impedance_backend = "cartesian"
    controller.emergency_stopped = False
    controller.execute_motion = True
    controller.arm_ready = True
    controller.arm_connected = True
    controller.arm = FakeArm()
    controller.joint_count = 7
    controller.jog = ArmJointJogState([(-3.0, 3.0)] * 7, 0.1)
    controller.jog.sync_target([0.0] * 7)
    controller.mit_trajectory = None
    controller.mit_command_rate = 100.0
    controller.last_mit_tick_time = None
    controller.gravity_model = FakeNeroModel()
    controller.cartesian_stiffness = [0.0] * 6
    controller.cartesian_damping = [0.0] * 6
    controller.cartesian_nullspace_stiffness = [0.4] * 7
    controller.cartesian_nullspace_damping = [0.1] * 7
    controller.cartesian_torque_limit = [8.0] * 7
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )

    controller.mit_tick()

    assert [command["joint_index"] for command in controller.arm.commands] == (
        [1, 2, 3, 4, 5, 6, 7]
    )
    assert [command["kp"] for command in controller.arm.commands] == [0.0] * 7
    assert [command["kd"] for command in controller.arm.commands] == [0.0] * 7
    assert [command["t_ff"] for command in controller.arm.commands] == (
        pytest.approx([0.0] * 6 + [-0.14])
    )


def test_cartesian_mit_uses_immediate_absolute_limit_not_torque_rate_limit(
    monkeypatch,
):
    class FakeArm:
        def __init__(self):
            self.commands = []

        def move_mit(self, **command):
            self.commands.append(command)

        def get_motor_states(self, joint_index):
            del joint_index
            return SimpleNamespace(msg=SimpleNamespace(velocity=0.0))

        def get_joint_angles(self):
            return SimpleNamespace(msg=[0.0] * 6)

    class FakeCartesianModel:
        def forward_kinematics(self, joints):
            pose = np.eye(4)
            pose[0, 3] = float(np.asarray(joints)[3])
            return pose

        def space_jacobian(self, joints):
            del joints
            return np.eye(6)

        def inverse_dynamics(self, positions, velocities, accelerations):
            del positions, velocities, accelerations
            return np.zeros(6)

    class FakeLogger:
        def info(self, message, **kwargs):
            del message, kwargs

        def warning(self, message, **kwargs):
            del message, kwargs

        def error(self, message, **kwargs):
            del message, kwargs

    controller = object.__new__(ArmKeyboardController)
    controller.impedance_enabled = True
    controller.impedance_backend = "cartesian"
    controller.emergency_stopped = False
    controller.execute_motion = True
    controller.arm_ready = True
    controller.arm_connected = True
    controller.arm = FakeArm()
    controller.joint_count = 6
    controller.jog = ArmJointJogState([(-3.0, 3.0)] * 6, 0.1)
    controller.jog.sync_target([0.0] * 6)
    controller.mit_trajectory = None
    controller.mit_command_rate = 100.0
    controller.last_mit_tick_time = None
    controller.gravity_model = FakeCartesianModel()
    controller.cartesian_stiffness = [0.0, 0.0, 0.0, 10.0, 0.0, 0.0]
    controller.cartesian_damping = [0.0] * 6
    controller.cartesian_torque_limit = [8.0] * 6
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )

    controller.mit_tick()
    controller.jog.sync_target([0.0, 0.0, 0.0, 2.0, 0.0, 0.0])
    controller.mit_tick()

    first_cycle = controller.arm.commands[:6]
    second_cycle = controller.arm.commands[6:]
    assert [command["t_ff"] for command in first_cycle] == [0.0] * 6
    assert [command["t_ff"] for command in second_cycle] == (
        pytest.approx([0.0, 0.0, 0.0, 8.0, 0.0, 0.0])
    )


def test_mit_tick_computes_gravity_from_measured_joint_positions(monkeypatch):
    class FakeArm:
        def __init__(self):
            self.commands = []

        def move_mit(self, **command):
            self.commands.append(command)

        def get_motor_states(self, joint_index):
            del joint_index
            return SimpleNamespace(msg=SimpleNamespace(velocity=0.0))

        def get_joint_angles(self):
            return SimpleNamespace(msg=[0.25, -0.5])

    class FakeGravityModel:
        def __init__(self):
            self.received = None

        def compensation(self, joints):
            self.received = list(joints)
            return np.array([2.0, -3.0])

    class FakeLogger:
        def info(self, message, **kwargs):
            del message, kwargs

        def warning(self, message, **kwargs):
            del message, kwargs

    controller = object.__new__(ArmKeyboardController)
    controller.impedance_enabled = True
    controller.emergency_stopped = False
    controller.execute_motion = True
    controller.arm_ready = True
    controller.arm = FakeArm()
    controller.joint_count = 2
    controller.jog = ArmJointJogState([(-1.0, 1.0)] * 2, 0.1)
    controller.jog.sync_target([0.8, 0.8])
    controller.mit_kp = [1.0] * 2
    controller.mit_kd = [0.2] * 2
    controller.mit_feedforward = [0.0] * 2
    controller.gravity_model = FakeGravityModel()
    controller.mit_gravity_scale = 1.0
    controller.mit_gravity_torque_limit = [6.0] * 2
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )
    monkeypatch.setattr(
        "armbycontroller.keyboard_controller.time.monotonic", lambda: 100.0
    )

    controller.mit_tick()

    assert controller.gravity_model.received == [0.25, -0.5]
    assert [command["t_ff"] for command in controller.arm.commands] == (
        pytest.approx([2.0, -3.0])
    )


def test_impedance_entry_refuses_incomplete_joint_feedback(monkeypatch):
    class FakeArm:
        def get_joint_angles(self):
            return None

        def get_motor_states(self, joint_index):
            del joint_index
            return SimpleNamespace(msg=SimpleNamespace(velocity=0.0))

    class FakeLogger:
        def __init__(self):
            self.errors = []

        def error(self, message, **kwargs):
            del kwargs
            self.errors.append(message)

    logger = FakeLogger()
    controller = object.__new__(ArmKeyboardController)
    controller.impedance_enabled = False
    controller.execute_motion = True
    controller.arm_ready = True
    controller.arm = FakeArm()
    controller.joint_count = 2
    controller.jog = ArmJointJogState([(-1.0, 1.0)] * 2, 0.1)
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: logger
    )

    controller.toggle_impedance()

    assert not controller.impedance_enabled
    assert logger.errors == [
        "cannot enter MIT: complete q/dq feedback is required"
    ]


def test_mit_tick_uses_trajectory_state_for_full_inverse_dynamics(monkeypatch):
    class FakeArm:
        def __init__(self):
            self.commands = []

        def move_mit(self, **command):
            self.commands.append(command)

        def get_motor_states(self, joint_index):
            del joint_index
            return SimpleNamespace(msg=SimpleNamespace(velocity=0.0))

        def get_joint_angles(self):
            return SimpleNamespace(msg=[0.2, -0.3])

    class FakeDynamicsModel:
        def inverse_dynamics(self, positions, velocities, accelerations):
            self.received = (
                np.asarray(positions), np.asarray(velocities),
                np.asarray(accelerations),
            )
            return np.array([1.5, -2.5])

    class FakeLogger:
        def info(self, message, **kwargs):
            del message, kwargs

        def warning(self, message, **kwargs):
            del message, kwargs

    controller = object.__new__(ArmKeyboardController)
    controller.impedance_enabled = True
    controller.emergency_stopped = False
    controller.execute_motion = True
    controller.arm_ready = True
    controller.arm = FakeArm()
    controller.joint_count = 2
    controller.jog = ArmJointJogState([(-1.0, 1.0)] * 2, 0.1)
    controller.jog.sync_target([0.5, -0.5])
    controller.mit_trajectory = JointTrajectoryState(
        2, [0.5] * 2, [1.0] * 2, [5.0] * 2
    )
    controller.mit_command_rate = 100.0
    controller.last_mit_tick_time = None
    controller.mit_kp = [1.0] * 2
    controller.mit_kd = [0.2] * 2
    controller.mit_feedforward = [0.0] * 2
    controller.gravity_model = FakeDynamicsModel()
    controller.mit_gravity_scale = 1.0
    controller.mit_gravity_torque_limit = [10.0] * 2
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )

    controller.mit_tick()

    positions, velocities, accelerations = controller.gravity_model.received
    assert positions == pytest.approx([0.2, -0.3])
    assert np.any(np.abs(velocities) > 0.0)
    assert np.any(np.abs(accelerations) > 0.0)
    assert [command["p_des"] for command in controller.arm.commands] != [
        0.5, -0.5
    ]
    assert [command["v_des"] for command in controller.arm.commands] == (
        pytest.approx(velocities)
    )
    assert [command["t_ff"] for command in controller.arm.commands] == [
        1.5, -2.5
    ]


def test_mit_tick_keeps_gains_fixed_and_limits_combined_torque(monkeypatch):
    class FakeArm:
        def __init__(self):
            self.commands = []

        def move_mit(self, **command):
            self.commands.append(command)

        def get_motor_states(self, joint_index):
            del joint_index
            return SimpleNamespace(msg=SimpleNamespace(velocity=0.0))

        def get_joint_angles(self):
            return SimpleNamespace(msg=[0.0, 0.0])

    class FakeDynamicsModel:
        def inverse_dynamics(self, positions, velocities, accelerations):
            del positions, velocities, accelerations
            return np.array([5.0, -5.0])

    class FakeLogger:
        def info(self, message, **kwargs):
            del message, kwargs

        def warning(self, message, **kwargs):
            del message, kwargs

    controller = object.__new__(ArmKeyboardController)
    controller.impedance_enabled = True
    controller.emergency_stopped = False
    controller.execute_motion = True
    controller.arm_ready = True
    controller.arm = FakeArm()
    controller.joint_count = 2
    controller.jog = ArmJointJogState([(-2.0, 2.0)] * 2, 0.1)
    controller.jog.sync_target([1.0, -1.0])
    controller.mit_trajectory = None
    controller.mit_command_rate = 100.0
    controller.last_mit_tick_time = None
    controller.mit_kp = [8.0, 8.0]
    controller.mit_kd = [0.2, 0.2]
    controller.mit_feedforward = [0.0, 0.0]
    controller.gravity_model = FakeDynamicsModel()
    controller.mit_gravity_scale = 1.0
    controller.mit_gravity_torque_limit = [10.0, 10.0]
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )

    controller.mit_tick()

    assert [command["kp"] for command in controller.arm.commands] == [8.0, 8.0]
    assert [command["kd"] for command in controller.arm.commands] == [0.2, 0.2]
    assert [command["t_ff"] for command in controller.arm.commands] == [2.0, -2.0]
    estimated_total = [
        command["kp"] * (command["p_des"] - measured_position)
        + command["kd"] * (command["v_des"] - 0.0)
        + command["t_ff"]
        for command, measured_position in zip(
            controller.arm.commands, [0.0, 0.0]
        )
    ]
    assert estimated_total == pytest.approx([10.0, -10.0])


def test_ik_joint_target_is_consumed_only_by_mit_backend(monkeypatch):
    class FakeArm:
        def __init__(self):
            self.mit_commands = []
            self.move_j_commands = []

        def move_mit(self, **command):
            self.mit_commands.append(command)

        def get_motor_states(self, joint_index):
            del joint_index
            return SimpleNamespace(msg=SimpleNamespace(velocity=0.0))

        def move_j(self, target):
            self.move_j_commands.append(target)

    class FakeLogger:
        def info(self, message, **kwargs):
            del message, kwargs

        def warning(self, message, **kwargs):
            del message, kwargs

        def error(self, message, **kwargs):
            del message, kwargs

    controller = object.__new__(ArmKeyboardController)
    controller.impedance_enabled = True
    controller.emergency_stopped = False
    controller.execute_motion = True
    controller.arm_ready = True
    controller.arm = FakeArm()
    controller.joint_count = 6
    controller.jog = ArmJointJogState([(-1.0, 1.0)] * 6, 0.1)
    controller.jog.sync_target([0.1, -0.2, 0.3, 0.0, 0.2, -0.1])
    controller.mit_kp = [1.0] * 6
    controller.mit_kd = [0.2] * 6
    controller.mit_feedforward = [0.0] * 6
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )

    controller.send_target("IK pose jog")
    controller.mit_tick()

    assert controller.arm.move_j_commands == []
    assert [command["p_des"] for command in controller.arm.mit_commands] == (
        pytest.approx(controller.jog.target_joints)
    )


def test_startup_reset_clears_latched_emergency_stop(monkeypatch):
    class FakeArm:
        def __init__(self):
            self.reset_calls = 0
            self.emergency_stopped = True

        def reset(self):
            self.reset_calls += 1
            self.emergency_stopped = False

        def get_arm_status(self):
            state = "EMERGENCY_STOP" if self.emergency_stopped else "NORMAL"
            return SimpleNamespace(msg=SimpleNamespace(arm_status=state))

    class FakeLogger:
        def info(self, message):
            del message

        def warning(self, message):
            del message

    controller = object.__new__(ArmKeyboardController)
    controller.arm = FakeArm()
    controller.emergency_reset_timeout = 0.1
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )

    assert controller.reset_emergency_stop()
    assert controller.arm.reset_calls == 1


def test_startup_home_zeros_joints_strictly_in_order(monkeypatch):
    class FakeArm:
        def __init__(self):
            self.joints = [1.0, 2.0, 3.0]
            self.commands = []
            self.limit_changes = []

        def get_joint_angles(self):
            return list(self.joints)

        def move_j(self, target):
            self.commands.append(list(target))
            self.joints = list(target)

        def get_joint_limits_enabled(self):
            return True

        def set_joint_limits_enabled(self, enabled):
            self.limit_changes.append(enabled)

    class FakeLogger:
        def info(self, message, **kwargs):
            del message, kwargs

        def error(self, message, **kwargs):
            del message, kwargs

    controller = object.__new__(ArmKeyboardController)
    controller.arm = FakeArm()
    controller.joint_count = 3
    controller.startup_home_timeout = 0.1
    controller.startup_home_tolerance = 0.01
    controller.jog = ArmJointJogState([(-4.0, 4.0)] * 3, 0.1)
    monkeypatch.setattr(
        ArmKeyboardController, "get_logger", lambda self: FakeLogger()
    )

    assert controller.move_home_and_wait()
    assert controller.arm.commands == [
        [0.0, 2.0, 3.0],
        [0.0, 0.0, 3.0],
        [0.0, 0.0, 0.0],
    ]
    assert controller.arm.limit_changes == [False, True]


def test_acceleration_limits_are_set_per_joint_not_with_255():
    class FakeArm:
        def __init__(self):
            self.calls = []

        def set_joint_acc_limits(self, **kwargs):
            self.calls.append(kwargs)
            return kwargs["joint_index"] != 255

    arm = FakeArm()
    result = set_joint_acceleration_limits(
        arm, 6, max_joint_acceleration=1.0, timeout=2.0
    )

    assert result == (True, None)
    assert [call["joint_index"] for call in arm.calls] == [
        1, 2, 3, 4, 5, 6
    ]
    assert all(call["timeout"] == 2.0 for call in arm.calls)


def test_position_mode_is_confirmed_before_startup_motion():
    class FakeArm:
        OPTIONS = SimpleNamespace(
            MOTION_MODE=SimpleNamespace(J="j")
        )

        def __init__(self):
            self.requested_modes = []

        def set_motion_mode(self, mode):
            self.requested_modes.append(mode)

        def get_arm_status(self):
            return SimpleNamespace(msg=SimpleNamespace(
                ctrl_mode="CAN_CTRL", mode_feedback="MOVE_J"
            ))

    arm = FakeArm()

    assert prepare_planned_joint_mode(arm, timeout=0.1, poll_period=0.0)
    assert arm.requested_modes == ["j"]


def test_explicit_emergency_stop_reset_waits_for_normal_status():
    class Status:
        def __init__(self, value):
            self.msg = type("Message", (), {"arm_status": value})()

    class FakeArm:
        def __init__(self):
            self.reset_called = False
            self.statuses = [Status("EMERGENCY_STOP"), Status("NORMAL")]

        def reset(self):
            self.reset_called = True

        def get_arm_status(self):
            return self.statuses.pop(0)

    class FakeLogger:
        def warning(self, message):
            del message

        def info(self, message):
            del message

    controller = object.__new__(ArmKeyboardController)
    controller.arm = FakeArm()
    controller.emergency_reset_timeout = 1.0
    controller.get_logger = lambda: FakeLogger()

    assert controller.reset_emergency_stop()
    assert controller.arm.reset_called


def test_revo2_normalized_mode_compatibility():
    from pyAgxArm.protocols.can_protocol.drivers.effector.revo2_touch import (
        default,
    )
    from armbycontroller.hand_controller import resolve_normalized_unit_mode

    mode = resolve_normalized_unit_mode(default.driver.Driver)
    assert mode is not None
    assert str(mode).endswith("Normalized")
