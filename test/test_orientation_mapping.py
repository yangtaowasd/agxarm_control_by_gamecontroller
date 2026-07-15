import math

from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config
from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD

from armbycontroller.piper_keyboard_controller import (
    KEY_COUNT,
    KEY_PAGEDOWN,
    KEY_PAGEUP,
    KEY_RIGHT,
    PiperKeyboardController,
)


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warning(self, *args, **kwargs):
        pass


def _normalize(vector):
    norm = math.sqrt(sum(value * value for value in vector))
    return [value / norm for value in vector]


def _tool_z_axis(pose):
    roll, pitch, yaw = pose[3:6]
    return [
        math.cos(yaw) * math.sin(pitch) * math.cos(roll)
        + math.sin(yaw) * math.sin(roll),
        math.sin(yaw) * math.sin(pitch) * math.cos(roll)
        - math.cos(yaw) * math.sin(roll),
        math.cos(pitch) * math.cos(roll),
    ]


def _make_controller():
    controller = object.__new__(PiperKeyboardController)
    controller.arm = AgxArmFactory.create_arm(
        create_agx_arm_config(
            robot=ArmModel.PIPER,
            firmeware_version=PiperFW.DEFAULT,
            channel="can0",
        )
    )
    controller.get_logger = lambda: _Logger()
    controller.target_joints = [0.0] * 6
    controller.wrist_j6_offset = 0.0
    controller.j3_correction_rad = 0.0
    controller.joint_limits = [
        tuple(ROBOT_JOINT_LIMIT_PRESET_RAD["piper"][f"joint{i}"])
        for i in range(1, 7)
    ]
    controller.joint_limits[3] = tuple(math.radians(value) for value in (-127.0, 127.0))
    controller.joint_limits[4] = tuple(math.radians(value) for value in (-89.5, 89.5))
    controller.joint_limits[5] = tuple(math.radians(value) for value in (-170.0, 170.0))
    controller.orientation_mapping = "square"
    controller.orientation_square_limit = 1.0
    controller.orientation_square_step = 0.02
    controller.orientation_square_max_tilt_rad = math.radians(80.0)
    controller.orientation_ik_damping = 0.2
    controller.orientation_ik_max_iterations = 40
    controller.orientation_ik_max_joint_step = 0.02
    controller.orientation_ik_fd_eps = 0.001
    controller.orientation_ik_line_search_steps = 6
    controller.orientation_ik_tolerance = 0.002
    controller.orientation_ik_refine_iterations = 10
    controller.orientation_ik_refine_damping = 0.05
    controller.orientation_ik_coordinate_iterations = 8
    controller.orientation_j5_limit_rad = math.radians(80.0)
    controller.orientation_yaw_joint_step_scale = 2.0
    controller.orientation_singularity_yaw_boost = 2.0
    controller.avoid_j5_zero_rad = 0.0
    controller.speed_percent = 40
    controller.j4_j6_speed_percent = 80
    controller.j3_correction_step_rad = 0.01
    controller.j3_correction_limit_rad = math.radians(10.0)
    controller.orientation_home_pitch = 0.0
    controller.orientation_home_yaw = 0.0
    controller.orientation_square_x = 0.0
    controller.orientation_square_y = 0.0
    controller.target_orientation_pitch = 0.0
    controller.target_orientation_yaw = 0.0
    controller.sync_orientation_target_from_fk()
    return controller


def _expected_square_axis(controller):
    home = _tool_z_axis(controller.arm.fk([0.0] * 6))
    horizontal = _normalize([-home[1], home[0], 0.0])
    down = [
        horizontal[1] * home[2] - horizontal[2] * home[1],
        horizontal[2] * home[0] - horizontal[0] * home[2],
        horizontal[0] * home[1] - horizontal[1] * home[0],
    ]
    square_x = controller.orientation_square_x / controller.orientation_square_limit
    square_y = controller.orientation_square_y / controller.orientation_square_limit
    polar_scale = max(abs(square_x), abs(square_y))
    if polar_scale < 1e-12:
        return home

    tangent = _normalize(
        [
            square_x * horizontal[index] + square_y * down[index]
            for index in range(3)
        ]
    )
    polar_angle = polar_scale * controller.orientation_square_max_tilt_rad
    return [
        math.cos(polar_angle) * home[index]
        + math.sin(polar_angle) * tangent[index]
        for index in range(3)
    ]


def _axis_error(actual, expected):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(actual, expected)))


def test_horizontal_square_motion_does_not_collapse_on_y_zero():
    controller = _make_controller()

    for _ in range(5):
        controller.apply_orientation_step(0, -1)

    actual = _tool_z_axis(controller.arm.fk(controller.target_joints))
    expected = _expected_square_axis(controller)

    assert abs(controller.orientation_square_x - 0.2) < 1e-12
    assert abs(controller.orientation_square_y) < 1e-12
    assert _axis_error(actual, expected) < 0.01
    assert abs(controller.target_joints[4]) > 0.05


def test_vertical_motion_crosses_pole_using_both_j5_signs():
    controller = _make_controller()

    for _ in range(5):
        controller.apply_orientation_step(1, 0)
    upper_j5 = controller.target_joints[4]

    for _ in range(10):
        controller.apply_orientation_step(-1, 0)
    lower_j5 = controller.target_joints[4]
    actual = _tool_z_axis(controller.arm.fk(controller.target_joints))
    expected = _expected_square_axis(controller)

    assert upper_j5 < -0.05
    assert lower_j5 > 0.05
    assert _axis_error(actual, expected) < 0.01


def test_solver_changes_wrist_chart_before_j4_limit_blocks_motion():
    controller = _make_controller()

    for _ in range(5):
        controller.apply_orientation_step(0, -1)
        controller.apply_orientation_step(-1, 0)

    for _ in range(10):
        controller.apply_orientation_step(1, 0)

    actual = _tool_z_axis(controller.arm.fk(controller.target_joints))
    expected = _expected_square_axis(controller)

    assert abs(controller.orientation_square_x - 0.2) < 1e-12
    assert abs(controller.orientation_square_y + 0.1) < 1e-12
    assert controller.target_joints[4] < -0.05
    assert _axis_error(actual, expected) < 0.01


def test_square_contains_j4_and_j5_at_60_degrees():
    controller = _make_controller()
    polar_scale = 60.0 / 80.0
    controller.orientation_square_x = (
        controller.orientation_square_limit * polar_scale
    )
    controller.orientation_square_y = (
        controller.orientation_square_limit * polar_scale / math.sqrt(3.0)
    )

    controller.apply_orientation_step(0, 0)

    assert abs(math.degrees(controller.target_joints[3]) - 60.0) < 0.5
    assert abs(math.degrees(controller.target_joints[4]) - 60.0) < 0.5
    assert abs(controller.target_joints[3] + controller.target_joints[5]) < 1e-9


def test_square_side_reaches_ninety_degree_azimuth_at_eighty_degree_tilt():
    controller = _make_controller()

    for _ in range(50):
        controller.apply_orientation_step(0, -1)

    actual = _tool_z_axis(controller.arm.fk(controller.target_joints))
    expected = _expected_square_axis(controller)

    assert abs(controller.orientation_square_x - 1.0) < 1e-12
    assert abs(abs(controller.target_joints[3]) - math.pi / 2.0) < 0.02
    assert abs(abs(controller.target_joints[4]) - math.radians(80.0)) < 0.02
    assert abs(controller.target_joints[5] + controller.target_joints[3]) < 1e-12
    assert _axis_error(actual, expected) < 0.01


def test_page_keys_move_j6_negative_then_positive():
    controller = _make_controller()
    controller.arrow_mode = "orientation"
    controller.step_rad = 0.01
    controller.key_state = [0] * KEY_COUNT

    controller.key_state[KEY_PAGEUP] = 1
    assert controller.apply_joint_step()
    assert abs(controller.target_joints[5] + 0.02) < 1e-12
    assert abs(controller.wrist_j6_offset + 0.02) < 1e-12

    controller.apply_orientation_step(0, -1)
    assert abs(
        controller.target_joints[5]
        + controller.target_joints[3]
        - controller.wrist_j6_offset
    ) < 1e-9

    controller.target_joints[5] = 0.0
    controller.target_joints[3] = 0.0
    controller.wrist_j6_offset = 0.0
    controller.key_state = [0] * KEY_COUNT
    controller.key_state[KEY_PAGEDOWN] = 1
    assert controller.apply_joint_step()
    assert abs(controller.target_joints[5] - 0.02) < 1e-12


def test_j4_j6_keyboard_steps_use_eighty_to_forty_speed_ratio():
    controller = _make_controller()
    controller.arrow_mode = "joint"
    controller.step_rad = 0.01
    controller.key_state = [0] * KEY_COUNT
    controller.key_state[KEY_RIGHT] = 1
    controller.key_state[KEY_PAGEDOWN] = 1

    assert controller.apply_joint_step()
    assert abs(controller.target_joints[3] - 0.02) < 1e-12
    assert abs(controller.target_joints[4]) < 1e-12
    assert abs(controller.target_joints[5]) < 1e-12


def test_j3_correction_extends_pitch_after_j5_reaches_limit():
    controller = _make_controller()

    for _ in range(50):
        controller.apply_orientation_step(1, 0)
    assert abs(controller.target_joints[4]) >= math.radians(79.5)

    old_j3 = controller.target_joints[2]
    controller.apply_orientation_step(1, 0)

    assert abs(controller.j3_correction_rad + 0.01) < 1e-12
    assert abs(controller.target_joints[2] - old_j3 + 0.01) < 1e-12
    assert abs(controller.target_joints[4]) >= math.radians(79.5)

    for _ in range(30):
        controller.apply_orientation_step(1, 0)

    assert abs(controller.j3_correction_rad + math.radians(10.0)) < 1e-12
    assert abs(controller.target_joints[2] + math.radians(10.0)) < 1e-12
    assert abs(controller.target_joints[4]) >= math.radians(79.5)


def test_reset_key_removes_only_j3_correction():
    controller = _make_controller()
    controller.target_joints[2] = -0.15
    controller.j3_correction_rad = -0.05

    assert controller.reset_j3_correction()
    assert abs(controller.j3_correction_rad) < 1e-12
    assert abs(controller.target_joints[2] + 0.1) < 1e-12
