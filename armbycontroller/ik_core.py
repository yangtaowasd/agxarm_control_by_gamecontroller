"""Shared screw-theory helpers for AGX arm controllers."""

from dataclasses import dataclass
import math
from pathlib import Path
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

from armbycontroller.lie import rotation_exp
from armbycontroller.lie import rotation_vector

VERIFIED_FIRMWARE = {"nero": "v112", "piper_l": "v188"}


def resolve_firmware_name(robot_model, requested_firmware):
    """Resolve ``auto`` to the firmware verified on the project hardware."""
    requested = str(requested_firmware).lower()
    if requested != "auto":
        return requested
    return VERIFIED_FIRMWARE[robot_model]


def set_joint_acceleration_limits(
    arm, joint_count, max_joint_acceleration, timeout
):
    """Set and verify limits per joint, avoiding Piper's broken 255 ACK."""
    for joint_index in range(1, joint_count + 1):
        if not arm.set_joint_acc_limits(
            joint_index=joint_index,
            max_joint_acc=max_joint_acceleration,
            timeout=timeout,
        ):
            return False, joint_index
    return True, None


def prepare_planned_joint_mode(arm, timeout, poll_period=0.05):
    """Enter and confirm CAN joint-position mode before the first target."""
    arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.J)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = arm.get_arm_status()
        if status is not None and hasattr(status, "msg"):
            ctrl_mode = str(getattr(status.msg, "ctrl_mode", ""))
            move_mode = str(getattr(status.msg, "mode_feedback", ""))
            if "CAN_CTRL" in ctrl_mode and "MOVE_J" in move_mode:
                return True
        time.sleep(poll_period)
    return False


def resolve_urdf_path(parameter_value, robot_model):
    """Resolve an AGX URDF from a parameter, install, or source tree."""
    if parameter_value:
        return Path(parameter_value).expanduser().resolve()

    candidates = []
    if robot_model == "nero":
        try:
            share = Path(get_package_share_directory("nero_screw_dynamics"))
            candidates.append(
                share / "agx_arm_urdf" / "nero" / "urdf"
                / "nero_description.urdf"
            )
        except PackageNotFoundError:
            pass
        candidates.append(
            Path(__file__).resolve().parents[2]
            / "nero_screw_dynamics" / "agx_arm_urdf" / "nero" / "urdf"
            / "nero_description.urdf"
        )
    try:
        share = Path(get_package_share_directory("armbycontroller"))
        candidates.append(
            share / "agx_arm_urdf" / robot_model / "urdf"
            / f"{robot_model}_description.urdf"
        )
    except PackageNotFoundError:
        pass
    candidates.append(
        Path(__file__).resolve().parents[1] / "agx_arm_urdf"
        / robot_model / "urdf" / f"{robot_model}_description.urdf"
    )
    try:
        share = Path(get_package_share_directory("agx_arm_description"))
        candidates.append(
            share / "agx_arm_urdf" / robot_model / "urdf"
            / f"{robot_model}_description.urdf"
        )
    except PackageNotFoundError:
        pass
    candidates.append(
        Path.home() / "agx_arm_ws" / "src" / "agx_arm_ros" / "src"
        / "agx_arm_description" / "agx_arm_urdf" / robot_model / "urdf"
        / f"{robot_model}_description.urdf"
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    searched = ", ".join(str(path) for path in candidates)
    raise ValueError(
        f"{robot_model} URDF was not found. Set urdf_path explicitly. "
        f"Searched: {searched}"
    )


def create_screw_solver(
    urdf_path, base_frame, tip_link, joint_count, timeout, tolerance
):
    """Create the shared PoE solver and verify the requested chain DOF."""
    from armbycontroller.screw_ik import ScrewIkSolver
    from armbycontroller.screw_model import UrdfScrewModel

    model = UrdfScrewModel(
        urdf_path,
        gravity=(0.0, 0.0, 0.0),
        base_link=base_frame,
        tip_link=tip_link,
        joint_count=joint_count,
    )
    # Keep the public timeout parameter meaningful without making results
    # depend on host speed: it scales a deterministic iteration budget.
    iterations = max(20, min(500, int(math.ceil(float(timeout) * 12000.0))))
    return ScrewIkSolver(
        model,
        max_iterations=iterations,
        position_tolerance=max(tolerance, 1e-4),
        orientation_tolerance=max(tolerance, 1e-4),
    )


def quaternion_to_rotation_matrix(x, y, z, w):
    """Convert a finite, non-zero quaternion to a 3x3 rotation matrix."""
    quaternion = np.asarray([x, y, z, w], dtype=float)
    if not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion contains a non-finite value")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z),
             2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w),
             1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotation_matrix_to_quaternion(rotation):
    """Convert a 3x3 rotation matrix to an x, y, z, w quaternion."""
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
        w = 0.25 * scale
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = math.sqrt(
                1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
            ) * 2.0
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
            w = (matrix[2, 1] - matrix[1, 2]) / scale
        elif axis == 1:
            scale = math.sqrt(
                1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
            ) * 2.0
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
            w = (matrix[0, 2] - matrix[2, 0]) / scale
        else:
            scale = math.sqrt(
                1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
            ) * 2.0
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
            w = (matrix[1, 0] - matrix[0, 1]) / scale
    quaternion = np.asarray([x, y, z, w], dtype=float)
    return quaternion / np.linalg.norm(quaternion)


def make_pointing_quaternion(direction, roll_reference):
    """Point tool-local +Z along direction with a stable first roll."""
    tool_z = np.asarray(direction, dtype=float)
    reference = np.asarray(roll_reference, dtype=float)
    if tool_z.shape != (3,) or not np.all(np.isfinite(tool_z)):
        raise ValueError("pointing_direction must contain three finite values")
    if reference.shape != (3,) or not np.all(np.isfinite(reference)):
        raise ValueError("roll_reference must contain three finite values")
    norm = float(np.linalg.norm(tool_z))
    if norm < 1e-12:
        raise ValueError("pointing_direction must be non-zero")
    tool_z /= norm
    tool_x = reference - np.dot(reference, tool_z) * tool_z
    if np.linalg.norm(tool_x) < 1e-12:
        fallback = np.zeros(3)
        fallback[int(np.argmin(np.abs(tool_z)))] = 1.0
        tool_x = fallback - np.dot(fallback, tool_z) * tool_z
    tool_x /= np.linalg.norm(tool_x)
    rotation = np.column_stack((tool_x, np.cross(tool_z, tool_x), tool_z))
    return rotation_matrix_to_quaternion(rotation)


def increment_tool_orientation(rotation, pitch, yaw, roll):
    """Apply base-frame pitch/yaw and tool-frame roll increments."""
    pitch_matrix = rotation_exp(np.asarray([0.0, 1.0, 0.0]), pitch)
    yaw_matrix = rotation_exp(np.asarray([0.0, 0.0, 1.0]), yaw)
    roll_matrix = rotation_exp(np.asarray([0.0, 0.0, 1.0]), roll)
    return yaw_matrix @ pitch_matrix @ rotation @ roll_matrix


def rotation_error_angle(actual, target):
    """Return the shortest rotation angle between rotation matrices."""
    return float(np.linalg.norm(rotation_vector(target @ actual.T)))


def pointing_error_angle(actual, target):
    """Return the angle between the tool-local +Z pointing axes."""
    cosine = float(np.dot(actual[:, 2], target[:, 2]))
    return math.acos(float(np.clip(cosine, -1.0, 1.0)))


def radial_workspace_check(position, minimum, maximum):
    """Check a position against a spherical shell around base_link."""
    distance = float(np.linalg.norm(np.asarray(position, dtype=float)))
    return minimum <= distance <= maximum, distance


def solve_pointing_ik(solver, position, target_rotation, seed, roll_samples):
    """Solve position and tool direction while leaving tool roll free."""
    best_solution = None
    best_distance = math.inf
    for angle in np.linspace(0.0, 2.0 * math.pi, roll_samples, endpoint=False):
        local_roll = rotation_exp(
            np.asarray([0.0, 0.0, 1.0]), float(angle)
        )
        solution = solver.ik(
            position, target_rotation @ local_roll, seed_jnt_values=seed
        )
        if solution is None:
            continue
        solution = np.asarray(solution, dtype=float)
        distance = float(np.linalg.norm(solution - seed))
        if distance < best_distance:
            best_solution, best_distance = solution, distance
    return best_solution


@dataclass(frozen=True)
class IkResult:
    """One FK-verified IK result."""

    joints: np.ndarray
    position_error: float
    orientation_error: float


class IkFailure(RuntimeError):
    """A safe, expected IK rejection with a user-facing reason."""


class AgxIkEngine:
    """Apply shared workspace, pointing-axis IK, and FK verification."""

    def __init__(
        self, solver, joint_count, workspace_min, workspace_max,
        position_tolerance, orientation_tolerance, roll_samples=8,
        pointing_axis_only=True,
    ):
        self.solver = solver
        self.joint_count = joint_count
        self.workspace_min = workspace_min
        self.workspace_max = workspace_max
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.roll_samples = roll_samples
        self.pointing_axis_only = pointing_axis_only

    def solve(self, position, rotation, seed):
        """Return a verified solution or raise IkFailure."""
        position = np.asarray(position, dtype=float)
        seed = np.asarray(seed, dtype=float)
        inside, distance = radial_workspace_check(
            position, self.workspace_min, self.workspace_max
        )
        if not inside:
            raise IkFailure(
                "workspace radius %.4f m is outside [%.4f, %.4f] m"
                % (distance, self.workspace_min, self.workspace_max)
            )
        if self.pointing_axis_only:
            joints = solve_pointing_ik(
                self.solver, position, rotation, seed, self.roll_samples
            )
        else:
            joints = self.solver.ik(
                position, rotation, seed_jnt_values=seed
            )
        if joints is None:
            raise IkFailure("screw IK found no solution")
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (self.joint_count,) or not np.all(np.isfinite(joints)):
            raise IkFailure("IK returned an invalid joint vector")

        fk_position, fk_rotation = self.solver.fk(joints)
        position_error = float(np.linalg.norm(fk_position - position))
        error_fn = (
            pointing_error_angle
            if self.pointing_axis_only else rotation_error_angle
        )
        orientation_error = error_fn(fk_rotation, rotation)
        if (position_error > self.position_tolerance or
                orientation_error > self.orientation_tolerance):
            raise IkFailure(
                "FK error %.6g m / %.6g rad"
                % (position_error, orientation_error)
            )
        return IkResult(joints, position_error, orientation_error)
