"""Shared Cartesian task-geometry contracts."""

from pathlib import Path

import numpy as np
import pytest

from armbycontroller.cartesian import geometric_jacobian
from armbycontroller.cartesian import joint_torque_from_wrench
from armbycontroller.cartesian import transform_matrix
from armbycontroller.cartesian import wrench_from_joint_torque


class FixedModel:
    """Minimal model with explicit pose and space Jacobian."""

    def __init__(self, pose, jacobian):
        self.pose = np.asarray(pose, dtype=float)
        self.jacobian = np.asarray(jacobian, dtype=float)

    def forward_kinematics(self, joints):
        del joints
        return self.pose.copy()

    def space_jacobian(self, joints):
        del joints
        return self.jacobian.copy()


def test_shared_geometry_converts_space_jacobian_to_tool_origin():
    pose = np.eye(4)
    pose[:3, 3] = [0.4, -0.2, 0.3]
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


def test_shared_virtual_work_mapping_is_bidirectional_at_regular_pose():
    jacobian = np.diag([1.0, 2.0, 4.0, 0.5, 1.0, 2.0])
    wrench = np.asarray([0.2, -0.4, 0.6, 1.0, -2.0, 3.0])

    torque = joint_torque_from_wrench(jacobian, wrench)
    recovered = wrench_from_joint_torque(jacobian, torque, damping=1e-8)

    assert recovered == pytest.approx(wrench, abs=1e-12)


def test_shared_transform_validation_rejects_non_rotation():
    invalid = np.eye(4)
    invalid[0, 0] = 2.0

    with pytest.raises(ValueError, match=r"SE\(3\)"):
        transform_matrix(invalid, "pose")


def test_impedance_and_admittance_do_not_import_each_other():
    root = Path(__file__).resolve().parents[1] / "armbycontroller"
    impedance_source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (root / "impedance").glob("*.py")
    )
    admittance_source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (root / "admittance").glob("*.py")
    )

    assert "armbycontroller.admittance" not in impedance_source
    assert "armbycontroller.impedance" not in admittance_source
    control_exports = (root / "control" / "__init__.py").read_text(
        encoding="utf-8"
    )
    assert "armbycontroller.admittance" not in control_exports
    assert "armbycontroller.impedance" not in control_exports
    assert not (root / "control" / "adapters.py").exists()
    assert not (root / "impedance" / "admittance.py").exists()
