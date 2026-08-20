"""Cartesian subspace selection in project ``[angular; linear]`` order."""

import re

import numpy as np

from armbycontroller.modeling.lie import rotation_from_vector


CARTESIAN_AXIS_NAMES = ("rx", "ry", "rz", "x", "y", "z")
_AXIS_INDEX = {
    name: index for index, name in enumerate(CARTESIAN_AXIS_NAMES)
}


def task_axis_mask(axes):
    """Return a six-axis mask for ``[rx, ry, rz, x, y, z]`` names."""
    if isinstance(axes, str):
        value = axes.strip().lower()
        if value == "all":
            return np.ones(6)
        names = [
            token for token in re.split(r"[\s,]+", value) if token
        ]
        if not names:
            raise ValueError("at least one Cartesian axis must be selected")
        mask = np.zeros(6)
        for name in names:
            if name not in _AXIS_INDEX:
                raise ValueError(
                    f"unknown Cartesian axis {name!r}; expected one of "
                    + ", ".join(CARTESIAN_AXIS_NAMES)
                )
            mask[_AXIS_INDEX[name]] = 1.0
        return mask

    values = np.asarray(axes, dtype=float)
    if (
        values.shape != (6,)
        or not np.all(np.isfinite(values))
        or np.any((values != 0.0) & (values != 1.0))
        or not np.any(values)
    ):
        raise ValueError(
            "Cartesian axis mask must contain six zeros/ones with at least "
            "one selected axis"
        )
    return values.copy()


def compliance_frame_rotation(frame, current_pose=None, rotation_vector=None):
    """Return the compliance-frame orientation expressed in the base frame."""
    name = str(frame).strip().lower()
    if name == "base":
        return np.eye(3)
    if name == "tool":
        pose = np.asarray(current_pose, dtype=float)
        rotation = pose[:3, :3] if pose.shape == (4, 4) else np.zeros((3, 3))
        if (
            pose.shape != (4, 4)
            or not np.all(np.isfinite(pose))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8)
        ):
            raise ValueError("tool compliance frame requires a finite pose")
        return rotation.copy()
    if name == "custom":
        vector = np.asarray(rotation_vector, dtype=float)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(
                "custom compliance frame requires a finite rotation vector"
            )
        return rotation_from_vector(vector)
    raise ValueError("compliance frame must be base, tool, or custom")


def task_subspace_projector(axes, frame_rotation=None):
    """Build a task projector in project ``[angular; linear]`` order."""
    mask = task_axis_mask(axes)
    rotation = (
        np.eye(3)
        if frame_rotation is None
        else np.asarray(frame_rotation, dtype=float)
    )
    if (
        rotation.shape != (3, 3)
        or not np.all(np.isfinite(rotation))
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8)
    ):
        raise ValueError("frame_rotation must be a finite SO(3) matrix")
    registration = np.zeros((6, 6))
    registration[:3, :3] = rotation
    registration[3:, 3:] = rotation
    projector = registration @ np.diag(mask) @ registration.T
    return 0.5 * (projector + projector.T)
