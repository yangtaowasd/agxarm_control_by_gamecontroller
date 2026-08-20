"""Cartesian subspace selection in project ``[angular; linear]`` order."""

import re

import numpy as np


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
