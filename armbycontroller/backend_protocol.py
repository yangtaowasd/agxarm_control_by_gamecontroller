"""Pure validation helpers shared by backend transport adapters."""

import ipaddress
import math

import numpy as np

from armbycontroller.control_protocol import ACTION_KEYS
from armbycontroller.model_profiles import get_arm_profile


API_VERSION = "v1"
MAX_BODY_BYTES = 16 * 1024


def finite_number(value, name):
    """Return a finite float or raise a client-facing validation error."""
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a number") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def extract_named_arm_joint_state(message, robot_model):
    """Extract and reorder only this arm's joints from a shared topic."""
    count = get_arm_profile(robot_model).joint_count
    expected = [f"joint{index}" for index in range(1, count + 1)]
    names = list(getattr(message, "name", []))
    positions = list(getattr(message, "position", []))
    if len(names) != len(positions) or len(set(names)) != len(names):
        return None
    lookup = {name: index for index, name in enumerate(names)}
    if any(name not in lookup for name in expected):
        return None
    indices = [lookup[name] for name in expected]
    values = np.asarray([positions[index] for index in indices], dtype=float)
    if not np.all(np.isfinite(values)):
        return None
    velocities = list(getattr(message, "velocity", []))
    ordered_velocities = []
    if len(velocities) == len(names):
        ordered_velocities = [float(velocities[index]) for index in indices]
        if not all(math.isfinite(value) for value in ordered_velocities):
            ordered_velocities = []
    return {
        "names": expected,
        "positions": values.tolist(),
        "velocities": ordered_velocities,
    }


def sanitize_pose_command(payload, default_frame="base_link"):
    """Validate and normalize one backend pose request."""
    if not isinstance(payload, dict):
        raise ValueError("pose command must be a JSON object")
    position = payload.get("position")
    orientation = payload.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise ValueError("position and orientation objects are required")
    normalized = {
        "frameId": str(payload.get("frameId") or default_frame),
        "position": {
            axis: finite_number(position.get(axis), f"position.{axis}")
            for axis in ("x", "y", "z")
        },
        "orientation": {
            axis: finite_number(
                orientation.get(axis), f"orientation.{axis}"
            )
            for axis in ("x", "y", "z", "w")
        },
    }
    quaternion = normalized["orientation"]
    norm = math.sqrt(sum(value * value for value in quaternion.values()))
    if norm < 1e-9:
        raise ValueError("orientation quaternion must be non-zero")
    normalized["orientation"] = {
        axis: value / norm for axis, value in quaternion.items()
    }
    return normalized


def sanitize_action(payload):
    """Validate a semantic backend action."""
    if not isinstance(payload, dict):
        raise ValueError("action command must be a JSON object")
    action = str(payload.get("action", "")).strip().lower()
    if action != "release" and action not in ACTION_KEYS:
        choices = ", ".join([*ACTION_KEYS, "release"])
        raise ValueError(f"action must be one of: {choices}")
    return action


def validate_bind_address(host, token):
    """Require authentication whenever the API leaves loopback."""
    normalized = str(host).strip()
    try:
        loopback = ipaddress.ip_address(normalized).is_loopback
    except ValueError:
        loopback = normalized.lower() == "localhost"
    if not loopback and not token:
        raise ValueError("api_token is required when api_host is not loopback")
