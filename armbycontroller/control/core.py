"""Robot- and transport-independent controller interface."""

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Any
from typing import Mapping
from typing import Protocol
from typing import runtime_checkable

import numpy as np


def _vector(values, name, size=None):
    result = np.asarray(values, dtype=float)
    expected = (int(size),) if size is not None else None
    if (
        result.ndim != 1
        or result.size < 1
        or (expected is not None and result.shape != expected)
        or not np.all(np.isfinite(result))
    ):
        suffix = f" {expected[0]}-vector" if expected else " finite vector"
        raise ValueError(f"{name} must be a{suffix}")
    return result.copy()


def _signals(values):
    copied = {}
    for name, value in dict(values).items():
        if isinstance(value, np.ndarray):
            copied[str(name)] = value.copy()
        elif isinstance(value, (list, tuple)):
            copied[str(name)] = np.asarray(value, dtype=float).copy()
        elif isinstance(value, (np.floating, np.integer)):
            copied[str(name)] = value.item()
        elif isinstance(value, (str, bool, int, float)) or value is None:
            copied[str(name)] = value
        else:
            raise ValueError(
                f"signal {name!r} must be numeric, text, boolean, or null"
            )
    return MappingProxyType(copied)


@dataclass(frozen=True)
class ControlState:
    """One measured joint state consumed by every controller adapter."""

    position: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray
    position_valid: bool = True
    velocity_valid: bool = True
    effort_valid: bool = True

    def __post_init__(self):
        position = _vector(self.position, "state.position")
        size = position.size
        object.__setattr__(self, "position", position)
        object.__setattr__(
            self, "velocity", _vector(self.velocity, "state.velocity", size)
        )
        object.__setattr__(
            self, "effort", _vector(self.effort, "state.effort", size)
        )

    @property
    def joint_count(self):
        return self.position.size


@dataclass(frozen=True)
class ControlReference:
    """Joint reference and optional external wrench for one control cycle."""

    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    external_wrench: np.ndarray

    def __post_init__(self):
        position = _vector(self.position, "reference.position")
        size = position.size
        object.__setattr__(self, "position", position)
        object.__setattr__(
            self,
            "velocity",
            _vector(self.velocity, "reference.velocity", size),
        )
        object.__setattr__(
            self,
            "acceleration",
            _vector(self.acceleration, "reference.acceleration", size),
        )
        object.__setattr__(
            self,
            "external_wrench",
            _vector(self.external_wrench, "reference.external_wrench", 6),
        )

    @classmethod
    def hold(cls, position, external_wrench=None):
        position = _vector(position, "position")
        zeros = np.zeros(position.size)
        return cls(
            position,
            zeros,
            zeros,
            np.zeros(6) if external_wrench is None else external_wrench,
        )


@dataclass(frozen=True)
class ControlInput:
    """Complete input to the controller seam for one bounded period."""

    timestamp: float
    period: float
    state: ControlState
    reference: ControlReference

    def __post_init__(self):
        timestamp = float(self.timestamp)
        period = float(self.period)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("period must be finite and positive")
        if self.state.joint_count != self.reference.position.size:
            raise ValueError("state and reference joint counts must match")
        object.__setattr__(self, "timestamp", timestamp)
        object.__setattr__(self, "period", period)


@dataclass(frozen=True)
class MitCommand:
    """One per-joint MIT command batch, before the hardware adapter."""

    position: np.ndarray
    velocity: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    feedforward: np.ndarray
    estimated_torque: np.ndarray

    def __post_init__(self):
        position = _vector(self.position, "command.position")
        size = position.size
        object.__setattr__(self, "position", position)
        for name in (
            "velocity", "kp", "kd", "feedforward", "estimated_torque"
        ):
            object.__setattr__(
                self,
                name,
                _vector(getattr(self, name), f"command.{name}", size),
            )
        if np.any(self.kp < 0.0) or np.any(self.kd < 0.0):
            raise ValueError("MIT gains must be nonnegative")

    @property
    def mode(self):
        return "mit"


@dataclass(frozen=True)
class PositionCommand:
    """One planned joint-position command before the hardware adapter."""

    position: np.ndarray

    def __post_init__(self):
        object.__setattr__(
            self, "position", _vector(self.position, "command.position")
        )

    @property
    def mode(self):
        return "position"


@dataclass(frozen=True)
class ControlResult:
    """Controller output plus algorithm-neutral diagnostic signals."""

    controller: str
    command: MitCommand | PositionCommand
    signals: Mapping[str, Any]
    raw: Any = None

    def __post_init__(self):
        name = str(self.controller).strip()
        if not name:
            raise ValueError("controller name must not be empty")
        object.__setattr__(self, "controller", name)
        object.__setattr__(self, "signals", _signals(self.signals))


class ControlSafetyError(RuntimeError):
    """A controller rejected an unsafe or incomplete cycle."""


@runtime_checkable
class ControllerAdapter(Protocol):
    """Minimal seam implemented by every control algorithm adapter."""

    name: str

    def reset(self, state: ControlState) -> None:
        """Capture any controller state needed before the next cycle."""

    def step(self, sample: ControlInput) -> ControlResult:
        """Evaluate one cycle without ROS or hardware access."""


class ControlEngine:
    """Own named controller adapters behind one checked interface."""

    def __init__(self, controllers=()):
        self._controllers = {}
        for controller in controllers:
            self.register(controller)

    @property
    def available(self):
        return tuple(sorted(self._controllers))

    def register(self, controller):
        if not isinstance(controller, ControllerAdapter):
            raise TypeError("controller must satisfy ControllerAdapter")
        name = str(controller.name).strip()
        if not name or name in self._controllers:
            raise ValueError(
                f"controller name is empty or duplicated: {name!r}"
            )
        self._controllers[name] = controller

    def _get(self, name):
        try:
            return self._controllers[str(name)]
        except KeyError as error:
            raise KeyError(
                f"unknown controller {name!r}; available={self.available}"
            ) from error

    def reset(self, name, state):
        self._get(name).reset(state)

    def step(self, name, sample):
        result = self._get(name).step(sample)
        if result.controller != str(name):
            raise RuntimeError(
                "controller result name does not match the selected adapter"
            )
        if result.command.position.size != sample.state.joint_count:
            raise RuntimeError("controller result joint count changed")
        return result


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


def control_sample(sample, result, *, robot_model, interaction_mode):
    """Return the stable JSON-compatible schema recorded by experiments."""
    command = result.command
    command_values = {
        "mode": command.mode,
        "position": command.position,
    }
    if isinstance(command, MitCommand):
        command_values.update({
            "velocity": command.velocity,
            "kp": command.kp,
            "kd": command.kd,
            "feedforward": command.feedforward,
            "estimated_torque": command.estimated_torque,
        })
    return _json_value({
        "schema_version": 1,
        "timestamp": sample.timestamp,
        "period": sample.period,
        "robot_model": str(robot_model),
        "interaction_mode": str(interaction_mode),
        "controller": result.controller,
        "state": {
            "position": sample.state.position,
            "velocity": sample.state.velocity,
            "effort": sample.state.effort,
            "position_valid": sample.state.position_valid,
            "velocity_valid": sample.state.velocity_valid,
            "effort_valid": sample.state.effort_valid,
        },
        "reference": {
            "position": sample.reference.position,
            "velocity": sample.reference.velocity,
            "acceleration": sample.reference.acceleration,
            "external_wrench": sample.reference.external_wrench,
        },
        "command": command_values,
        "signals": result.signals,
    })
