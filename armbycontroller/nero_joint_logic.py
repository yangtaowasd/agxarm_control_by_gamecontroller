"""Pure state logic for independent NERO joint keyboard jogging."""

from dataclasses import dataclass
from typing import Sequence


JOINT_COUNT = 7
KEY_JOINT_1 = 0
KEY_JOINT_7 = 6
KEY_DECREASE = 7
KEY_INCREASE = 8
KEY_HOME = 9
KEY_ESTOP = 10
KEY_COUNT = 11


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


@dataclass(frozen=True)
class JogUpdate:
    selected_joint: int
    target_changed: bool = False
    selection_changed: bool = False
    home_requested: bool = False
    estop_requested: bool = False


class NeroJointJogState:
    """Track the selected joint and a limit-safe seven-joint target."""

    def __init__(
        self,
        joint_limits: Sequence[Sequence[float]],
        step_rad: float,
        initial_joints: Sequence[float] | None = None,
    ) -> None:
        if len(joint_limits) != JOINT_COUNT:
            raise ValueError("NERO requires exactly 7 joint limits")
        if step_rad <= 0.0:
            raise ValueError("step_rad must be > 0")

        self.joint_limits = [tuple(map(float, limit)) for limit in joint_limits]
        if any(low >= high for low, high in self.joint_limits):
            raise ValueError("each joint limit must satisfy low < high")

        self.step_rad = float(step_rad)
        self.selected_joint = 0
        self.target_joints = [0.0] * JOINT_COUNT
        self.previous_keys = [0] * KEY_COUNT
        if initial_joints is not None:
            self.sync_target(initial_joints)

    def sync_target(self, joints: Sequence[float]) -> None:
        if len(joints) != JOINT_COUNT:
            raise ValueError("NERO joint target must contain 7 values")
        self.target_joints = [
            clamp(float(value), low, high)
            for value, (low, high) in zip(joints, self.joint_limits)
        ]

    def update(self, keys: Sequence[int]) -> JogUpdate:
        if len(keys) < KEY_COUNT:
            raise ValueError(f"keyboard state must contain {KEY_COUNT} values")

        pressed = [1 if value else 0 for value in keys[:KEY_COUNT]]
        rising = [
            bool(current and not previous)
            for current, previous in zip(pressed, self.previous_keys)
        ]

        selection_changed = False
        for index in range(KEY_JOINT_1, KEY_JOINT_7 + 1):
            if rising[index]:
                selection_changed = self.selected_joint != index
                self.selected_joint = index

        home_requested = rising[KEY_HOME]
        estop_requested = rising[KEY_ESTOP]
        target_changed = False

        if home_requested:
            home = [
                clamp(0.0, low, high)
                for low, high in self.joint_limits
            ]
            target_changed = home != self.target_joints
            self.target_joints = home
        elif not estop_requested:
            direction = pressed[KEY_INCREASE] - pressed[KEY_DECREASE]
            if direction:
                low, high = self.joint_limits[self.selected_joint]
                old_value = self.target_joints[self.selected_joint]
                self.target_joints[self.selected_joint] = clamp(
                    old_value + direction * self.step_rad,
                    low,
                    high,
                )
                target_changed = self.target_joints[self.selected_joint] != old_value

        self.previous_keys = pressed
        return JogUpdate(
            selected_joint=self.selected_joint,
            target_changed=target_changed,
            selection_changed=selection_changed,
            home_requested=home_requested,
            estop_requested=estop_requested,
        )
