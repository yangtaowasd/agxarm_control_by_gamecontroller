"""Convert the fixed keyboard protocol into limit-safe control intent."""

from dataclasses import dataclass
from typing import Sequence


KEY_JOINT_1, KEY_JOINT_7 = 0, 6
KEY_DECREASE, KEY_INCREASE = 7, 8
KEY_HOME, KEY_ESTOP, KEY_MODE_TOGGLE = 9, 10, 11
KEY_FORWARD, KEY_BACKWARD, KEY_Z_UP, KEY_Z_DOWN = 12, 13, 14, 15
KEY_IMPEDANCE_TOGGLE = 16
KEY_ARROW_UP, KEY_ARROW_DOWN = 17, 18
KEY_ARROW_LEFT, KEY_ARROW_RIGHT = 19, 20
KEY_ROLL_LEFT, KEY_ROLL_RIGHT = 21, 22
KEY_ADMITTANCE_TOGGLE = 23
KEY_HYBRID_TOGGLE = 24
KEY_COUNT = 25


def _clamp(value, low, high):
    return min(max(value, low), high)


@dataclass(frozen=True)
class JogUpdate:
    """One edge-detected update from the keyboard state."""

    selected_joint: int
    target_changed: bool = False
    selection_changed: bool = False
    home_requested: bool = False
    estop_requested: bool = False
    mode_toggle_requested: bool = False
    impedance_toggle_requested: bool = False
    admittance_toggle_requested: bool = False
    hybrid_toggle_requested: bool = False


class ArmJointJogState:
    """Track selection, key edges, and a limit-safe joint target."""

    def __init__(self, joint_limits, step_rad, initial_joints=None):
        if not 1 <= len(joint_limits) <= 7 or step_rad <= 0.0:
            raise ValueError("arm requires 1-7 limits and step_rad > 0")
        self.joint_count = len(joint_limits)
        self.joint_limits = [
            tuple(map(float, limit)) for limit in joint_limits
        ]
        if any(low >= high for low, high in self.joint_limits):
            raise ValueError("each joint limit must satisfy low < high")
        self.step_rad = float(step_rad)
        self.selected_joint = 0
        self.target_joints = [0.0] * self.joint_count
        self.previous_keys = [0] * KEY_COUNT
        if initial_joints is not None:
            self.sync_target(initial_joints)

    def sync_target(
        self, joints: Sequence[float], clamp_to_limits: bool = True
    ):
        if len(joints) != self.joint_count:
            raise ValueError("joint target count does not match the arm")
        values = [float(value) for value in joints]
        if clamp_to_limits:
            values = [
                _clamp(value, low, high)
                for value, (low, high) in zip(values, self.joint_limits)
            ]
        self.target_joints = values

    def update(self, keys: Sequence[int]):
        """Apply one keyboard sample and return edge-triggered intent."""
        if len(keys) < KEY_COUNT:
            raise ValueError(f"keyboard state must contain {KEY_COUNT} values")
        pressed = [bool(value) for value in keys[:KEY_COUNT]]
        rising = [
            current and not previous
            for current, previous in zip(pressed, self.previous_keys)
        ]
        old_selection = self.selected_joint
        for index in range(KEY_JOINT_1, self.joint_count):
            if rising[index]:
                self.selected_joint = index

        home = rising[KEY_HOME]
        estop = rising[KEY_ESTOP]
        toggle = rising[KEY_MODE_TOGGLE]
        impedance_toggle = rising[KEY_IMPEDANCE_TOGGLE]
        admittance_toggle = rising[KEY_ADMITTANCE_TOGGLE]
        hybrid_toggle = rising[KEY_HYBRID_TOGGLE]
        changed = False
        if home:
            target = [
                _clamp(0.0, low, high) for low, high in self.joint_limits
            ]
            changed, self.target_joints = target != self.target_joints, target
        elif not any((
            estop,
            toggle,
            impedance_toggle,
            admittance_toggle,
            hybrid_toggle,
        )):
            direction = pressed[KEY_INCREASE] - pressed[KEY_DECREASE]
            if direction:
                low, high = self.joint_limits[self.selected_joint]
                old = self.target_joints[self.selected_joint]
                if old < low:
                    new = (
                        min(old + self.step_rad, low)
                        if direction > 0 else old
                    )
                elif old > high:
                    new = (
                        max(old - self.step_rad, high)
                        if direction < 0 else old
                    )
                else:
                    new = _clamp(
                        old + direction * self.step_rad, low, high
                    )
                self.target_joints[self.selected_joint] = new
                changed = new != old
        self.previous_keys = pressed
        return JogUpdate(
            self.selected_joint,
            changed,
            self.selected_joint != old_selection,
            home,
            estop,
            toggle,
            impedance_toggle,
            admittance_toggle,
            hybrid_toggle,
        )
