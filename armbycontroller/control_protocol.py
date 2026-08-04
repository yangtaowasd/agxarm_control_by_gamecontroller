"""Stable key-state protocol shared by native and backend controllers."""


KEY_JOINT_1, KEY_JOINT_7 = 0, 6
KEY_DECREASE, KEY_INCREASE = 7, 8
KEY_HOME, KEY_ESTOP, KEY_MODE_TOGGLE = 9, 10, 11
KEY_FORWARD, KEY_BACKWARD, KEY_Z_UP, KEY_Z_DOWN = 12, 13, 14, 15
KEY_IMPEDANCE_TOGGLE = 16
KEY_ARROW_UP, KEY_ARROW_DOWN = 17, 18
KEY_ARROW_LEFT, KEY_ARROW_RIGHT = 19, 20
KEY_ROLL_LEFT, KEY_ROLL_RIGHT = 21, 22
KEY_COUNT = 23


ACTION_KEYS = {
    "home": KEY_HOME,
    "estop": KEY_ESTOP,
    "toggle_control_mode": KEY_MODE_TOGGLE,
    "toggle_impedance": KEY_IMPEDANCE_TOGGLE,
}


def sanitize_controller_keys(keys):
    """Validate and normalize one complete controller key-state frame."""
    if not isinstance(keys, list) or len(keys) != KEY_COUNT:
        raise ValueError(
            f"controller state must contain exactly {KEY_COUNT} keys"
        )
    normalized = []
    for value in keys:
        if value is True or value == 1:
            normalized.append(1)
        elif value is False or value == 0:
            normalized.append(0)
        else:
            raise ValueError("controller key values must be 0 or 1")
    return normalized
