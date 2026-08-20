"""Transport-neutral interaction-mode command and state contracts."""

import math
from collections.abc import Callable
from dataclasses import dataclass

from armbycontroller.control.interaction import INTERACTION_MODES


INTERACTION_API_SCHEMA_VERSION = 1
PUBLIC_INTERACTION_MODES = ("normal", "impedance", "admittance")


@dataclass(frozen=True)
class InteractionModeRequestResult:
    """Stable result returned by every public mode-command adapter."""

    success: bool
    requested_mode: str
    active_mode: str
    changed: bool
    message: str

    def to_payload(self):
        """Return a JSON-safe schema-v1 service response."""
        return {
            "schema_version": INTERACTION_API_SCHEMA_VERSION,
            "success": self.success,
            "requested_mode": self.requested_mode,
            "active_mode": self.active_mode,
            "changed": self.changed,
            "message": self.message,
        }


class InteractionModeInterface:
    """Set public modes idempotently through the mandatory normal state."""

    def __init__(
        self,
        *,
        current_mode: Callable[[], str],
        enter_normal: Callable[[str], object],
        enter_impedance: Callable[[str], object],
        enter_admittance: Callable[[str], object],
    ):
        callbacks = {
            "current_mode": current_mode,
            "enter_normal": enter_normal,
            "enter_impedance": enter_impedance,
            "enter_admittance": enter_admittance,
        }
        if not all(callable(callback) for callback in callbacks.values()):
            raise TypeError("interaction interface callbacks must be callable")
        self._current_mode = current_mode
        self._handlers = {
            "normal": enter_normal,
            "impedance": enter_impedance,
            "admittance": enter_admittance,
        }

    @staticmethod
    def _public_mode(value):
        mode = str(value).strip().lower()
        if mode not in PUBLIC_INTERACTION_MODES:
            raise ValueError(
                "public interaction mode must be normal, impedance, or "
                "admittance"
            )
        return mode

    def _active_mode(self):
        active = str(self._current_mode()).strip().lower()
        if active not in INTERACTION_MODES:
            raise RuntimeError(f"invalid active interaction mode: {active}")
        return active

    def _invoke(self, mode, reason):
        try:
            self._handlers[mode](reason)
        except Exception as error:
            return error
        return None

    @staticmethod
    def _result(initial, requested, active, success, message):
        return InteractionModeRequestResult(
            bool(success),
            requested,
            active,
            active != initial,
            str(message),
        )

    def request(self, mode, *, source="external"):
        """Set one public mode and report the observed committed state."""
        requested = self._public_mode(mode)
        initial = self._active_mode()
        if requested == initial:
            return self._result(
                initial,
                requested,
                initial,
                True,
                f"interaction mode already {requested}",
            )
        source = str(source).strip() or "external"
        reason = f"{source} requested {requested}"
        if initial != "normal" and requested != "normal":
            error = self._invoke("normal", reason)
            active = self._active_mode()
            if error is not None or active != "normal":
                detail = f": {error}" if error is not None else ""
                return self._result(
                    initial,
                    requested,
                    active,
                    False,
                    f"failed to leave {initial} for normal mode{detail}",
                )
        error = self._invoke(requested, reason)
        active = self._active_mode()
        success = error is None and active == requested
        message = (
            f"interaction mode set to {requested}"
            if success
            else (
                f"failed to enter {requested}; active mode is {active}"
                + (f": {error}" if error is not None else "")
            )
        )
        return self._result(
            initial, requested, active, success, message
        )


def interaction_state_payload(
    interaction_mode,
    *,
    timestamp,
    robot_model,
    **fields,
):
    """Build the stable JSON state consumed by external clients."""
    active = str(interaction_mode).strip().lower()
    if active not in INTERACTION_MODES:
        raise ValueError(f"invalid interaction mode: {active}")
    timestamp = float(timestamp)
    if not math.isfinite(timestamp):
        raise ValueError("interaction-state timestamp must be finite")
    reserved = {
        "schema_version",
        "timestamp",
        "robot_model",
        "interaction_mode",
        "available_modes",
    }
    if reserved.intersection(fields):
        raise ValueError("interaction-state fields contain a reserved name")
    return {
        "schema_version": INTERACTION_API_SCHEMA_VERSION,
        "timestamp": timestamp,
        "robot_model": str(robot_model),
        "interaction_mode": active,
        "available_modes": list(PUBLIC_INTERACTION_MODES),
        **fields,
    }
