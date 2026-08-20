"""Pure lifecycle rules for mutually exclusive interaction controllers."""

from dataclasses import dataclass


INTERACTION_MODES = ("normal", "impedance", "admittance")


@dataclass(frozen=True)
class InteractionTransition:
    """Required committed states for one requested interaction mode."""

    source: str
    target: str
    path: tuple[str, ...]


class InteractionModeLifecycle:
    """Own the normal/impedance/admittance transition invariant."""

    def __init__(self, active="normal"):
        self._active = self._mode(active)

    @staticmethod
    def _mode(value):
        mode = str(value).strip().lower()
        if mode not in INTERACTION_MODES:
            raise ValueError(
                "interaction mode must be normal, impedance, or admittance"
            )
        return mode

    @property
    def active(self):
        return self._active

    def synchronize(self, impedance_enabled, admittance_enabled):
        """Import legacy boolean state while enforcing mutual exclusion."""
        impedance = bool(impedance_enabled)
        admittance = bool(admittance_enabled)
        if impedance and admittance:
            raise RuntimeError(
                "impedance and admittance cannot be active together"
            )
        observed = (
            "impedance"
            if impedance else "admittance" if admittance else "normal"
        )
        self._active = observed
        return observed

    def plan(self, target):
        """Return the only legal committed path to a requested target."""
        target = self._mode(target)
        if target == self._active:
            target = "normal"
        if self._active == "normal" or target == "normal":
            path = (target,)
        else:
            path = ("normal", target)
        return InteractionTransition(self._active, target, path)

    def commit(self, mode):
        """Commit one completed state and reject direct cross transitions."""
        mode = self._mode(mode)
        if (
            self._active != "normal"
            and mode != "normal"
            and mode != self._active
        ):
            raise RuntimeError(
                "interaction controllers must transition through normal mode"
            )
        self._active = mode
        return self._active
