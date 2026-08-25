"""Classical static Stribeck friction model."""

import numpy as np


def _parameter_size(*values):
    sizes = [np.asarray(value, dtype=float).size for value in values]
    vector_sizes = {size for size in sizes if size != 1}
    if len(vector_sizes) > 1:
        raise ValueError("Stribeck parameter vectors must have equal lengths")
    return vector_sizes.pop() if vector_sizes else 1


def _parameter_vector(values, size, name, *, positive=False):
    result = np.asarray(values, dtype=float)
    if result.ndim == 0 or result.size == 1:
        result = np.full(size, float(result.reshape(-1)[0]))
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be one finite value or {size} values")
    if positive and np.any(result <= 0.0):
        raise ValueError(f"{name} must be positive")
    if not positive and np.any(result < 0.0):
        raise ValueError(f"{name} must be nonnegative")
    return result.copy()


class StribeckFrictionModel:
    r"""
    Evaluate the classical velocity-dependent Stribeck equation.

    The returned signed friction magnitude is

    ``tau_f(v) = [tau_c + (tau_s - tau_c) exp(-(|v|/v_s)^a)]
                  sign(v) + b v``.

    It has the direction of motion and is normally subtracted from the drive
    torque (or added as a compensation torque).  This static model uses the
    explicit convention ``sign(0) = 0``; presliding/stick dynamics require a
    dynamic friction model and are intentionally not invented here.
    """

    def __init__(
        self,
        static_friction,
        coulomb_friction,
        stribeck_velocity,
        viscous_coefficient=0.0,
        exponent=2.0,
    ):
        self.joint_count = _parameter_size(
            static_friction,
            coulomb_friction,
            stribeck_velocity,
            viscous_coefficient,
            exponent,
        )
        self.static_friction = _parameter_vector(
            static_friction, self.joint_count, "static_friction"
        )
        self.coulomb_friction = _parameter_vector(
            coulomb_friction, self.joint_count, "coulomb_friction"
        )
        self.stribeck_velocity = _parameter_vector(
            stribeck_velocity,
            self.joint_count,
            "stribeck_velocity",
            positive=True,
        )
        self.viscous_coefficient = _parameter_vector(
            viscous_coefficient,
            self.joint_count,
            "viscous_coefficient",
        )
        self.exponent = _parameter_vector(
            exponent, self.joint_count, "exponent", positive=True
        )
        if np.any(self.static_friction < self.coulomb_friction):
            raise ValueError(
                "static_friction must be greater than or equal to "
                "coulomb_friction"
            )

    def evaluate(self, velocity):
        """Return ``tau_f`` for one finite joint-velocity vector."""
        velocity = np.asarray(velocity, dtype=float)
        if velocity.ndim == 0 and self.joint_count == 1:
            velocity = velocity.reshape(1)
        if (
            velocity.shape != (self.joint_count,)
            or not np.all(np.isfinite(velocity))
        ):
            raise ValueError(
                f"velocity must contain {self.joint_count} finite values"
            )
        ratio = np.abs(velocity) / self.stribeck_velocity
        stribeck = np.exp(-np.power(ratio, self.exponent))
        dry_friction = self.coulomb_friction + (
            self.static_friction - self.coulomb_friction
        ) * stribeck
        return (
            dry_friction * np.sign(velocity)
            + self.viscous_coefficient * velocity
        )

    __call__ = evaluate
