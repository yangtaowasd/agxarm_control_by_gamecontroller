"""Zero-force and resistive Cartesian admittance laws."""

import numpy as np

from armbycontroller.admittance.core import CartesianAdmittance
from armbycontroller.cartesian import spatial_vector


class ZeroForceAdmittance(CartesianAdmittance):
    """
    Track near-zero wrench with anti-drift hold and light virtual friction.

    This is deliberately not an ideal zero-force integrator. A weak holding
    spring, viscous damping, and a bounded stick/slip wrench prevent observer
    bias from becoming motion while preserving hand-guided compliance.
    """

    mode = "zero_force"

    def __init__(
        self,
        virtual_mass,
        damping,
        holding_stiffness,
        friction,
        stiction_velocity,
        wrench_deadband,
        wrench_limit,
        offset_limit,
        velocity_limit,
        wrench_filter_hz=5.0,
    ):
        self.damping = spatial_vector(
            damping, "zero_force_damping", positive=True
        )
        self.holding_stiffness = spatial_vector(
            holding_stiffness,
            "zero_force_holding_stiffness",
            positive=True,
        )
        self.friction = spatial_vector(
            friction, "zero_force_friction", positive=True
        )
        self.stiction_velocity = spatial_vector(
            stiction_velocity,
            "zero_force_stiction_velocity",
            positive=True,
        )
        super().__init__(
            virtual_mass=virtual_mass,
            wrench_deadband=wrench_deadband,
            wrench_limit=wrench_limit,
            offset_limit=offset_limit,
            velocity_limit=velocity_limit,
            wrench_filter_hz=wrench_filter_hz,
        )

    def _mode_acceleration(self, applied_wrench, period):
        elastic_viscous = (
            self.holding_stiffness * self.offset
            + self.damping * self.velocity
        )
        drive = applied_wrench - elastic_viscous
        moving = np.abs(self.velocity) > self.stiction_velocity
        sticking = (~moving) & (np.abs(drive) <= self.friction)
        direction = np.where(
            moving,
            np.sign(self.velocity),
            np.sign(drive),
        )
        friction_wrench = self.friction * direction
        friction_wrench[sticking] = drive[sticking]
        resisting = elastic_viscous + friction_wrench
        acceleration = (applied_wrench - resisting) / self.virtual_mass

        # Cancel the small remaining velocity in one sample once the virtual
        # contact is inside the configured stick region.
        acceleration[sticking] = -self.velocity[sticking] / period
        resisting[sticking] = (
            applied_wrench[sticking]
            - self.virtual_mass[sticking] * acceleration[sticking]
        )
        return acceleration, resisting


class ResistiveAdmittance(CartesianAdmittance):
    """Add virtual damping and a spring toward the captured anchor pose."""

    mode = "resistive"

    def __init__(
        self,
        virtual_mass,
        damping,
        stiffness,
        wrench_deadband,
        wrench_limit,
        offset_limit,
        velocity_limit,
        wrench_filter_hz=5.0,
    ):
        self.damping = spatial_vector(
            damping, "resistive_damping", positive=True
        )
        self.stiffness = spatial_vector(
            stiffness, "resistive_stiffness", positive=True
        )
        super().__init__(
            virtual_mass=virtual_mass,
            wrench_deadband=wrench_deadband,
            wrench_limit=wrench_limit,
            offset_limit=offset_limit,
            velocity_limit=velocity_limit,
            wrench_filter_hz=wrench_filter_hz,
        )

    def _mode_acceleration(self, applied_wrench, period):
        del period
        resisting = self.damping * self.velocity + self.stiffness * self.offset
        return (applied_wrench - resisting) / self.virtual_mass, resisting
