"""Damped and restoring Cartesian admittance."""

from armbycontroller.admittance.core import CartesianAdmittance
from armbycontroller.cartesian import spatial_vector


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
