"""Cartesian admittance modes with one shared safety envelope."""

from armbycontroller.admittance.core import CartesianAdmittance
from armbycontroller.admittance.core import CartesianAdmittanceState
from armbycontroller.admittance.controller import (
    ADMITTANCE_MIT_TORQUE_LIMIT_MAX,
)
from armbycontroller.admittance.controller import CartesianAdmittanceController
from armbycontroller.admittance.resistive import ResistiveAdmittance
from armbycontroller.admittance.zero_force import ZeroForceAdmittance
from armbycontroller.ik.screw import BoundedScrewVelocityIk


ADMITTANCE_MODES = ("zero_force", "resistive")
BoundedVelocityIk = BoundedScrewVelocityIk


def create_cartesian_admittance(
    mode,
    *,
    virtual_mass,
    zero_force_damping,
    zero_force_holding_stiffness,
    zero_force_friction,
    zero_force_stiction_velocity,
    resistive_damping,
    resistive_stiffness,
    **common_settings,
):
    """Create one explicit admittance mode from validated ROS settings."""
    selected = str(mode).strip().lower()
    if selected == "zero_force":
        return ZeroForceAdmittance(
            virtual_mass=virtual_mass,
            damping=zero_force_damping,
            holding_stiffness=zero_force_holding_stiffness,
            friction=zero_force_friction,
            stiction_velocity=zero_force_stiction_velocity,
            **common_settings,
        )
    if selected == "resistive":
        return ResistiveAdmittance(
            virtual_mass=virtual_mass,
            damping=resistive_damping,
            stiffness=resistive_stiffness,
            **common_settings,
        )
    raise ValueError(
        "admittance_mode must be zero_force or resistive"
    )


__all__ = [
    "ADMITTANCE_MODES",
    "ADMITTANCE_MIT_TORQUE_LIMIT_MAX",
    "BoundedScrewVelocityIk",
    "BoundedVelocityIk",
    "CartesianAdmittance",
    "CartesianAdmittanceController",
    "CartesianAdmittanceState",
    "ResistiveAdmittance",
    "ZeroForceAdmittance",
    "create_cartesian_admittance",
]
