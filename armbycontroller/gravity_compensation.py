"""Compatibility names for the unified screw-theory robot model."""

from armbycontroller.screw_model import project_gravity_vector
from armbycontroller.screw_model import UrdfScrewModel


def nero_mount_gravity(mount):
    """Return base-frame gravity for one explicit Nero mounting choice."""
    try:
        return list(project_gravity_vector(mount))
    except ValueError as error:
        raise ValueError("nero_mount must be horizontal or side") from error


# Keep the old import stable for downstream code while all kinematics and
# dynamics now run through the one PoE/RNEA implementation.
UrdfGravityModel = UrdfScrewModel
