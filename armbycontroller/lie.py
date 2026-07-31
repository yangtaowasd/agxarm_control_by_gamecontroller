"""Small SE(3) and spatial-vector algebra primitives."""

import numpy as np


def skew(vector, xp=np):
    """Return the 3x3 cross-product matrix of a length-three vector."""
    x, y, z = vector
    zero = xp.asarray(0.0, dtype=xp.asarray(vector).dtype)
    return xp.stack((
        xp.stack((zero, -z, y)),
        xp.stack((z, zero, -x)),
        xp.stack((-y, x, zero)),
    ))


def rotation_exp(axis, angle, xp=np):
    """Exponentiate one unit rotation axis."""
    axis = xp.asarray(axis)
    axis_hat = skew(axis, xp)
    identity = xp.eye(3, dtype=axis.dtype)
    return (
        identity
        + xp.sin(angle) * axis_hat
        + (1.0 - xp.cos(angle)) * (axis_hat @ axis_hat)
    )


def transform(rotation=None, translation=None, xp=np):
    """Build a homogeneous transform without in-place array updates."""
    rotation = xp.eye(3) if rotation is None else xp.asarray(rotation)
    translation = (
        xp.zeros(3) if translation is None else xp.asarray(translation)
    )
    top = xp.concatenate((rotation, translation.reshape(3, 1)), axis=1)
    bottom = xp.asarray([[0.0, 0.0, 0.0, 1.0]], dtype=rotation.dtype)
    return xp.concatenate((top, bottom), axis=0)


def transform_inverse(matrix, xp=np):
    """Invert an SE(3) homogeneous transform."""
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    return transform(rotation.T, -(rotation.T @ translation), xp)


def adjoint(matrix, xp=np):
    """Return the 6x6 motion adjoint for [angular; linear] twists."""
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    zero = xp.zeros((3, 3), dtype=rotation.dtype)
    return xp.concatenate((
        xp.concatenate((rotation, zero), axis=1),
        xp.concatenate((skew(translation, xp) @ rotation, rotation), axis=1),
    ), axis=0)


def motion_cross(twist, xp=np):
    """Return Featherstone's spatial motion cross-product matrix."""
    angular = twist[:3]
    linear = twist[3:]
    zero = xp.zeros((3, 3), dtype=xp.asarray(twist).dtype)
    return xp.concatenate((
        xp.concatenate((skew(angular, xp), zero), axis=1),
        xp.concatenate((skew(linear, xp), skew(angular, xp)), axis=1),
    ), axis=0)


def force_cross(twist, xp=np):
    """Return the spatial force cross-product matrix."""
    return -motion_cross(twist, xp).T


def joint_transform(kind, axis, position, xp=np):
    """Return the local joint exponential exp([S]q)."""
    axis = xp.asarray(axis)
    if kind in ("revolute", "continuous"):
        return transform(rotation_exp(axis, position, xp), xp=xp)
    if kind == "prismatic":
        return transform(translation=axis * position, xp=xp)
    return transform(xp=xp)


def twist_exp(screw, position, xp=np):
    """Exponentiate a unit space screw axis."""
    screw = xp.asarray(screw)
    angular = screw[:3]
    linear = screw[3:]
    angular_norm = float(np.linalg.norm(np.asarray(angular)))
    if angular_norm < 1e-12:
        return transform(translation=linear * position, xp=xp)
    angular = angular / angular_norm
    linear = linear / angular_norm
    angle = angular_norm * position
    angular_hat = skew(angular, xp)
    rotation = rotation_exp(angular, angle, xp)
    identity = xp.eye(3, dtype=screw.dtype)
    translation = (
        identity * angle
        + (1.0 - xp.cos(angle)) * angular_hat
        + (angle - xp.sin(angle)) * (angular_hat @ angular_hat)
    ) @ linear
    return transform(rotation, translation, xp)


def spatial_inertia(mass, center, inertia_center, xp=np):
    """Construct link-frame spatial inertia about the link origin."""
    center = xp.asarray(center)
    inertia_center = xp.asarray(inertia_center)
    center_hat = skew(center, xp)
    identity = xp.eye(3, dtype=center.dtype)
    rotational = inertia_center - mass * (center_hat @ center_hat)
    return xp.concatenate((
        xp.concatenate((rotational, mass * center_hat), axis=1),
        xp.concatenate((-mass * center_hat, mass * identity), axis=1),
    ), axis=0)
