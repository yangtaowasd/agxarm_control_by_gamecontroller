"""Read-only URDF/Xacro rigid-body dynamics for robot-arm trees."""

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def _vector(element, attribute, default):
    if element is None or attribute not in element.attrib:
        return np.asarray(default, dtype=float)
    values = np.fromstring(element.attrib[attribute], sep=" ", dtype=float)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError(f"invalid URDF {attribute!r} vector")
    return values


def _rpy_rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cosine, sine = math.cos(angle), math.sin(angle)
    one_minus_cosine = 1.0 - cosine
    return np.array(
        [
            [cosine + x * x * one_minus_cosine,
             x * y * one_minus_cosine - z * sine,
             x * z * one_minus_cosine + y * sine],
            [y * x * one_minus_cosine + z * sine,
             cosine + y * y * one_minus_cosine,
             y * z * one_minus_cosine - x * sine],
            [z * x * one_minus_cosine - y * sine,
             z * y * one_minus_cosine + x * sine,
             cosine + z * z * one_minus_cosine],
        ],
        dtype=float,
    )


def _transform(xyz=None, rotation=None):
    result = np.eye(4, dtype=float)
    if xyz is not None:
        result[:3, 3] = xyz
    if rotation is not None:
        result[:3, :3] = rotation
    return result


@dataclass(frozen=True)
class _LinkMass:
    mass: float
    center: np.ndarray
    inertia: np.ndarray
    inertia_rotation: np.ndarray


_ZERO_LINK_MASS = _LinkMass(
    0.0, np.zeros(3), np.zeros((3, 3)), np.eye(3)
)


@dataclass(frozen=True)
class _UrdfJoint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


class UrdfGravityModel:
    """Compute arm inverse dynamics including attached model branches."""

    def __init__(
        self,
        urdf_path,
        base_link,
        tip_link,
        joint_count,
        gravity=(0.0, 0.0, -9.80665),
    ):
        self.urdf_path = Path(urdf_path).resolve()
        self.base_link = str(base_link)
        self.tip_link = str(tip_link)
        self.joint_count = int(joint_count)
        self.gravity = np.asarray(gravity, dtype=float)
        if self.gravity.shape != (3,) or not np.all(np.isfinite(self.gravity)):
            raise ValueError("gravity must contain three finite values")
        self.links, self.joints = self._read_model()
        self.joints_by_child = {joint.child: joint for joint in self.joints}
        self.children = {}
        for joint in self.joints:
            self.children.setdefault(joint.parent, []).append(joint)
        self.movable_joint_names = self._controlled_chain()
        if len(self.movable_joint_names) != self.joint_count:
            raise ValueError(
                f"URDF chain must have {self.joint_count} movable joints, "
                f"got {len(self.movable_joint_names)}"
            )

        self.controlled_indices = {
            name: index for index, name in enumerate(self.movable_joint_names)
        }
        reachable = self._reachable_links()
        self.modeled_mass = sum(
            link.mass for name, link in self.links.items()
            if name in reachable and name != self.base_link
        )

    def _resolve_include(self, including_path, filename):
        marker = "/agx_arm_urdf/"
        if marker in filename:
            relative = filename.split(marker, 1)[1]
            asset_root = next(
                (parent for parent in including_path.parents
                 if parent.name == "agx_arm_urdf"),
                None,
            )
            if asset_root is None:
                raise ValueError("cannot resolve agx_arm_urdf Xacro include")
            return asset_root / relative
        candidate = Path(filename)
        if not candidate.is_absolute():
            candidate = including_path.parent / candidate
        return candidate.resolve()

    def _expanded_robot(self, path, include_stack=()):
        path = Path(path).resolve()
        if path in include_stack:
            raise ValueError(f"recursive URDF/Xacro include: {path}")
        root = ET.parse(path).getroot()
        for element in list(root):
            if not element.tag.endswith("}include"):
                continue
            filename = element.attrib.get("filename", "")
            included_path = self._resolve_include(path, filename)
            included = self._expanded_robot(
                included_path, include_stack + (path,)
            )
            position = list(root).index(element)
            root.remove(element)
            for child in list(included):
                root.insert(position, child)
                position += 1
        return root

    def _read_model(self):
        root = self._expanded_robot(self.urdf_path)
        links = {}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                links[link.attrib["name"]] = _ZERO_LINK_MASS
                continue
            mass_element = inertial.find("mass")
            if mass_element is None:
                raise ValueError(f"link {link.attrib['name']} has no mass")
            mass = float(mass_element.attrib["value"])
            inertial_origin = inertial.find("origin")
            center = _vector(inertial_origin, "xyz", (0.0, 0.0, 0.0))
            inertia_rotation = _rpy_rotation(
                _vector(inertial_origin, "rpy", (0.0, 0.0, 0.0))
            )
            if not math.isfinite(mass) or mass < 0.0:
                raise ValueError(f"link {link.attrib['name']} has invalid mass")
            inertia_element = inertial.find("inertia")
            if inertia_element is None:
                raise ValueError(f"link {link.attrib['name']} has no inertia")
            inertia = np.array(
                [
                    [float(inertia_element.attrib["ixx"]),
                     float(inertia_element.attrib["ixy"]),
                     float(inertia_element.attrib["ixz"])],
                    [float(inertia_element.attrib["ixy"]),
                     float(inertia_element.attrib["iyy"]),
                     float(inertia_element.attrib["iyz"])],
                    [float(inertia_element.attrib["ixz"]),
                     float(inertia_element.attrib["iyz"]),
                     float(inertia_element.attrib["izz"])],
                ],
                dtype=float,
            )
            if not np.all(np.isfinite(inertia)):
                raise ValueError(f"link {link.attrib['name']} has invalid inertia")
            links[link.attrib["name"]] = _LinkMass(
                mass, center, inertia, inertia_rotation
            )

        joints = []
        child_names = set()
        for joint in root.findall("joint"):
            parent = joint.find("parent").attrib["link"]
            child = joint.find("child").attrib["link"]
            if child in child_names:
                raise ValueError(f"link {child} has more than one parent joint")
            child_names.add(child)
            kind = joint.attrib["type"]
            if kind not in ("fixed", "revolute", "continuous", "prismatic"):
                raise ValueError(f"unsupported joint type {kind!r}")
            origin_element = joint.find("origin")
            xyz = _vector(origin_element, "xyz", (0.0, 0.0, 0.0))
            rpy = _vector(origin_element, "rpy", (0.0, 0.0, 0.0))
            axis = _vector(joint.find("axis"), "xyz", (1.0, 0.0, 0.0))
            axis_norm = float(np.linalg.norm(axis))
            if kind != "fixed" and axis_norm < 1e-12:
                raise ValueError(f"joint {joint.attrib['name']} has zero axis")
            if axis_norm >= 1e-12:
                axis = axis / axis_norm
            joints.append(
                _UrdfJoint(
                    joint.attrib["name"], kind, parent, child,
                    _transform(xyz, _rpy_rotation(rpy)), axis,
                )
            )
        return links, joints

    def _controlled_chain(self):
        reverse_chain = []
        child_name = self.tip_link
        visited = set()
        while child_name != self.base_link:
            if child_name in visited:
                raise ValueError("URDF contains a cycle in the requested chain")
            visited.add(child_name)
            joint = self.joints_by_child.get(child_name)
            if joint is None:
                raise ValueError(
                    f"no URDF chain from {self.base_link} to {self.tip_link}"
                )
            if joint.kind in ("revolute", "continuous"):
                reverse_chain.append(joint.name)
            elif joint.kind != "fixed":
                raise ValueError("controlled arm chain must use revolute joints")
            child_name = joint.parent
        return list(reversed(reverse_chain))

    def _reachable_links(self):
        reachable = set()
        pending = [self.base_link]
        while pending:
            link = pending.pop()
            if link in reachable:
                raise ValueError("URDF contains a cycle below the base link")
            reachable.add(link)
            pending.extend(
                joint.child for joint in self.children.get(link, ())
            )
        return reachable

    def _positions(self, joint_positions, name):
        positions = np.asarray(joint_positions, dtype=float)
        if positions.shape != (self.joint_count,) or not np.all(
            np.isfinite(positions)
        ):
            raise ValueError(
                f"{name} must contain {self.joint_count} finite values"
            )
        return positions

    def _kinematic_state(self, joint_positions):
        positions = self._positions(joint_positions, "joint_positions")
        joint_origins = [None] * self.joint_count
        joint_axes = [None] * self.joint_count
        link_states = []

        def visit(link_name, transform, active_joints):
            link = self.links.get(link_name, _ZERO_LINK_MASS)
            center_world = (
                transform @ np.append(link.center, 1.0)
            )[:3]
            inertia_world_rotation = (
                transform[:3, :3] @ link.inertia_rotation
            )
            inertia_world = (
                inertia_world_rotation @ link.inertia
                @ inertia_world_rotation.T
            )
            link_states.append(
                (tuple(active_joints), link.mass, center_world, inertia_world)
            )
            for joint in self.children.get(link_name, ()):
                joint_frame = transform @ joint.origin
                child_transform = joint_frame
                child_active = list(active_joints)
                controlled_index = self.controlled_indices.get(joint.name)
                if controlled_index is not None:
                    joint_origins[controlled_index] = joint_frame[:3, 3].copy()
                    joint_axes[controlled_index] = (
                        joint_frame[:3, :3] @ joint.axis
                    )
                    rotation = _axis_rotation(
                        joint.axis, float(positions[controlled_index])
                    )
                    child_transform = joint_frame @ _transform(rotation=rotation)
                    child_active.append(controlled_index)
                # Accessory joints are evaluated at their URDF zero position.
                visit(joint.child, child_transform, child_active)

        visit(self.base_link, np.eye(4, dtype=float), [])
        if any(value is None for value in joint_origins + joint_axes):
            raise ValueError("not all controlled joints are reachable from base")
        return joint_origins, joint_axes, link_states

    def compensation(self, joint_positions):
        """Return torques that oppose gravity at the measured configuration."""
        joint_origins, joint_axes, link_states = self._kinematic_state(
            joint_positions
        )

        gravitational_torque = np.zeros(self.joint_count, dtype=float)
        for joint_index, (origin, axis) in enumerate(zip(joint_origins, joint_axes)):
            for active_joints, mass, center, _ in link_states:
                if joint_index not in active_joints or mass == 0.0:
                    continue
                force = mass * self.gravity
                gravitational_torque[joint_index] += np.dot(
                    axis, np.cross(center - origin, force)
                )
        return -gravitational_torque

    def mass_matrix(self, joint_positions):
        """Return the joint-space mass matrix at one arm configuration."""
        joint_origins, joint_axes, link_states = self._kinematic_state(
            joint_positions
        )
        matrix = np.zeros((self.joint_count, self.joint_count), dtype=float)
        for active_joints, mass, center, inertia_world in link_states:
            if not active_joints or mass == 0.0:
                continue
            linear_jacobian = np.zeros((3, self.joint_count), dtype=float)
            angular_jacobian = np.zeros((3, self.joint_count), dtype=float)
            for joint_index in active_joints:
                axis = joint_axes[joint_index]
                origin = joint_origins[joint_index]
                linear_jacobian[:, joint_index] = np.cross(
                    axis, center - origin
                )
                angular_jacobian[:, joint_index] = axis
            matrix += mass * linear_jacobian.T @ linear_jacobian
            matrix += angular_jacobian.T @ inertia_world @ angular_jacobian
        return 0.5 * (matrix + matrix.T)

    def _recursive_inverse_dynamics(
        self, joint_positions, joint_velocities, joint_accelerations, gravity
    ):
        """Evaluate inverse dynamics by recursive Newton-Euler traversal."""
        positions = self._positions(joint_positions, "joint_positions")
        velocities = self._positions(joint_velocities, "joint_velocities")
        accelerations = self._positions(
            joint_accelerations, "joint_accelerations"
        )
        gravity = np.asarray(gravity, dtype=float)
        if gravity.shape != (3,) or not np.all(np.isfinite(gravity)):
            raise ValueError("gravity must contain three finite values")
        states = {}
        joint_frames = {}

        def forward(link_name, transform, omega, alpha, origin_acceleration):
            link = self.links.get(link_name, _ZERO_LINK_MASS)
            rotation = transform[:3, :3]
            center_offset = rotation @ link.center
            center_acceleration = (
                origin_acceleration
                + np.cross(alpha, center_offset)
                + np.cross(omega, np.cross(omega, center_offset))
            )
            inertia_rotation = rotation @ link.inertia_rotation
            inertia_world = (
                inertia_rotation @ link.inertia @ inertia_rotation.T
            )
            force = link.mass * center_acceleration
            center_moment = (
                inertia_world @ alpha
                + np.cross(omega, inertia_world @ omega)
            )
            states[link_name] = {
                "position": transform[:3, 3].copy(),
                "force": force,
                "moment": center_moment + np.cross(center_offset, force),
            }
            for joint in self.children.get(link_name, ()):
                joint_frame = transform @ joint.origin
                joint_position = joint_frame[:3, 3]
                offset = joint_position - transform[:3, 3]
                child_origin_acceleration = (
                    origin_acceleration
                    + np.cross(alpha, offset)
                    + np.cross(omega, np.cross(omega, offset))
                )
                child_transform = joint_frame
                child_omega = omega
                child_alpha = alpha
                controlled_index = self.controlled_indices.get(joint.name)
                axis_world = joint_frame[:3, :3] @ joint.axis
                if controlled_index is not None:
                    joint_velocity = velocities[controlled_index]
                    joint_acceleration = accelerations[controlled_index]
                    child_omega = omega + axis_world * joint_velocity
                    child_alpha = (
                        alpha
                        + axis_world * joint_acceleration
                        + np.cross(omega, axis_world * joint_velocity)
                    )
                    child_transform = joint_frame @ _transform(
                        rotation=_axis_rotation(
                            joint.axis, positions[controlled_index]
                        )
                    )
                joint_frames[joint.name] = (joint_position, axis_world)
                forward(
                    joint.child,
                    child_transform,
                    child_omega,
                    child_alpha,
                    child_origin_acceleration,
                )

        forward(
            self.base_link,
            np.eye(4, dtype=float),
            np.zeros(3),
            np.zeros(3),
            -gravity,
        )
        torque = np.zeros(self.joint_count, dtype=float)

        def backward(link_name):
            state = states[link_name]
            total_force = state["force"].copy()
            total_moment = state["moment"].copy()
            link_position = state["position"]
            for joint in self.children.get(link_name, ()):
                child_force, child_moment = backward(joint.child)
                joint_position, axis_world = joint_frames[joint.name]
                controlled_index = self.controlled_indices.get(joint.name)
                if controlled_index is not None:
                    torque[controlled_index] = np.dot(
                        axis_world, child_moment
                    )
                total_force += child_force
                total_moment += (
                    child_moment
                    + np.cross(joint_position - link_position, child_force)
                )
            return total_force, total_moment

        backward(self.base_link)
        return torque

    def coriolis(self, joint_positions, joint_velocities):
        """Return Coriolis and centrifugal torque at one arm state."""
        return self._recursive_inverse_dynamics(
            joint_positions,
            joint_velocities,
            np.zeros(self.joint_count),
            np.zeros(3),
        )

    def inverse_dynamics(
        self, joint_positions, joint_velocities, joint_accelerations
    ):
        """Return M(q)ddq + C(q,dq)dq + g(q) feed-forward torque."""
        return self._recursive_inverse_dynamics(
            joint_positions,
            joint_velocities,
            joint_accelerations,
            self.gravity,
        )
