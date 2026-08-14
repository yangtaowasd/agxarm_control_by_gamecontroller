"""Generic read-only URDF model using PoE and spatial-vector RNEA."""

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import modern_robotics as mr
import numpy as np
from armbycontroller.modeling.lie import adjoint
from armbycontroller.modeling.lie import force_cross
from armbycontroller.modeling.lie import joint_transform
from armbycontroller.modeling.lie import motion_cross
from armbycontroller.modeling.lie import rotation_exp
from armbycontroller.modeling.lie import spatial_inertia
from armbycontroller.modeling.lie import transform
from armbycontroller.modeling.lie import transform_inverse


def project_gravity_vector(orientation):
    """Return project-owned base-frame gravity for one orientation."""
    vectors = {
        "horizontal": (0.0, 0.0, -9.80665),
        "side": (-9.80665, 0.0, 0.0),
    }
    name = str(orientation).lower()
    if name not in vectors:
        raise ValueError(
            "compensation_orientation must be horizontal or side"
        )
    return vectors[name]


def _vector(element, attribute, default):
    if element is None or attribute not in element.attrib:
        return np.asarray(default, dtype=float)
    value = np.fromstring(element.attrib[attribute], sep=" ", dtype=float)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError(f"invalid URDF {attribute} vector")
    return value


def _rpy_rotation(rpy):
    roll, pitch, yaw = rpy
    return (
        rotation_exp(np.asarray([0.0, 0.0, 1.0]), yaw)
        @ rotation_exp(np.asarray([0.0, 1.0, 0.0]), pitch)
        @ rotation_exp(np.asarray([1.0, 0.0, 0.0]), roll)
    )


@dataclass(frozen=True)
class _Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    q_index: int = -1


class UrdfScrewModel:
    """PoE kinematics and tree RNEA with accessory joints fixed at zero."""

    def __init__(
        self,
        urdf_path,
        base_link="base_link",
        tip_link="link7",
        joint_count=None,
        gravity=(0.0, 0.0, -9.80665),
    ):
        self.urdf_path = Path(urdf_path).expanduser().resolve()
        if not self.urdf_path.is_file():
            raise ValueError(f"URDF/Xacro does not exist: {self.urdf_path}")
        self.base_link = str(base_link)
        self.tip_link = str(tip_link)
        self.gravity = np.asarray(gravity, dtype=float)
        if self.gravity.shape != (3,) or not np.all(np.isfinite(self.gravity)):
            raise ValueError("gravity must contain three finite values")
        root = self._expanded_robot(self.urdf_path)
        self._link_inertia, self._link_mass = self._parse_links(root)
        parsed_joints = self._parse_joints(root)
        controlled = self._controlled_chain(parsed_joints)
        self.joint_names = tuple(controlled)
        self.movable_joint_names = list(controlled)
        self.joint_count = len(controlled)
        if joint_count is not None and self.joint_count != int(joint_count):
            raise ValueError(
                f"URDF chain must have {int(joint_count)} movable joints, "
                f"got {self.joint_count}"
            )
        joints_by_name = {joint.name: joint for joint in parsed_joints}
        self.joint_limits = np.asarray(
            [
                (joints_by_name[name].lower, joints_by_name[name].upper)
                for name in controlled
            ],
            dtype=float,
        )
        controlled_index = {name: i for i, name in enumerate(controlled)}
        self._joints = tuple(
            _Joint(
                joint.name, joint.kind, joint.parent, joint.child,
                joint.origin, joint.axis, joint.lower, joint.upper,
                controlled_index.get(joint.name, -1),
            )
            for joint in parsed_joints
        )
        self._build_topology()
        self.modeled_mass = sum(
            self._link_mass.get(name, 0.0) for name in self._links[1:]
        )
        self._build_home_screws()
        self._build_modern_robotics_dynamics()

    def _resolve_include(self, including_path, filename):
        marker = "/agx_arm_urdf/"
        if marker in filename:
            relative = filename.split(marker, 1)[1]
            root = next(
                (parent for parent in including_path.parents
                 if parent.name == "agx_arm_urdf"),
                None,
            )
            if root is None:
                raise ValueError("cannot resolve agx_arm_urdf include")
            return (root / relative).resolve()
        path = Path(filename)
        return (
            path if path.is_absolute() else including_path.parent / path
        ).resolve()

    def _expanded_robot(self, path, stack=()):
        path = Path(path).resolve()
        if path in stack:
            raise ValueError(f"recursive Xacro include: {path}")
        root = ET.parse(path).getroot()
        for element in list(root):
            if not element.tag.endswith("}include"):
                continue
            included = self._expanded_robot(
                self._resolve_include(path, element.attrib["filename"]),
                stack + (path,),
            )
            position = list(root).index(element)
            root.remove(element)
            for child in list(included):
                root.insert(position, child)
                position += 1
        return root

    def _parse_links(self, root):
        links = {}
        masses = {}
        for link in root.findall("link"):
            name = link.attrib["name"]
            inertial = link.find("inertial")
            if inertial is None:
                links[name] = np.zeros((6, 6))
                masses[name] = 0.0
                continue
            origin = inertial.find("origin")
            center = _vector(origin, "xyz", (0.0, 0.0, 0.0))
            rotation = _rpy_rotation(
                _vector(origin, "rpy", (0.0, 0.0, 0.0))
            )
            mass = float(inertial.find("mass").attrib["value"])
            masses[name] = mass
            inertia = inertial.find("inertia")
            matrix = np.asarray([
                [float(inertia.attrib["ixx"]), float(inertia.attrib["ixy"]),
                 float(inertia.attrib["ixz"])],
                [float(inertia.attrib["ixy"]), float(inertia.attrib["iyy"]),
                 float(inertia.attrib["iyz"])],
                [float(inertia.attrib["ixz"]), float(inertia.attrib["iyz"]),
                 float(inertia.attrib["izz"])],
            ])
            links[name] = spatial_inertia(
                mass, center, rotation @ matrix @ rotation.T
            )
        return links, masses

    def _parse_joints(self, root):
        joints = []
        for element in root.findall("joint"):
            kind = element.attrib["type"]
            if kind not in ("fixed", "revolute", "continuous", "prismatic"):
                raise ValueError(f"unsupported joint type: {kind}")
            origin = element.find("origin")
            axis = _vector(element.find("axis"), "xyz", (1.0, 0.0, 0.0))
            norm = np.linalg.norm(axis)
            if kind != "fixed" and norm < 1e-12:
                raise ValueError(f"zero joint axis: {element.attrib['name']}")
            if norm > 1e-12:
                axis = axis / norm
            limit = element.find("limit")
            if kind == "fixed":
                lower, upper = 0.0, 0.0
            elif kind == "continuous":
                lower, upper = -math.pi, math.pi
            elif limit is None:
                # Some dynamics-only URDFs omit limits. Keep those usable
                # while giving the IK solver a conservative finite interval.
                lower, upper = -math.pi, math.pi
            else:
                lower = float(limit.attrib["lower"])
                upper = float(limit.attrib["upper"])
                if (
                    not math.isfinite(lower)
                    or not math.isfinite(upper)
                    or lower >= upper
                ):
                    raise ValueError(
                        f"joint {element.attrib['name']} has invalid limits"
                    )
            joints.append(_Joint(
                element.attrib["name"],
                kind,
                element.find("parent").attrib["link"],
                element.find("child").attrib["link"],
                transform(
                    _rpy_rotation(_vector(origin, "rpy", (0.0, 0.0, 0.0))),
                    _vector(origin, "xyz", (0.0, 0.0, 0.0)),
                ),
                axis,
                lower,
                upper,
            ))
        return joints

    def _controlled_chain(self, joints):
        by_child = {joint.child: joint for joint in joints}
        names = []
        child = self.tip_link
        while child != self.base_link:
            joint = by_child.get(child)
            if joint is None:
                raise ValueError("no URDF chain from base_link to link7")
            if joint.kind != "fixed":
                names.append(joint.name)
            child = joint.parent
        names.reverse()
        return names

    def _build_topology(self):
        children = {}
        for joint in self._joints:
            children.setdefault(joint.parent, []).append(joint)
        self._links = [self.base_link]
        self._edges = []

        def visit(parent):
            parent_index = self._links.index(parent)
            for joint in children.get(parent, ()):
                child_index = len(self._links)
                self._links.append(joint.child)
                self._edges.append((parent_index, child_index, joint))
                visit(joint.child)

        visit(self.base_link)
        self._spatial_inertias = tuple(
            self._link_inertia.get(name, np.zeros((6, 6)))
            for name in self._links
        )
        if self.tip_link not in self._links:
            raise ValueError("tip link is not reachable from base")

    def _build_home_screws(self):
        transforms = {self.base_link: np.eye(4)}
        screws = [None] * self.joint_count
        for parent_index, child_index, joint in self._edges:
            parent = self._links[parent_index]
            child = self._links[child_index]
            joint_frame = transforms[parent] @ joint.origin
            transforms[child] = joint_frame
            if joint.q_index < 0:
                continue
            axis = joint_frame[:3, :3] @ joint.axis
            if joint.kind == "prismatic":
                screw = np.concatenate((np.zeros(3), axis))
            else:
                point = joint_frame[:3, 3]
                screw = np.concatenate((axis, -np.cross(axis, point)))
            screws[joint.q_index] = screw
        self.space_screws = np.stack(screws, axis=1)
        self.home_tip_transform = transforms[self.tip_link]
        self._home_transforms = transforms

    def _build_modern_robotics_dynamics(self):
        """Build MR chain data, folding fixed branches into each body."""
        body_frames = [None] * self.joint_count
        link_owner = {self.base_link: -1}
        for parent_index, child_index, joint in self._edges:
            parent = self._links[parent_index]
            child = self._links[child_index]
            owner = link_owner[parent]
            if joint.q_index >= 0:
                owner = joint.q_index
                body_frames[owner] = child
            link_owner[child] = owner

        if any(frame is None for frame in body_frames):
            raise ValueError("could not construct the serial MR chain")

        inertias = [np.zeros((6, 6)) for _ in range(self.joint_count)]
        for link, inertia in zip(self._links, self._spatial_inertias):
            owner = link_owner[link]
            if owner < 0:
                continue
            body_to_link = (
                transform_inverse(self._home_transforms[body_frames[owner]])
                @ self._home_transforms[link]
            )
            link_motion_from_body = adjoint(
                transform_inverse(body_to_link)
            )
            inertias[owner] += (
                link_motion_from_body.T
                @ inertia
                @ link_motion_from_body
            )

        frames = [
            self._home_transforms[frame] for frame in body_frames
        ]
        relative = [frames[0]]
        relative.extend(
            transform_inverse(frames[index - 1]) @ frames[index]
            for index in range(1, self.joint_count)
        )
        relative.append(
            transform_inverse(frames[-1]) @ self.home_tip_transform
        )
        self.mr_mlist = np.asarray(relative)
        self.mr_glist = np.asarray(inertias)

    def _state(self, values, name):
        values = np.asarray(values, dtype=float)
        if (
            values.shape != (self.joint_count,)
            or not np.all(np.isfinite(values))
        ):
            raise ValueError(
                f"{name} must contain {self.joint_count} finite values"
            )
        return values

    def forward_kinematics(self, joint_positions):
        """Compute the base-to-tip pose with modern_robotics space PoE."""
        positions = self._state(joint_positions, "joint_positions")
        return mr.FKinSpace(
            self.home_tip_transform,
            self.space_screws,
            positions,
        )

    def urdf_forward_kinematics(self, joint_positions):
        """Compute the base-to-tip pose by direct URDF-tree propagation."""
        positions = self._state(joint_positions, "joint_positions")
        transforms = [np.eye(4) for _ in self._links]
        for parent, child, joint in self._edges:
            coordinate = (
                positions[joint.q_index] if joint.q_index >= 0 else 0.0
            )
            transforms[child] = (
                transforms[parent]
                @ joint.origin
                @ joint_transform(joint.kind, joint.axis, coordinate)
            )
        return transforms[self._links.index(self.tip_link)]

    def space_jacobian(self, joint_positions):
        """Compute the 6xn Jacobian with modern_robotics."""
        positions = self._state(joint_positions, "joint_positions")
        return mr.JacobianSpace(self.space_screws, positions)

    def _rnea_backend(
        self, joint_positions, joint_velocities, joint_accelerations,
        gravity, xp,
    ):
        positions = xp.asarray(joint_positions)
        velocities = xp.asarray(joint_velocities)
        accelerations = xp.asarray(joint_accelerations)
        gravity = xp.asarray(gravity)
        node_count = len(self._links)
        twists = [
            xp.zeros(6, dtype=positions.dtype) for _ in range(node_count)
        ]
        spatial_acceleration = [
            xp.zeros(6, dtype=positions.dtype) for _ in range(node_count)
        ]
        spatial_acceleration[0] = xp.concatenate((
            xp.zeros(3, dtype=positions.dtype), -gravity
        ))
        x_up = [None] * len(self._edges)
        motion_subspaces = [None] * len(self._edges)
        for edge_index, (parent, child, joint) in enumerate(self._edges):
            coordinate = (
                positions[joint.q_index] if joint.q_index >= 0
                else xp.asarray(0.0, dtype=positions.dtype)
            )
            parent_to_child = (
                xp.asarray(joint.origin)
                @ joint_transform(joint.kind, joint.axis, coordinate, xp)
            )
            x_motion = adjoint(transform_inverse(parent_to_child, xp), xp)
            x_up[edge_index] = x_motion
            if joint.kind in ("revolute", "continuous"):
                subspace = xp.concatenate((
                    xp.asarray(joint.axis),
                    xp.zeros(3, dtype=positions.dtype),
                ))
            elif joint.kind == "prismatic":
                subspace = xp.concatenate((
                    xp.zeros(3, dtype=positions.dtype),
                    xp.asarray(joint.axis),
                ))
            else:
                subspace = xp.zeros(6, dtype=positions.dtype)
            motion_subspaces[edge_index] = subspace
            speed = (
                velocities[joint.q_index] if joint.q_index >= 0
                else xp.asarray(0.0, dtype=positions.dtype)
            )
            acceleration = (
                accelerations[joint.q_index] if joint.q_index >= 0
                else xp.asarray(0.0, dtype=positions.dtype)
            )
            joint_twist = subspace * speed
            twists[child] = x_motion @ twists[parent] + joint_twist
            spatial_acceleration[child] = (
                x_motion @ spatial_acceleration[parent]
                + subspace * acceleration
                + motion_cross(twists[child], xp) @ joint_twist
            )
        forces = []
        for index in range(node_count):
            inertia = xp.asarray(self._spatial_inertias[index])
            momentum = inertia @ twists[index]
            forces.append(
                inertia @ spatial_acceleration[index]
                + force_cross(twists[index], xp) @ momentum
            )
        torques = [
            xp.asarray(0.0, dtype=positions.dtype)
            for _ in range(self.joint_count)
        ]
        for edge_index in range(len(self._edges) - 1, -1, -1):
            parent, child, joint = self._edges[edge_index]
            if joint.q_index >= 0:
                torques[joint.q_index] = (
                    motion_subspaces[edge_index] @ forces[child]
                )
            forces[parent] = (
                forces[parent] + x_up[edge_index].T @ forces[child]
            )
        return xp.stack(torques)

    def inverse_dynamics(
        self, joint_positions, joint_velocities, joint_accelerations
    ):
        """Return M(q)ddq + C(q,dq)dq + g(q) using MR RNEA."""
        q = self._state(joint_positions, "joint_positions")
        dq = self._state(joint_velocities, "joint_velocities")
        ddq = self._state(joint_accelerations, "joint_accelerations")
        return mr.InverseDynamics(
            q,
            dq,
            ddq,
            self.gravity,
            np.zeros(6),
            self.mr_mlist,
            self.mr_glist,
            self.space_screws,
        )

    def gravity_torque(self, joint_positions):
        """Return g(q) using modern_robotics."""
        q = self._state(joint_positions, "joint_positions")
        return mr.GravityForces(
            q,
            self.gravity,
            self.mr_mlist,
            self.mr_glist,
            self.space_screws,
        )

    def compensation(self, joint_positions):
        """Return gravity compensation torque for existing controllers."""
        return self.gravity_torque(joint_positions)

    def coriolis_torque(self, joint_positions, joint_velocities):
        """Return C(q,dq)dq using modern_robotics."""
        q = self._state(joint_positions, "joint_positions")
        dq = self._state(joint_velocities, "joint_velocities")
        return mr.VelQuadraticForces(
            q,
            dq,
            self.mr_mlist,
            self.mr_glist,
            self.space_screws,
        )

    def coriolis(self, joint_positions, joint_velocities):
        """Return Coriolis torque for existing controllers."""
        return self.coriolis_torque(joint_positions, joint_velocities)

    def mass_matrix(self, joint_positions):
        """Return M(q) using modern_robotics."""
        q = self._state(joint_positions, "joint_positions")
        return mr.MassMatrix(
            q,
            self.mr_mlist,
            self.mr_glist,
            self.space_screws,
        )

    def _spatial_momentum_state(self, joint_positions, joint_velocities):
        """Return link twists, transforms, subspaces, and momenta in O(n)."""
        positions = self._state(joint_positions, "joint_positions")
        velocities = self._state(joint_velocities, "joint_velocities")
        node_count = len(self._links)
        twists = [np.zeros(6) for _ in range(node_count)]
        x_up = [None] * len(self._edges)
        subspaces = [None] * len(self._edges)
        for edge_index, (parent, child, joint) in enumerate(self._edges):
            coordinate = (
                positions[joint.q_index] if joint.q_index >= 0 else 0.0
            )
            parent_to_child = (
                joint.origin
                @ joint_transform(joint.kind, joint.axis, coordinate)
            )
            x_motion = adjoint(transform_inverse(parent_to_child))
            if joint.kind in ("revolute", "continuous"):
                subspace = np.concatenate((joint.axis, np.zeros(3)))
            elif joint.kind == "prismatic":
                subspace = np.concatenate((np.zeros(3), joint.axis))
            else:
                subspace = np.zeros(6)
            speed = (
                velocities[joint.q_index] if joint.q_index >= 0 else 0.0
            )
            twists[child] = (
                x_motion @ twists[parent] + subspace * speed
            )
            x_up[edge_index] = x_motion
            subspaces[edge_index] = subspace
        momenta = [
            self._spatial_inertias[index] @ twists[index]
            for index in range(node_count)
        ]
        return twists, momenta, x_up, subspaces

    def generalized_momentum(self, joint_positions, joint_velocities):
        """Return ``p=M(q)qdot`` by a spatial O(n) backward recursion."""
        _, momenta, x_up, subspaces = self._spatial_momentum_state(
            joint_positions, joint_velocities
        )
        result = np.zeros(self.joint_count)
        for edge_index in range(len(self._edges) - 1, -1, -1):
            parent, child, joint = self._edges[edge_index]
            if joint.q_index >= 0:
                result[joint.q_index] = (
                    subspaces[edge_index] @ momenta[child]
                )
            momenta[parent] = (
                momenta[parent] + x_up[edge_index].T @ momenta[child]
            )
        return result

    def kinetic_energy(self, joint_positions, joint_velocities):
        """Return rigid-link kinetic energy from spatial inertia and twist."""
        twists, momenta, _, _ = self._spatial_momentum_state(
            joint_positions, joint_velocities
        )
        return 0.5 * sum(
            float(twist @ momentum)
            for twist, momentum in zip(twists, momenta)
        )

    def kinetic_energy_gradient(
        self, joint_positions, joint_velocities, step=1e-6
    ):
        """Return ``dT/dq=C(q,qdot).T qdot`` by central differences."""
        positions = self._state(joint_positions, "joint_positions")
        velocities = self._state(joint_velocities, "joint_velocities")
        step = float(step)
        if not math.isfinite(step) or step <= 0.0:
            raise ValueError("kinetic energy gradient step must be positive")
        gradient = np.zeros(self.joint_count)
        for index in range(self.joint_count):
            offset = np.zeros(self.joint_count)
            offset[index] = step
            gradient[index] = (
                self.kinetic_energy(positions + offset, velocities)
                - self.kinetic_energy(positions - offset, velocities)
            ) / (2.0 * step)
        return gradient

    def momentum_observer_terms(self, joint_positions, joint_velocities):
        """Return generalized momentum and ``beta=g-dT/dq``."""
        positions = self._state(joint_positions, "joint_positions")
        velocities = self._state(joint_velocities, "joint_velocities")
        momentum = self.generalized_momentum(positions, velocities)
        speed = float(np.linalg.norm(velocities))
        if speed < 1e-12:
            momentum_derivative = np.zeros(self.joint_count)
        else:
            step = 1e-6 / max(1.0, speed)
            momentum_derivative = (
                self.generalized_momentum(
                    positions + step * velocities, velocities
                )
                - self.generalized_momentum(
                    positions - step * velocities, velocities
                )
            ) / (2.0 * step)
        # Mdot*qdot is the directional derivative of p=M(q)qdot along qdot.
        # Since Mdot=C+C.T, beta=g-C.T*qdot=g+C*qdot-Mdot*qdot.
        beta = (
            self.inverse_dynamics(
                positions, velocities, np.zeros(self.joint_count)
            )
            - momentum_derivative
        )
        return momentum, beta
