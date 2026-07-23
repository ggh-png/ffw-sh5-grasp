"""MJCF-derived kinematics shared by arm and whole-body IK.

MuJoCo still compiles the MJCF, resolving defaults and includes, but :class:`KinematicTree`
copies the resulting body/joint/site topology and fixed transforms.  FK, the world-aligned
geometric Jacobian, and iterative IK then run from that immutable tree with NumPy only;
they do not allocate ``MjData`` or call ``mujoco.mj_forward``.

Collision-distance helpers intentionally remain live-state MuJoCo queries.  They depend on
the physics engine's current contacts and geometry closest points, unlike tree kinematics.
"""

from dataclasses import dataclass
import itertools
from pathlib import Path

import mujoco
import numpy as np


@dataclass(frozen=True)
class SiteKinematics:
    """World pose and world-aligned geometric Jacobian of one MuJoCo site."""

    position: np.ndarray
    quaternion: np.ndarray
    jacobian: np.ndarray


@dataclass(frozen=True)
class KinematicJoint:
    """One joint copied from a compiled MJCF model."""

    id: int
    name: str
    body_id: int
    kind: int
    kind_name: str
    qpos_adr: int
    dof_adr: int
    position: np.ndarray
    axis: np.ndarray
    limited: bool
    range: np.ndarray


@dataclass(frozen=True)
class KinematicBody:
    """One body node and its fixed parent-to-body transform."""

    id: int
    name: str
    parent_id: int
    position: np.ndarray
    quaternion: np.ndarray
    joint_ids: tuple


@dataclass(frozen=True)
class KinematicSite:
    """A fixed site transform attached to a body node."""

    id: int
    name: str
    body_id: int
    position: np.ndarray
    quaternion: np.ndarray


@dataclass(frozen=True)
class CollisionPair:
    """One geometry pair monitored by the ROS-free collision barrier."""

    name: str
    geom_a: int
    geom_b: int
    mode: str = "geom"


@dataclass(frozen=True)
class CollisionConstraint:
    """Signed separation distance and its controlled-DOF gradient."""

    name: str
    distance: float
    gradient: np.ndarray
    point_a: np.ndarray
    point_b: np.ndarray


def normalize_quaternion(quaternion):
    """Return a finite unit MuJoCo quaternion (w, x, y, z) with canonical sign."""
    result = np.asarray(quaternion, dtype=float).copy()
    norm = float(np.linalg.norm(result))
    if not np.isfinite(norm) or norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    result /= norm
    if result[0] < 0.0:
        result *= -1.0
    return result


def shortest_orientation_error(target_quaternion, current_quaternion):
    """Shortest world-frame rotation vector taking ``current`` to ``target``.

    Forming ``target * inverse(current)`` makes the axis world-aligned, matching MuJoCo's
    rotational site Jacobian.  Canonicalizing the quaternion pair also makes q and -q
    exactly equivalent instead of allowing sign flips near 180 degrees.
    """
    target = normalize_quaternion(target_quaternion)
    current = normalize_quaternion(current_quaternion)
    if float(np.dot(target, current)) < 0.0:
        target *= -1.0
    current_inverse = current.copy()
    current_inverse[1:] *= -1.0
    error = np.zeros(4)
    mujoco.mju_mulQuat(error, target, current_inverse)
    error = normalize_quaternion(error)
    vector_norm = float(np.linalg.norm(error[1:]))
    if vector_norm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(vector_norm, max(error[0], 0.0))
    return error[1:] * (angle / vector_norm)


def _rotation_from_quaternion(quaternion):
    """Convert a ``(w, x, y, z)`` quaternion to a 3x3 rotation matrix."""
    w, x, y, z = normalize_quaternion(quaternion)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ])


def _quaternion_from_rotation(rotation):
    """Convert a 3x3 rotation matrix to a canonical ``(w, x, y, z)`` quaternion."""
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(max(trace + 1.0, 0.0))
        quaternion = np.array([
            0.25 * scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
        ])
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = 2.0 * np.sqrt(max(1.0 + matrix[0, 0] - matrix[1, 1]
                                      - matrix[2, 2], 0.0))
            quaternion = np.array([
                (matrix[2, 1] - matrix[1, 2]) / scale,
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
            ])
        elif index == 1:
            scale = 2.0 * np.sqrt(max(1.0 + matrix[1, 1] - matrix[0, 0]
                                      - matrix[2, 2], 0.0))
            quaternion = np.array([
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
            ])
        else:
            scale = 2.0 * np.sqrt(max(1.0 + matrix[2, 2] - matrix[0, 0]
                                      - matrix[1, 1], 0.0))
            quaternion = np.array([
                (matrix[1, 0] - matrix[0, 1]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
            ])
    return normalize_quaternion(quaternion)


def _axis_rotation(axis, angle):
    """Rodrigues rotation for a normalized joint axis."""
    axis = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        raise ValueError("kinematic joint axis must be non-zero")
    x, y, z = axis / norm
    sine, cosine = np.sin(angle), np.cos(angle)
    one_minus_cosine = 1.0 - cosine
    return np.array([
        [cosine + x * x * one_minus_cosine,
         x * y * one_minus_cosine - z * sine,
         x * z * one_minus_cosine + y * sine],
        [y * x * one_minus_cosine + z * sine,
         cosine + y * y * one_minus_cosine,
         y * z * one_minus_cosine - x * sine],
        [z * x * one_minus_cosine - y * sine,
         z * y * one_minus_cosine + x * sine,
         cosine + z * z * one_minus_cosine],
    ])


class KinematicTree:
    """Immutable body/joint/site tree copied from a compiled MJCF model.

    The MuJoCo compiler remains the source of truth for MJCF defaults, nested includes,
    angles, and asset-independent transforms.  Once copied, site FK and its geometric
    Jacobian are evaluated directly from this tree without a MuJoCo runtime state.
    """

    def __init__(self, model):
        self.nq = int(model.nq)
        self.qpos0 = np.asarray(model.qpos0, dtype=float).copy()
        self.bodies = tuple(self._copy_body(model, body_id)
                            for body_id in range(model.nbody))
        self.joints = tuple(self._copy_joint(model, joint_id)
                            for joint_id in range(model.njnt))
        self.sites = tuple(self._copy_site(model, site_id)
                           for site_id in range(model.nsite))
        self.body_by_name = {body.name: body for body in self.bodies if body.name}
        self.joint_by_name = {joint.name: joint for joint in self.joints if joint.name}
        self.site_by_name = {site.name: site for site in self.sites if site.name}
        children_by_body = [[] for _ in self.bodies]
        for body in self.bodies[1:]:
            children_by_body[body.parent_id].append(body.id)
        sites_by_body = [[] for _ in self.bodies]
        for site in self.sites:
            sites_by_body[site.body_id].append(site.id)
        self.children_by_body = tuple(tuple(ids) for ids in children_by_body)
        self.sites_by_body = tuple(tuple(ids) for ids in sites_by_body)
        self.site_paths = {
            site.id: self._body_path(site.body_id) for site in self.sites
        }

    @staticmethod
    def _name(model, object_type, object_id):
        return mujoco.mj_id2name(model, object_type, object_id) or ""

    @classmethod
    def _copy_body(cls, model, body_id):
        joint_address = int(model.body_jntadr[body_id])
        joint_count = int(model.body_jntnum[body_id])
        joint_ids = (() if joint_count == 0 else
                     tuple(range(joint_address, joint_address + joint_count)))
        return KinematicBody(
            id=body_id,
            name=cls._name(model, mujoco.mjtObj.mjOBJ_BODY, body_id),
            parent_id=int(model.body_parentid[body_id]),
            position=np.asarray(model.body_pos[body_id], dtype=float).copy(),
            quaternion=normalize_quaternion(model.body_quat[body_id]),
            joint_ids=joint_ids,
        )

    @classmethod
    def _copy_joint(cls, model, joint_id):
        kind = int(model.jnt_type[joint_id])
        kind_names = {
            int(mujoco.mjtJoint.mjJNT_FREE): "free",
            int(mujoco.mjtJoint.mjJNT_BALL): "ball",
            int(mujoco.mjtJoint.mjJNT_SLIDE): "slide",
            int(mujoco.mjtJoint.mjJNT_HINGE): "hinge",
        }
        return KinematicJoint(
            id=joint_id,
            name=cls._name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id),
            body_id=int(model.jnt_bodyid[joint_id]),
            kind=kind,
            kind_name=kind_names[kind],
            qpos_adr=int(model.jnt_qposadr[joint_id]),
            dof_adr=int(model.jnt_dofadr[joint_id]),
            position=np.asarray(model.jnt_pos[joint_id], dtype=float).copy(),
            axis=np.asarray(model.jnt_axis[joint_id], dtype=float).copy(),
            limited=bool(model.jnt_limited[joint_id]),
            range=np.asarray(model.jnt_range[joint_id], dtype=float).copy(),
        )

    @classmethod
    def _copy_site(cls, model, site_id):
        return KinematicSite(
            id=site_id,
            name=cls._name(model, mujoco.mjtObj.mjOBJ_SITE, site_id),
            body_id=int(model.site_bodyid[site_id]),
            position=np.asarray(model.site_pos[site_id], dtype=float).copy(),
            quaternion=normalize_quaternion(model.site_quat[site_id]),
        )

    def _body_path(self, body_id):
        path = []
        while body_id != 0:
            path.append(body_id)
            body_id = self.bodies[body_id].parent_id
        path.reverse()
        return tuple(path)

    def _forward_body(self, qpos, body_id):
        """Propagate one body path and return its world pose plus joint frames."""
        position = np.zeros(3)
        rotation = np.eye(3)
        joint_frames = {}
        hinge = int(mujoco.mjtJoint.mjJNT_HINGE)
        slide = int(mujoco.mjtJoint.mjJNT_SLIDE)

        for path_body_id in self._body_path(body_id):
            body = self.bodies[path_body_id]
            position = position + rotation @ body.position
            rotation = rotation @ _rotation_from_quaternion(body.quaternion)
            for joint_id in body.joint_ids:
                joint = self.joints[joint_id]
                axis_world = rotation @ joint.axis
                anchor_world = position + rotation @ joint.position
                joint_frames[joint_id] = (joint.kind, axis_world, anchor_world)
                displacement = qpos[joint.qpos_adr] - self.qpos0[joint.qpos_adr]
                if joint.kind == slide:
                    position = position + axis_world * displacement
                elif joint.kind == hinge:
                    rotation = rotation @ _axis_rotation(joint.axis, displacement)
                    position = anchor_world - rotation @ joint.position
                else:
                    raise NotImplementedError(
                        f"body path contains unsupported joint {joint.name!r}; "
                        "tree kinematics currently supports scalar hinge and slide joints")
        return position, rotation, joint_frames

    @staticmethod
    def _point_jacobian_from_frames(point_world, joint_ids, joint_frames):
        """Build a selected-column translational Jacobian for a world point."""
        point_world = np.asarray(point_world, dtype=float)
        jacobian = np.zeros((3, len(joint_ids)))
        hinge = int(mujoco.mjtJoint.mjJNT_HINGE)
        slide = int(mujoco.mjtJoint.mjJNT_SLIDE)
        for column, joint_id in enumerate(joint_ids):
            frame = joint_frames.get(int(joint_id))
            if frame is None:
                continue
            kind, axis_world, anchor_world = frame
            if kind == slide:
                jacobian[:, column] = axis_world
            elif kind == hinge:
                jacobian[:, column] = np.cross(
                    axis_world, point_world - anchor_world)
        return jacobian

    def point_jacobian(self, qpos, body_id, point_world, joint_ids):
        """Return the 3xN Jacobian of a body-fixed world point from this tree."""
        qpos = np.asarray(qpos, dtype=float)
        if qpos.shape != (self.nq,):
            raise ValueError(f"expected qpos shape ({self.nq},), got {qpos.shape}")
        if body_id < 0 or body_id >= len(self.bodies):
            raise ValueError(f"invalid body id: {body_id}")
        _, _, joint_frames = self._forward_body(qpos, body_id)
        return self._point_jacobian_from_frames(
            point_world, joint_ids, joint_frames)

    def forward_site(self, qpos, site_id, joint_ids):
        """Return world pose and selected-column geometric Jacobian for one site."""
        qpos = np.asarray(qpos, dtype=float)
        if qpos.shape != (self.nq,):
            raise ValueError(f"expected qpos shape ({self.nq},), got {qpos.shape}")
        if site_id < 0 or site_id >= len(self.sites):
            raise ValueError(f"invalid site id: {site_id}")

        site = self.sites[site_id]
        position, rotation, joint_frames = self._forward_body(qpos, site.body_id)
        site_position = position + rotation @ site.position
        site_rotation = rotation @ _rotation_from_quaternion(site.quaternion)
        jacobian = np.zeros((6, len(joint_ids)))
        jacobian[:3] = self._point_jacobian_from_frames(
            site_position, joint_ids, joint_frames)
        hinge = int(mujoco.mjtJoint.mjJNT_HINGE)
        for column, joint_id in enumerate(joint_ids):
            frame = joint_frames.get(int(joint_id))
            if frame is not None and frame[0] == hinge:
                jacobian[3:, column] = frame[1]
        return SiteKinematics(
            position=site_position,
            quaternion=_quaternion_from_rotation(site_rotation),
            jacobian=jacobian,
        )


def collision_distance_gradient(model, data, pair, tree, joint_ids, max_distance):
    """Return a Cyclo-style signed distance gradient, or ``None`` when far away.

    MuJoCo supplies geometry closest points; :class:`KinematicTree` supplies both point
    Jacobians.  For separated geometry the derivative is

    ``n.T @ (J_b - J_a)``, where ``n`` points from A to B.  MuJoCo reverses the closest
    segment inside penetration, so the sign is flipped there just as Cyclo's FCL/Pinocchio
    implementation does.

    A few convex mesh/box combinations in MuJoCo 3.10 can return exactly zero while also
    returning visibly separated closest points.  Treating the segment length as the
    distance in that inconsistent case prevents a false collision constraint at rest.
    """
    max_distance = max(float(max_distance), 0.0)
    if pair.mode == "table_top":
        return _table_top_distance_gradient(
            model, data, pair, tree, joint_ids, max_distance)
    if pair.mode == "bounding_sphere":
        return _bounding_sphere_distance_gradient(
            model, data, pair, tree, joint_ids, max_distance)
    fromto = np.zeros(6)
    raw_distance = float(mujoco.mj_geomDistance(
        model, data, pair.geom_a, pair.geom_b, max_distance, fromto))
    point_a, point_b = fromto[:3].copy(), fromto[3:].copy()
    segment = point_b - point_a
    segment_length = float(np.linalg.norm(segment))

    # Above distmax MuJoCo returns distmax and leaves fromto at zero.
    if raw_distance >= max_distance - 1e-12 and segment_length < 1e-12:
        return None
    distance = raw_distance
    if abs(raw_distance) < 1e-12 and segment_length > 1e-7:
        distance = segment_length
    if distance > max_distance:
        return None

    if segment_length > 1e-10:
        normal = segment / segment_length
    else:
        normal = _contact_normal(data, pair.geom_a, pair.geom_b)
        if normal is None:
            center_delta = data.geom_xpos[pair.geom_b] - data.geom_xpos[pair.geom_a]
            center_norm = float(np.linalg.norm(center_delta))
            normal = (center_delta / center_norm if center_norm > 1e-10
                      else np.array([1.0, 0.0, 0.0]))
        point_a = data.geom_xpos[pair.geom_a].copy()
        point_b = data.geom_xpos[pair.geom_b].copy()

    body_a = int(model.geom_bodyid[pair.geom_a])
    body_b = int(model.geom_bodyid[pair.geom_b])
    jacobian_a = tree.point_jacobian(data.qpos, body_a, point_a, joint_ids)
    jacobian_b = tree.point_jacobian(data.qpos, body_b, point_b, joint_ids)
    gradient = normal @ (jacobian_b - jacobian_a)
    if raw_distance < 0.0:
        gradient *= -1.0
    if not np.isfinite(distance) or not np.isfinite(gradient).all():
        return None
    return CollisionConstraint(
        pair.name, float(distance), np.asarray(gradient, dtype=float), point_a, point_b)


def default_collision_pairs(model):
    """Build the useful whole-body pairs without blocking wheels or grasp contacts.

    The model has detailed finger geometry, but finger joints are not WBIK variables and
    their intended contacts with objects must remain possible.  Monitoring one collision
    geometry per arm link/palm covers arm-arm, folded-arm/body, and arm/table collisions at
    predictable cost.  Floor and wheel pairs are deliberately excluded.
    """
    collision_geoms_by_body = {}
    for geom_id in range(model.ngeom):
        if int(model.geom_contype[geom_id]) == 0:
            continue
        body_id = int(model.geom_bodyid[geom_id])
        collision_geoms_by_body.setdefault(body_id, []).append(geom_id)

    body_geom = {}
    for body_name in (
            *(f"arm_{side}_link{i}" for side in ("r", "l") for i in range(1, 8)),
            "hx5_r_base", "hx5_l_base", "base_link", "lift_link", "arm_base_link",
            "head_link1", "head_link2"):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            continue
        candidates = collision_geoms_by_body.get(body_id, []).copy()
        if candidates:
            # Collision meshes/palm boxes use group 3.  The mobile base's collision box is
            # group 0, so the fallback retains it while visual meshes (contype=0) stay out.
            candidates.sort(key=lambda geom_id: int(model.geom_group[geom_id]) != 3)
            body_geom[body_name] = candidates[0]

    table_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table")
    pairs = {}

    def add(kind, body_a, body_b, geom_b=None, mode="geom"):
        geom_a = body_geom.get(body_a)
        other = body_geom.get(body_b) if geom_b is None else geom_b
        if geom_a is None or other is None or geom_a == other:
            return
        key = tuple(sorted((int(geom_a), int(other))))
        if key not in pairs:
            if mode == "table_top":
                pairs[key] = CollisionPair(
                    f"{kind}:{body_a}/{body_b}", int(geom_a), int(other), mode)
            else:
                pairs[key] = CollisionPair(f"{kind}:{body_a}/{body_b}", *key, mode)

    arms = {
        side: [f"arm_{side}_link{i}" for i in range(1, 8)] + [f"hx5_{side}_base"]
        for side in ("r", "l")
    }
    # Every cross-arm combination from link 2 onward; shoulder link 1 is fixed close to
    # its symmetric counterpart by construction and cannot usefully avoid it.
    for body_a, body_b in itertools.product(arms["r"][1:], arms["l"][1:]):
        mode = ("bounding_sphere" if body_a.startswith("hx5_")
                and body_b.startswith("hx5_") else "geom")
        add("cross-arm", body_a, body_b, mode=mode)

    # Same-arm pairs separated by at least two intervening bodies.  Adjacent and next-near
    # links legitimately sit within 3-6 cm in the home pose.
    for side in ("r", "l"):
        for first, second in itertools.combinations(range(len(arms[side])), 2):
            if second - first >= 3:
                add("folded-arm", arms[side][first], arms[side][second])

    central_bodies = ("base_link", "lift_link", "arm_base_link", "head_link1", "head_link2")
    for side in ("r", "l"):
        for arm_body, central_body in itertools.product(arms[side][2:], central_bodies):
            add("body", arm_body, central_body)

    if table_id >= 0:
        for side in ("r", "l"):
            for arm_body in arms[side][1:]:
                # The generic convex query is accurate for link meshes.  Only the oriented
                # palm-box/table-box combination exhibits MuJoCo's zero-distance jump.
                mode = "table_top" if arm_body.startswith("hx5_") else "geom"
                add("workspace", arm_body, "table", table_id, mode)
    return tuple(pairs.values())


def _table_top_distance_gradient(model, data, pair, tree, joint_ids, max_distance):
    """Stable support-point clearance above the finite table top.

    MuJoCo 3.10's generic convex distance occasionally jumps between zero and the correct
    positive value for an oriented palm box over a box table.  The support point of each
    monitored geom's local AABB gives a conservative, continuous top clearance instead.
    """
    robot_geom, table_geom = pair.geom_a, pair.geom_b
    robot_rotation = data.geom_xmat[robot_geom].reshape(3, 3)
    table_rotation = data.geom_xmat[table_geom].reshape(3, 3)
    robot_local_center = model.geom_aabb[robot_geom, :3]
    robot_half_size = model.geom_aabb[robot_geom, 3:]
    table_local_center = model.geom_aabb[table_geom, :3]
    table_half_size = model.geom_aabb[table_geom, 3:]
    robot_center = data.geom_xpos[robot_geom] + robot_rotation @ robot_local_center
    table_center = data.geom_xpos[table_geom] + table_rotation @ table_local_center
    table_normal = table_rotation[:, 2]

    # Only the finite tabletop footprint counts.  The physical contact model handles its
    # sides/legs; this analytical barrier must not behave like an infinite floor.
    robot_in_table = table_rotation.T @ (robot_center - table_center)
    relative_rotation = table_rotation.T @ robot_rotation
    robot_extent_table = np.abs(relative_rotation) @ robot_half_size
    for axis in (0, 1):
        gap = abs(robot_in_table[axis]) - (
            table_half_size[axis] + robot_extent_table[axis])
        if gap > max_distance:
            return None

    normal_local = robot_rotation.T @ table_normal
    support_local = robot_local_center - np.sign(normal_local) * robot_half_size
    point_robot = data.geom_xpos[robot_geom] + robot_rotation @ support_local
    point_table = point_robot - table_normal * (
        table_normal @ (point_robot - table_center) - table_half_size[2])
    distance = float(table_normal @ (point_robot - table_center) - table_half_size[2])
    if distance > max_distance:
        return None

    jacobian_robot = tree.point_jacobian(
        data.qpos, int(model.geom_bodyid[robot_geom]), point_robot, joint_ids)
    jacobian_table = tree.point_jacobian(
        data.qpos, int(model.geom_bodyid[table_geom]), point_table, joint_ids)
    gradient = table_normal @ (jacobian_robot - jacobian_table)
    return CollisionConstraint(
        pair.name, distance, np.asarray(gradient, dtype=float),
        point_robot.copy(), point_table.copy())


def _bounding_sphere_distance_gradient(model, data, pair, tree, joint_ids,
                                       max_distance):
    """Continuous conservative distance for the palm-box/palm-box pair."""
    geom_a, geom_b = pair.geom_a, pair.geom_b
    rotation_a = data.geom_xmat[geom_a].reshape(3, 3)
    rotation_b = data.geom_xmat[geom_b].reshape(3, 3)
    center_a = data.geom_xpos[geom_a] + rotation_a @ model.geom_aabb[geom_a, :3]
    center_b = data.geom_xpos[geom_b] + rotation_b @ model.geom_aabb[geom_b, :3]
    radius_a = float(np.linalg.norm(model.geom_aabb[geom_a, 3:]))
    radius_b = float(np.linalg.norm(model.geom_aabb[geom_b, 3:]))
    delta = center_b - center_a
    center_distance = float(np.linalg.norm(delta))
    if center_distance < 1e-10:
        normal = np.array([1.0, 0.0, 0.0])
    else:
        normal = delta / center_distance
    distance = center_distance - radius_a - radius_b
    if distance > max_distance:
        return None

    jacobian_a = tree.point_jacobian(
        data.qpos, int(model.geom_bodyid[geom_a]), center_a, joint_ids)
    jacobian_b = tree.point_jacobian(
        data.qpos, int(model.geom_bodyid[geom_b]), center_b, joint_ids)
    gradient = normal @ (jacobian_b - jacobian_a)
    return CollisionConstraint(
        pair.name, float(distance), np.asarray(gradient, dtype=float),
        center_a + radius_a * normal, center_b - radius_b * normal)


def _contact_normal(data, geom_a, geom_b):
    """Find a MuJoCo contact normal oriented from ``geom_a`` toward ``geom_b``."""
    for contact in data.contact:
        first, second = int(contact.geom1), int(contact.geom2)
        if first == geom_a and second == geom_b:
            return np.asarray(contact.frame[:3], dtype=float).copy()
        if first == geom_b and second == geom_a:
            return -np.asarray(contact.frame[:3], dtype=float).copy()
    return None


DEFAULT_DAMPING = 0.05
DEFAULT_MAX_JOINT_DELTA = 0.05
DEFAULT_MAX_ITER = 50
POSITION_TOLERANCE = 1e-4
ORIENTATION_TOLERANCE = 1e-3


class KinematicsSolver:
    """MJCF-tree FK, geometric Jacobian, and damped least-squares IK solver."""

    def __init__(self, model, site_name, joint_names, damping=DEFAULT_DAMPING,
                 max_joint_delta=DEFAULT_MAX_JOINT_DELTA, *, tree=None):
        self.model = model
        self.tree = KinematicTree(model) if tree is None else tree
        try:
            site = self.tree.site_by_name[site_name]
            joints = tuple(self.tree.joint_by_name[name] for name in joint_names)
        except KeyError as error:
            raise ValueError(
                f"kinematics solver references a site or joint absent from the model: "
                f"{error.args[0]!r}") from error
        unsupported = [joint.name for joint in joints if joint.kind not in (
            int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))]
        if unsupported:
            raise ValueError(
                "controlled joints must be scalar hinge/slide joints: "
                + ", ".join(unsupported))
        if len({joint.id for joint in joints}) != len(joints):
            raise ValueError("controlled joint names must be unique")

        self.site_name = site_name
        self.joint_names = tuple(joint_names)
        self.site_id = site.id
        self.joint_ids = np.array([joint.id for joint in joints], dtype=int)
        self.dof_ids = np.array([joint.dof_adr for joint in joints], dtype=int)
        self.qpos_adrs = np.array([joint.qpos_adr for joint in joints], dtype=int)
        self.joint_ranges = np.array([joint.range for joint in joints], dtype=float)
        self.joint_limited = np.array([joint.limited for joint in joints], dtype=bool)
        self.damping = float(damping)
        self.max_joint_delta = float(max_joint_delta)
        self.n = len(joints)
        if self.damping <= 0.0 or self.max_joint_delta <= 0.0:
            raise ValueError("damping and max_joint_delta must be positive")

    @classmethod
    def from_mjcf(cls, path, site_name, joint_names, **kwargs):
        """Compile an MJCF file, parse its kinematic tree, and construct a solver."""
        model = mujoco.MjModel.from_xml_path(str(Path(path)))
        return cls(model, site_name, joint_names, **kwargs)

    def _configuration(self, q, context_qpos=None):
        q = np.asarray(q, dtype=float)
        if q.shape != (self.n,):
            raise ValueError(f"expected {self.n} joint positions, got {q.shape}")
        if context_qpos is None:
            qpos = self.tree.qpos0.copy()
        else:
            context = np.asarray(context_qpos, dtype=float)
            if context.shape != (self.tree.nq,):
                raise ValueError(
                    f"expected context_qpos shape ({self.tree.nq},), got {context.shape}")
            qpos = context.copy()
        qpos[self.qpos_adrs] = self._clamp_to_limits(q)
        return qpos

    def forward(self, q, context_qpos=None):
        """Compute site FK and a 6xN world-aligned Jacobian from the parsed tree."""
        return self.tree.forward_site(
            self._configuration(q, context_qpos), self.site_id, self.joint_ids)

    def forward_kinematics(self, q, context_qpos=None):
        """Compatibility name used by the phase tests and older callers."""
        return self.forward(q, context_qpos)

    def _clamp_to_limits(self, q):
        result = np.asarray(q, dtype=float).copy()
        result[self.joint_limited] = np.clip(
            result[self.joint_limited],
            self.joint_ranges[self.joint_limited, 0],
            self.joint_ranges[self.joint_limited, 1])
        return result

    @staticmethod
    def _pose_error(state, target_position, target_quaternion):
        position_error = np.asarray(target_position, dtype=float) - state.position
        orientation_error = shortest_orientation_error(
            target_quaternion, state.quaternion)
        return position_error, orientation_error

    def solve_pose(self, q_init, target_pos, target_quat, max_iter=DEFAULT_MAX_ITER,
                   pos_tol=POSITION_TOLERANCE, ori_tol=ORIENTATION_TOLERANCE,
                   ori_weight=0.3, context_qpos=None):
        """Solve a pose with position-priority DLS and backtracking line search.

        Position is solved first.  Orientation correction is projected away from the
        position task, then every candidate step is evaluated through :meth:`forward`.
        Returns ``(q_solution, position_error_norm, orientation_error_norm_radians)``.
        """
        q = self._clamp_to_limits(np.asarray(q_init, dtype=float))
        if q.shape != (self.n,):
            raise ValueError(f"expected {self.n} initial joint positions, got {q.shape}")
        state = self.forward(q, context_qpos)
        position_error, orientation_error = self._pose_error(
            state, target_pos, target_quat)
        position_norm = float(np.linalg.norm(position_error))
        orientation_norm = float(np.linalg.norm(orientation_error))
        damping_squared = self.damping ** 2

        for _ in range(max(0, int(max_iter))):
            if position_norm < pos_tol and orientation_norm < ori_tol:
                break
            position_jacobian = state.jacobian[:3]
            rotation_jacobian = state.jacobian[3:]
            position_system = (
                position_jacobian @ position_jacobian.T
                + damping_squared * np.eye(3))

            position_delta = position_jacobian.T @ np.linalg.solve(
                position_system, position_error)
            orientation_gradient = rotation_jacobian.T @ orientation_error
            projected_gradient = np.linalg.solve(
                position_system, position_jacobian @ orientation_gradient)
            orientation_delta = (
                orientation_gradient - position_jacobian.T @ projected_gradient)
            full_delta = np.clip(
                position_delta + orientation_delta,
                -self.max_joint_delta, self.max_joint_delta)

            current_cost = position_norm + ori_weight * orientation_norm
            best = None
            step = 1.0
            for _ in range(6):
                candidate_q = self._clamp_to_limits(q + step * full_delta)
                candidate_state = self.forward(candidate_q, context_qpos)
                candidate_position_error, candidate_orientation_error = self._pose_error(
                    candidate_state, target_pos, target_quat)
                candidate_position_norm = float(
                    np.linalg.norm(candidate_position_error))
                candidate_orientation_norm = float(
                    np.linalg.norm(candidate_orientation_error))
                cost = (candidate_position_norm
                        + ori_weight * candidate_orientation_norm)
                if best is None or cost < best[0]:
                    best = (cost, candidate_q, candidate_state,
                            candidate_position_error, candidate_orientation_error,
                            candidate_position_norm, candidate_orientation_norm)
                if cost < current_cost:
                    break
                step *= 0.5

            (_, q, state, position_error, orientation_error,
             position_norm, orientation_norm) = best
        return q, position_norm, orientation_norm

    def solve_pose_multistart(self, q_init, target_pos, target_quat, rng, n_restarts=8,
                              max_iter=250, success_pos_tol=0.005,
                              success_ori_tol=np.radians(5), context_qpos=None):
        """Retry pose IK from random valid configurations to escape local minima."""
        initial = np.asarray(q_init, dtype=float)
        if initial.shape != (self.n,):
            raise ValueError(f"expected {self.n} initial joint positions, got {initial.shape}")
        lower = np.where(self.joint_limited, self.joint_ranges[:, 0], initial - np.pi)
        upper = np.where(self.joint_limited, self.joint_ranges[:, 1], initial + np.pi)
        candidates = [initial]
        candidates.extend(rng.uniform(lower, upper) for _ in range(max(0, int(n_restarts))))
        best = None
        for candidate in candidates:
            q, position_error, orientation_error = self.solve_pose(
                candidate, target_pos, target_quat, max_iter=max_iter,
                context_qpos=context_qpos)
            if (position_error < success_pos_tol
                    and orientation_error < success_ori_tol):
                return q, position_error, orientation_error, True
            if (best is None
                    or position_error + orientation_error < best[1] + best[2]):
                best = (q, position_error, orientation_error)
        return best[0], best[1], best[2], False
