"""ROS-free MuJoCo forward kinematics shared by arm and whole-body IK.

Cyclo Control keeps pose and ``LOCAL_WORLD_ALIGNED`` Jacobian evaluation behind one
kinematics interface.  This module provides the same useful boundary for this project using
only MuJoCo and NumPy: every pose quaternion is normalized, every rotational Jacobian is in
the world frame, and callers may evaluate either live state or an isolated scratch state.
"""

from dataclasses import dataclass
import itertools

import mujoco
import numpy as np


@dataclass(frozen=True)
class SiteKinematics:
    """World pose and world-aligned geometric Jacobian of one MuJoCo site."""

    position: np.ndarray
    quaternion: np.ndarray
    jacobian: np.ndarray


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


def evaluate_site(model, data, site_id, dof_ids=None):
    """Read a site's pose and geometric Jacobian from an already-forwarded state."""
    if site_id < 0 or site_id >= model.nsite:
        raise ValueError(f"invalid site id: {site_id}")
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    columns = slice(None) if dof_ids is None else np.asarray(dof_ids, dtype=int)
    quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(quaternion, data.site_xmat[site_id])
    return SiteKinematics(
        position=data.site_xpos[site_id].copy(),
        quaternion=normalize_quaternion(quaternion),
        jacobian=np.vstack((jacp[:, columns], jacr[:, columns])),
    )


def collision_distance_gradient(model, data, pair, dof_ids, max_distance):
    """Return a Cyclo-style signed distance gradient, or ``None`` when far away.

    MuJoCo supplies the closest points while ``mj_jac`` supplies their translational
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
        return _table_top_distance_gradient(model, data, pair, dof_ids, max_distance)
    if pair.mode == "bounding_sphere":
        return _bounding_sphere_distance_gradient(
            model, data, pair, dof_ids, max_distance)
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
    jacobian_a = np.zeros((3, model.nv))
    jacobian_b = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacobian_a, None, point_a, body_a)
    mujoco.mj_jac(model, data, jacobian_b, None, point_b, body_b)
    gradient = normal @ (jacobian_b[:, dof_ids] - jacobian_a[:, dof_ids])
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
    body_geom = {}
    for body_name in (
            *(f"arm_{side}_link{i}" for side in ("r", "l") for i in range(1, 8)),
            "hx5_r_base", "hx5_l_base", "base_link", "lift_link", "arm_base_link",
            "head_link1", "head_link2"):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            continue
        candidates = [
            geom_id for geom_id in range(model.ngeom)
            if int(model.geom_bodyid[geom_id]) == body_id
            and int(model.geom_contype[geom_id]) != 0
        ]
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


def _table_top_distance_gradient(model, data, pair, dof_ids, max_distance):
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

    jacobian_robot = np.zeros((3, model.nv))
    jacobian_table = np.zeros((3, model.nv))
    mujoco.mj_jac(
        model, data, jacobian_robot, None, point_robot,
        int(model.geom_bodyid[robot_geom]))
    mujoco.mj_jac(
        model, data, jacobian_table, None, point_table,
        int(model.geom_bodyid[table_geom]))
    gradient = table_normal @ (
        jacobian_robot[:, dof_ids] - jacobian_table[:, dof_ids])
    return CollisionConstraint(
        pair.name, distance, np.asarray(gradient, dtype=float),
        point_robot.copy(), point_table.copy())


def _bounding_sphere_distance_gradient(model, data, pair, dof_ids, max_distance):
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

    jacobian_a = np.zeros((3, model.nv))
    jacobian_b = np.zeros((3, model.nv))
    mujoco.mj_jac(
        model, data, jacobian_a, None, center_a, int(model.geom_bodyid[geom_a]))
    mujoco.mj_jac(
        model, data, jacobian_b, None, center_b, int(model.geom_bodyid[geom_b]))
    gradient = normal @ (jacobian_b[:, dof_ids] - jacobian_a[:, dof_ids])
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


class KinematicsSolver:
    """Scratch-state FK/Jacobian evaluator that never mutates live simulation data."""

    def __init__(self, model, site_name, joint_names):
        self.model = model
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        self.joint_ids = np.array([
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in joint_names
        ], dtype=int)
        if self.site_id < 0 or np.any(self.joint_ids < 0):
            raise ValueError("kinematics solver references a site or joint absent from the model")
        self.dof_ids = np.array([model.jnt_dofadr[jid] for jid in self.joint_ids], dtype=int)
        self.qpos_adrs = np.array([model.jnt_qposadr[jid] for jid in self.joint_ids], dtype=int)
        self.joint_ranges = np.array([model.jnt_range[jid] for jid in self.joint_ids], dtype=float)
        self.joint_limited = np.array(
            [bool(model.jnt_limited[jid]) for jid in self.joint_ids])
        self.data = mujoco.MjData(model)

    def update(self, q, context_qpos=None):
        """Update the private scratch state and return its site FK/Jacobian result."""
        q = np.asarray(q, dtype=float)
        if q.shape != self.qpos_adrs.shape:
            raise ValueError(f"expected {len(self.qpos_adrs)} joint positions, got {q.shape}")
        mujoco.mj_resetData(self.model, self.data)
        if context_qpos is not None:
            context = np.asarray(context_qpos, dtype=float)
            if context.shape != self.data.qpos.shape:
                raise ValueError(
                    f"expected context_qpos shape {self.data.qpos.shape}, got {context.shape}")
            self.data.qpos[:] = context
        bounded_q = q.copy()
        bounded_q[self.joint_limited] = np.clip(
            bounded_q[self.joint_limited],
            self.joint_ranges[self.joint_limited, 0],
            self.joint_ranges[self.joint_limited, 1])
        self.data.qpos[self.qpos_adrs] = bounded_q
        mujoco.mj_forward(self.model, self.data)
        return evaluate_site(self.model, self.data, self.site_id, self.dof_ids)

    def forward(self, q, context_qpos=None):
        """Compute a site pose and Jacobian without exposing the mutable scratch buffer."""
        return self.update(q, context_qpos)
