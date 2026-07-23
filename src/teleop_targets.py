"""Target-pose and Cyclo-style bimanual helpers for teleop_app.

This module owns the math that connects three views of the same command:

- UI values: home-relative XYZ/RPY targets in ``app.targets``.
- Render values: world-space marker/gizmo poses.
- IK values: world-space site goals passed into ``whole_body_ik.WholeBodyIK``.

It deliberately duck-types on ``app`` instead of importing ``teleop_app`` so the main app
can remain the only composition point.
"""

import math

import mujoco
import numpy as np

SIDES = ("r", "l")


def _multiply_quaternions(*quaternions):
    """Multiply MuJoCo-format quaternions from left to right."""
    result = np.array([1.0, 0.0, 0.0, 0.0])
    for quaternion in quaternions:
        product = np.zeros(4)
        mujoco.mju_mulQuat(product, result, quaternion)
        result = product
    return result


def _inverse_quaternion(quaternion):
    result = np.zeros(4)
    mujoco.mju_negQuat(result, quaternion)
    return result


def rpy_deg_to_quat(rpy_deg):
    r, p, y = (math.radians(v) for v in rpy_deg)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_rpy_deg(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def set_home_references(app):
    site_states = {
        side: app.whole_body_solver.site_state(app.data, side)
        for side in SIDES
    }
    for side, state in site_states.items():
        setattr(app, f"home_quat_{side}", state.quaternion.copy())
    app.home_pos_local = {
        side: world_to_base_pos(app, state.position)
        for side, state in site_states.items()
    }
    base_x, base_y, base_yaw, _cy, _sy, base_quat = base_pose(app)
    app.target_anchor_xy = np.array([base_x, base_y], dtype=float)
    app.target_anchor_yaw = float(base_yaw)
    app.target_anchor_quat = base_quat.copy()
    app.home_pos_world = {
        side: state.position.copy() for side, state in site_states.items()
    }
    app.home_quat_world = {
        "r": app.home_quat_r.copy(),
        "l": app.home_quat_l.copy(),
    }


def carry_world_targets_with_base(app, previous_base_pose, current_base_pose):
    """Apply measured manual-base SE(2) motion to every world target reference.

    Whole-body targets normally stay fixed in world space so IK-driven base motion reduces
    their error.  Manual driving has different semantics: it repositions the whole robot,
    so its targets must ride with the chassis.  Transforming the references (rather than
    changing UI offsets) also preserves independent and captured-bimanual commands.
    """
    old_x, old_y, old_yaw = (float(v) for v in previous_base_pose)
    new_x, new_y, new_yaw = (float(v) for v in current_base_pose)
    delta_yaw = (new_yaw - old_yaw + math.pi) % (2.0 * math.pi) - math.pi
    c, s = math.cos(delta_yaw), math.sin(delta_yaw)
    old_xy = np.array([old_x, old_y])
    new_xy = np.array([new_x, new_y])

    def transform_position(position):
        result = np.asarray(position, dtype=float).copy()
        relative = result[:2] - old_xy
        result[:2] = new_xy + np.array([
            c * relative[0] - s * relative[1],
            s * relative[0] + c * relative[1],
        ])
        return result

    delta_quat = np.array([math.cos(delta_yaw / 2.0), 0.0, 0.0,
                           math.sin(delta_yaw / 2.0)])
    app.target_anchor_xy = transform_position(app.target_anchor_xy)[:2]
    app.target_anchor_yaw += delta_yaw
    anchor_quat = np.zeros(4)
    mujoco.mju_mulQuat(anchor_quat, delta_quat, app.target_anchor_quat)
    app.target_anchor_quat = anchor_quat
    for side in SIDES:
        app.home_pos_world[side] = transform_position(app.home_pos_world[side])
        app.home_quat_world[side] = _multiply_quaternions(
            delta_quat, app.home_quat_world[side]
        )


def base_pose(app):
    base_x = app.data.qpos[app.base_x_qadr]
    base_y = app.data.qpos[app.base_y_qadr]
    base_yaw = app.data.qpos[app.base_yaw_qadr]
    cy, sy = math.cos(base_yaw), math.sin(base_yaw)
    base_quat = np.array([math.cos(base_yaw / 2), 0.0, 0.0, math.sin(base_yaw / 2)])
    return base_x, base_y, base_yaw, cy, sy, base_quat


def local_to_world_pos(app, p_local):
    base_x, base_y, _base_yaw, cy, sy, _base_quat = base_pose(app)
    x, y, z = p_local
    return np.array([base_x + cy * x - sy * y, base_y + sy * x + cy * y, z])


def world_to_base_pos(app, p_world):
    base_x, base_y, _base_yaw, cy, sy, _base_quat = base_pose(app)
    dx, dy = p_world[0] - base_x, p_world[1] - base_y
    return np.array([cy * dx + sy * dy, -sy * dx + cy * dy, p_world[2]])


def anchor_local_to_world_pos(app, p_local):
    """Transform through the startup base pose, not the moving live base pose."""
    cy, sy = math.cos(app.target_anchor_yaw), math.sin(app.target_anchor_yaw)
    x, y, z = p_local
    return np.array([
        app.target_anchor_xy[0] + cy * x - sy * y,
        app.target_anchor_xy[1] + sy * x + cy * y,
        z,
    ])


def world_to_anchor_local_pos(app, p_world):
    cy, sy = math.cos(app.target_anchor_yaw), math.sin(app.target_anchor_yaw)
    dx = p_world[0] - app.target_anchor_xy[0]
    dy = p_world[1] - app.target_anchor_xy[1]
    return np.array([cy * dx + sy * dy, -sy * dx + cy * dy, p_world[2]])


def target_pos_to_base_pos(app, side, pos_target):
    return app.home_pos_local[side] + np.array(pos_target)


def target_pos_to_world_pos(app, side, pos_target):
    if getattr(app, "whole_body_enabled", False):
        offset = np.asarray(pos_target, dtype=float)
        cy, sy = math.cos(app.target_anchor_yaw), math.sin(app.target_anchor_yaw)
        rotated = np.array([
            cy * offset[0] - sy * offset[1],
            sy * offset[0] + cy * offset[1],
            offset[2],
        ])
        return app.home_pos_world[side] + rotated
    return local_to_world_pos(app, target_pos_to_base_pos(app, side, pos_target))


def world_to_target_pos(app, side, world_pos):
    if getattr(app, "whole_body_enabled", False):
        delta = np.asarray(world_pos, dtype=float) - app.home_pos_world[side]
        cy, sy = math.cos(app.target_anchor_yaw), math.sin(app.target_anchor_yaw)
        return [cy * delta[0] + sy * delta[1],
                -sy * delta[0] + cy * delta[1], float(delta[2])]
    return (world_to_base_pos(app, world_pos) - app.home_pos_local[side]).tolist()


def target_rpy_to_world_quat(app, side, rpy_deg):
    """Convert a hand's home-relative RPY value using the active target frame."""
    delta_quat = rpy_deg_to_quat(rpy_deg)
    if getattr(app, "whole_body_enabled", False):
        return _multiply_quaternions(app.home_quat_world[side], delta_quat)
    *_unused, base_quat = base_pose(app)
    home_quat = getattr(app, f"home_quat_{side}")
    return _multiply_quaternions(base_quat, home_quat, delta_quat)


def target_world_quat(app, side):
    return target_rpy_to_world_quat(app, side, app.targets[f"rpy_{side}"])


def world_quat_to_target_rpy(app, side, world_quat):
    if getattr(app, "whole_body_enabled", False):
        delta = _multiply_quaternions(
            _inverse_quaternion(app.home_quat_world[side]), world_quat
        )
        return quat_to_rpy_deg(delta)
    home_quat = getattr(app, f"home_quat_{side}")
    *_unused, base_quat = base_pose(app)
    rpy_delta_quat = _multiply_quaternions(
        _inverse_quaternion(home_quat),
        _inverse_quaternion(base_quat),
        world_quat,
    )
    return quat_to_rpy_deg(rpy_delta_quat)


def world_quat_to_virtual_rpy(app, world_quat):
    if getattr(app, "whole_body_enabled", False):
        base_quat = app.target_anchor_quat
    else:
        *_unused, base_quat = base_pose(app)
    rpy_delta_quat = _multiply_quaternions(
        _inverse_quaternion(base_quat), world_quat
    )
    return quat_to_rpy_deg(rpy_delta_quat)


def quat_to_mat(quat):
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, quat)
    return mat.reshape(3, 3)


def mat_to_quat(mat):
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.asarray(mat).reshape(9))
    return quat


def target_world_pose(app, side):
    return target_pos_to_world_pos(app, side, app.targets[f"pos_{side}"]), target_world_quat(app, side)


def virtual_object_world_pose(app):
    if getattr(app, "whole_body_enabled", False):
        pos = anchor_local_to_world_pos(app, app.targets["virtual_object_pos"])
        base_quat = app.target_anchor_quat
    else:
        pos = local_to_world_pos(app, app.targets["virtual_object_pos"])
        *_unused, base_quat = base_pose(app)
    quat = _multiply_quaternions(
        base_quat, rpy_deg_to_quat(app.targets["virtual_object_rpy"])
    )
    return pos, quat


def sync_virtual_object_to_hand_targets(app):
    pos_r, _quat_r = target_world_pose(app, "r")
    pos_l, _quat_l = target_world_pose(app, "l")
    midpoint = 0.5 * (pos_r + pos_l)
    if getattr(app, "whole_body_enabled", False):
        app.targets["virtual_object_pos"] = world_to_anchor_local_pos(app, midpoint).tolist()
    else:
        app.targets["virtual_object_pos"] = world_to_base_pos(app, midpoint).tolist()
    app.targets["virtual_object_rpy"] = [0.0, 0.0, 0.0]


def capture_grasp(app):
    """Record both hand target poses relative to the virtual object marker."""
    sync_virtual_object_to_hand_targets(app)
    obj_pos, obj_quat = virtual_object_world_pose(app)
    obj_R = quat_to_mat(obj_quat)
    offsets = {}
    for side in SIDES:
        hand_pos, hand_quat = target_world_pose(app, side)
        offsets[side] = {
            "pos": obj_R.T @ (hand_pos - obj_pos),
            "mat": obj_R.T @ quat_to_mat(hand_quat),
        }
    app.cyclo_capture_offsets = offsets
    app.cyclo_grasp_captured = True
    app.cyclo_controller = "bimanual_movel"
    app.cyclo_status = "captured virtual object"


def release_grasp(app):
    app.cyclo_grasp_captured = False
    app.cyclo_capture_offsets = None
    app.cyclo_status = "released"


def apply_virtual_object_target(app):
    if not app.cyclo_grasp_captured or app.cyclo_capture_offsets is None:
        return
    obj_pos, obj_quat = virtual_object_world_pose(app)
    obj_R = quat_to_mat(obj_quat)
    for side, offset in app.cyclo_capture_offsets.items():
        hand_pos = obj_pos + obj_R @ offset["pos"]
        hand_quat = mat_to_quat(obj_R @ offset["mat"])
        app.targets[f"pos_{side}"] = world_to_target_pos(app, side, hand_pos)
        app.targets[f"rpy_{side}"] = world_quat_to_target_rpy(app, side, hand_quat)


def bimanual_marker_visible(app):
    return (getattr(app, "cyclo_controller", "movel") == "bimanual_movel"
            and bool(getattr(app, "cyclo_grasp_captured", False)))


def sync_marker_visibility(app):
    if not hasattr(app, "virtual_object_marker_geom_id"):
        return
    alpha_scale = 1.0 if bimanual_marker_visible(app) else 0.0
    geom_rgba = app.virtual_object_marker_rgba["geom"].copy()
    site_rgba = app.virtual_object_marker_rgba["site"].copy()
    geom_rgba[3] *= alpha_scale
    site_rgba[3] *= alpha_scale
    app.model.geom_rgba[app.virtual_object_marker_geom_id] = geom_rgba
    app.model.site_rgba[app.virtual_object_marker_site_id] = site_rgba


def active_gizmo_target(app):
    if app.cyclo_controller == "bimanual_movel" and app.cyclo_grasp_captured:
        return "virtual"
    side = getattr(app, "jog_side", "r")
    return side if side in ("l", "r") else "r"


def gizmo_target_world_pose(app, target):
    if target == "virtual":
        return virtual_object_world_pose(app)
    return target_world_pose(app, target)


def set_gizmo_target_world_pose(app, target, world_pos, world_quat):
    if target == "virtual":
        if getattr(app, "whole_body_enabled", False):
            app.targets["virtual_object_pos"] = world_to_anchor_local_pos(app, world_pos).tolist()
        else:
            app.targets["virtual_object_pos"] = world_to_base_pos(app, world_pos).tolist()
        app.targets["virtual_object_rpy"] = world_quat_to_virtual_rpy(app, world_quat)
        apply_virtual_object_target(app)
    else:
        app.targets[f"pos_{target}"] = world_to_target_pos(app, target, world_pos)
        app.targets[f"rpy_{target}"] = world_quat_to_target_rpy(app, target, world_quat)


def sync_ik_mocaps_from_targets(app):
    if not hasattr(app, "ik_target_mocap_ids"):
        return
    for side, mocap_id in app.ik_target_mocap_ids.items():
        pos, quat = target_world_pose(app, side)
        app.data.mocap_pos[mocap_id] = pos
        app.data.mocap_quat[mocap_id] = quat
    if hasattr(app, "virtual_object_mocap_id"):
        pos, quat = virtual_object_world_pose(app)
        app.data.mocap_pos[app.virtual_object_mocap_id] = pos
        app.data.mocap_quat[app.virtual_object_mocap_id] = quat
    sync_marker_visibility(app)
