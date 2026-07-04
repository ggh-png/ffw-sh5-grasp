"""Phase 2 -- grasp synergy mapping + contact-based grasp verification.

Two independent scalars, matching the reference video's "Right grasp / Right thumb"
sliders:
  - `grasp` (0..1): index + middle finger curl (pip/dip/tip).
  - `thumb` (0..1): thumb curl (mcp_pitch, ip). Thumb CMC/MCP-yaw (abduction) are a fixed
    pre-shape, not part of either scalar -- found by FK sweep in Phase 2 (see NOTES.md) so
    the thumb opposes the index/middle convergence zone around the can.

Ring and pinky are locked at range=0 in models/hand_only.xml (3-point grasp fallback from
PLAN.md's own guidance), so this module only ever drives 3 digits.

Both scalars map to a *sub-range* of each joint's travel, not [lo, hi]: starting fully
extended left ~10cm of empty travel before any contact, which meant the freely-falling can
(hand_only has no table) dropped out of reach before the gentle force-limited actuators
could close far enough to catch it. OPEN_FRAC is tuned (empirically, via fingertip-to-can
gap) so `grasp=0`/`thumb=0` sits ~20mm short of the can surface -- see models/hand_only.xml's
"pregrasp" keyframe, which encodes this exact pose.

Widened from an initial ~2mm margin once Phase 3 started chaining IK + a real (imperfect)
arm servo in front of it: the arm settles with a real but small (~15-20mm) residual site
error (see NOTES.md "Phase 3" -- a torque/kinematics limitation of the specific reach
configuration, not a bug), and a razor-thin capture margin that only worked with
millimeter-perfect hand placement just knocked the can away instead of wrapping around it.
The wider start still closes fully within Phase 2's own ramp+settle timing, so this doesn't
regress Phase 2's fixed-hand grasp (still 10/10, see NOTES.md).
"""

import mujoco
import numpy as np

FINGER_CURL_JOINTS = {
    "index": ("finger_r_joint6", "finger_r_joint7", "finger_r_joint8"),
    "middle": ("finger_r_joint10", "finger_r_joint11", "finger_r_joint12"),
}
THUMB_CURL_JOINTS = ("finger_r_joint3", "finger_r_joint4")
THUMB_PRESHAPE = {
    "finger_r_joint1": 0.131,   # CMC abduction
    "finger_r_joint2": -1.309,  # MCP yaw
}

FINGER_OPEN_FRAC = 0.20
THUMB_OPEN_FRAC = 0.0

FINGER_BODY_GROUPS = {
    "thumb": ("finger_r_link1", "finger_r_link2", "finger_r_link3", "finger_r_link4"),
    "index": ("finger_r_link5", "finger_r_link6", "finger_r_link7", "finger_r_link8"),
    "middle": ("finger_r_link9", "finger_r_link10", "finger_r_link11", "finger_r_link12"),
}

CAN_GEOM_NAME = "can_geom"


def _actuator_for_joint(model, jid):
    for aid in range(model.nu):
        if model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT and model.actuator_trnid[aid, 0] == jid:
            return aid
    return None


def _set_joint_ctrl(model, data, joint_name, value):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    aid = _actuator_for_joint(model, jid)
    data.ctrl[aid] = value


def apply_grasp(model, data, grasp: float, thumb: float):
    """Map the two synergy scalars (each clamped to [0, 1]) to actuator ctrl targets.

    grasp -> index + middle pip/dip/tip, ramped over [FINGER_OPEN_FRAC, 1.0] of each
             joint's range.
    thumb -> thumb mcp_pitch/ip, ramped over [THUMB_OPEN_FRAC, 1.0] of each joint's range.
    Thumb CMC/yaw are always held at the fixed pre-shape regardless of either scalar.
    """
    grasp = float(np.clip(grasp, 0.0, 1.0))
    thumb = float(np.clip(thumb, 0.0, 1.0))

    for name, value in THUMB_PRESHAPE.items():
        _set_joint_ctrl(model, data, name, value)

    for joint_name in THUMB_CURL_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        lo, hi = model.jnt_range[jid]
        frac = THUMB_OPEN_FRAC + thumb * (1.0 - THUMB_OPEN_FRAC)
        _set_joint_ctrl(model, data, joint_name, lo + frac * (hi - lo))

    for finger_joints in FINGER_CURL_JOINTS.values():
        for joint_name in finger_joints:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            lo, hi = model.jnt_range[jid]
            frac = FINGER_OPEN_FRAC + grasp * (1.0 - FINGER_OPEN_FRAC)
            _set_joint_ctrl(model, data, joint_name, lo + frac * (hi - lo))


def get_finger_can_contacts(model, data):
    """Return {finger_group_name: total_normal_force} for fingers currently touching the can.

    finger_group_name is one of "thumb", "index", "middle" (see FINGER_BODY_GROUPS).
    Normal force is the contact-frame normal component magnitude, summed over every
    contact point belonging to that finger group this step.
    """
    can_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, CAN_GEOM_NAME)
    body_to_group = {}
    for group, bodies in FINGER_BODY_GROUPS.items():
        for b in bodies:
            body_to_group[b] = group

    forces = {}
    force_vec = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if can_gid not in (c.geom1, c.geom2):
            continue
        other = c.geom1 if c.geom2 == can_gid else c.geom2
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[other])
        group = body_to_group.get(bname)
        if group is None:
            continue
        mujoco.mj_contactForce(model, data, i, force_vec)
        normal_force = abs(force_vec[0])  # contact frame: index 0 is the normal component
        forces[group] = forces.get(group, 0.0) + normal_force
    return forces


def is_grasped(model, data, min_fingers=2, min_total_force=0.05, require_thumb=True):
    """Contact-force-based grasp check (no position/attachment cheating).

    True if >= min_fingers distinct finger groups touch the can (thumb required by
    default, matching PLAN.md), and the summed normal force exceeds min_total_force (N).
    """
    forces = get_finger_can_contacts(model, data)
    if require_thumb and "thumb" not in forces:
        return False
    if len(forces) < min_fingers:
        return False
    return sum(forces.values()) >= min_total_force
