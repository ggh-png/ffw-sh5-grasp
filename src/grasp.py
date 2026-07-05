"""Phase 2 -- grasp synergy mapping + contact-based grasp verification.
Phase 4 -- generalized to either hand via a `side` ('l'/'r') parameter; default stays 'r'
so every Phase 1-3 call site (single-hand models, right hand only) is unchanged.

Two independent scalars, matching the reference video's "Right grasp / Right thumb"
sliders:
  - `grasp` (0..1): index + middle finger curl (pip/dip/tip).
  - `thumb` (0..1): thumb curl (mcp_pitch, ip). Thumb CMC/MCP-yaw (abduction) are a fixed
    pre-shape, not part of either scalar -- found by FK sweep in Phase 2 (see NOTES.md) so
    the thumb opposes the index/middle convergence zone around the can.

Ring and pinky mcp (spread) stay locked at range=0 (3-point grasp fallback from PLAN.md's
own guidance), so this module never actively grasps with them. Their pip/dip/tip *do* get a
small cosmetic curl here (Session 8 후속 4), scaled by the same `grasp` scalar as index/
middle but capped at RING_PINKY_MAX_FRAC (0.35, not 1.0) of their own range -- purely so the
hand doesn't look like two fingers are frozen open while the other three visibly close.
0.35 is not an arbitrary cosmetic choice: it's the exact ceiling found by sweeping this
fraction against tests/test_phase_4.py's pick success (see NOTES.md "Phase 5 후속 3") --
20-40% all held 10/10, 45%+ dropped to 0/10 (ring/pinky start interfering with the real
3-point grasp past that point). Scaling by `grasp` (0 at rest, 0.35 at grasp=1.0) rather than
holding 0.35 as a flat constant regardless of `grasp` was a deliberate change from the
previous session's approach: a *constant* curl meant ring/pinky started the session already
curled even at grasp=0/rest, which looked wrong and differed from every other digit's
fully-extended rest pose (see NOTES.md "Phase 5 후속 4").

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

Left-hand thumb pre-shape (Phase 4, models/full_scene.xml): mirrored from the right's
FK-sweep result, not independently re-swept -- thumb_l_cmc's range is symmetric with
thumb_r_cmc's so the CMC value carries over unchanged (0.131), but thumb_l_mcp_yaw's range
is the mirror image of thumb_r_mcp_yaw's ([0, pi] vs [-pi, 0]) so that value is negated
(+1.309). The left hand has no can of its own in this project (single shared can, right-hand
regression only, see NOTES.md "Phase 4") so this mirror hasn't been independently validated
against real contact-force grasp success the way the right hand's has -- it's provided for
teleop completeness, not claimed to be as thoroughly tuned.
"""

import mujoco
import numpy as np

FINGER_CURL_JOINTS = {
    "l": {
        "index": ("finger_l_joint6", "finger_l_joint7", "finger_l_joint8"),
        "middle": ("finger_l_joint10", "finger_l_joint11", "finger_l_joint12"),
    },
    "r": {
        "index": ("finger_r_joint6", "finger_r_joint7", "finger_r_joint8"),
        "middle": ("finger_r_joint10", "finger_r_joint11", "finger_r_joint12"),
    },
}
THUMB_CURL_JOINTS = {
    "l": ("finger_l_joint3", "finger_l_joint4"),
    "r": ("finger_r_joint3", "finger_r_joint4"),
}
RING_PINKY_CURL_JOINTS = {
    "l": ("finger_l_joint14", "finger_l_joint15", "finger_l_joint16",
          "finger_l_joint18", "finger_l_joint19", "finger_l_joint20"),
    "r": ("finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
          "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"),
}
THUMB_PRESHAPE = {
    "l": {
        "finger_l_joint1": 0.131,   # CMC abduction (symmetric range, same value as right)
        "finger_l_joint2": 1.309,   # MCP yaw (mirrored range, sign-flipped from right)
    },
    "r": {
        "finger_r_joint1": 0.131,   # CMC abduction
        "finger_r_joint2": -1.309,  # MCP yaw
    },
}

FINGER_OPEN_FRAC = 0.20
THUMB_OPEN_FRAC = 0.0
RING_PINKY_MAX_FRAC = 0.35

FINGER_BODY_GROUPS = {
    "l": {
        "thumb": ("finger_l_link1", "finger_l_link2", "finger_l_link3", "finger_l_link4"),
        "index": ("finger_l_link5", "finger_l_link6", "finger_l_link7", "finger_l_link8"),
        "middle": ("finger_l_link9", "finger_l_link10", "finger_l_link11", "finger_l_link12"),
    },
    "r": {
        "thumb": ("finger_r_link1", "finger_r_link2", "finger_r_link3", "finger_r_link4"),
        "index": ("finger_r_link5", "finger_r_link6", "finger_r_link7", "finger_r_link8"),
        "middle": ("finger_r_link9", "finger_r_link10", "finger_r_link11", "finger_r_link12"),
    },
}

CAN_GEOM_NAME = "can_geom"

# apply_grasp runs once per physics *substep* (thousands of calls per pick trial), and each
# call used to re-resolve every joint name via mj_name2id plus a linear O(nu) Python scan
# over every actuator to find its transmission -- harmless at the original 10 lookups/call,
# but adding RING_PINKY_CURL_JOINTS's 6 more (Session 8 후속 4) measurably slowed
# tests/test_phase_4.py's already-long pick trials (measured: ~1.1ms/step with the scan vs
# ~0.1ms/step for mj_step alone). Caching each (model, joint_name) -> (jid, aid) lookup the
# first time it's resolved removes the repeat cost without changing any behavior -- keyed by
# id(model) since tests construct a fresh MjModel per test file.
_JOINT_ACTUATOR_CACHE = {}


def _resolve_joint_actuator(model, joint_name):
    key = (id(model), joint_name)
    cached = _JOINT_ACTUATOR_CACHE.get(key)
    if cached is not None:
        return cached
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    aid = None
    if jid != -1:
        for a in range(model.nu):
            if model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT and model.actuator_trnid[a, 0] == jid:
                aid = a
                break
    _JOINT_ACTUATOR_CACHE[key] = (jid, aid)
    return jid, aid


def _set_joint_ctrl(model, data, joint_name, value):
    _jid, aid = _resolve_joint_actuator(model, joint_name)
    data.ctrl[aid] = value


def apply_grasp(model, data, grasp: float, thumb: float, side: str = "r"):
    """Map the two synergy scalars (each clamped to [0, 1]) to actuator ctrl targets.

    grasp -> index + middle pip/dip/tip, ramped over [FINGER_OPEN_FRAC, 1.0] of each
             joint's range.
    thumb -> thumb mcp_pitch/ip, ramped over [THUMB_OPEN_FRAC, 1.0] of each joint's range.
    Thumb CMC/yaw are always held at the fixed pre-shape regardless of either scalar.
    side selects which hand ('l' or 'r'); default 'r' matches every pre-Phase-4 call site.
    """
    grasp = float(np.clip(grasp, 0.0, 1.0))
    thumb = float(np.clip(thumb, 0.0, 1.0))

    for name, value in THUMB_PRESHAPE[side].items():
        _set_joint_ctrl(model, data, name, value)

    for joint_name in THUMB_CURL_JOINTS[side]:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        lo, hi = model.jnt_range[jid]
        frac = THUMB_OPEN_FRAC + thumb * (1.0 - THUMB_OPEN_FRAC)
        _set_joint_ctrl(model, data, joint_name, lo + frac * (hi - lo))

    for finger_joints in FINGER_CURL_JOINTS[side].values():
        for joint_name in finger_joints:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            lo, hi = model.jnt_range[jid]
            frac = FINGER_OPEN_FRAC + grasp * (1.0 - FINGER_OPEN_FRAC)
            _set_joint_ctrl(model, data, joint_name, lo + frac * (hi - lo))

    for joint_name in RING_PINKY_CURL_JOINTS[side]:
        # jid can be -1 (hand_only.xml/arm_hand.xml have no left hand at all) and aid can be
        # None even when jid is valid (those two models still hard-lock ring/pinky pip/dip/
        # tip at range="0 0" with no actuator, unlike full_scene.xml -- see NOTES.md "Phase 5
        # 후속 4"). Must check both explicitly: data.ctrl[None] silently broadcasts to the
        # *entire* ctrl array instead of raising (the exact bug Session 2 already hit once,
        # see NOTES.md "Phase 2").
        jid, aid = _resolve_joint_actuator(model, joint_name)
        if jid == -1 or aid is None:
            continue
        lo, hi = model.jnt_range[jid]
        frac = grasp * RING_PINKY_MAX_FRAC
        data.ctrl[aid] = lo + frac * (hi - lo)


def get_finger_can_contacts(model, data, side: str = "r"):
    """Return {finger_group_name: total_normal_force} for fingers currently touching the can.

    finger_group_name is one of "thumb", "index", "middle" (see FINGER_BODY_GROUPS).
    Normal force is the contact-frame normal component magnitude, summed over every
    contact point belonging to that finger group this step.
    """
    can_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, CAN_GEOM_NAME)
    body_to_group = {}
    for group, bodies in FINGER_BODY_GROUPS[side].items():
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


def is_grasped(model, data, min_fingers=2, min_total_force=0.05, require_thumb=True, side: str = "r"):
    """Contact-force-based grasp check (no position/attachment cheating).

    True if >= min_fingers distinct finger groups touch the can (thumb required by
    default, matching PLAN.md), and the summed normal force exceeds min_total_force (N).
    """
    forces = get_finger_can_contacts(model, data, side=side)
    if require_thumb and "thumb" not in forces:
        return False
    if len(forces) < min_fingers:
        return False
    return sum(forces.values()) >= min_total_force
