"""Phase 2 -- grasp synergy mapping + contact-based grasp verification.
Phase 4 -- generalized to either hand via a `side` ('l'/'r') parameter; default stays 'r'
so every Phase 1-3 call site (single-hand models, right hand only) is unchanged.

Two independent scalars, matching the reference video's "Right grasp / Right thumb"
sliders:
  - `grasp` (0..1): index + middle finger curl (pip/dip/tip).
  - `thumb` (0..1): thumb curl (mcp_pitch, ip) *and* (this session) thumb MCP-yaw, ramped
    from a safe rest angle to a wider "folds toward the palm" angle -- see
    THUMB_YAW_REST/THUMB_YAW_CURL below for why yaw is no longer a flat constant. Thumb
    CMC (abduction) is still a genuinely fixed pre-shape, independent of either scalar --
    found by FK sweep in Phase 2 (see NOTES.md) so the thumb opposes the index/middle
    convergence zone around the can.

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
thumb_r_cmc's so the CMC value carries over unchanged (now -0.131, see below), but
thumb_l_mcp_yaw's range is the mirror image of thumb_r_mcp_yaw's ([0, pi] vs [-pi, 0]) so
THUMB_YAW_REST/THUMB_YAW_CURL negate the right hand's values for "l". The left hand has
no can of its own in this project (single shared can, right-hand regression only, see
NOTES.md "Phase 4") so this mirror hasn't been independently validated against real
contact-force grasp success the way the right hand's has -- it's provided for teleop
completeness, not claimed to be as thoroughly tuned.

Thumb pre-shape was revisited twice this session after visual inspection showed the thumb
folding out to the side rather than toward the palm at thumb=1.0. First pass: CMC
abduction negated (0.131 -> -0.131 rad -- confirmed by direct render comparison to barely
affect the overall thumb direction, since CMC only adjusts spread, not which way the thumb
faces) and MCP yaw -- the joint that actually sets which way the thumb's curl plane faces --
moved from a flat -1.309 rad first to -1.5708 (the midpoint of thumb_r_mcp_yaw's [-pi, 0]
range, found by rendering a sweep across the full range at grasp=0/thumb=1 and picking the
point where the curled thumb visibly swings in to meet the index/middle convergence zone),
then nudged further to -2.0326 rad once that direction was confirmed to look right. That
landed as a flat constant in THUMB_PRESHAPE, same as CMC -- and every fixed constant in
THUMB_PRESHAPE is applied on *every* apply_grasp call regardless of `thumb`'s value,
including during a pregrasp approach where `thumb=0`.

That turned out to be a real bug, not just a style choice: tests/test_phase_4.py's
integration pick success dropped from ~80-90% to 20% because the wider MCP-yaw angle is
now wide enough that the thumb physically clips the can while the arm swings from its
folded home pose to the pregrasp hover position -- confirmed by instrumenting a single
pick trial (contact registers on the thumb ~47% through that swing, can drifts up to
85mm by the time the arm arrives at pregrasp; reverting just this one joint back to
-1.309 for that same swing drops the drift back to a normal ~4.5mm). hand_only.xml's own
tests never move the arm at all, so they couldn't have caught this.

Fix: MCP yaw is no longer a flat constant. THUMB_YAW_REST is the original, collision-safe
angle (identical to the value used for years before this session); THUMB_YAW_CURL is the
wide angle confirmed to look right at thumb=1. apply_grasp ramps linearly between them by
the `thumb` scalar itself, the same one that already drives the curl joints -- so the
angle stays safe while `thumb=0` (i.e. while the arm is still approaching, in every call
site in this codebase), and only swings out to the wide angle once the caller actually
starts closing the thumb, by which point the arm has already stopped moving next to the
can. Re-verified tests/test_phase_2.py (10/10, unchanged -- hand_only never moves the arm
so it was never exposed to begin with) and tests/test_phase_4.py (back to 9/10, matching
the pre-regression baseline).
"""

import mujoco
import numpy as np

# 검지/중지 각 손가락의 pip/dip/tip 관절 이름 (좌/우 손 각각). grasp 스칼라 하나가
# 이 여섯 관절 전부의 목표 각도로 동시에 퍼진다.
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
# 엄지 mcp_pitch/ip 관절 (thumb 스칼라가 매핑되는 대상).
THUMB_CURL_JOINTS = {
    "l": ("finger_l_joint3", "finger_l_joint4"),
    "r": ("finger_r_joint3", "finger_r_joint4"),
}
# Session 8 후속 6 -- unlike FINGER_CURL_JOINTS (index/middle curl the same physical
# direction on both hands, so both hands' ranges start at 0 = extended), the thumb's
# mcp_pitch/ip joints are mirrored: finger_r_joint3/4's range is [0, 1.5708] (lo=extended,
# hi=curled) but finger_l_joint3/4's is the geometric mirror [-1.5708, 0] (so *hi*=extended,
# lo=curled -- the sign convention flips, it isn't just relabeled). apply_grasp used to
# always take `lo` as the open end regardless of side, which for the left hand commanded
# the thumb toward its curled extreme even at thumb=0 -- self-colliding finger_l_link3/4
# into the palm (measured ~40N contact, actuator pinned at its force limit, ~26 deg short of
# target) instead of reaching a clean, contact-free extended pose the way the right hand
# does. This also silently inverted the left thumb's curl *direction* (thumb=1 commanded it
# toward 0 = MORE extended, not more curled) -- a real behavioral bug, not just a rest-pose
# cosmetic one, though never exercised since the left hand's grasp has no can of its own to
# test against (see module docstring).
# 손별 보간 방향 플래그: 오른손은 range의 lo가 "편 상태", 왼손은 range 자체가
# 부호까지 미러링돼 있어 hi가 "편 상태"다 -- 이걸 안 나누면 왼손 엄지가 항상
# 반대 방향(펴야 할 때 굽힘)으로 명령을 받는다.
THUMB_CURL_OPEN_AT_HI = {"l": True, "r": False}
# 약지/새끼의 pip/dip/tip (mcp는 range=0으로 잠겨 있어 여기 없음) -- 실제 grasp에는
# 참여하지 않고, grasp 스칼라에 비례해 보기 좋으라고만 살짝 굽힌다(아래 apply_grasp
# 참고).
RING_PINKY_CURL_JOINTS = {
    "l": ("finger_l_joint14", "finger_l_joint15", "finger_l_joint16",
          "finger_l_joint18", "finger_l_joint19", "finger_l_joint20"),
    "r": ("finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
          "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"),
}
# 엄지 CMC(벌림) 관절은 grasp/thumb 스칼라와 무관하게 항상 이 고정값으로 유지된다 --
# Phase 2에서 FK 그리드 서치로 찾은, 검지·중지 수렴 지점을 엄지가 마주보게 하는 각도.
# CMC만 여기 있다 -- MCP yaw는 더 이상 고정값이 아니라 thumb 스칼라로 램프된다
# (THUMB_YAW_REST/THUMB_YAW_CURL, 바로 아래 참고).
THUMB_PRESHAPE = {
    "l": {"finger_l_joint1": -0.131},  # CMC abduction (symmetric range, same value as right)
    "r": {"finger_r_joint1": -0.131},  # CMC abduction
}
# MCP yaw(엄지가 어느 방향을 "바라보는지")는 한때 THUMB_PRESHAPE 안에서 완전히
# 고정값이었다 -- 그런데 그 고정값을 -1.309 -> -2.0326으로 넓혀 thumb=1일 때
# 손바닥 쪽으로 보기 좋게 접히도록 만들었더니, 이 값이 "고정"이라 grasp/thumb=0인
# 접근(pregrasp) 단계에서도 그대로 적용돼 팔이 홈 자세에서 캔 쪽으로 스윙하는 동안
# 엄지가 캔을 미리 쳐서 날려버렸다(실측: full_scene.xml 통합 pick 테스트에서 캔이
# pregrasp 이동 중 최대 85mm 밀려나 pick 성공률 20%로 급락, hand_only 단독 테스트
# 에서는 팔이 안 움직이므로 이 문제가 안 드러남). 그래서 MCP yaw는 이제 CMC처럼
# 완전히 고정이 아니라 thumb 스칼라로 THUMB_YAW_REST(접근 중 안전하게 검증된 각도,
# 과거의 고정값과 동일) -> THUMB_YAW_CURL(사용자가 보기 좋다고 확인한 넓은 각도)로
# 램프한다 -- thumb=0(아직 쥐기 전, 팔이 움직이는 동안)에는 안전한 각도를 유지하고,
# thumb이 실제로 올라갈 때(팔은 이미 멈춰 캔 옆에 있을 때)만 넓은 각도로 돌아간다.
THUMB_YAW_REST = {"l": 1.309, "r": -1.309}
THUMB_YAW_CURL = {"l": 2.0326, "r": -2.0326}

# grasp/thumb=0일 때도 관절 range 전체(lo)까지 펴지 않고 이만큼 남겨둔다 --
# "접촉 직전" 자세를 만들어 자유낙하하는 캔을 놓치지 않기 위함(모듈 docstring 참고).
FINGER_OPEN_FRAC = 0.20
THUMB_OPEN_FRAC = 0.0
# 약지/새끼 pip/dip/tip이 grasp=1.0일 때 굽는 최대 비율(자기 range의 35%까지만) --
# pick 성공률을 0.20~0.60으로 스윕해서 찾은 안전한 상한(0.40/0.45 사이가 절벽).
RING_PINKY_MAX_FRAC = 0.35

# 접촉력 판정(get_finger_can_contacts)에서 "이 body가 어느 손가락 그룹 소속인지"
# 조회하는 데 쓰는 역방향 매핑의 원본 데이터.
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
BOX_GEOM_NAME = "box_geom"
BOX_HAND_BODIES = {
    "l": ("hx5_l_base",) + tuple(f"finger_l_link{i}" for i in range(1, 21)),
    "r": ("hx5_r_base",) + tuple(f"finger_r_link{i}" for i in range(1, 21)),
}

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
    """관절 이름 -> (joint id, actuator id) 조회를 캐싱한다.

    관절 자체는 mj_name2id로 바로 찾지만, "이 관절을 움직이는 actuator가 몇 번인지"는
    MuJoCo가 직접 안 알려줘서 전체 actuator를 선형 탐색(O(nu))해야 한다 -- 매 물리
    스텝마다 이 탐색을 반복하면 비용이 커서(실측: mj_step 단독 0.1ms/스텝 vs 캐싱 전
    1.1ms/스텝) 처음 조회한 결과를 (model, joint_name) 키로 캐싱해둔다.
    해당 관절에 actuator가 아예 없는 모델(hand_only.xml/arm_hand.xml의 약지·새끼)에서는
    aid가 None으로 캐싱된다 -- 호출부에서 반드시 None 체크할 것.
    """
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
    """관절 이름으로 바로 목표값(ctrl)을 써주는 편의 함수.

    주의: 여기선 aid가 None인지 확인하지 않는다 -- 이 함수를 호출하는 쪽(엄지/검지/
    중지 루프)은 항상 actuator가 존재함을 전제하기 때문. actuator가 없을 수도 있는
    약지/새끼 루프는 apply_grasp 안에서 직접 _resolve_joint_actuator를 불러 None을
    가드한다(data.ctrl[aid]에 aid=None이 들어가면 numpy가 배열 전체에 broadcast
    대입해버리는 사고가 나므로 -- 아래 apply_grasp 주석 참고).
    """
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

    # 1) 엄지 CMC는 항상 고정 pre-shape (grasp/thumb 스칼라와 무관).
    for name, value in THUMB_PRESHAPE[side].items():
        _set_joint_ctrl(model, data, name, value)
    # MCP yaw는 thumb 스칼라로 REST(안전, 접근 중에도 캔을 안 침)에서
    # CURL(사용자가 확인한, 손바닥 쪽으로 접힌 모습)까지 램프한다 -- 위 THUMB_YAW_REST/
    # THUMB_YAW_CURL 주석 참고.
    yaw_joint = f"finger_{side}_joint2"
    yaw_value = THUMB_YAW_REST[side] + thumb * (THUMB_YAW_CURL[side] - THUMB_YAW_REST[side])
    _set_joint_ctrl(model, data, yaw_joint, yaw_value)

    # 2) 엄지 mcp_pitch/ip -- thumb 스칼라를 [THUMB_OPEN_FRAC, 1.0] 구간에 매핑.
    #    손별로 range의 "편 상태"가 lo/hi 중 어느 쪽인지 다르므로(THUMB_CURL_OPEN_AT_HI)
    #    보간 방향을 뒤집어준다.
    for joint_name in THUMB_CURL_JOINTS[side]:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        lo, hi = model.jnt_range[jid]
        frac = THUMB_OPEN_FRAC + thumb * (1.0 - THUMB_OPEN_FRAC)
        if THUMB_CURL_OPEN_AT_HI[side]:
            value = hi - frac * (hi - lo)
        else:
            value = lo + frac * (hi - lo)
        _set_joint_ctrl(model, data, joint_name, value)

    # 3) 검지/중지 pip/dip/tip -- grasp 스칼라를 [FINGER_OPEN_FRAC, 1.0] 구간에 매핑.
    #    실제로 캔을 쥐는 3점 파지(엄지+검지+중지)의 핵심 구동부.
    for finger_joints in FINGER_CURL_JOINTS[side].values():
        for joint_name in finger_joints:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            lo, hi = model.jnt_range[jid]
            frac = FINGER_OPEN_FRAC + grasp * (1.0 - FINGER_OPEN_FRAC)
            _set_joint_ctrl(model, data, joint_name, lo + frac * (hi - lo))

    # 4) 약지/새끼 pip/dip/tip -- 실제 grasp에는 참여하지 않고, grasp 스칼라에 비례해
    #    자기 range의 RING_PINKY_MAX_FRAC까지만 코스메틱하게 굽는다(0=rest에서 완전히
    #    펴짐, grasp=1에서 0.35까지).
    for joint_name in RING_PINKY_CURL_JOINTS[side]:
        # jid can be -1 (hand_only.xml/arm_hand.xml have no left hand at all) and aid can be
        # None even when jid is valid (those two models still hard-lock ring/pinky pip/dip/
        # tip at range="0 0" with no actuator, unlike full_scene.xml -- see NOTES.md "Phase 5
        # 후속 4"). Must check both explicitly: data.ctrl[None] silently broadcasts to the
        # *entire* ctrl array instead of raising (the exact bug Session 2 already hit once,
        # see NOTES.md "Phase 2").
        # (한글) jid/aid가 없을 수 있는 모델(hand_only/arm_hand)이 있으므로 반드시 둘 다
        # None/-1 여부를 확인하고 건너뛴다 -- data.ctrl[None]은 에러 없이 배열 전체를
        # 덮어써버리는 numpy의 함정이라 이 프로젝트에서 세 번이나 반복 재발했던 버그.
        jid, aid = _resolve_joint_actuator(model, joint_name)
        if jid == -1 or aid is None:
            continue
        lo, hi = model.jnt_range[jid]
        frac = grasp * RING_PINKY_MAX_FRAC
        data.ctrl[aid] = lo + frac * (hi - lo)


def apply_open_hand(model, data, side: str = "r"):
    """Command every actuated finger joint to the fully-open neutral pose."""
    for i in range(1, 21):
        joint_name = f"finger_{side}_joint{i}"
        jid, aid = _resolve_joint_actuator(model, joint_name)
        if jid == -1 or aid is None:
            continue
        data.ctrl[aid] = 0.0


def get_finger_can_contacts(model, data, side: str = "r"):
    """Return {finger_group_name: total_normal_force} for fingers currently touching the can.

    finger_group_name is one of "thumb", "index", "middle" (see FINGER_BODY_GROUPS).
    Normal force is the contact-frame normal component magnitude, summed over every
    contact point belonging to that finger group this step.
    """
    can_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, CAN_GEOM_NAME)
    # body 이름 -> "thumb"/"index"/"middle" 그룹 이름 역방향 조회 테이블.
    body_to_group = {}
    for group, bodies in FINGER_BODY_GROUPS[side].items():
        for b in bodies:
            body_to_group[b] = group

    forces = {}
    force_vec = np.zeros(6)
    # 이번 스텝에 발생한 접촉(data.contact) 전체를 훑어서, 캔과 맞닿은 접촉만 골라
    # 어느 손가락 그룹인지 확인하고 법선력을 합산한다 -- 위치가 아니라 실제 접촉력을
    # 근거로 판정하기 위함(이 프로젝트의 핵심 규칙).
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
    # 엄지가 반드시 포함되고(기본값), 서로 다른 손가락 그룹 2개 이상이 닿아 있으며,
    # 합산 법선력이 임계값을 넘어야 "쥐었다"고 판정한다 -- 셋 다 접촉력 기반이라
    # 위치/부착 치팅이 끼어들 여지가 없다.
    if require_thumb and "thumb" not in forces:
        return False
    if len(forces) < min_fingers:
        return False
    return sum(forces.values()) >= min_total_force


def get_box_hand_contacts(model, data):
    """Return {'l': normal_force, 'r': normal_force} for hand contacts on box_geom."""
    box_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, BOX_GEOM_NAME)
    if box_gid == -1:
        return {"l": 0.0, "r": 0.0}

    body_to_side = {}
    for side, bodies in BOX_HAND_BODIES.items():
        for name in bodies:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid != -1:
                body_to_side[bid] = side

    forces = {"l": 0.0, "r": 0.0}
    force_vec = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if box_gid not in (c.geom1, c.geom2):
            continue
        other = c.geom1 if c.geom2 == box_gid else c.geom2
        side = body_to_side.get(model.geom_bodyid[other])
        if side is None:
            continue
        mujoco.mj_contactForce(model, data, i, force_vec)
        forces[side] += abs(force_vec[0])
    return forces


def is_box_held(model, data, min_force_per_hand=1.0):
    """True when both hands are pressing box_geom above the per-hand force threshold."""
    forces = get_box_hand_contacts(model, data)
    return forces["l"] >= min_force_per_hand and forces["r"] >= min_force_per_hand
