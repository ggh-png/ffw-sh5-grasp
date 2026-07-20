"""Hand-synergy commands and contact-force grasp detection.

``grasp`` curls the index and middle fingers, while ``thumb`` controls thumb curl and
yaw.  Ring and pinky joints receive only a small cosmetic curl.  Both hands share this
mapping, with explicit interpolation direction for the mirrored left-thumb ranges.

The tuned open fractions keep the fingers near the object without starting in contact.
The thumb yaw remains collision-safe during approach and rotates toward the palm only as
the thumb closes.  Grasp detection uses MuJoCo contact forces rather than object pose.
"""

import mujoco
import numpy as np

import mj_util

SIDES = ("l", "r")

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
# Right-thumb ranges open at their lower bound; mirrored left-thumb ranges open at their
# upper bound.  Keep the interpolation direction explicit instead of relying on signs.
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
# MCP yaw stays at a collision-safe angle during approach, then rotates toward the palm as
# the thumb closes.  Left-hand values mirror the tuned right-hand values.
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
BODY_TO_FINGER_GROUP = {
    side: {
        body: group
        for group, bodies in groups.items()
        for body in bodies
    }
    for side, groups in FINGER_BODY_GROUPS.items()
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


def _validate_side(side):
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}, got {side!r}")


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
    aid = mj_util.find_actuator_for_joint(model, jid) if jid != -1 else None
    _JOINT_ACTUATOR_CACHE[key] = (jid, aid)
    return jid, aid


def _ramp_value(lo, hi, frac, open_at_hi=False):
    """관절 range의 [lo, hi] 구간을 frac(0~1) 비율로 보간한다.

    open_at_hi=False면 lo가 "펼침"(frac=0 -> lo, frac=1 -> hi), True면 그 반대
    (frac=0 -> hi, frac=1 -> lo) -- 왼손 엄지처럼 range 자체가 미러링돼 "펼침"이
    hi 쪽인 관절에 쓴다(THUMB_CURL_OPEN_AT_HI 참고).
    """
    if open_at_hi:
        return hi - frac * (hi - lo)
    return lo + frac * (hi - lo)


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


def _set_joint_fraction(model, data, joint_name, fraction, *, open_at_hi=False):
    """Interpolate an actuated joint across its configured range."""
    joint_id, actuator_id = _resolve_joint_actuator(model, joint_name)
    lo, hi = model.jnt_range[joint_id]
    data.ctrl[actuator_id] = _ramp_value(lo, hi, fraction, open_at_hi)


def apply_grasp(model, data, grasp: float, thumb: float, side: str = "r"):
    """Map the two synergy scalars (each clamped to [0, 1]) to actuator ctrl targets.

    grasp -> index + middle pip/dip/tip, ramped over [FINGER_OPEN_FRAC, 1.0] of each
             joint's range.
    thumb -> thumb mcp_pitch/ip, ramped over [THUMB_OPEN_FRAC, 1.0] of each joint's range.
    Thumb CMC is fixed; thumb yaw interpolates between safe approach and curled poses.
    side selects which hand ('l' or 'r'); default 'r' matches every pre-Phase-4 call site.
    """
    _validate_side(side)
    grasp = float(np.clip(grasp, 0.0, 1.0))
    thumb = float(np.clip(thumb, 0.0, 1.0))
    finger_fraction = FINGER_OPEN_FRAC + grasp * (1.0 - FINGER_OPEN_FRAC)
    thumb_fraction = THUMB_OPEN_FRAC + thumb * (1.0 - THUMB_OPEN_FRAC)

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
        _set_joint_fraction(
            model,
            data,
            joint_name,
            thumb_fraction,
            open_at_hi=THUMB_CURL_OPEN_AT_HI[side],
        )

    # 3) 검지/중지 pip/dip/tip -- grasp 스칼라를 [FINGER_OPEN_FRAC, 1.0] 구간에 매핑.
    #    실제로 캔을 쥐는 3점 파지(엄지+검지+중지)의 핵심 구동부.
    for finger_joints in FINGER_CURL_JOINTS[side].values():
        for joint_name in finger_joints:
            _set_joint_fraction(model, data, joint_name, finger_fraction)

    # 4) 약지/새끼 pip/dip/tip -- 실제 grasp에는 참여하지 않고, grasp 스칼라에 비례해
    #    자기 range의 RING_PINKY_MAX_FRAC까지만 코스메틱하게 굽는다(0=rest에서 완전히
    #    펴짐, grasp=1에서 0.35까지).
    for joint_name in RING_PINKY_CURL_JOINTS[side]:
        # jid can be -1 (hand_only.xml/arm_hand.xml have no left hand at all) and aid can be
        # None even when jid is valid (those two models still hard-lock ring/pinky pip/dip/
        # tip at range="0 0" with no actuator, unlike full_scene.xml). Must check both
        # explicitly: data.ctrl[None] silently broadcasts to the *entire* ctrl array instead
        # of raising (the exact bug this project has hit more than once).
        # (한글) jid/aid가 없을 수 있는 모델(hand_only/arm_hand)이 있으므로 반드시 둘 다
        # None/-1 여부를 확인하고 건너뛴다 -- data.ctrl[None]은 에러 없이 배열 전체를
        # 덮어써버리는 numpy의 함정이라 이 프로젝트에서 세 번이나 반복 재발했던 버그.
        jid, aid = _resolve_joint_actuator(model, joint_name)
        if jid == -1 or aid is None:
            continue
        lo, hi = model.jnt_range[jid]
        data.ctrl[aid] = _ramp_value(lo, hi, grasp * RING_PINKY_MAX_FRAC)


def get_finger_can_contacts(model, data, side: str = "r"):
    """Return {finger_group_name: total_normal_force} for fingers currently touching the can.

    finger_group_name is one of "thumb", "index", "middle" (see FINGER_BODY_GROUPS).
    Normal force is the contact-frame normal component magnitude, summed over every
    contact point belonging to that finger group this step.
    """
    _validate_side(side)
    can_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, CAN_GEOM_NAME)
    body_to_group = BODY_TO_FINGER_GROUP[side]

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
    default), and the summed normal force exceeds min_total_force (N).
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
