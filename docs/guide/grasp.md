# `src/grasp.py` — grasp synergy 매핑 + 접촉력 기반 파지 판정

## 이 파일이 하는 일

두 개의 스칼라(`grasp`, `thumb`, 각각 0~1)를 손가락 20개 관절의 목표 각도로 펼쳐주는
`apply_grasp()`, 그리고 캔과의 실제 접촉력을 근거로 "지금 쥐고 있는지"를 판정하는
`is_grasped()`/`get_finger_can_contacts()`를 제공한다. 물리 모델(`model`)과 상태
(`data`)는 인자로 받을 뿐 이 모듈 자신은 `MjModel`/`MjData`를 만들지 않는다 — 어떤
씬(`hand_only.xml`/`arm_hand.xml`/`full_scene.xml`)에도 똑같이 쓸 수 있는 순수 로직
모듈이다.

## 구현: 스칼라 하나 → 관절 여러 개

핵심은 `apply_grasp()`가 매 호출마다 하는 네 단계다:

```python title="src/grasp.py — apply_grasp 본문"
# 1) 엄지 CMC/MCP-yaw는 항상 고정 pre-shape (grasp/thumb 스칼라와 무관).
for name, value in THUMB_PRESHAPE[side].items():
    _set_joint_ctrl(model, data, name, value)

# 2) 엄지 mcp_pitch/ip -- thumb 스칼라를 [THUMB_OPEN_FRAC, 1.0] 구간에 매핑.
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
for finger_joints in FINGER_CURL_JOINTS[side].values():
    for joint_name in finger_joints:
        ...
        frac = FINGER_OPEN_FRAC + grasp * (1.0 - FINGER_OPEN_FRAC)
        _set_joint_ctrl(model, data, joint_name, lo + frac * (hi - lo))

# 4) 약지/새끼 -- 실제 grasp에는 참여하지 않고, 보기 좋으라고만 살짝 굽힌다.
for joint_name in RING_PINKY_CURL_JOINTS[side]:
    jid, aid = _resolve_joint_actuator(model, joint_name)
    if jid == -1 or aid is None:
        continue
    ...
```

몇 가지가 여기서 반복되는 패턴이다.

**"0~1을 그대로 [lo, hi]에 매핑하지 않는다."** `FINGER_OPEN_FRAC=0.20`, `THUMB_OPEN_FRAC=0.0`,
`RING_PINKY_MAX_FRAC=0.35` 같은 상수들이 전부 이 서브-레인지 매핑을 위한 것이다.
예를 들어 `grasp=0`일 때도 손가락을 관절 range의 맨 끝(완전히 편 상태)까지 펴지
않고 20% 지점에서 멈춘다 — 캔에 닿기 직전 자세를 만들어, 자유낙하하는 캔을 손이
따라잡을 시간을 벌기 위해서다.

**"손별로 range의 부호/방향이 다르면 보간 방향을 뒤집는다."** `THUMB_CURL_OPEN_AT_HI =
{"l": True, "r": False}`가 이걸 담당한다. 오른손 엄지 관절은 `range=[0, 1.5708]`
(lo=편 상태), 왼손은 그 미러 이미지인 `[-1.5708, 0]`(hi=편 상태)이라, 한쪽 부호만
믿고 매핑하면 반대쪽 손이 정반대로 움직인다.

**"actuator가 없을 수도 있는 관절은 반드시 None 체크."** 약지/새끼 관절은
`hand_only.xml`/`arm_hand.xml`에는 아예 actuator가 없다(`range="0 0"`으로 고정만
되어 있음). `_resolve_joint_actuator()`가 `(jid, aid)`를 캐싱해서 반환하는데,
`aid`가 `None`일 수 있다는 걸 호출부에서 명시적으로 확인하지 않으면
`data.ctrl[None] = value`가 numpy에서 **배열 전체에 스칼라를 broadcast**해버리는
사고로 이어진다(`arr[None]`이 `arr[np.newaxis]`로 해석되기 때문) — 이 프로젝트에서
같은 패턴으로 세 번 재발했던 버그라 `apply_grasp`의 4단계만 명시적으로 가드한다.

파지 판정은 별도 함수 두 개로 분리돼 있다:

```python title="src/grasp.py — 접촉력 기반 판정"
def get_finger_can_contacts(model, data, side="r"):
    # data.ncon(이번 스텝의 접촉) 전체를 훑어, 캔과 맞닿은 접촉의 손가락 그룹별
    # 법선력(mj_contactForce)을 합산한다. 위치가 아니라 힘을 근거로 삼는다.
    ...

def is_grasped(model, data, min_fingers=2, min_total_force=0.05, require_thumb=True, side="r"):
    forces = get_finger_can_contacts(model, data, side=side)
    if require_thumb and "thumb" not in forces:
        return False
    return len(forces) >= min_fingers and sum(forces.values()) >= min_total_force
```

`apply_grasp`가 관절각을 "지시"하는 쪽이라면, 이쪽은 "실제로 됐는지"를 물리 엔진의
접촉력으로만 확인하는 쪽 — 캔이 어디 있는지, 손가락이 캔에 "붙어 있는지" 같은
위치 기반 치팅이 끼어들 여지가 없다.

## 이 파일이 다른 파일과 합쳐지는 방식

`grasp.py`는 이 프로젝트에서 **가장 의존성이 적은 리프 모듈**이다 — `mujoco`와
`numpy` 말고는 아무것도 import하지 않고, 다른 프로젝트 파일도 이 파일을 import하지
않는다(호출만 받는 쪽).

- **`teleop_app.py`**가 유일한 실시간 호출자다: `TeleopApp._step_physics()`의 물리
  서브스텝 루프 안에서 매 스텝(`steps_per_frame`번) `grasp.apply_grasp(model, data,
  grasp=targets["grasp_r"], thumb=targets["thumb_r"], side="r")`를 양손 각각 호출한다.
  슬라이더 값(`targets["grasp_r"]` 등)은 `teleop_ui.py`가 매 프레임 갱신해두는
  값이므로, 흐름은 **teleop_ui(슬라이더 읽기) → teleop_app(값 보관) → grasp.py(관절
  ctrl로 변환) → mj_step(물리 반영)** 순이다.
- `tests/test_phase_2.py`, `test_phase_3.py`, `test_phase_4.py`도 각자의 검증
  루프 안에서 동일하게 `apply_grasp`/`is_grasped`를 직접 호출해 회귀 테스트한다 —
  `teleop_app.py`를 거치지 않고 `grasp.py`만 단독으로 실행 가능하다는 뜻이다.
- `arm_control.py`/`ik.py`/`base_teleop.py`와는 서로 호출 관계가 전혀 없다 — 팔
  움직임과 손가락 움직임은 독립적인 액추에이터 집합이라, `teleop_app.py`의 물리
  스텝 안에서 "나란히" 호출될 뿐 서로의 존재를 모른다.
