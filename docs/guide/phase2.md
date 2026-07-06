# Phase 2 — 잡는다는 것의 물리, 이 프로젝트의 핵심

`src/grasp.py` · `models/hand_only.xml`

Phase 2는 이 프로젝트에서 가장 중요한 통찰을 담고 있다: **손가락이 캔의 모양에 "알아서
순응해서" 감기는 현상은, 복잡한 제어 알고리즘이 아니라 딱 두 가지 XML 속성
(force-limited position actuator)의 조합만으로 나온다.**

## position actuator + forcerange — 순응(compliance)이 공짜로 나오는 원리

!!! info "핵심 개념 · 왜 손가락이 캔 모양에 알아서 맞춰지는가"
    각 손가락 관절은 `<position kp="0.7" forcerange="-1.0 1.0"/>`처럼 정의된다 —
    "목표 각도로 가라, 단 그 과정에서 낼 수 있는 최대 토크는 ±1.0N·m뿐"이라는
    뜻이다. grasp 스칼라를 0→1로 서서히 올리면 모든 손가락 관절의 목표 각도가
    동시에 "더 굽혀라"로 바뀐다. 그런데 **캔에 먼저 닿은 마디는 그 지점에서 토크가
    forcerange 상한에 막혀 더 못 굽혀지고 멈추는 반면, 아직 안 닿은 마디는 계속
    굽혀진다**. 결과적으로 각 마디가 독립적으로 "닿으면 멈추고, 안 닿으면 계속
    간다"를 반복하면서 손 전체가 캔의 실제 표면 형상에 맞춰 감싸듯 닫힌다 — 이게
    바로 **underactuation(저구동)의 순응 그립**이고, 이 프로젝트는 별도의
    force/torque 피드백 제어기를 전혀 쓰지 않는다. forcerange가 너무 크면 관통해서
    캔을 뚫고 지나가고, 너무 작으면 아예 못 든다 — 이 균형을 찾는 게 튜닝의 전부다.

```python title="src/grasp.py — grasp synergy 매핑 (핵심 부분 발췌)"
def apply_grasp(model, data, grasp: float, thumb: float, side: str = "r"):
    grasp = float(np.clip(grasp, 0.0, 1.0))
    for finger_joints in FINGER_CURL_JOINTS[side].values():
        for joint_name in finger_joints:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            lo, hi = model.jnt_range[jid]
            frac = FINGER_OPEN_FRAC + grasp * (1.0 - FINGER_OPEN_FRAC)
            _set_joint_ctrl(model, data, joint_name, lo + frac * (hi - lo))
```

스칼라 하나(`grasp`, 0~1)가 검지·중지 세 마디 전체의 목표 각도로 퍼져 나간다 — 이게
"grasp synergy"다. 손가락 20개 관절을 사람이 하나하나 조작하는 대신, 실제 로봇 손
텔레옵에서 흔히 쓰는 방식대로 저차원 스칼라 하나로 손 전체를 제어한다. 또한
`[lo, hi]` 전체가 아니라 `[FINGER_OPEN_FRAC, 1.0]` 구간에만 매핑한다는 점도 중요하다
— `grasp=0`이 "완전히 편 상태"가 아니라 "캔 표면 바로 앞(접촉 직전)"이 되도록 미리
여유를 깎아둔 것이다. 자유낙하하는 캔을 force-limited 액추에이터가 따라잡기엔 너무
느리기 때문에, Phase 2에서는 아예 캔 아래에 정적 지지대를 둬서 "쥐기 전까지 그 자리에
가만히 있게" 만들었다.

## contact force로 "진짜 쥐었는지" 판정하기

이 프로젝트는 "손가락이 캔 근처에 있다"거나 "캔이 손 안에 있다" 같은 **위치 기반
판정을 절대 쓰지 않는다**. 대신 `mj_contactForce`로 각 접촉점의 실제 수직
항력(normal force)을 읽어서, 서로 다른 손가락 2개 이상(엄지 포함)이 캔에 충분한
힘으로 닿고 있는지를 직접 확인한다.

```python title="src/grasp.py — contact force 기반 판정"
for i in range(data.ncon):
    c = data.contact[i]
    if can_gid not in (c.geom1, c.geom2):
        continue
    mujoco.mj_contactForce(model, data, i, force_vec)
    normal_force = abs(force_vec[0])   # 접촉 프레임에서 index 0 = 법선 성분

def is_grasped(model, data, min_fingers=2, min_total_force=0.05, require_thumb=True):
    forces = get_finger_can_contacts(model, data)
    if require_thumb and "thumb" not in forces:
        return False
    return len(forces) >= min_fingers and sum(forces.values()) >= min_total_force
```

!!! info "핵심 개념 · data.contact와 mj_contactForce"
    `data.ncon`은 이번 스텝에 발생한 접촉의 개수, `data.contact[i]`는 각 접촉점의
    geom 쌍/위치/법선 방향 등을 담은 구조체다. 힘 자체는 별도 함수
    `mj_contactForce(model, data, i, out)`로 읽어야 하는데, 반환되는 6차원 벡터는
    **월드 좌표가 아니라 그 접촉점 고유의 로컬 프레임**(법선 1 + 마찰 2 +
    비틀림/굴림 3)이라는 점에 주의한다. 인덱스 0이 법선(수직) 성분이다.

## keyframe으로 좋은 자세 저장하기 + 실제로 만난 버그 2개

좋은 pre-grasp 자세를 찾으면 `<keyframe>`에 `qpos`/`ctrl` 스냅샷으로 저장해 재현성을
확보한다. `mujoco.mj_resetDataKeyframe(model, data, key_id)` 한 번으로 그 상태를
그대로 복원할 수 있다.

!!! bug "실제로 만난 버그 · data.ctrl[None] = value가 배열 전체를 조용히 덮어씀"
    **증상**: 관통 테스트가 계속 "0.000mm PASS"만 찍음 — 에러는 없는데 결과가
    이상하게 완벽함.

    **원인**: 일부 관절은 actuator가 아예 없는데(예: range를 0으로 고정한 관절), 그
    관절의 actuator id를 찾는 조회가 `None`을 반환했다. numpy는 `arr[None]`을
    `arr[np.newaxis]`로 해석해서 **스칼라 값을 배열 전체에 broadcast로 대입**해버린다
    — 에러 없이.

    **해결**: `if aid is None: continue` 가드 추가. 이 버그는 이 프로젝트에서 **같은
    패턴으로 세 번** 재발했다 — 매번 "새 관절이 어떤 모델 변형(hand_only/arm_hand/
    full_scene)에는 actuator가 있고 어떤 데는 없다"는 걸 놓쳐서였다.

    **배울 점**: lookup 실패로 `None`/`-1`이 나올 수 있는 값을 numpy 배열 인덱스에
    그대로 쓰지 말 것. numpy는 죽지 않고 조용히 엉뚱한 곳에 값을 쓴다. 특히 이
    프로젝트처럼 **모델 변형이 여러 개** 있는 경우, "내가 지금 테스트한 모델에 있으니
    됐다"가 아니라 그 코드가 실행될 수 있는 모든 모델에 실제로 해당 actuator가
    있는지 확인해야 한다.

!!! bug "실제로 만난 버그 · range=\"0 0\"만으로는 관절이 안 잠긴다"
    **증상**: 약지·새끼 mcp 관절을 "0으로 잠갔다"고 문서화해뒀는데, 60초 방치
    시뮬레이션에서 그 관절이 0.66rad까지 서서히(단조 증가) 움직임.

    **원인**: `limited="true"`를 깜빡함. MuJoCo의 `autolimits`는 명시적인 `[0, 0]`과
    "range 미지정"을 구분하지 못해서, `limited`가 없으면 이 range 자체가 적용되지
    않는다. 즉 이 관절은 사실상 free였고, 미세한 중력 토크가 수십 초에 걸쳐 서서히
    밀어낸 것.

    **해결**: 유효한 미세 범위(`range="-0.0001 0.0001"`, MuJoCo는 `limited`에
    `range[0] < range[1]`을 요구해서 정확히 0-0은 안 된다) + `limited="true"` 명시.

    **배울 점**: "설계 의도가 XML/문서에 적혀 있다"는 것과 "실제로 그렇게 동작한다"는
    것은 다른 문제다. 컴파일된 `model.jnt_limited` 값을 직접 찍어보기 전까지는
    확신하지 말 것. 그리고 이런 버그는 **초 단위가 아니라 분 단위**의 시간 상수를
    가질 수 있으므로, 짧은 자동 테스트만으로는 절대 못 잡는다.

결과: ±5mm 랜덤 노이즈를 준 grasp+lift(10cm 들어올려 5초 유지) 10회 반복, **10/10
성공**(목표 8/10). 5지 전부를 쓰는 대신 계획서의 fallback대로 **엄지+검지+중지 3점
파지**로 단순화한 결정이 여기서 나온다 — 약지·새끼는 XML에서 range를 고정(파이썬에서
"안 움직이게 막기"가 아니라 물리 자체를 XML로 제한).

---

다음: [Phase 3 — 팔에 IK 붙이기](phase3.md)
