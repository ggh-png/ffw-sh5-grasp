[← 전체 안내](../ros2-guide.md)

# Part 5 — 손 제어: `src/grasp.py` {: #part-5 }

!!! info "함께 볼 개발자 가이드"
    함수별 입력·출력과 접촉 판정 조건은 [`grasp.py` 개발자 가이드](../grasp.md)에서
    실제 호출 관점으로 확인할 수 있다.

## 기능 구현 요약

| 구분 | 내용 |
|---|---|
| 해결할 문제 | 많은 손가락 관절을 소수의 입력으로 움직이고, 물체를 실제 접촉력으로 잡았는지 판정해야 한다. |
| 해결 방법 | `grasp`, `thumb` 두 synergy 값을 관절 범위에 보간하고, 캔과 닿은 손가락별 normal force를 합산해 grasp 조건을 검사한다. |
| 사용 수식 | 먼저 \(f=f_{open}+s(1-f_{open})\)를 만든다. open이 range의 lo이면 \(q=lo+f(hi-lo)\), hi이면 \(q=hi-f(hi-lo)\)로 미러링한다. 총 접촉력은 \(F_{total}=\sum_iF_i\)다. 각 항과 판정 조건은 5.2와 5.3에서 설명한다. |
| 코드 구현 과정 | `apply_grasp()` → `_set_joint_fraction()` → `_ramp_value()` → `_set_joint_ctrl()` 순서로 actuator 목표를 기록한다. 판정은 `get_finger_can_contacts()`가 힘을 모으고 `is_grasped()`가 손가락 수·총 힘·엄지 조건을 확인한다. |
| 수식 없이 사용하는 함수 | `_validate_side()`는 좌우 손 입력을 검증하고, `_resolve_joint_actuator()`는 `mj_util.find_actuator_for_joint()`로 joint와 actuator 연결을 찾고 캐시한다. `mujoco.mj_contactForce()`는 접촉력 원시값을 읽는다. |

## 5.1 grasp synergy란 무엇인가 {: #part-5-1 }

실제 로봇 손(HX5-D20)은 손가락 마디마다 관절이 있지만(엄지 4개, 검지/중지/약지/
새끼 각 3~4개), 텔레옵에서 사람이 그 관절 하나하나를 슬라이더로 조작하는 건
비현실적이다. 그래서 **"synergy"**(스칼라 하나가 여러 관절을 동시에 움직이는
매핑)를 쓴다 — ROS2의 `control_msgs/action/GripperCommand`가 "그리퍼를 얼마나
닫을지" 스칼라 하나(`position`)만 받는 것과 똑같은 아이디어다. 이 프로젝트는
스칼라 2개를 쓴다:

| 슬라이더 | 매핑 대상 |
|---|---|
| `grasp` (0~1) | 검지+중지의 pip/dip/tip 관절 전부 동시에 |
| `thumb` (0~1) | 엄지의 mcp_pitch/ip 관절 + 엄지 MCP-yaw(방향) |

## 5.2 관절 지도와 상수 {: #part-5-2 }

```python
FINGER_CURL_JOINTS = {
    "r": {
        "index": ("finger_r_joint6", "finger_r_joint7", "finger_r_joint8"),
        "middle": ("finger_r_joint10", "finger_r_joint11", "finger_r_joint12"),
    }, ...
}
THUMB_CURL_JOINTS = {"r": ("finger_r_joint3", "finger_r_joint4"), ...}
FINGER_OPEN_FRAC = 0.20   # grasp=0이어도 range 전체를 펴지 않고 20% 남겨둠
RING_PINKY_MAX_FRAC = 0.35  # 약지/새끼는 판정 대상이 아니며 35%까지 보조 명령
```

성공 판정은 엄지·검지·중지 body group만 집계한다. 약지·새끼는 최대 35%의
보조 동작 명령을 받고 물리적으로 접촉할 수 있지만, `is_grasped()`의 손가락 수와
접촉력 합에는 포함되지 않는다. MCP(스프레드) 관절은 XML에서 `range="0 0"`으로
잠겨 있다. 이건 튜닝 실패가 아니라 처음부터 제시된 fallback이다:
"5지 전부"에 집착하지 않고 3점 파지로 단순화하는 게 "정직한 물리 파지"라는
목표에 더 맞다는 판단.

### 관절 매핑 공식

**이 공식이 왜 필요한가**: 슬라이더/그리퍼 명령은 스칼라 \(s\in[0,1]\) 하나뿐인데,
실제로 액추에이터에 넣어야 하는 값은 그 관절의 절대 각도(라디안)다. 이 둘을 잇는
가장 단순한 방법은 "0=완전히 편 상태(`lo`), 1=완전히 굽힌 상태(`hi`)"로 range 전체를
그대로 선형 보간하는 것이다. 하지만 이 프로젝트는 실제로 겪은 문제 때문에 그보다
한 단계 더 들어간 식을 쓴다: `hand_only.xml`에는 아직 테이블이 없어서 캔이
자유낙하하는데, 손가락 액추에이터는 일부러 힘을 약하게(force-limited) 설정해뒀다
— 그래야 접촉하는 순간 토크가 포화돼서 손가락이 캔 형상에 순응(compliant)하며
감기기 때문이다(Part 2.6). 그런데 range 전체(`lo`~`hi`, 완전히 편 상태부터)를 다
써서 닫으면, 그 약한 액추에이터가 손가락을 다 오므리기도 전에 캔이 이미 낙하해서
손 밖으로 벗어나 버렸다. 그래서 "완전히 편 상태여도 이미 캔 표면 근처까지 살짝
오므려둔" 시작점을 만들고, 그 지점부터만 나머지 구간을 보간한다.

이 시작점을 정하는 여유 비율이 `open_frac`(검지/중지는 `FINGER_OPEN_FRAC=0.20`,
엄지 curl은 `THUMB_OPEN_FRAC=0.0`)이고, 실제 보간 비율은 여기서부터 \(s\)에 비례해
1.0까지 선형으로 올라간다(\(s=0\)일 때 \(\text{frac}=\text{open\_frac}\), \(s=1\)일
때 \(\text{frac}=1\)이 되도록 나머지 구간 \((1-\text{open\_frac})\)을 \(s\)로
스케일):

\[
\text{frac} = \text{open\_frac} + s\,(1 - \text{open\_frac})
\]

그다음 이 비율로 range를 선형 보간해서 실제 목표각을 구한다. 여기서 두 갈래로
나뉘는 이유는 순전히 XML 관절 정의 때문이다: 대부분의 관절은 `lo`가 "편 상태",
`hi`가 "굽힌 상태"이지만, 왼손 엄지(`finger_l_joint3/4`)는 오른손을 기하학적으로
거울에 비춘 모델이라 range 자체의 부호가 뒤집혀 있어서 `hi`가 "편 상태"다(Part
13 사례 1 — 이 차이를 놓쳤을 때 실제로 자가충돌 버그가 났다). 그래서 어느 쪽이
"편 상태"인지를 `open_at_hi` 플래그로 나눠 보간 방향 자체를 뒤집는다:

\[
\theta =
\begin{cases}
lo + \text{frac}\,(hi - lo) & \text{open\_at\_hi = False (검지/중지, 오른손 엄지 등 대부분)} \\
hi - \text{frac}\,(hi - lo) & \text{open\_at\_hi = True (왼손 엄지 -- range 부호 자체가 미러링됨)}
\end{cases}
\]

이 두 식이 `grasp.py`의 `_ramp_value(lo, hi, frac, open_at_hi)` 함수 그 자체다.

약지/새끼는 위와 다른, 더 단순한 식을 쓴다 — 이유는 이 두 손가락이 애초에
"자유낙하하는 캔을 놓치지 않게 미리 오므려둘" 필요 자체가 없기 때문이다(3점
파지에 참여하지 않으므로, 이 프로젝트 설계 초기부터의 fallback 결정). 그래서 `open_frac`으로
시작점을 당겨줄 이유가 없고, 대신 순전히 보기 좋으라고(다른 손가락이 다 오므릴 때
이 둘만 뻣뻣하게 펴진 채로 있으면 어색해 보여서) `grasp` 스칼라에 비례해 자기
range의 일부(`RING_PINKY_MAX_FRAC=0.35`)까지만 코스메틱하게 움직인다 — 이 상한값
자체도 임의로 고른 게 아니라 0.20~0.60 구간을 스윕해서 0.40/0.45 사이에서 pick
성공률이 10/10→0/10으로 무너지는 절벽을 찾고, 그보다 충분히 낮은 값을 고른
것이다(Part 13에서 다루는 "절벽 근처 대신 여유를 둔 값을 고른다" 패턴):

\[
\theta_{\text{ring/pinky}} = lo + \big(s \cdot \text{RING\_PINKY\_MAX\_FRAC}\big)\,(hi - lo)
\]

## 5.3 접촉력 기반 판정 — `is_grasped` {: #part-5-3 }

이 프로젝트에서 "쥐었다"의 정의는 **위치가 아니라 순전히 접촉력**이다:

```python
def is_grasped(model, data, min_fingers=2, min_total_force=0.05,
               require_thumb=True, side="r"):
    forces = get_finger_can_contacts(model, data, side=side)
    if require_thumb and "thumb" not in forces:
        return False
    if len(forces) < min_fingers:
        return False
    return sum(forces.values()) >= min_total_force
```

`get_finger_can_contacts`는 이번 스텝에 발생한 모든 접촉(`data.contact`)을
순회하며, 캔과 맞닿은 접촉만 골라 `mj_contactForce()`로 법선력을 읽어 손가락
그룹별로 합산한다. ROS2로 치면 실제 F/T(force-torque) 센서 토픽을 구독해서
"그리퍼가 물체를 쥐었는지" 판정하는 노드와 같은 역할이다 — 다만 진짜 센서가
아니라 물리 엔진이 계산한 접촉력을 직접 읽는다는 차이가 있을 뿐, **"위치가
가까우면 쥔 걸로 친다" 같은 치팅은 하지 않는다.**

## 5.4 사례 연구 — 엄지 프리쉐입 버그 {: #part-5-4 }

이 버그는 "고정값처럼 보이는 상수가 사실은 안전하지 않았다"는, 시뮬레이션
디버깅에서 자주 나오는 패턴을 잘 보여준다.

- **증상**: 엄지가 옆으로 벌어지는 대신 손바닥 쪽으로 접히길 원해서, MCP-yaw
  각도를 `-1.309 → -2.0326 rad`로 넓혔다.
- **회귀**: 그런데 `tests/test_phase_4.py`의 통합 pick 성공률이 80~90%에서
  20%로 급락했다.
- **원인**: 이 각도가 `THUMB_PRESHAPE` 안의 **완전 고정값**이라, `thumb=0`인
  **pregrasp 접근 단계**(팔이 아직 캔 쪽으로 스윙하는 중)에도 그대로 적용됐다.
  넓어진 각도 때문에 엄지가 스윙 도중 캔을 미리 쳐서 최대 85mm까지 밀어냈다
  (계측: 접촉이 스윙의 47% 지점에서 시작). `hand_only.xml` 기반 테스트는 팔
  자체가 없어서 이 문제를 원천적으로 못 잡는다.
- **수정**: MCP-yaw를 고정값이 아니라 `thumb` 스칼라로 `THUMB_YAW_REST`(안전한
  기존 각도) → `THUMB_YAW_CURL`(넓은 새 각도) 사이를 **선형 램프**하게 바꿨다.
  `thumb=0`(접근 중)엔 안전한 각도, `thumb`이 실제로 올라갈 때(팔이 이미 멈춘
  뒤)만 넓은 각도로 전환된다.

\[
\text{yaw}(\text{thumb}) = \text{yaw}_{\text{rest}} + \text{thumb}\,\big(\text{yaw}_{\text{curl}} - \text{yaw}_{\text{rest}}\big)
\]

코드로는 그대로:

```python
yaw_value = THUMB_YAW_REST[side] + thumb * (THUMB_YAW_CURL[side] - THUMB_YAW_REST[side])
```

`thumb=0`이면 항상 안전한 `yaw_rest`, `thumb=1`이면 사용자가 확인한 `yaw_curl`이 되고
그 사이는 선형 보간된다 — 5.2의 관절 매핑 공식과 똑같은 선형 램프이지만, 대상이
`grasp`/`thumb` 곡선 자체가 아니라 "커브가 어느 각도에서 어느 각도로 이어지는가"라는
점이 다르다.

**교훈**(일반화): "이 값은 스칼라와 무관하게 항상 적용된다"는 설계는, 그
스칼라가 취할 수 있는 **모든** 상태(여기서는 `thumb=0`인 접근 단계까지 포함)에서
안전할 때만 유효하다. 값을 하나 바꿀 때는 "지금 보고 있는 상태"뿐 아니라 그
값이 적용되는 **모든** 호출 시점을 따져봐야 한다.

---

[← Part 4](./04-runtime-architecture.md) · [전체 안내](../ros2-guide.md) · [Part 6 →](./06-inverse-kinematics.md)
