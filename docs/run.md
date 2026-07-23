# 화면과 조작

앱을 설치하고 처음 실행하는 단계는 [10분 빠른 시작](getting-started.md)에 있다. 이
문서는 실행 중 어떤 입력이 어떤 상태를 바꾸는지 찾아보는 운영 매뉴얼이다.

## 화면 구성

```text
┌────────────── MuJoCo main ──────────────┐  ┌──── Control Center ────┐
│ 3D scene · target marker · gizmo        │  │ Target | R Arm | L Arm │
│ ┌ Status & Windows ┐                    │  │       | Robot/Grasp     │
│ └──────────────────┘                    │  └─────────────────────────┘
└─────────────────────────────────────────┘  ┌──── Diagnostics ────────┐
                                             │ Kinematic Tree | Joints │
                                             └─────────────────────────┘
```

두 워크스페이스는 별도 OS 창으로 메인 3D 창 밖에 뜨며 drag/resize/close할 수 있다.
닫은 창은 **Status & Windows → Workspaces**에서 다시 연다. **Detach tools outside**와
**Return tools to main**으로
외부 창 배치와 주 창 내부 배치를 즉시 전환할 수 있다.

### Kinematic Tree

기본값은 양손 IK target에 실제로 연결된 body chain만 표시한다. `Right`, `Left`,
`Both arms`로 범위를 바꾸거나 **Show full MJCF tree**로 전체 body/joint/site 구조를
볼 수 있다. `[controlled]`는 Whole-Body IK가 푸는 관절, `[IK target]`은 손끝 site다.

## 키보드와 마우스

| 입력 | 기능 | 입력 방식 |
|---|---|---|
| Mouse left drag | 카메라 orbit | 연속 |
| Mouse right drag | 카메라 pan | 연속 |
| Mouse wheel | 카메라 zoom | 연속 |
| `Up` / `Down` | base 전진 / 후진 | 누르는 동안 |
| `Left` / `Right` | base yaw 좌 / 우 | 누르는 동안 |
| `[` / `]` | base strafe 좌 / 우 | 누르는 동안 |
| `Q` / `E` | lift 하강 / 상승 | 누르는 동안 |
| `R` | can reset | 누른 순간 한 번 |
| `G` | physical contact point/force 표시 | toggle |
| `V` | collision geometry/CBF 최근접점 표시 | toggle |
| `C` | overview / right-hand camera 전환 | 누른 순간 한 번 |

!!! warning "키보드 focus"
    ImGui slider나 text field가 키보드를 잡고 있으면 drive key가 로봇에 전달되지 않는다.
    3D scene을 클릭해 focus를 돌린 뒤 다시 시도한다.

## Status 읽는 법

상태줄은 “사용자가 무엇을 요청했나”보다 “controller가 지금 실제로 무엇을 내보내나”를
확인하는 곳이다.

| 표시 | 의미 |
|---|---|
| `CAN | movel/bimanual_movel` | 현재 Cyclo-style target controller |
| `marker` | jog/gizmo가 조작할 target |
| `sim`, `wall`, `Hz` | simulation 시간, 실제 시간, UI loop 주파수 |
| `IK err L/R` | 실제 손과 target의 위치 오차; FK 팔은 `FK` 표시 |
| `Base x/y/yaw` | 실제 MuJoCo base pose |
| `Whole-body IK ON/OFF` | 자동 IK의 base/lift 참여 상태 |
| `body cmd vx/vy/wz` | swerve에 전달되는 최종 body-frame 명령 |
| `Collision CBF viz` | active pair, 최소 거리, soft slack |

`body cmd`가 zero인데 base가 잠깐 움직이는 것은 물리 제동일 수 있다. 반대로 command가
계속 nonzero면 입력 우선순위나 WBIK target을 확인한다.

## Control Center → Target

### Controller 선택

| 선택 | target 편집 방식 |
|---|---|
| MoveL | 오른손/왼손을 독립적으로 선택하고 이동 |
| Bimanual MoveL | Capture 뒤 virtual object를 움직여 양손을 함께 이동 |

모드 조합의 전체 의미는 [모드 선택](control-modes.md)에 있다.

`Move time` slider는 현재 Cyclo UI 호환 상태값만 보관하며 trajectory duration을
재계산하지 않는다. 실제 목표 응답 속도는 frame target rate limit과 controller gain이
결정한다.

### Marker 선택

- MoveL: `Right goal` 또는 `Left goal` 선택
- Captured Bimanual MoveL: `Virtual object`만 조작

### Jog

- Position step 기본값: 0.005 m
- Rotation step 기본값: 2°
- `X-/X+`, `Y-/Y+`, `Z-/Z+`를 누르고 있으면 반복 입력
- target 범위는 손 position offset ±0.35 m, RPY ±90°로 clamp

### 3D gizmo

marker에서 translation 화살표 또는 rotation ring을 drag한다. gizmo는 world pose를
출력하고 `teleop_targets.py`가 현재 Whole-body mode의 target 좌표계로 역변환한다.

## Control Center → Right Arm / Left Arm

각 팔은 독립적으로 IK와 FK를 선택할 수 있다.

=== "IK pose"

    - Position X/Y/Z는 home에서의 offset
    - Roll/Pitch/Yaw는 home orientation에 대한 local delta
    - slider, jog, gizmo가 같은 target 값을 갱신
    - Whole-body solver의 active arm으로 참여

=== "FK joints"

    - J1~J7 joint angle을 degree로 직접 지정
    - 해당 팔은 Cartesian solver에서 제외
    - 상태줄 IK error 대신 `FK` 표시

전환 순간 목표 점프를 막기 위해 IK→FK는 현재 관절 목표를 복사하고, FK→IK는 현재
실제 손 pose에서 Cartesian target을 다시 계산한다.

## Can Grasp

오른손과 왼손 각각:

- `Grab`/`Release`: grasp와 thumb target을 시간에 따라 ramp
- grasp slider: 검지/중지의 주 파지 curl과 약지/새끼의 작은 cosmetic curl
- thumb slider: 엄지 preshape/flexion synergy

캔은 손에 강제로 붙지 않는다. finger-can contact force가 임계값을 만족하고 실제
마찰로 물체가 따라와야 한다.

## Lift / Utilities

### Whole-body Control

| 버튼 상태 | 자동 IK | 수동 입력 |
|---|---|---|
| ON | base 3축 + lift + IK 팔 | keyboard base가 우선, lift slider/Q/E 가능 |
| OFF (arm-only) | IK 팔만 | keyboard base와 lift slider/Q/E 가능 |

전환은 world target 보존, smoothing 동기화, solver rebase, cached base twist zero를 한
동작으로 수행한다.

### Lift target

범위는 -0.5~0.0 m다. `Q/E`와 slider는 같은 target을 갱신한다.

- ON: lift target은 WBIK nominal/posture와 함께 solve에 들어간다.
- OFF: WBIK lift velocity는 0이지만 slider target은 독립 lift actuator command로 적용된다.

### Reset / Visualization / Camera

| 버튼 | 키 | 동작 |
|---|---|---|
| Reset Can | `R` | can을 home 근처 ±2 cm로 재배치하고 grasp/Cyclo capture reset |
| Contact Viz | `G` | 실제 MuJoCo contact point/force 표시 |
| Collision CBF Viz | `V` | monitored geometry와 거리 constraint 표시 |
| Camera | `C` | camera preset 전환 |

## Collision 색상

| 색 | signed distance | 해석 |
|---|---:|---|
| 노랑 | 1~3 cm | monitoring buffer 안, safe distance 밖 |
| 주황 | 0~1 cm | safe distance 안, 분리 CBF 적극 적용 |
| 빨강 | <0 cm | geometry 관통 |

상태줄의 `slack`은 모든 요구를 동시에 만족하기 어려울 때 남은 최대 CBF 위반 속도다.
0에 가까울수록 제약을 잘 만족한 것이다.

## 추천 작업 절차

### 한 손 target 확인

1. MoveL
2. 해당 팔 IK
3. Whole-body OFF
4. 작은 XYZ jog
5. IK error와 `V` 확인

### 양손 전신 이동

1. 양손 IK
2. Bimanual MoveL
3. Capture Grasp
4. Whole-body ON
5. virtual object를 작은 step으로 이동
6. base command, 양손 error, collision line 확인

### 수동 재배치 후 자동 제어 재개

1. arrow/`[/]`로 base 이동
2. 키를 놓고 물리 제동 대기
3. body cmd와 실제 속도가 zero인지 확인
4. 새 손 target 입력
5. WBIK가 현재 위치에서 이어지는지 확인

## 테스트와 문서 빌드

```bash
python tests/test_phase_6.py
python tests/test_whole_body.py
mkdocs build --strict
```

전체 suite와 출력 해석은 [테스트와 검증](testing.md), 증상별 진단은
[문제 해결](troubleshooting.md)에 있다.
