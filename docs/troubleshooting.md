# 문제 해결

증상에서 시작해 확인 순서, 정상 기준, 관련 테스트를 연결한다. 먼저 저장소 루트에서
명령을 실행하고 있는지와 현재 branch가 맞는지 확인한다.

```bash
git status --short --branch
python -c "import mujoco, numpy, glfw; from imgui_bundle import imgui; print('imports OK')"
```

## 빠른 진단표

| 증상 | 가장 먼저 확인 | 관련 테스트 |
|---|---|---|
| 창이 안 열림 | display/GLFW/X11과 Python import | `test_phase_6.py`는 headless 분리 진단 |
| 손이 안 움직임 | 팔이 FK인지, marker 선택, collision line | `test_phase_6.py`, `test_whole_body.py` |
| base가 자동으로 안 움직임 | Whole-body OFF 여부, target 크기 | `test_whole_body.py` |
| 키를 놓아도 바퀴가 계속 돎 | 0.5초 뒤에도 지속하는지 | `test_phase_5.py`, `test_whole_body.py` |
| 키를 놓으면 원래 위치로 돌아감 | manual handover/rebase 회귀 | `test_whole_body.py` |
| collision 기준이 답답함 | `V`, active pair와 최소 거리 | `test_whole_body.py` |
| Bimanual marker가 안 보임 | Capture Grasp 상태 | `test_phase_6.py` |
| RPY 축이 예상과 다름 | home-relative RPY와 Euler coupling | `test_phase_6.py` |
| 동작이 느림 | frame rate, collision, 도달 불가 target | latency/physical WBIK gate |

## 창이 열리지 않는다 {: #window-startup }

### `ModuleNotFoundError`

가상환경을 활성화하고 runtime 패키지를 다시 확인한다.

```bash
source .venv/bin/activate
python -m pip install mujoco numpy glfw imgui-bundle
python -c "import mujoco, glfw; from imgui_bundle import imgui"
```

### GLFW 초기화 또는 OpenGL context 오류

앱은 PyOpenGL/imgui-bundle과 일관된 context를 얻기 위해 GLFW의 X11 platform hint를
사용한다. 다음을 확인한다.

```bash
printf '%s\n' "$DISPLAY"
printf '%s\n' "$XDG_SESSION_TYPE"
```

- 원격 shell이면 X forwarding 또는 실제 데스크톱 세션이 필요하다.
- Wayland 세션에서도 XWayland가 설치되어 있어야 한다.
- headless 서버에서는 GUI 대신 테스트만 실행할 수 있다.

```bash
python tests/test_phase_6.py
python tests/test_whole_body.py
```

headless 테스트가 PASS인데 앱만 실패하면 IK/모델 문제가 아니라 window/context 문제로
범위를 좁힐 수 있다.

## 손이 marker를 따라가지 않는다

순서대로 확인한다.

1. 상태줄의 controller와 selected marker를 확인한다.
2. 해당 Arm panel이 `IK pose`인지 확인한다. FK면 Cartesian target을 풀지 않는다.
3. Bimanual MoveL이면 Capture Grasp 이후 virtual object를 움직이고 있는지 확인한다.
4. `V`를 켜 active collision line이 목표 방향을 막는지 확인한다.
5. target offset이 팔 가동범위를 벗어나지 않았는지 확인한다.

marker는 target이고 손은 실제 물리 state다. 큰 target은 rate limit과 torque를 거쳐
따라가므로 순간적으로 떨어져 보일 수 있다. IK error가 지속적으로 줄어들면 정상이다.

## Whole-body ON인데 base가 움직이지 않는다

- 한 손만 작게 움직였거나 오차가 작은 경우 팔이 대부분 흡수할 수 있다.
- base command는 손 위치 오차 8 cm, 자세 오차 0.25 rad 안에서 점차 fade한다.
- 수동 키를 막 놓았다면 차체 feedback이 정지를 확인할 때까지 WBIK가 대기한다.
- collision CBF가 base 접근 방향을 제한할 수 있다.
- 상태줄에서 실제 `body cmd vx/vy/wz`를 확인한다.

양손 target을 같은 방향으로 12 cm 정도 이동하는 headless gate는 base와 lift, 양팔이
모두 사용되는지 검사한다.

```bash
python tests/test_whole_body.py
```

출력의 `Whole-body solver gate`와 `Physical WBIK longitudinal/lateral/yaw`를 본다.

## Whole-body OFF인데 base 또는 lift가 움직인다

OFF가 막는 것은 **IK에서 나온 자동 base/lift 명령**이다.

- 화살표, `[/]`를 누르면 base는 수동으로 움직인다.
- `Q/E`나 lift slider를 바꾸면 lift는 수동으로 움직인다.
- 물리 제동 중에는 이전 운동량으로 잠깐 움직일 수 있다.

아무 입력도 없고 제동 시간이 지난 뒤에도 자동 명령이 남는다면 상태줄의 body cmd를
확인한다. 정상 OFF에서는 `vx=0`, `vy=0`, `wz=0`이다. 자동 gate는 다음 줄을 출력한다.

```text
Arm-only solver gate: body_qdot_zero=True twist_zero=True ...: OK
```

## 키를 놓아도 바퀴가 계속 돈다 {: #wheel-keeps-rolling }

정상과 버그를 시간을 기준으로 구분한다.

| 관찰 | 판정 |
|---|---|
| 키 해제 직후 짧은 제동 | 정상 물리 반응 |
| 차체 약 0.20초, 바퀴 약 0.32초 내 정지 | 현재 회귀 기준 정상 |
| 0.5초 이상 wheel velocity가 유지 | 비정상 가능성 |
| 멈춘 뒤 원래 위치 쪽으로 수 cm 이상 복귀 | handover/rebase 문제 가능성 |

```bash
python tests/test_phase_5.py
python tests/test_whole_body.py
```

`Manual release physical gate`에서 `base_stop`, `wheel_stop`, `return` 값을 확인한다.
현재 기준은 모두 0.5초 미만, 역방향 복귀 5 mm 미만이다.

## 키를 놓으면 차체가 원래 위치로 돌아간다

수동 주행 전의 WBIK reference가 남아 있으면 수동 이동을 새 task error로 해석할 수
있다. 현재 구현은 다음 순서로 막는다.

1. 수동 주행 중 target frame을 실제 base SE(2)만큼 운반
2. 키 해제 뒤 zero command로 물리 제동
3. 정지 handover에서 현재 target으로 solver `rebase()`
4. 이후 새 target이 생길 때만 WBIK 재개

`test_whole_body.py`의 `Manual handover gate`와 `Manual release physical gate`가 이
경로를 독립/물리 수준에서 모두 검사한다.

## Collision avoidance가 너무 빡빡하거나 느리다

현재 기본값은 다음과 같다.

| 값 | 기본값 | 의미 |
|---|---:|---|
| monitoring buffer | 3 cm | 이 거리 안의 pair를 solver가 평가 |
| safe distance | 1 cm | 안쪽 접근을 강하게 제한하고 분리 속도 요구 |
| visualization | OFF | `V` 또는 checkbox로 켬 |

먼저 `V`를 켜 어느 pair가 활성인지 확인한다. 노란 선만 보이면 아직 안전거리 밖이며,
주황/빨강은 목표가 너무 가깝거나 관통했음을 뜻한다. Reactive controller는 우회 경로를
만들지 않으므로 target을 안전한 중간점으로 나누는 것이 우선이다.

기준을 코드에서 바꾸려면 `WholeBodyIK`의 `collision_buffer`가
`collision_safe_distance`보다 커야 한다. 단순히 0으로 끄면 self/table 회귀가 보장하던
안전층이 사라진다. 변경 뒤 반드시 `test_whole_body.py`를 실행한다.

## RPY가 예상한 축으로 돌지 않는다

RPY slider는 world 절대 Euler angle이 아니라 손의 home orientation에 곱하는 local
delta다. 0/0/0이 시작 자세다. 한 축씩 작은 값으로 움직일 때는 손 local Roll/Pitch/Yaw와
직관적으로 맞지만, 여러 Euler angle을 동시에 크게 주면 회전 순서에 따른 coupling이
생긴다. 이는 3-parameter Euler 표현의 성질이다.

큰 회전은 다음을 권장한다.

- 한 번에 한 축씩 작은 step으로 jog
- 3D rotation gizmo 사용
- 중간 pose를 나눠 이동

## Bimanual virtual marker가 보이지 않는다

virtual marker는 다음 조건에서만 보인다.

```text
controller == Bimanual MoveL
and Capture Grasp가 활성
```

`Capture Grasp`를 누르면 현재 양손 target의 상대 transform을 저장하고 virtual marker를
표시한다. `Release Grasp` 뒤 숨는 것은 정상이다.

## 테스트 하나가 실패한다

전체를 반복하기 전에 실패 영역만 실행한다.

| 실패 영역 | 명령 |
|---|---|
| 원본 모델/중력 안정성 | `python tests/test_phase_0.py` |
| 손가락 collision | `python tests/test_phase_1.py` |
| grasp/lift | `python tests/test_phase_2.py` |
| 단일 팔 FK/IK | `python tests/test_phase_3.py` |
| 전신 hold/pick | `python tests/test_phase_4.py` |
| keyboard/swerve/제동 | `python tests/test_phase_5.py` |
| UI target/Bimanual/toggle | `python tests/test_phase_6.py` |
| WBIK/collision/mobile 통합 | `python tests/test_whole_body.py` |

테스트 의미와 성공 기준은 [테스트와 검증](testing.md)에 정리되어 있다.
