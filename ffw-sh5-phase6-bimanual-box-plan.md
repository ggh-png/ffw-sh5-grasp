# FFW-SH5 Phase 6 실행 계획/검토 결과

목표는 작은 상자를 양손으로 마주 눌러서 들고, 든 상태로 리프트/베이스 이동을
유지하는 것이다. 이번 검토에서 사용자 피드백을 반영해 제어 기준을 다시 확정했다.

중요 수정 사항:

- EE 제어는 화면 마커 드래그가 아니라 패널의 X/Y/Z 위치와 Roll/Pitch/Yaw 각도 제어가
  기준이다.
- 조작감은 ROBOTIS Cyclo Control의 `MoveL`/`Bimanual MoveL` marker UI 구조를 따른다.
  capture 전에는 `right_goal_marker`/`left_goal_marker`, capture 후에는
  `virtual_object_marker`를 3D 화살표/회전 링으로 움직여 양손 목표를 파생한다.
- 화면의 IK target 마커는 현재 숫자 target을 보여주는 표시용이다.
- box 시나리오에서 사용자가 jog 또는 X/Y/Z/RPY 값을 직접 바꾸면 자동 box tracking은 즉시 꺼져
  수동 target이 다음 프레임에 덮어써지지 않는다.
- 상자는 기존보다 작게 줄이고, 작업 위치는 검증 가능한 방향으로 10cm 뒤로 조정했다.

현재 확정값:

```python
BOX_HALF_EXTENTS = [0.10, 0.10, 0.14]   # 20 x 20 x 28 cm
BOX_HOME_QPOS    = [0.4055, 0.0, 0.8716, 1.0, 0.0, 0.0, 0.0]
BOX_MASS         = 0.2
SQUEEZE_KP_SCALE = 0.08
DRIFT_GAIN       = 0.2
```

`x=0.3055`에서 10cm를 어느 방향으로 볼지 실제 squeeze/lift/drive 테스트로 비교했고,
`x=0.2055`는 lift가 불안정해 제외했다. 최종값 `x=0.4055`는 squeeze, lift, 주행 유지
검증을 통과했다.

## Step 0. 상자 모델과 물리 게이트

상자는 `models/full_scene.xml`의 `box_body`로 추가되어 있고, can 시나리오와 box
시나리오는 실행 인자로 분리한다.

```bash
python3 src/teleop_app.py can
python3 src/teleop_app.py box
```

검증 기준:

- keyframe qpos 길이가 모델 `nq`와 일치해야 한다.
- box 홈 리셋 후 idle 상태에서 큰 드리프트나 폭주가 없어야 한다.
- box가 테이블에 안정적으로 놓여야 한다.
- can pick 및 mobile base 회귀가 없어야 한다.

상태: 완료.

## Step 1. 숫자 기반 EE 포즈 제어

양손 target은 UI 패널에서 다음 6축으로 직접 제어한다.

- position: X, Y, Z
- orientation: Roll, Pitch, Yaw

Roll/Pitch/Yaw는 각 손의 home pose 기준 상대 자세다. 수동 제어가 들어오면:

- 해당 target 값이 `app.targets["pos_l/r"]`, `app.targets["rpy_l/r"]`에 반영된다.
- IK가 이 target을 따라 팔 관절 목표를 계산한다.
- box 모드에서는 `box_tracking=False`가 되어 자동 정렬이 수동 입력을 덮지 않는다.
- IK target marker는 이 숫자 target을 따라가는 display-only indicator로 동기화된다.

마우스는 일반 MuJoCo 카메라 조작에 사용한다. Ctrl+drag 기반 target pose 조작은 현재
요구사항과 맞지 않아 제거했다.

상태: 완료.

## Step 1.5. Cyclo Control marker UI

슬라이더를 직접 끄는 방식은 target pose를 큰 폭으로 튕기기 쉽고, 작은 물체를 양손으로
맞추는 작업에서는 조작감이 나쁘다. 이를 보완하기 위해 `teleop_ui.py`에
`Cyclo Control` 패널을 추가했다.

동작:

- `MoveL`/`Bimanual MoveL` controller type을 고른다.
- capture 전에는 `right_goal_marker`/`left_goal_marker` 중 조작 대상을 고른다.
- `Bimanual MoveL`에서 `Capture Grasp (/capture_grasp true)`를 누르면 현재 양손 target의
  상대 transform을 `virtual_object_marker` 기준으로 기록한다.
- capture 후에는 `virtual_object_marker`를 움직이면 양손 target이 함께 파생된다.
- 3D 화면에는 선택된 marker의 X/Y/Z 이동 화살표와 Roll/Pitch/Yaw 회전 링을 ImGuizmo로
  띄운다. 사용자는 이 화살표/링을 직접 드래그해 pose를 제어한다.
- `Position step`과 `RPY step`을 고른다.
- 패널의 `X/Y/Z`와 `Roll/Pitch/Yaw` +/- 버튼은 같은 target을 해당 step만큼 증분 이동하는
  보조 입력이다.
- FK 모드인 손은 jog 대상에서 제외된다.
- box 모드에서 jog를 쓰면 자동 box tracking이 꺼져 수동 target이 유지된다.

테스트:

- `Cyclo marker jog gate`가 양손 jog, FK 손 제외, target clamp, box tracking 해제를 검증한다.
- `Cyclo bimanual virtual object gate`가 capture/release 및 virtual object pose에서 양손
  target이 파생되는지 검증한다.
- `Cyclo 3D gizmo pose gate`가 ImGuizmo matrix와 hand/virtual object target 변환을 검증한다.

상태: 완료.

## Step 2. Box 자동 정렬과 squeeze

box 시나리오에서는 `[Grab]`과 squeeze gap이 양손을 상자 양옆으로 대칭 배치한다.
자동 정렬은 수동 제어를 대신하는 기능이 아니라, 초기 접근을 빠르게 맞추는 보조 기능이다.

핵심 동작:

- `Auto-align XYZ targets to box`가 켜져 있으면 상자 pose 기준으로 좌우 손 target을
  계산한다.
- squeeze gap은 양손 target 간격을 줄여 접촉 압력을 만든다.
- `BOX_TARGET_SITE_TO_PALM_MARGIN=0.060`을 적용해 `grasp_target` site와 실제 손바닥
  collision surface 사이의 차이를 보정한다.
- 접촉 중 팔 stiffness가 너무 강해 box를 과하게 누르지 않도록 `box_squeeze_kp_scale`
  기본값은 `0.08`이다.

상태: 완료.

## Step 3. Box held 판정

상자 파지는 can 전용 `is_grasped()`와 분리해 box 전용 접촉 판정을 사용한다.

기준:

- 왼손과 오른손이 모두 `box_geom`에 접촉해야 한다.
- 각 손의 접촉력이 임계값 이상이어야 한다.
- 한쪽만 닿은 상태는 held가 아니라 pushing/contact로 본다.

UI에는 양손 접촉력과 held 상태를 표시한다.

상태: 완료.

## Step 4. Bimanual constraint

상자를 들고 움직일 때 핵심은 두 손의 상대 pose를 유지하는 것이다. 이를 위해
`src/bimanual_constraint.py`에서 양팔 joint delta를 grasp manifold에 투영한다.

적용 방식:

- box held가 성립하면 현재 양손의 상대 pose를 snapshot으로 기록한다.
- 이후 양손 IK 결과의 joint delta를 결합해 projection을 수행한다.
- 수치 drift를 줄이기 위해 drift correction을 함께 적용한다.
- squeeze와 constraint가 서로 싸우지 않도록 held 이후에는 gap을 lock한다.

현재 튜닝:

```python
drift_gain = 0.2
```

기존 `0.8`은 접촉이 많은 box lift에서 과하게 보정되어 lift/drive 안정성을 해쳤다.

상태: 완료.

## Step 5. 들기와 이동

검증해야 하는 실제 시나리오는 다음 순서다.

1. box 시나리오로 시작한다.
2. 양손을 box 양옆 pregrasp target으로 보낸다.
3. squeeze로 양손 접촉력을 만든다.
4. held 판정 이후 bimanual constraint를 활성화한다.
5. Z target 또는 lift 조작으로 box를 들어 올린다.
6. 든 상태에서 mobile base를 이동해도 box가 손 사이에 유지되는지 본다.

현재 headless 테스트에서 통과한 대표 수치:

- squeeze force: 좌우 약 5.3N
- lift: 약 153mm 상승, 상대 drift 약 1.1mm
- drive while held: base 약 0.74m 이동, 상대 drift 약 2.1mm
- manual XYZ/RPY IK: 위치 오차 0.01mm 수준, 자세 오차 0.05deg 수준

상태: 완료.

## Step 6. 통합 테스트

`tests/test_phase_6.py`는 이제 단순 groundwork가 아니라 Phase 6 전체 게이트다.

포함된 검증:

- model/keyframe gate
- box idle/drop
- bimanual constraint projection residual
- box pregrasp IK
- squeeze stability
- scripted lift
- drive while held
- manual pose edit gate
- Cyclo marker jog gate: 양손 marker jog, FK 제외, clamp, box tracking 해제 검증
- Cyclo bimanual virtual object gate: capture 후 virtual object 목표에서 양손 목표 파생 검증
- Cyclo 3D gizmo pose gate: 3D 화살표/회전 링 pose matrix와 target 변환 검증
- display marker gate: 마우스 target 제어 경로가 다시 들어오지 않았는지 확인
- numeric target -> marker sync gate: X/Y/Z/RPY 숫자 target이 표시용 marker pose를 결정하는지 확인
- box auto-align target math gate: 자동 box 정렬 target 수식 검증
- manual XYZ/RPY IK gate

회귀 확인 대상:

```bash
python3 tests/test_phase_4.py
python3 tests/test_phase_5.py
python3 tests/test_phase_6.py
python3 -m py_compile src/teleop_app.py src/teleop_ui.py src/bimanual_constraint.py tests/test_phase_6.py
mkdocs build --strict
```

상태: 완료.

## 변경 파일

주요 변경 파일:

- `models/full_scene.xml`: 작은 box 크기/위치/질량 조정
- `src/teleop_app.py`: box tracking, squeeze, display-only target marker, constraint 연동
- `src/teleop_ui.py`: 숫자 X/Y/Z/RPY 수동 제어 우선, 수동 편집 시 box tracking 해제
- `src/bimanual_constraint.py`: drift gain 튜닝
- `tests/test_phase_6.py`: Phase 6 통합 게이트 확장
- `README.md`, `docs/*`: Phase 6 및 숫자 기반 제어 문서화

주의:

- `src/imgui.ini`는 런타임 UI 레이아웃 파일이라 기능 변경의 핵심 파일은 아니다.
- 이 계획 파일은 원래 untracked 상태였고, 현재 내용은 실제 구현/검증 결과에 맞춰 다시
  정리한 것이다.

## 남은 점검 메모

현재 코드 기준으로 큰 기능 누락은 보이지 않는다. 다음 점검의 우선순위는 실제 GUI에서
다음 두 가지를 눈으로 확인하는 것이다.

- box 모드에서 X/Y/Z와 Roll/Pitch/Yaw 슬라이더를 움직였을 때 양손 target marker가
  정확히 따라가고, box tracking이 다시 덮어쓰지 않는지
- 실제 렌더 화면에서 줄인 box 크기와 `x=0.4055` 위치가 조작감상 자연스러운지
