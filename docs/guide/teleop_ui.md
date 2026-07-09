# `src/teleop_ui.py` — ImGui 슬라이더 패널

## 이 파일이 하는 일

공개 진입점은 `draw_panel(app)` 하나다. 내부 구현은 상태 요약, Cyclo/marker,
오른팔/왼팔, can grasp, 리프트/유틸, 관절 모니터 helper로 나뉜다.
`teleop_app.py`의 `TeleopApp` 인스턴스 전체(`app`)를 인자로 받아, 슬라이더/버튼
조작 결과를 `app`의 상태에 곧바로 되써넣는다.

## 구현: 왜 `app` 객체 전체를 받는가

```python title="src/teleop_ui.py — 모듈 docstring"
Split out of teleop_app.py because it's the one genuinely independent piece of that
file: this module only reads/writes the app's already-public state (`app.targets`,
`app.contact_viz`, ...) and never touches physics or the 3D render -- it doesn't need to
know how IK, grasp synergy, or mj_step work, only what a slider's current value is. This
module doesn't import teleop_app (avoids a circular import); it just duck-types on
whatever `app` object `draw_panel` is called with.
```

개별 값을 하나하나 인자로 받는 대신 `app` 자체를 넘기는 이유는 실용적이다 —
패널이 다루는 상태가 `targets`/`contact_viz`/`camera_preset`/`fk_q_deg`/`arm_mode`/
`ik_err_mm`/`monitor_qposadr` 등 십수 개라, 전부 개별 매개변수로 뽑으면 호출부
(`teleop_app.py`)와 함수 시그니처 양쪽이 더 지저분해진다. 대신 `teleop_ui.py`는
`teleop_app`을 import하지 않고 **duck typing**으로 `app.xxx` 속성에 접근한다 —
그래서 `teleop_app.py`가 `import teleop_ui`를 해도 순환 임포트가 생기지 않는다.

패널 상단에는 항상 보이는 상태 요약이 있다. Cyclo controller, 선택된 marker,
IK 오차, 베이스 pose, 주요 키 바인딩을
한 곳에 모아 라이브 조작 중 시선을 덜 움직이게 했다.

그 아래에는 `Cyclo / Marker Control`이 있다. ROBOTIS Cyclo Control 문서의 marker 기반
`MoveL`/`Bimanual MoveL` 흐름을 이 MuJoCo 텔레옵에 맞춘 영역이다. `MoveL`에서는
`Right goal`/`Left goal` marker가 각 손 목표이고, `Bimanual MoveL`에서
`Capture Grasp`를 누른 뒤에는 `virtual_object_marker`가 양손을 함께 움직이는 목표가 된다.
이 파란 virtual marker는 양팔 제어가 capture된 동안에만 보이고, MoveL 상태나
Release 이후에는 숨긴다.
선택된 marker에는 3D 화면 위에 transform gizmo가 뜬다. X/Y/Z 화살표를 드래그하면
위치 target이 바뀌고, Roll/Pitch/Yaw 회전 링을 드래그하면 자세 target이 바뀐다.
패널의 +/- 버튼은 같은 target을 작은 step으로 미세 조정하는 보조 입력이다.
FK 모드인 손은 jog 대상에서 제외된다.

```python title="src/teleop_ui.py — Cyclo marker jog"
def _apply_cartesian_jog(app, side, pos_delta=(0.0, 0.0, 0.0), rpy_delta=(0.0, 0.0, 0.0)):
    if side == "virtual":
        app.targets["virtual_object_pos"] += pos_delta
        app.targets["virtual_object_rpy"] += rpy_delta
        app.apply_virtual_object_target()
        return
    ...
    _note_manual_pose_edit(app)
```

`capture_grasp()`는 현재 양손 target pose를 `virtual_object_marker` 기준 상대 transform으로
기록한다. 이후 `virtual_object_pos/rpy`가 바뀌면 `apply_virtual_object_target()`이 그
상대 transform을 다시 펼쳐 양손 `pos_l/r`, `rpy_l/r` target을 만든다. 즉 ROS topic을
쓰지는 않지만, 의미상 Cyclo의 `/capture_grasp`와 `/virtual_object_goal_move`를 같은
상태 흐름으로 구현한 것이다.

3D 화살표/회전 링은 `teleop_render.py`의 `draw_transform_gizmo()`가 ImGuizmo를 이용해
MuJoCo 렌더 위 foreground draw list에 그린다. ImGuizmo가 돌려준 pose matrix는
`_set_gizmo_target_world_pose()`에서 다시 base-local X/Y/Z와 home-relative RPY target으로
변환된다.

손별 패널은 `Right Arm`/`Left Arm`으로 정리되어 있고, IK/FK 모드에 따라 다른
슬라이더 집합을 그린다:

```python title="src/teleop_ui.py — 손 패널의 모드 분기"
def _draw_arm_panel(app, targets, side):
    mode = app.arm_mode[side]
    imgui.text(f"Mode: {'IK pose' if mode == 'ik' else 'FK joints'}")
    imgui.same_line()
    if imgui.button(f"Switch to {'FK' if mode == 'ik' else 'IK'}##{side}mode"):
        app.set_arm_mode(side, "fk" if mode == "ik" else "ik")

    if mode == "ik":
        _draw_ik_pose_controls(app, targets, side)
    else:
        _draw_fk_joint_controls(app, side)
```

IK 포즈 슬라이더와 가상 오브젝트 슬라이더는 같은 `_draw_vector_sliders()` helper를
쓴다. 그래서 X/Y/Z, Roll/Pitch/Yaw 축 라벨과 clamp 범위가 한 곳에서 일관되게
처리된다.

```python title="src/teleop_ui.py — 반복 슬라이더 helper"
def _draw_vector_sliders(prefix, values, axes, lo, hi, fmt, on_change=None):
    for i, axis in enumerate(axes):
        changed, values[i] = _slider_float_clamped(...)
        if changed and on_change is not None:
            on_change()
```

모드 전환 버튼은 직접 `app.arm_mode[side]`를 바꾸지 않고 `app.set_arm_mode(side,
...)`를 호출한다 — 전환 순간 포즈가 튀지 않게 하는 동기화 로직(팔 목표 관절각을
재계산하는 것)은 `teleop_app.py`의 책임이라, UI는 "전환해라"라는 의도만 전달한다.
"Reset Can"/"Toggle Contact Viz"/"Cycle Camera" 버튼도 같은 패턴이다 —
`app.reset_active_object()`/`app.contact_viz = not app.contact_viz`/`app.cycle_camera()`처럼
직접 상태를 바꾸거나 `TeleopApp`의 메서드를 호출할 뿐, 그 메서드 안에서 실제로
무슨 일이 일어나는지는 이 파일이 몰라도 된다.

Can grasp 패널은 손별 `grasp`/`thumb` synergy와 grab/release 버튼만 다룬다.
양팔 동시 pose 제어는 별도 squeeze 패널이 아니라 `Bimanual MoveL` capture 후
`virtual_object_marker`와 X/Y/Z + Roll/Pitch/Yaw target으로 처리한다.

IK 오차 표시도 모드에 따라 달라진다:

```python title="src/teleop_ui.py — _ik_err_text"
def _ik_err_text(app, side):
    # FK 모드인 손은 IK를 아예 안 풀므로 mm 오차 자체가 의미 없다.
    if app.arm_mode[side] == "ik":
        return f"{app.ik_err_mm[side]:.2f}mm"
    return "FK"
```

`_begin_expanded`는 `imgui.begin()`의 반환 타입이 imgui-bundle 버전에 따라
`bool`이거나 `(expanded, opened)` 튜플이거나 달라서 이를 정규화하는 작은 헬퍼다.
`_section()`은 ImGui collapsing header를 감싸서 기본 열림/닫힘 정책을 한 곳에서
맞춘다. 관절 모니터처럼 보조 정보 성격이 강한 패널은 기본 닫힘이고, 조작에 필요한
패널들은 기본 열림이다.

## 이 파일이 다른 파일과 합쳐지는 방식

- **`teleop_app.py`**가 유일한 호출자다. `TeleopApp._draw_ui_panel()`은 사실상
  `teleop_ui.draw_panel(self)` 한 줄이다 — 이 프로젝트에서 "위젯을 추가/수정하려면
  이 파일만 보면 된다"는 명확한 경계가 여기서 생긴다.
- 이 파일은 **다른 어떤 프로젝트 모듈도 import하지 않는다**(`math`, `time`,
  `imgui_bundle`뿐) — `ik`/`arm_control`/`grasp`/`base_teleop`이 무엇을 하는지
  전혀 몰라도, `app`이 노출하는 속성/메서드 이름만 알면 패널을 그릴 수 있다.
- 데이터 흐름은 항상 한 방향이다: **jog/슬라이더 조작 → `app.targets`(또는
  `app.fk_q_deg`, `app.contact_viz` 등) 갱신 → 다음 물리 스텝에서
  `TeleopApp._step_physics()`가 그 값을 읽음.** 이 패널이 물리 상태(`data`)를
  직접 읽는 건 HUD 표시(현재 시각, 베이스 위치, 관절각 모니터)뿐이고, 쓰는
  경우는 전혀 없다.
