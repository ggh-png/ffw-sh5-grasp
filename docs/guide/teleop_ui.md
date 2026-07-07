# `src/teleop_ui.py` — ImGui 슬라이더 패널

## 이 파일이 하는 일

`draw_panel(app)` 함수 하나가 사실상 전부다. `teleop_app.py`의 `TeleopApp` 인스턴스
전체(`app`)를 인자로 받아, HUD·양손 EE 포즈/관절각 슬라이더·grasp/thumb 슬라이더·
리프트·관절 모니터까지 패널 전체를 매 프레임 그리고, 슬라이더/버튼 조작 결과를
`app`의 상태에 곧바로 되써넣는다.

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

패널은 손마다 IK/FK 모드에 따라 다른 슬라이더 집합을 그린다:

```python title="src/teleop_ui.py — 손 패널의 모드 분기"
mode = app.arm_mode[side]
imgui.text(f"Mode: {'IK (pose)' if mode == 'ik' else 'FK (joint)'}")
imgui.same_line()
if imgui.button(f"Switch to {'FK' if mode == 'ik' else 'IK'}##{side}mode"):
    app.set_arm_mode(side, "fk" if mode == "ik" else "ik")

if mode == "ik":
    pos = targets[f"pos_{side}"]
    rpy = targets[f"rpy_{side}"]
    for i, axis in enumerate(("X", "Y", "Z")):
        _, pos[i] = imgui.slider_float(f"{axis}##{side}pos", pos[i], -0.2, 1.2, "%.3f m")
    ...
else:
    imgui.text("Joint angles (deg)")
    fk_deg = app.fk_q_deg[side]
    for i, (lo, hi) in enumerate(app.arm_joint_ranges_deg[side]):
        _, fk_deg[i] = imgui.slider_float(f"J{i+1}##{side}fk", fk_deg[i], lo, hi, "%.1f deg")
```

모드 전환 버튼은 직접 `app.arm_mode[side]`를 바꾸지 않고 `app.set_arm_mode(side,
...)`를 호출한다 — 전환 순간 포즈가 튀지 않게 하는 동기화 로직(팔 목표 관절각을
재계산하는 것)은 `teleop_app.py`의 책임이라, UI는 "전환해라"라는 의도만 전달한다.
"Reset Can"/"Toggle Contact Viz"/"Cycle Camera" 버튼도 같은 패턴이다 —
`app.reset_can()`/`app.contact_viz = not app.contact_viz`/`app.cycle_camera()`처럼
직접 상태를 바꾸거나 `TeleopApp`의 메서드를 호출할 뿐, 그 메서드 안에서 실제로
무슨 일이 일어나는지는 이 파일이 몰라도 된다.

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

## 이 파일이 다른 파일과 합쳐지는 방식

- **`teleop_app.py`**가 유일한 호출자다. `TeleopApp._draw_ui_panel()`은 사실상
  `teleop_ui.draw_panel(self)` 한 줄이다 — 이 프로젝트에서 "위젯을 추가/수정하려면
  이 파일만 보면 된다"는 명확한 경계가 여기서 생긴다.
- 이 파일은 **다른 어떤 프로젝트 모듈도 import하지 않는다**(`math`, `time`,
  `imgui_bundle`뿐) — `ik`/`arm_control`/`grasp`/`base_teleop`이 무엇을 하는지
  전혀 몰라도, `app`이 노출하는 속성/메서드 이름만 알면 패널을 그릴 수 있다.
- 데이터 흐름은 항상 한 방향이다: **슬라이더 조작 → `app.targets`(또는
  `app.fk_q_deg`, `app.contact_viz` 등) 갱신 → 다음 물리 스텝에서
  `TeleopApp._step_physics()`가 그 값을 읽음.** 이 패널이 물리 상태(`data`)를
  직접 읽는 건 HUD 표시(현재 시각, 베이스 위치, 관절각 모니터)뿐이고, 쓰는
  경우는 전혀 없다.
