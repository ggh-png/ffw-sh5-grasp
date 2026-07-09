# `src/teleop_render.py` — 렌더링, 카메라, 3D gizmo

## 이 파일이 하는 일

`teleop_app.py`에서 시각화 전용 책임을 분리한 모듈이다. GLFW 창 생성, ImGui frame
시작/종료, MuJoCo 저수준 렌더 API 호출, 마우스 카메라 조작, ImGuizmo transform gizmo,
프레임 pacing을 담당한다.

`teleop_ui.py`와 마찬가지로 `TeleopApp` 클래스를 import하지 않는다. 대신
`setup_render(app, ...)`, `render_scene(app)`처럼 `app` 객체를 받아 필요한 속성만
duck typing으로 읽고 쓴다. 덕분에 `teleop_app.py`는 물리 상태와 실행 순서를 담당하고,
시각화 구현 세부사항은 이 파일에서만 보면 된다.

## 구현: 프레임 흐름

메인 루프에서 렌더 모듈은 세 번 호출된다.

```python title="src/teleop_app.py — run"
io = teleop_render.begin_frame(self)
teleop_render.handle_camera_mouse(self, io)
...
teleop_render.render_scene(self)
teleop_render.end_frame(self, t0)
```

`begin_frame()`은 `glfw.poll_events()`, ImGui backend 입력 처리, `imgui.new_frame()`을
묶어서 새 UI 프레임을 시작한다. `handle_camera_mouse()`는 ImGui나 gizmo가 마우스를
잡고 있지 않을 때만 MuJoCo 카메라 orbit/pan/zoom을 처리한다.

```python title="src/teleop_render.py — handle_camera_mouse"
if io.want_capture_mouse or app.gizmo_mouse_active:
    return
...
mujoco.mjv_moveCamera(app.model, action, dx / win_h, dy / win_h, app.scene, app.cam)
```

## 구현: MuJoCo + ImGui 합성

`render_scene()`은 먼저 mocap marker pose와 marker 표시 상태를 현재 target에 맞춘 뒤,
MuJoCo 장면을 갱신하고, ImGuizmo를 foreground draw list에 올리고, 마지막에 ImGui 패널을
같은 framebuffer에 합성한다.

```python title="src/teleop_render.py — render_scene"
app._sync_ik_mocaps_from_targets()
app.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = app.contact_viz
app.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = app.contact_viz
mujoco.mjv_updateScene(app.model, app.data, app.opt, app.pert, app.cam,
                       mujoco.mjtCatBit.mjCAT_ALL, app.scene)
mujoco.mjr_render(viewport, app.scene, app.context)
draw_transform_gizmo(app, viewport)
imgui.render()
app.impl.render(imgui.get_draw_data())
glfw.swap_buffers(app.window)
```

## 구현: ImGuizmo pose 변환

3D 화살표/회전 링은 `draw_transform_gizmo()`가 그린다. 실제 target이 오른손, 왼손,
virtual object 중 무엇인지는 `app._active_gizmo_target()`에 묻고, pose 변환은
`pose_to_imguizmo_matrix()`와 `imguizmo_matrix_to_pose()`가 맡는다.

```python title="src/teleop_render.py — draw_transform_gizmo"
target = app._active_gizmo_target()
world_pos, world_quat = app._gizmo_target_world_pose(target)
object_matrix = pose_to_imguizmo_matrix(app, world_pos, world_quat)
...
if changed_translate or changed_rotate:
    new_pos, new_quat = imguizmo_matrix_to_pose(app, object_matrix)
    app._set_gizmo_target_world_pose(target, new_pos, new_quat)
```

`TeleopApp` 안에는 테스트 호환용 wrapper만 남아 있다. Phase 6 테스트가 matrix roundtrip과
target pose 반영을 직접 검증하기 때문이다.

## 이 파일이 다른 파일과 합쳐지는 방식

- **teleop_app.py**가 유일한 사용자다. `_setup_render()`와 `run()`에서 이 모듈의
  함수들을 호출한다.
- **teleop_ui.py**는 위젯을 그리고 target 값을 바꾼다. `teleop_render.py`는 그 target을
  화면의 mocap marker/gizmo로 보여준다.
- 물리 계산은 하지 않는다. `mj_step`은 여전히 `teleop_app.py`의 `_step_physics()`에만
  있다.
