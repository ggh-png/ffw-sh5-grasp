# `src/teleop_render.py`

GLFW 창, MuJoCo scene render, 카메라, 3D gizmo를 담당한다.

UI·목표 상태와 gizmo가 연결되는 전체 흐름은
[Part 9 — Cyclo Control UI](ros2/09-teleoperation-ui.md)에서 확인한다.

## 역할

| 항목 | 내용 |
|---|---|
| Window | GLFW window 생성/종료 |
| UI backend | ImGui context와 GLFW renderer |
| Scene | MuJoCo `MjvScene`, `MjrContext` 렌더링 |
| Camera | mouse orbit/pan/zoom |
| Gizmo | ImGuizmo translate/rotate 조작 |

## 함수

| 함수 | 역할 |
|---|---|
| `set_camera_preset(cam, preset)` | overview/right-hand close-up 카메라 설정 |
| `setup_render(app, window_w, window_h)` | GLFW, ImGui, MuJoCo render context 생성 |
| `begin_frame(app)` | event poll, ImGui input 처리, 새 frame 시작 |
| `shutdown(app)` | ImGui backend와 GLFW 종료 |
| `handle_camera_mouse(app, io)` | 마우스 입력을 MuJoCo camera move로 변환 |
| `pose_to_imguizmo_matrix(app, world_pos, world_quat)` | world pose를 ImGuizmo matrix로 변환 |
| `imguizmo_matrix_to_pose(app, matrix)` | ImGuizmo matrix를 world pose로 변환 |
| `_imguizmo_camera_matrices(app, viewport)` | ImGuizmo용 view/projection matrix 생성 |
| `draw_transform_gizmo(app, viewport)` | 현재 target의 translate/rotate gizmo 렌더링 및 결과 반영 |
| `render_scene(app)` | marker sync, MuJoCo render, gizmo, ImGui draw, swap buffers |
| `end_frame(app, t0)` | frame frequency update와 sleep |

## 함수 흐름

```mermaid
flowchart TD
    A["teleop_app.run<br>전체 프레임 루프"] --> B["begin_frame()<br>프레임 입력과 ImGui 시작 처리"]
    B --> C["glfw.poll_events()<br>창/키보드/마우스 이벤트 수집"]
    C --> D["imgui.new_frame()<br>새 UI frame 시작"]
    A --> E["handle_camera_mouse()<br>마우스 입력으로 카메라 조작"]
    E --> F["mjv_moveCamera()<br>MuJoCo 카메라 pose 갱신"]
    A --> G["render_scene()<br>3D scene, gizmo, UI를 그리는 메인 함수"]
    G --> H["app._sync_ik_mocaps_from_targets()<br>target에 맞춰 marker mocap 동기화"]
    H --> I["mjv_updateScene()<br>MuJoCo scene geometry 갱신"]
    I --> J["mjr_render()<br>MuJoCo 3D 화면 렌더링"]
    J --> K["draw_transform_gizmo()<br>현재 target의 이동/회전 gizmo 표시"]
    K --> L["pose_to_imguizmo_matrix()<br>world pose를 gizmo matrix로 변환"]
    K --> M["imguizmo.manipulate()<br>사용자 drag 결과 matrix 계산"]
    M --> N["imguizmo_matrix_to_pose()<br>gizmo matrix를 world pose로 복원"]
    N --> O["app._set_gizmo_target_world_pose()<br>world pose를 target 상태에 반영"]
    K --> P["imgui.render()<br>ImGui draw command 생성"]
    P --> Q["swap_buffers()<br>렌더링 결과를 화면에 표시"]
    A --> R["end_frame()<br>FPS 계산과 frame sleep 처리"]
```

## Frame 순서

```text
begin_frame()
handle_camera_mouse()
teleop_ui.draw_panel()
teleop_app._step_physics()
render_scene()
end_frame()
```

## 데이터 변경

| 읽기 | 쓰기 |
|---|---|
| `app.model`, `app.data`, `app.targets`, camera state | `app.cam`, `app.gizmo_mouse_active`, target wrapper 호출 |

렌더 모듈은 직접 IK나 physics step을 수행하지 않는다.
