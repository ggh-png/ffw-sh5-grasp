# `src/teleop_targets.py`

UI target, 3D marker/gizmo pose, IK world pose 사이의 변환을 담당한다.

## Target 의미

| 값 | 의미 |
|---|---|
| `pos_r`, `pos_l` | 각 손의 시작 위치 기준 XYZ offset |
| `rpy_r`, `rpy_l` | 각 손의 시작 자세 기준 RPY delta |
| `virtual_object_pos` | base frame의 virtual object 위치 |
| `virtual_object_rpy` | base frame 기준 virtual object RPY |

## 함수

| 함수 | 역할 |
|---|---|
| `rpy_deg_to_quat(rpy_deg)` | RPY degree를 quaternion으로 변환 |
| `quat_to_rpy_deg(q)` | quaternion을 RPY degree로 변환 |
| `set_home_references(app)` | 양손 시작 위치/자세를 target 기준으로 저장 |
| `base_pose(app)` | base x/y/yaw, sin/cos, yaw quaternion 반환 |
| `local_to_world_pos(app, p_local)` | base-local 위치를 world 위치로 변환 |
| `world_to_base_pos(app, p_world)` | world 위치를 base-local 위치로 변환 |
| `target_pos_to_base_pos(app, side, pos_target)` | 손별 home-relative offset을 base-local 위치로 변환 |
| `target_pos_to_world_pos(app, side, pos_target)` | 손별 target 위치를 world 위치로 변환 |
| `world_to_target_pos(app, side, world_pos)` | world 위치를 손별 target offset으로 변환 |
| `target_world_quat(app, side)` | 손별 RPY target을 world quaternion으로 변환 |
| `world_quat_to_target_rpy(app, side, world_quat)` | world quaternion을 손별 RPY target으로 변환 |
| `world_quat_to_virtual_rpy(app, world_quat)` | world quaternion을 virtual object RPY로 변환 |
| `quat_to_mat(quat)` | quaternion을 3x3 rotation matrix로 변환 |
| `mat_to_quat(mat)` | 3x3 rotation matrix를 quaternion으로 변환 |
| `target_world_pose(app, side)` | 손 target의 world position/quaternion 반환 |
| `virtual_object_world_pose(app)` | virtual object의 world position/quaternion 반환 |
| `sync_virtual_object_to_hand_targets(app)` | virtual object를 양손 target 중점으로 이동 |
| `capture_grasp(app)` | 양손 target을 virtual object 기준 상대 transform으로 저장 |
| `release_grasp(app)` | Bimanual MoveL capture 해제 |
| `apply_virtual_object_target(app)` | virtual object pose에서 양손 target 재계산 |
| `bimanual_marker_visible(app)` | virtual marker 표시 여부 반환 |
| `sync_marker_visibility(app)` | virtual marker alpha 갱신 |
| `active_gizmo_target(app)` | 현재 gizmo 대상 반환 |
| `gizmo_target_world_pose(app, target)` | gizmo 대상의 world pose 반환 |
| `set_gizmo_target_world_pose(app, target, world_pos, world_quat)` | gizmo 결과를 target 값으로 반영 |
| `sync_ik_mocaps_from_targets(app)` | 손/virtual marker mocap pose를 target과 동기화 |

## 함수 흐름

```mermaid
flowchart TD
    A["UI sliders / marker jog<br>사용자가 target 값을 직접 조정"] --> B["app.targets<br>IK/FK/virtual object 목표 상태 저장소"]
    C["ImGuizmo drag world pose<br>3D marker를 world 좌표에서 조작"] --> D["set_gizmo_target_world_pose()<br>gizmo 결과를 app target 좌표계로 변환"]
    D --> B
    B --> E["target_world_pose(side)<br>손별 IK 목표 world pose 계산"]
    E --> F["target_pos_to_world_pos()<br>home-relative 위치를 world 위치로 변환"]
    E --> G["target_world_quat()<br>target RPY를 world quaternion으로 변환"]
    F --> H["IK target world pose<br>IK solver에 넘길 최종 목표"]
    G --> H
    H --> I["teleop_app -> ik.solve_pose()<br>world pose를 관절 목표로 변환"]

    J["Capture Grasp<br>양손 상대 관계 저장 시작"] --> K["capture_grasp()<br>현재 양손 pose를 virtual object 기준으로 캡처"]
    K --> L["sync_virtual_object_to_hand_targets()<br>virtual object를 양손 중앙에 배치"]
    L --> M["store hand offsets from virtual object<br>object 기준 양손 상대 transform 저장"]
    N["Move virtual object<br>양손을 하나의 강체처럼 이동"] --> O["apply_virtual_object_target()<br>virtual object pose에서 양손 target 재계산"]
    O --> P["world_to_target_pos()<br>world 위치를 target 좌표로 변환"]
    O --> Q["world_quat_to_target_rpy()<br>world 자세를 target RPY로 변환"]
    P --> B
    Q --> B
    B --> R["sync_ik_mocaps_from_targets()<br>시각 marker mocap pose 동기화"]
```

## Bimanual MoveL 상태 흐름

```text
MoveL
  right/left target을 독립 조작

Capture Grasp
  현재 양손 target pose를 virtual object 기준 상대 transform으로 저장

Bimanual MoveL
  virtual object pose 변경
  -> 저장된 상대 transform 적용
  -> left/right target 자동 갱신

Release Grasp
  다시 독립 MoveL 상태
```

## 사용 위치

- `teleop_app.py`: target wrapper와 물리 step에서 호출
- `teleop_render.py`: gizmo pose 조회/반영 wrapper를 통해 사용
- `teleop_ui.py`: capture/release/apply 메서드를 통해 사용
