# `src/teleop_targets.py`

UI target, 3D marker/gizmo pose, IK world pose 사이의 변환을 담당한다.

## Target 의미

| 값 | Whole-body ON | Whole-body OFF (arm-only) |
|---|---|---|
| `pos_r`, `pos_l` | 시작 world pose 기준 XYZ offset | live base 기준 home-relative offset |
| `rpy_r`, `rpy_l` | 시작 world 자세 기준 RPY delta | live base 기준 home-relative RPY delta |
| `virtual_object_pos/rpy` | startup anchor frame | live base frame |

모드 전환 함수는 먼저 현재 world pose를 저장하고 새 표현으로 역변환하므로 같은
숫자를 억지로 재사용하지 않는다. 따라서 ON/OFF 버튼을 눌러도 marker 목표는 이동하지
않는다.

## 수식

> **왜 startup anchor가 필요한가**: 입력 축은 로봇 시작 방향의 앞/옆/위를
> 유지하되 최종 target world pose는 고정돼야 한다. 현재 base pose에 target을 붙이면
> whole-body IK가 베이스를 움직일 때 goal도 똑같이 움직여 오차가 줄지 않는다. 자세한 이유는
> [ROS2 개발자를 위한 튜토리얼 Part 10.3](ros2-guide.md#part-10-3) 참고.

startup-anchor 위치 \((x,y,z)\) → world 위치, 앱 시작 시 캡처한 베이스 pose
\((x_{b0},y_{b0},\theta_{b0})\)만큼 2D 회전 후 평행이동한다:

\[
\begin{pmatrix} X\\Y\\Z \end{pmatrix} =
\begin{pmatrix} x_{b0}\\y_{b0}\\0 \end{pmatrix} +
\begin{pmatrix} \cos\theta_{b0} & -\sin\theta_{b0} & 0\\ \sin\theta_{b0} & \cos\theta_{b0} & 0\\ 0&0&1 \end{pmatrix}
\begin{pmatrix} x\\y\\z \end{pmatrix}
\]

손 target의 world quaternion(`target_world_quat`)은 세 쿼터니언의 곱:

\[
q_{world} = q_{home\_world} \otimes q_{rpy\_delta}
\]

양손으로 물건을 함께 드는 건 두 손이 서로에 대한 상대 pose를 유지한 채(보이지
않는 막대로 이어진 강체처럼) 같이 움직인다는 뜻이다 — virtual object가 그
기준이고, capture 시점의 상대 오프셋을 한 번 저장해두면 이후 virtual object만
옮겨도 그 관계가 그대로 재적용된다. Bimanual MoveL의 world→virtual-object-local
오프셋 캡처(`capture_grasp`)와 그 역변환(`apply_virtual_object_target`),
\(R^{-1}=R^{T}\)(회전행렬은 직교행렬이라 역행렬이 전치행렬과 같다):

\[
p_{\text{offset}} = R_{obj}^{T}(p_{hand}-p_{obj}), \quad R_{\text{offset}} = R_{obj}^{T}R_{hand}
\qquad\Longleftrightarrow\qquad
p_{hand} = p_{obj} + R_{obj}\,p_{\text{offset}}, \quad R_{hand} = R_{obj}\,R_{\text{offset}}
\]

## 함수

| 함수 | 역할 |
|---|---|
| `rpy_deg_to_quat(rpy_deg)` | RPY degree를 quaternion으로 변환 |
| `quat_to_rpy_deg(q)` | quaternion을 RPY degree로 변환 |
| `set_home_references(app)` | 양손 시작 위치/자세를 target 기준으로 저장 |
| `base_pose(app)` | base x/y/yaw, sin/cos, yaw quaternion 반환 |
| `local_to_world_pos(app, p_local)` | base-local 위치를 world 위치로 변환 |
| `world_to_base_pos(app, p_world)` | world 위치를 base-local 위치로 변환 |
| `anchor_local_to_world_pos(app, p_local)` | startup base anchor 위치를 world로 변환 |
| `world_to_anchor_local_pos(app, p_world)` | world 위치를 startup anchor로 역변환 |
| `carry_world_targets_with_base(app, previous, current)` | 수동 주행 SE(2) 변환을 hand/virtual target frame에 적용 |
| `target_pos_to_base_pos(app, side, pos_target)` | 손별 home-relative offset을 base-local 위치로 변환 |
| `target_pos_to_world_pos(app, side, pos_target)` | 손별 target 위치를 world 위치로 변환 |
| `world_to_target_pos(app, side, world_pos)` | world 위치를 손별 target offset으로 변환 |
| `target_world_quat(app, side)` | 손별 RPY target을 world quaternion으로 변환 |
| `target_rpy_to_world_quat(app, side, rpy)` | raw/smoothed RPY를 활성 target frame의 world quaternion으로 변환 |
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
    E --> F["target_pos_to_world_pos()<br>startup anchor 기준 offset을 고정 world 위치로 변환"]
    E --> G["target_world_quat()<br>target RPY를 world quaternion으로 변환"]
    F --> H["IK target world pose<br>IK solver에 넘길 최종 목표"]
    G --> H
    H --> I["teleop_app -> whole_body_ik.solve()<br>world pose를 base/lift/양팔 명령으로 변환"]

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
