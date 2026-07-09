# `src/teleop_targets.py` — 손 목표 pose와 Cyclo-style bimanual target

## 이 파일이 하는 일

`teleop_targets.py`는 UI 숫자 target, 3D marker/gizmo pose, IK solver에 들어가는
world pose 사이의 변환을 담당한다. `teleop_app.py`에 있던 home-relative XYZ/RPY
좌표계 변환과 `Bimanual MoveL` capture/apply 로직을 분리한 모듈이다.

핵심 계약은 세 가지다.

- 손별 `pos_r`/`pos_l`은 world 좌표가 아니라 시작 손 위치 기준 XYZ offset이다.
- 손별 `rpy_r`/`rpy_l`은 시작 손 자세 기준 local Roll/Pitch/Yaw delta다.
- `Bimanual MoveL` capture 이후에는 `virtual_object_pos/rpy`가 명령 원천이고,
  오른손/왼손 target은 capture 당시의 상대 transform에서 다시 파생된다.

## 구현: target 좌표계

앱 시작 시 `set_home_references(app)`가 양손의 기준 pose를 저장한다.

```python title="src/teleop_targets.py — set_home_references"
app.home_quat_r = ...
app.home_quat_l = ...
app.home_pos_local = {
    "r": world_to_base_pos(app, app.data.site_xpos[app.site_r]),
    "l": world_to_base_pos(app, app.data.site_xpos[app.site_l]),
}
```

따라서 UI에서 `pos_r = [0, 0, 0]`이면 "오른손을 시작 위치에 둔다"는 뜻이다.
실제 IK에는 `home_pos_local["r"] + pos_r`를 base pose로 world 변환한 값이 들어간다.

```python title="src/teleop_targets.py — target_world_pose"
def target_world_pose(app, side):
    return target_pos_to_world_pos(app, side, app.targets[f"pos_{side}"]), \
           target_world_quat(app, side)
```

반대로 3D gizmo가 world pose를 돌려주면 `world_to_target_pos()`와
`world_quat_to_target_rpy()`가 다시 UI target 값으로 환산한다. 그래서 숫자 슬라이더,
marker, IK target이 같은 목표를 서로 다른 표현으로 공유한다.

## 구현: Bimanual MoveL

`capture_grasp(app)`는 현재 양손 target pose를 virtual object marker 기준 상대
transform으로 저장한다.

```python title="src/teleop_targets.py — capture_grasp"
obj_pos, obj_quat = virtual_object_world_pose(app)
obj_R = quat_to_mat(obj_quat)
for side in ("r", "l"):
    hand_pos, hand_quat = target_world_pose(app, side)
    offsets[side] = {
        "pos": obj_R.T @ (hand_pos - obj_pos),
        "mat": obj_R.T @ quat_to_mat(hand_quat),
    }
```

이후 `apply_virtual_object_target(app)`은 현재 virtual object pose에 저장된 상대
transform을 다시 곱해서 양손 target을 갱신한다.

```python title="src/teleop_targets.py — apply_virtual_object_target"
hand_pos = obj_pos + obj_R @ offset["pos"]
hand_quat = mat_to_quat(obj_R @ offset["mat"])
app.targets[f"pos_{side}"] = world_to_target_pos(app, side, hand_pos)
app.targets[f"rpy_{side}"] = world_quat_to_target_rpy(app, side, hand_quat)
```

즉 MuJoCo 안에서 ROS topic을 쓰지는 않지만, 의미상 Cyclo Control의
`capture_grasp`/`virtual_object_goal_move` 흐름과 같은 상태 전이를 구현한다.

## 이 파일이 다른 파일과 합쳐지는 방식

- **teleop_app.py**가 유일한 직접 사용자다. 기존 테스트와 렌더 코드 호환을 위해
  `TeleopApp._target_world_pose()` 같은 얇은 wrapper는 남겨 두고, 실제 구현은 이
  모듈 함수로 위임한다.
- **teleop_ui.py**는 `app.targets` 값을 바꾼다. 이 모듈은 그 값을 world pose로
  해석하거나, world pose를 다시 target 값으로 환산한다.
- **teleop_render.py**는 gizmo drag 결과를 `app._set_gizmo_target_world_pose()`로
  넘긴다. wrapper를 거쳐 이 모듈의 `set_gizmo_target_world_pose()`가 target을 갱신한다.
- **ik.py**는 이 모듈을 모른다. `teleop_app.py`의 물리 스텝이 이 모듈로 계산된
  world pose를 `solve_pose()`에 넘길 뿐이다.
