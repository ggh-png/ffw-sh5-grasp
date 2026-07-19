# API 치트시트

## 프로젝트 핵심 API

| API | 역할 |
|---|---|
| `TeleopApp.set_whole_body_enabled(enabled)` | world target 보존 + rebase + cached twist zero를 포함한 안전 전환 |
| `TeleopApp.toggle_whole_body_control()` | UI 버튼용 ON/OFF wrapper |
| `WholeBodyIK.solve(..., whole_body_enabled=True)` | base/lift/active arms bounded solve; false면 base/lift hard pin |
| `WholeBodyIK.rebase(data, target_poses)` | 현재 base/hand를 공통 motion reference로 재설정 |
| `WholeBodyIK.collision_distances(data)` | controller와 visualization이 공유하는 active distance query |
| `kinematics.evaluate_site(...)` | 정규화 world pose + world-aligned geometric Jacobian |
| `kinematics.collision_distance_gradient(...)` | signed distance, 최근접점, controlled-DOF gradient |
| `teleop_targets.target_world_pose(app, side)` | 현재 mode 표현의 hand target을 world pose로 변환 |
| `teleop_targets.world_to_target_pos(...)` | world position을 현재 mode의 target offset으로 역변환 |
| `teleop_targets.target_rpy_to_world_quat(...)` | raw/smoothed RPY를 현재 mode의 world quaternion으로 변환 |
| `base_teleop.BodyTwist(vx, vy, wz)` | robot body-frame 평면 속도 값 객체 |
| `SwerveDrive.update_twist(...)` | body twist + wheel feedback → wheel steer/drive command |

## 현재 기본값

| 설정 | 값 | 위치 |
|---|---:|---|
| UI loop | 25 Hz | `teleop_app.LOOP_HZ` |
| target position ramp | 0.03 m/frame | `MAX_POS_STEP_PER_FRAME` |
| target orientation ramp | 8°/frame | `MAX_RPY_STEP_PER_FRAME_DEG` |
| lift range | -0.5~0.0 m | `LIFT_RANGE` |
| collision buffer | 0.03 m | `WholeBodyIK` 기본값 |
| collision safe distance | 0.01 m | `WholeBodyIK` 기본값 |
| base linear velocity limit | 0.55 m/s | `DEFAULT_VELOCITY_LIMITS` |
| base yaw velocity limit | 1.2 rad/s | `DEFAULT_VELOCITY_LIMITS` |
| lift velocity limit | 0.25 m/s | `DEFAULT_VELOCITY_LIMITS` |
| arm velocity limit | 2.0 rad/s | `DEFAULT_VELOCITY_LIMITS` fallback |

## MuJoCo Python API

| API | 사용 위치 | 역할 |
|---|---|---|
| `MjModel.from_xml_path()` | `teleop_app.py`, tests | XML 모델 로드 |
| `MjData(model)` | app, IK, tests | 시뮬레이션 상태 생성 |
| `mj_forward()` | app, IK, tests | 현재 qpos 기준 계산 갱신 |
| `mj_step()` | app, tests | 물리 timestep 진행 |
| `mj_resetData()` | IK, tests | data 초기화 |
| `mj_resetDataKeyframe()` | app, tests | keyframe으로 초기화 |
| `mj_name2id()` | 대부분 모듈 | 이름을 id로 변환 |
| `mj_id2name()` | `grasp.py` | id를 이름으로 변환 |
| `mj_jacSite()` | `kinematics.py`, `ik.py` | site Jacobian 계산 |
| `mj_contactForce()` | `grasp.py` | contact force 읽기 |
| `mju_subQuat()` | legacy `bimanual_constraint.py` | quaternion 상대 회전 계산 |
| `mju_mat2Quat()` | target/render/IK | matrix를 quaternion으로 변환 |
| `mju_quat2Mat()` | `teleop_targets.py` | quaternion을 matrix로 변환 |
| `mju_mulQuat()` | target/IK | quaternion 곱 |
| `mju_negQuat()` | target | quaternion inverse/conjugate |
| `MjvScene` | `teleop_render.py` | 렌더 scene |
| `MjvCamera` | `teleop_render.py` | 카메라 |
| `MjvOption` | `teleop_render.py` | 렌더 옵션 |
| `MjvPerturb` | `teleop_render.py` | perturb 구조체 |
| `MjrContext` | `teleop_render.py` | 렌더 context |
| `mjv_updateScene()` | `teleop_render.py` | scene 갱신 |
| `mjr_render()` | `teleop_render.py` | scene 렌더 |
| `mjv_moveCamera()` | `teleop_render.py` | mouse camera 조작 |

## MJCF 요소

| 요소/속성 | 역할 |
|---|---|
| `<body>` | 강체 |
| `<joint>` | 자유도 |
| `<freejoint>` | 6DOF 자유물체 |
| `<geom>` | 시각/충돌 형상 |
| `<site>` | 참조 좌표계 |
| `<actuator>` | joint 구동 |
| `<position>` | 위치 actuator |
| `<motor>` | 토크 actuator |
| `<velocity>` | 속도 actuator |
| `<keyframe>` | 초기 상태 저장 |
| `<equality><weld>` | body 간 제약 |
| `mocap="true"` | 외부에서 pose를 지정하는 kinematic marker body |
| `solref`, `solimp` | 접촉 solver 파라미터 |
| `friction` | 마찰 |
| `condim` | contact 차원 |
| `priority` | 접촉 파라미터 우선순위 |
| `<exclude>` | body pair collision 제외 |
| `<pair>` | geom pair 접촉 파라미터 |
