# API 치트시트

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
| `mj_jacSite()` | `ik.py` | site Jacobian 계산 |
| `mj_contactForce()` | `grasp.py` | contact force 읽기 |
| `mju_subQuat()` | `ik.py` | quaternion 오차 계산 |
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
