[← 전체 안내](../ros2-guide.md)

# Part 14 — 용어 사전 (ROS2 ↔ 이 프로젝트) {: #part-14 }

| ROS2/로보틱스 용어 | 이 프로젝트 용어 | 짧은 설명 |
|---|---|---|
| node | (없음, `TeleopApp` 인스턴스) | 프로세스가 하나뿐 |
| topic | (없음, `app.targets` dict) | 함수 인자로 직접 전달 |
| tf2 frame | `site`, base/target/world 좌표계 | 자동 트리 없음, `teleop_targets.py`가 수동 변환 |
| URDF | MJCF(`models/*.xml`) | Part 2.1 |
| `joint_state_publisher` | `data.qpos` 직접 읽기 | 퍼블리시 안 함 |
| `ros2_control` 컨트롤러 | `ArmTorqueController`, `apply_grasp`, `SwerveDrive` | 파이썬 클래스/함수 |
| MoveIt IK/Servo | `whole_body_ik.WholeBodyIK` | base/lift/양팔 weighted differential IK + reactive collision CBF; 전역 플래너와 ROS 없음 |
| MoveIt Cartesian path | (없음) | 프레임마다 목표까지 한 스텝만 풂(연속 텔레옵) |
| RViz Interactive Marker | ImGuizmo + mocap body | Part 9.3~9.4 |
| `GripperCommand` action | `grasp.apply_grasp(grasp, thumb)` | 스칼라 2개 synergy |
| force/torque sensor topic | `mj_contactForce()` | 물리 엔진이 직접 계산 |
| `cmd_vel`(Twist) | `base_teleop.BodyTwist` | ROS 메시지가 아닌 순수 Python 값 객체 |
| `swerve_drive_controller` | `SwerveDrive` | ROBOTIS 원본 구조 이식 |
| `nav2`/`twist_mux` | `teleop_app.py`의 명령 우선순위 | 키보드 입력 중에는 수동 twist, 그 외에는 whole-body IK twist |
| launch 파일 | (없음, `python3 src/teleop_app.py`) | 노드가 하나라 불필요 |
| 파라미터 서버 | 모듈 최상단 상수 | 런타임 재설정 불가 |
| `colcon test`/`launch_testing` | `tests/test_phase_{0..6}.py`, `test_whole_body.py` | headless, 직접 실행 |
| Gazebo `SetModelState` 치팅 | `data.qpos[...]` 직접 대입 | **금지** (reset/초기배치 제외) |
| `dynamic_reconfigure`/`rqt` | `teleop_ui.py` (ImGui) | 같은 프로세스 내 위젯 |

---

## 다음으로 읽을 문서

- 함수 단위로 더 빠르게 찾고 싶다면: [`API 치트시트`](../cheatsheet.md), [`체크리스트`](../pitfalls.md)
- 파일별 함수 표/Mermaid 흐름도만 빠르게 보고 싶다면: [`코드 가이드 홈`](../index.md)부터 시작해서
  읽는 순서표를 따라가면 된다(`grasp.py` → `whole_body_ik.py` → `arm_control.py` → `base_teleop.py` →
  `teleop_targets.py` → `teleop_ui.py` → `teleop_render.py` → `teleop_app.py`).
- 프로젝트가 왜 세 번째 시도인지, 전체 설계 근거를 더 보고 싶다면: [`프로젝트 개요`](../../overview.md)
- 실행/조작만 빠르게 필요하다면: [`직접 실행하기`](../../run.md)

---

[← Part 13](./13-bug-cases.md) · [전체 안내](../ros2-guide.md)
