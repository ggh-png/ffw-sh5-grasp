# ffw-sh5-grasp

ROBOTIS FFW-SH5 양팔 로봇을 MuJoCo에서 구동하는 물리 기반 텔레오퍼레이션 프로젝트.
캔 grasp, 모바일 베이스 주행, Cyclo Control 스타일 양팔 target 조작을 하나의 네이티브
창에서 실행한다.

[![demo video](https://img.youtube.com/vi/2LV_RsAGdz8/hqdefault.jpg)](https://www.youtube.com/watch?v=2LV_RsAGdz8)

프로젝트 링크: https://ggh-png.github.io/ffw-sh5-grasp/

## Features

- FFW-SH5 전신 모델: 양팔 7DOF x2, HX5-D20 5지 핸드 x2, 리프트, 헤드, 모바일 베이스
- contact force 기반 캔 grasp: 물체를 로봇에 붙이는 kinematic override 없음
- 토크 기반 팔 제어: PD + gravity/Coriolis feedforward
- whole-body IK: `base_x/base_y/base_yaw + lift + 양팔 14축`이 손별 6DOF target을 함께 추종
- 전신 제어 ON/OFF: UI 버튼으로 전신 IK와 arm-only IK를 무점프 전환
- collision avoidance: 3 cm 감시·1 cm 안전거리의 arm-arm/body/table signed-distance CBF
- Cyclo-style UI: `MoveL`, `Bimanual MoveL`, virtual object marker
- 3D gizmo: XYZ 화살표와 RPY 회전 링으로 target 조작
- 실제 바퀴 마찰 기반 스워브 주행: 조향각/구동속도 명령으로 베이스 이동
- ROS 비의존: AIWorker/Cyclo의 제어 알고리즘만 NumPy + MuJoCo 순수 Python으로 반영
- headless 회귀 테스트: Phase 0-6 + mobile whole-body 독립 실행

## Quick Start

```bash
pip install --break-system-packages mujoco numpy trimesh pillow glfw imgui-bundle
python3 src/teleop_app.py
```

테스트:

```bash
for p in 0 1 2 3 4 5 6; do python3 tests/test_phase_$p.py; done
python3 tests/test_whole_body.py
```

문서:

```bash
pip install --break-system-packages mkdocs mkdocs-material
mkdocs serve
```

## Teleop Controls

| 입력 | 기능 |
|---|---|
| Mouse left drag | 카메라 orbit |
| Mouse right drag | 카메라 pan |
| Mouse wheel | 카메라 zoom |
| `Up` / `Down` | 베이스 전진 / 후진 |
| `Left` / `Right` | 베이스 yaw 회전 |
| `[` / `]` | 베이스 좌 / 우 strafe |
| `Q` / `E` | 리프트 하강 / 상승 |
| `R` | 캔 리셋 |
| `G` | contact point/force 표시 토글 |
| `V` | collision geometry와 활성 CBF 최근접점/연결선 표시 토글 |
| `C` | 카메라 프리셋 전환 |

`Lift / Utilities`의 **Whole-body Control** 버튼은 다음 두 모드를 전환한다.

- **ON**: 베이스 3축, 리프트, IK 모드 팔이 손 목표 추종에 함께 참여한다.
- **OFF (arm-only)**: 베이스와 리프트를 IK에서 정확히 고정하고 팔만 푼다. 키보드
  주행과 `Lift target` 수동 제어는 계속 사용할 수 있다.

전환 시 손/virtual-object의 현재 world 목표는 그대로 유지되고 이전 전신 베이스
명령은 즉시 0으로 초기화된다.

## Code Structure

```text
src/
├── teleop_app.py        # 앱 조립, 메인 루프, 입력, 물리 step
├── teleop_ui.py         # ImGui 패널과 버튼/슬라이더
├── teleop_render.py     # GLFW, MuJoCo 렌더링, 카메라, ImGuizmo
├── teleop_targets.py    # 손 target pose 변환, marker sync, Bimanual MoveL 상태
├── base_teleop.py       # body twist, swerve 역/정기구학, feedback controller
├── kinematics.py        # 공용 pose/FK/Jacobian + collision distance gradient
├── whole_body_ik.py     # 베이스+리프트+양팔 IK + joint/collision CBF
├── ik.py                # 기존 단일 팔 6DOF IK와 독립 회귀 테스트
├── arm_control.py       # 팔 토크 제어기
├── grasp.py             # 손가락 synergy와 접촉 기반 grasp 판정
├── bimanual_constraint.py # legacy box constraint helper
└── mj_util.py            # joint -> actuator 탐색 등 공용 MuJoCo 헬퍼
```

## Tests

| 파일 | 검증 내용 |
|---|---|
| `tests/test_phase_0.py` | FFW-SH5 원본 모델 로드, 5초 안정성 |
| `tests/test_phase_1.py` | 손가락 collision 관통 깊이 |
| `tests/test_phase_2.py` | 고정 손 캔 grasp/lift |
| `tests/test_phase_3.py` | 오른팔 FK/Jacobian 중앙차분, IK와 pick 통합 |
| `tests/test_phase_4.py` | 전신 모델 hold, IK, pick |
| `tests/test_phase_5.py` | 입력 응답/FSM/조향 limiter 단위 + 전후·strafe·yaw·복합·반전 물리 회귀 |
| `tests/test_phase_6.py` | Cyclo marker, Bimanual MoveL, XYZ/RPY target, 전신 ON/OFF 무점프 전환 |
| `tests/test_whole_body.py` | ROS-free, box-QP/관절·충돌 CBF/rigid-grasp, arm-only hard gate, 거리 gradient 수치검증, 무작위 solver 40회, 실제 바퀴 기반 추종 |

## Docs

MkDocs 문서는 `docs/`에 있고, 배포된 문서는 https://ggh-png.github.io/ffw-sh5-grasp/ 에서 확인할 수 있다.

- `docs/index.md`: 기능 요약
- `docs/run.md`: 실행 방법
- `docs/overview.md`: 구조 요약
- `docs/guide/`: 모듈별 함수 역할 정리
