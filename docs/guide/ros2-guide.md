# ROS2 관점의 FFW-SH5 시스템 해설

이 섹션은 ROS2/Gazebo 경험을 MuJoCo 기반 단일 프로세스 제어 구조에 연결해 설명한다.
기존의 긴 단일 페이지를 주제별 하위 페이지로 나눠, 필요한 개념과 구현만 골라 읽을 수
있도록 구성했다.

!!! tip "실행 방법을 찾는 중이라면"
    이 섹션은 설계와 내부 동작을 이해하기 위한 심화 자료다. 실행은
    [빠른 시작](../getting-started.md), 조작은 [화면과 조작](../run.md),
    문제 진단은 [문제 해결](../troubleshooting.md)이 더 빠르다.

## 추천 읽기 경로

| 목적 | 권장 순서 |
|---|---|
| ROS2와 구조 차이 파악 | Part 1 → 2 → 4 |
| 제어 알고리즘 이해 | Part 5 → 6 → 7 → 8 |
| 3D 조작과 좌표계 이해 | Part 9 → 10 |
| 검증과 유지보수 | Part 11 → 13 → 14 |
| 직접 실행 | Part 12 |

## 시작과 구조

| 페이지 | 내용 |
|---|---|
| <span id="part-1"></span>[Part 1 — ROS2와 개념 지도](ros2/01-concepts.md) | 노드·토픽·tf·controller 개념을 현재 구조와 비교 |
| <span id="part-2"></span>[Part 2 — MuJoCo model과 data](ros2/02-mujoco-model-data.md) | MJCF, actuator, contact, 물리 상태 |
| <span id="part-3"></span>[Part 3 — 프로젝트 정체성](ros2/03-project-identity.md) | 목표, 불변식, Phase 이력 |
| <span id="part-4"></span>[Part 4 — 런타임 아키텍처](ros2/04-runtime-architecture.md) | 파일 지도와 한 frame의 실행 순서 |

## 제어 알고리즘

| 페이지 | 내용 |
|---|---|
| <span id="part-5"></span>[Part 5 — 손 제어](ros2/05-hand-control.md) | grasp synergy, 관절 보간, 접촉 판정 |
| <span id="part-6"></span>[Part 6 — 역기구학](ros2/06-inverse-kinematics.md) | DLS, task priority, 회전 오차, line search |
| <span id="part-7"></span>[Part 7 — 팔 토크 제어](ros2/07-arm-torque-control.md) | PD와 bias force feedforward |
| <span id="part-8"></span>[Part 8 — 모바일 베이스](ros2/08-mobile-base.md) | 스워브 역기구학과 feedback 제어 |

## 조작과 좌표계

| 페이지 | 내용 |
|---|---|
| <span id="part-9"></span>[Part 9 — 3D 텔레오퍼레이션 UI](ros2/09-teleoperation-ui.md) | MoveL, bimanual 상태, gizmo |
| <span id="part-10"></span>[Part 10 — 좌표계](ros2/10-coordinate-frames.md) | startup anchor, world target, 변환 함수 |

## 검증과 참고

| 페이지 | 내용 |
|---|---|
| <span id="part-11"></span>[Part 11 — 테스트와 검증](ros2/11-testing.md) | Phase gate와 release 전략 |
| <span id="part-12"></span>[Part 12 — 직접 실행](ros2/12-running.md) | 설치와 실행 명령 |
| <span id="part-13"></span>[Part 13 — 버그 사례집](ros2/13-bug-cases.md) | 실제 결함과 일반화된 교훈 |
| <span id="part-14"></span>[Part 14 — 용어 사전](ros2/14-glossary.md) | ROS2와 현재 프로젝트 용어 비교 |

## 수식 설명 원칙

수식은 기호의 뜻과 차원을 먼저 제시하고, 중간 대수 전개를 생략하지 않는 것을 원칙으로
한다. 특히 DLS, SVD gain, null-space projector의 전체 유도는
[DLS와 위치 우선 IK 수학](ik-math.md)에 한 흐름으로 정리되어 있다.

## 30초 요약

이 프로젝트는 ROS2 노드가 아니라 `python3 src/teleop_app.py` 하나로 실행되는 단일
프로세스 프로그램이다. 입력으로 target을 갱신하고, whole-body IK와 각 actuator
controller가 `data.ctrl`을 만든 다음 MuJoCo 물리 step과 렌더링을 순서대로 수행한다.

```text
입력 → target 갱신 → whole-body IK
    → 팔·손·바퀴 actuator command → mj_step → 렌더링
```

캔은 좌표를 강제로 붙여 드는 것이 아니라 손가락 접촉력과 마찰로 실제로 지지한다.
