# ffw-sh5-grasp

FFW-SH5 양팔 모바일 로봇을 MuJoCo 물리에서 조작하는 ROS-free 텔레오퍼레이션
프로젝트다. 손의 목표 자세를 지정하면 팔, 리프트, 스워브 베이스가 함께 움직이며,
실제 actuator·마찰·접촉을 거쳐 목표를 추종한다.

<figure class="hero-figure" markdown>
  ![full_scene.xml 렌더](assets/hero.jpg)
  <figcaption>양팔 14축, 양손, 리프트, 3모듈 스워브 베이스와 캔을 포함한 전체 장면.</figcaption>
</figure>

[![GitHub](https://img.shields.io/badge/GitHub-Repository-black?logo=github)](https://github.com/ggh-png/ffw-sh5-grasp)
[![Release](https://img.shields.io/badge/release-1.1.1-indigo)](https://github.com/ggh-png/ffw-sh5-grasp/releases/tag/1.1.1)

## 처음이라면 이 순서로 읽기

설명서를 처음부터 끝까지 읽을 필요는 없다.

1. [빠른 시작](getting-started.md)에서 설치하고 앱을 실행한다.
2. [화면과 조작](run.md)에서 키와 패널의 역할을 확인한다.
3. [모드 선택](control-modes.md)에서 MoveL, IK/FK, Whole-body 조합을 고른다.

문제가 생기면 증상별 [문제 해결](troubleshooting.md)로 바로 이동한다.

## 목적에 맞는 읽기 경로

=== "로봇을 조작하고 싶다"

    **빠른 시작 → 화면과 조작 → 모드 선택** 순서만 읽으면 된다.

    - [빠른 시작](getting-started.md)
    - [화면과 조작](run.md)
    - [모드 선택](control-modes.md)

=== "코드를 수정하고 싶다"

    먼저 target, command, physics의 차이를 이해한 뒤 코드 흐름을 따라간다.

    - [동작 원리](concepts.md)
    - [아키텍처와 데이터 흐름](overview.md)
    - [코드 읽기 시작](guide/index.md)
    - [테스트와 검증](testing.md)

=== "ROS2 경험으로 이해하고 싶다"

    기본 구조를 먼저 본 다음 ROS2 구성요소와 대응시킨다.

    - [아키텍처와 데이터 흐름](overview.md)
    - [ROS2 개발자용 대응표](guide/ros2-guide.md)

## 한눈에 보는 동작 흐름

```mermaid
flowchart LR
    U["키보드 · 슬라이더 · 3D gizmo"] --> T["손 또는 virtual-object target"]
    T --> IK["Whole-body / arm-only IK"]
    IK --> C["팔 · 리프트 · base 명령"]
    C --> S["swerve와 actuator 제어"]
    S --> P["MuJoCo physics"]
    P --> F["실제 pose · contact · 화면"]
    F --> IK
```

여기서 가장 중요한 구분은 다음 세 가지다.

| 구분 | 의미 | 예 |
|---|---|---|
| Target | 사용자가 원하는 상태 | 손 XYZ/RPY, virtual object pose |
| Command | 제어기가 계산한 입력 | 팔 목표각, lift 위치, base twist |
| State | 물리가 만든 실제 상태 | `qpos`, `qvel`, contact, 실제 손 pose |

UI는 로봇 상태를 순간 이동시키지 않는다. target을 바꾸면 solver와 controller가
command를 계산하고, MuJoCo physics가 다음 state를 만든다.

## 무엇을 할 수 있나

| 기능 | 요약 |
|---|---|
| 손 목표 | 양손 home-relative XYZ/RPY, jog, slider, 3D gizmo |
| 전신 제어 | base x/y/yaw + lift + 양팔 14축 bounded differential IK |
| Arm-only | Whole-body OFF에서 base/lift 자동 참여를 hard gate |
| 양팔 이동 | Bimanual MoveL, virtual object, captured relative pose |
| 충돌 대응 | 팔-팔·팔-몸체·팔/손-table reactive collision CBF |
| 모바일 베이스 | 실제 steer/drive actuator와 wheel-ground contact로 이동 |
| 파지 | 손가락 synergy와 contact force를 사용하며 물체를 강제로 붙이지 않음 |

!!! info "기능 범위"
    Collision avoidance는 가까운 장애물에 반응하는 안전 계층이지 경로 플래너가
    아니다. Whole-body OFF는 자동 IK의 base/lift 참여만 끄며 수동 주행까지 막지 않는다.

## Demo

<div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; border-radius: 8px;">
  <iframe
    src="https://www.youtube.com/embed/2LV_RsAGdz8"
    title="ffw-sh5-grasp demo"
    style="position: absolute; inset: 0; width: 100%; height: 100%; border: 0;"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
</div>
