# ffw-sh5-grasp

ROBOTIS FFW-SH5 양팔 로봇의 MuJoCo 기반 텔레오퍼레이션 프로젝트.
캔 grasp, 모바일 베이스 주행, Cyclo Control 스타일 양팔 target 조작을 제공한다.

<figure class="hero-figure" markdown>
  ![full_scene.xml 렌더](assets/hero.jpg)
  <figcaption>전신 FFW-SH5 모델: 양팔, 양손, 리프트, 헤드, 모바일 베이스, 캔.</figcaption>
</figure>

[![GitHub](https://img.shields.io/badge/GitHub-Repository-black?logo=github)](https://github.com/ggh-png/ffw-sh5-grasp)

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

## 기능

| 구분 | 내용 |
|---|---|
| 로봇 모델 | FFW-SH5 전신, 양팔 7DOF x2, HX5-D20 5지 핸드 x2 |
| 팔 제어 | 6DOF IK + 토크 제어(PD + gravity/Coriolis feedforward) |
| 손 제어 | grasp/thumb synergy, contact force 기반 grasp 판정 |
| 베이스 | 실제 바퀴 조향/구동 액추에이터와 지면 마찰 기반 주행 |
| UI | GLFW + MuJoCo renderer + ImGui |
| Target 조작 | 숫자 XYZ/RPY, 3D gizmo 화살표/회전 링 |
| 양팔 제어 | Cyclo-style `MoveL`, `Bimanual MoveL`, virtual object marker |
| 검증 | Phase 0-6 headless 테스트 |

## 문서 바로가기

- **[ROS2 개발자를 위한 튜토리얼](guide/ros2-guide.md)** — 처음이라면 여기부터. ROS2는
  알지만 MuJoCo/이 프로젝트는 처음인 사람을 위해 개념부터 순서대로 설명한다.
- [프로젝트 구조](overview.md) — 이미 구조를 아는 사람을 위한 빠른 요약(표/다이어그램 위주).
- [코드 가이드](guide/index.md) — `src/` 파일별 함수 레퍼런스, 필요할 때 찾아보는 용도.
- [실행 방법](run.md) — 설치, 조작법, 테스트 명령.

## 핵심 파일

```text
src/
├── teleop_app.py        # 앱 조립과 메인 루프
├── teleop_ui.py         # ImGui UI
├── teleop_render.py     # 렌더링/카메라/gizmo
├── teleop_targets.py    # target pose와 Cyclo 상태
├── base_teleop.py       # swerve drive
├── ik.py                # IK solver
├── arm_control.py       # 팔 토크 제어
└── grasp.py             # 손 synergy와 grasp 판정
```
