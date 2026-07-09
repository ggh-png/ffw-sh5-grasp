# ffw-sh5-grasp

**ROBOTIS FFW-SH5**(양팔 7DOF×2 + HX5-D20 5지 핸드×2 + 모바일 베이스)가 **kinematic 치팅
없이, 오직 contact force만으로** 테이블 위 캔을 집어 드는 MuJoCo 물리 기반 텔레오퍼레이션
시뮬레이터다. 모바일 베이스는 실제 바퀴-지면 마찰로 구동되고, 텔레옵은 사람이 슬라이더와
키보드로 직접 조작하는 단일 네이티브 창 애플리케이션이다.

<figure class="hero-figure" markdown>
  ![full_scene.xml 렌더](assets/hero.jpg)
  <figcaption>models/full_scene.xml — 양팔 7DOF×2, HX5-D20 5지 핸드×2, 모바일 베이스(바퀴
  3개 실제 지면 마찰 구동), 실제 라벨 텍스처를 두른 캔.</figcaption>
</figure>

[![GitHub](https://img.shields.io/badge/GitHub-Repository-black?logo=github)](https://github.com/ggh-png/ffw-sh5-grasp)

---

## 이 문서 사이트는 두 갈래로 나뉜다

- ***정직한 물리***: 손가락-캔 접촉은 위치 대입이 아니라 force-limited position
  actuator가 만드는 실제 접촉력으로만 성립한다. `data.qpos[...]`로 물리 상태를 직접
  덮어쓰는 코드는 리셋/초기 배치를 제외하면 이 레포 어디에도 없다.
- ***실제 바퀴 마찰 주행***: 모바일 베이스는 가상 액추에이터가 아니라 바퀴 3개의 실제
  조향+구동 관절과 지면 마찰 접촉으로 밀린다(슬립 0.8%로 확인).
- ***MuJoCo 3.10 (Python)***: 별도 물리 엔진 래퍼 없이 MuJoCo의 Python 바인딩만으로
  IK, 토크 제어, grasp synergy, 렌더링까지 전부 직접 구현한다.

Phase 0–6 전체 완료. `tests/test_phase_{0,1,2,3,4,5,6}.py`는 캔 pick 회귀, 실제
바퀴 주행, 작은 상자 양손 squeeze/lift/drive, Cyclo Control식 bimanual MoveL UI와 수동 XYZ/RPY IK target 제어를 headless로
검증한다.

- **[프로젝트 개요](overview.md)** — 왜 이런 구조로 만들었는가. 앞서 실패한 두 번의
  시도, 세 모델로 나눈 이유, 설계 판단 6가지, 가장 비직관적이었던 버그들, 현재 상태.
- **[MuJoCo 튜토리얼](guide/index.md)** — `src/` 코드 파일 하나하나가 각각 무엇을
  구현하는지, 그리고 그 파일들이 서로 어떻게 호출/연결되어 하나의 텔레옵 앱으로
  합쳐지는지를 파일 단위로 설명한다.
- **[직접 실행하기](run.md)** — 설치, 테스트, 텔레옵 앱 실행 방법.

---

## Features and Updates

### (Session 8, 2026-07) Phase 5 — 모바일 베이스 완성
- 평면 가상 액추에이터 → **실제 바퀴 3개 조향/구동 + 지면 마찰**로 재작업
- 바퀴-바닥 접촉 강성 튜닝(로봇 실제 무게의 28배였던 반발력 문제 해결)
- 팔/손가락 미러링 버그 다수 수정, 캔에 실제 STL 형상 위 라벨 텍스처 적용

### (Session 11+, 2026-07) Phase 6 — 작은 상자 양손 squeeze lift
- `can`/`box` 실행 시나리오 분리, 작은 상자(20×20×28cm) 추가
- 양손이 상자 양옆을 눌러 마찰로 들고, bimanual constraint로 상대 pose 유지
- EE 제어는 Cyclo Control 패널의 `MoveL`/`Bimanual MoveL` marker target과 숫자 X/Y/Z + Roll/Pitch/Yaw 기준

### (Session 6-7) Phase 4 — 전신 조립 + 텔레옵 UI
- 양팔/양손/헤드/리프트 전체 조립, 단일 네이티브 창(GLFW+ImGui) 텔레옵
- 팔 액추에이터를 `<position>`에서 `<motor>` + PD + 중력 feedforward로 교체

### (Session 4-5) Phase 3 — 6DOF IK
- 계층형 damped-least-squares IK, backtracking line search, multistart
- 통합 pick(접근 → 파지 → 들어올리기) 10/10

### (Session 2-3) Phase 1-2 — 손 콜리전 + 고정 손 grasp
- 메시 콜리전 → 캡슐 근사(실측 AABB 기반)
- grasp synergy 스칼라 두 개 + force-limited actuator로 순응 그립 구현

See more details in [프로젝트 개요](overview.md) and [MuJoCo 튜토리얼](guide/index.md).
