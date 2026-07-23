# 릴리스 기록

## 1.2.0 — Custom Kinematics & Compact Multi-Viewport UI

2026-07-23 발행. [GitHub Release](https://github.com/ggh-png/ffw-sh5-grasp/releases/tag/1.2.0)

### 사용자에게 보이는 변화

- ImGui 도구가 메인 MuJoCo 창 밖의 실제 네이티브 OS 창으로 분리
- 6개 도구 창을 `Control Center`와 `Diagnostics` 두 워크스페이스로 통합
- Target, 양팔 IK/FK, Robot/Grasp, Kinematic Tree, Joint Monitor를 탭으로 정리
- multi-viewport desktop 좌표를 반영해 3D gizmo 중심을 선택 target에 정확히 정렬
- 수식 전개, 기능별 알고리즘→코드 대응, 제어 다이어그램을 개발자 문서에 확장

### 기구학과 제어

- 컴파일된 MJCF의 body·joint·site 고정 변환을 `KinematicTree`로 한 번 복사
- hinge/slide FK와 world-aligned 6×N geometric Jacobian을 NumPy로 직접 계산
- collision 최근접점의 translational Jacobian도 같은 트리에서 직접 계산
- 단일 팔 DLS IK와 18-DOF Whole-Body IK가 하나의 custom tree를 공유
- 앱 런타임에서 `mj_forward`, MuJoCo Jacobian API, `site_xpos/site_xmat` FK 우회 제거
- MuJoCo는 `mj_step` 기반 접촉·동역학·actuator·렌더링에만 사용

### 검증

- 런타임 custom-kinematics dependency gate 통과
- FK/Jacobian 중앙 유한차분 최대 오차 `2.33e-10`
- collision distance gradient 중앙 유한차분 최대 오차 `7.39e-11`
- 단일 팔 무작위 IK 100/100 수렴, pick 10/10
- compact UI gate: workspace 2개, 실제 플랫폼 viewport는 주 창 포함 3개
- Phase 0–6, Whole-Body 통합, strict 문서 빌드 통과

전체 diff: [1.1.1...1.2.0](https://github.com/ggh-png/ffw-sh5-grasp/compare/1.1.1...1.2.0)

## 1.1.1 — Whole-body Control Toggle

2026-07-19 발행. [GitHub Release](https://github.com/ggh-png/ffw-sh5-grasp/releases/tag/1.1.1)

### 사용자에게 보이는 변화

- `Lift / Utilities`에 **Whole-body Control ON/OFF** 버튼 추가
- 상태줄에 `ON` 또는 `OFF (arm-only)`와 실제 body command 표시
- 전환 시 양손/virtual-object target의 world pose 보존
- OFF에서도 keyboard base와 manual lift 사용 가능
- `V`/checkbox collision CBF 시각화와 완화된 3 cm/1 cm 기준

### 제어와 알고리즘

- OFF에서 base x/y/yaw와 lift differential velocity hard pin
- 공용 ROS-free pose/FK/Jacobian 계층
- bounded whole-body BVLS와 joint-limit CBF
- Bimanual rigid-grasp relative-pose task
- arm-arm/body/table signed-distance collision CBF
- 키 해제 제동과 manual-to-WBIK rebase 안정화

### 검증

- Phase 0–6 전체 통과
- 일반/Bimanual ON→OFF→ON pose 불변성
- arm-only base/lift zero + 양팔 error descent
- collision gradient/CBF와 visualization consistency
- 무작위 WBIK 40회와 실제 바퀴 4방향 추종
- strict 문서 빌드

전체 diff: [1.1.0...1.1.1](https://github.com/ggh-png/ffw-sh5-grasp/compare/1.1.0...1.1.1)

## 1.1.0 — ROS-free Whole-body IK

- base 3축 + lift + 양팔 14축 differential IK
- 실제 steer/drive actuator와 wheel-ground contact 기반 mobile control
- 키보드 해제 후 잔류 wheel command와 원점 복귀 방지

## 이전 버전

`1.0.0`, `0.1.0`, `0.0.2`, `0.0.1`과 `phase-0`~`phase-4` 태그가 있다. 최신
사용법과 검증 기준은 항상 현재 문서를 우선한다.
