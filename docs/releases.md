# 릴리스 기록

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
