# Changelog

## 1.2.0 — 2026-07-23

- 컴파일된 MJCF에서 body–joint–site 트리를 만드는 `KinematicTree`와 자체 FK 구현
- hinge/slide site Jacobian과 충돌점 Jacobian을 직접 계산하고 런타임의
  `mj_forward`/`mj_jacSite`/`mj_jac` 및 engine site-pose 우회 제거
- 단일 팔 DLS IK와 18-DOF Whole-Body IK가 동일한 custom kinematics 계층 공유
- ImGui 네이티브 multi-viewport로 도구를 실제 OS 창으로 분리하고 gizmo 좌표 정렬
- 6개 도구 창을 `Control Center`와 `Diagnostics` 두 tabbed workspace로 통합
- FK·Jacobian·DLS·collision CBF 수식 전개와 코드 대응, 다이어그램 중심 문서 재구성
- custom kinematics 의존성, 수치 Jacobian, UI workspace를 포함한 회귀 gate 강화

## 1.1.1 — 2026-07-19

- 전신 제어 ON/OFF UI와 무점프 whole-body/arm-only 전환
- OFF 상태에서 base x/y/yaw와 lift IK 속도를 정확히 0으로 고정
- 공용 FK/Jacobian 기반 ROS-free IK 개선과 Bimanual rigid-grasp 제약
- arm/상체/table reactive collision CBF, 시각화 토글, 완화된 3 cm 감시·1 cm 안전거리
- 스워브 정지/수동 handover 안정화 및 반복 headless 회귀 테스트

## 1.1.0

- ROS-free 18-DOF whole-body differential IK
- 실제 steer/drive actuator와 wheel-ground contact 기반 모바일 제어
- 키보드 해제 후 잔류 주행과 원점 복귀 방지
