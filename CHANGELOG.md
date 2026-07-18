# Changelog

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
