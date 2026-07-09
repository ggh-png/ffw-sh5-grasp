# `src/base_teleop.py`

키보드 입력을 ROBOTIS-style swerve drive 명령으로 변환한다.

## 역할

| 단계 | 내용 |
|---|---|
| 입력 smoothing | 전진/후진/strafe/yaw 키를 부드러운 body-frame 속도로 변환 |
| swerve IK | body velocity를 3개 wheel module의 조향각/바퀴 속도로 변환 |
| 반전 처리 | 180도 steering flip, 감속-조향-가속 sequence |
| 안전 처리 | steering rate limit, alignment gating, wheel speed clamp |

## 클래스와 함수

| 이름 | 역할 |
|---|---|
| `BaseTeleop` | 키 입력을 smoothed velocity command로 변환 |
| `BaseTeleop.update(keys, dt, yaw=0.0)` | `vx_world, vy_world, wz`를 반환 |
| `ReversalPhase` | wheel 방향 반전 상태 enum |
| `SwerveDrive` | 3-wheel swerve command generator |
| `SwerveDrive.update(keys, dt, yaw, steering_positions, wheel_velocities)` | wheel별 `(steer_angle, drive_angvel)` 반환 |
| `_limit_steering_rate(current, target, dt)` | 조향각 변화량 제한 |
| `_normalize_angle(angle)` | 각도를 `[-pi, pi)`로 정규화 |
| `_shortest_angular_distance(start, target)` | 최단 각도 차 계산 |
| `_clamp(value, lo, hi)` | 값 clamp |

## 함수 흐름

```mermaid
flowchart TD
    A["teleop_app._read_drive_and_lift_keys<br>키보드 주행 입력 수집"] --> B["drive_keys<br>전후/좌우/회전 명령 상태"]
    B --> C["SwerveDrive.update(keys, dt, yaw, steering, wheel_vel)<br>swerve wheel 목표각/속도 생성"]
    C --> D["BaseTeleop.update()<br>입력 가속도 제한과 속도 smoothing"]
    D --> E["smoothed vx/vy/wz<br>부드럽게 변한 body velocity"]
    E --> F["per-wheel swerve inverse kinematics<br>각 바퀴별 조향각과 구동속도 계산"]
    F --> G["180 deg steering optimization<br>조향을 덜 돌리도록 바퀴 회전 방향 반전"]
    G --> H["_update_reversal_phase()<br>반전 상태 전환을 안정적으로 관리"]
    H --> I["_limit_steering_rate()<br>조향 모터 회전 속도 제한"]
    I --> J["alignment gating<br>조향 정렬 전 과한 구동 억제"]
    J --> K["wheel speed clamp<br>바퀴 속도 상한 적용"]
    K --> L["{wheel: steer_angle, drive_angvel}<br>바퀴별 최종 명령 반환"]
    L --> M["teleop_app writes data.ctrl<br>반환 명령을 actuator에 기록"]
```

## 출력 형식

```python
{
    "left_wheel": (steer_angle_rad, drive_angvel_rad_s),
    "right_wheel": (steer_angle_rad, drive_angvel_rad_s),
    "rear_wheel": (steer_angle_rad, drive_angvel_rad_s),
}
```

## 사용 위치

`teleop_app.py`가 매 render frame마다 한 번 호출하고, 반환된 wheel command를 물리
substep마다 `data.ctrl`에 반복 적용한다.
