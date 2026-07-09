# `src/base_teleop.py` — 모바일 베이스 조작감 + ROBOTIS식 스워브 컨트롤러

## 이 파일이 하는 일

MjModel/MjData를 전혀 참조하지 않는 **순수 계산 모듈**이다. 두 개의 클래스가
단계적으로 쌓인다:

- **`BaseTeleop`** — 눌린 키를 "부드러운" 몸체-프레임(body-frame) 속도/각속도로
  바꾼다. 목표 속도로 순간이동하지 않고 지수함수로 가속/제동한다.
- **`SwerveDrive`** — `BaseTeleop`을 감싸서, 그 몸체 속도를 바퀴 3개 각각의
  (조향각, 구동각속도) 명령으로 변환한다. 구조는 ROBOTIS AI Worker의
  `ffw_swerve_drive_controller` 알고리즘을 Python/MuJoCo용으로 옮긴 것이다.

## 구현: 가속/제동 스무딩

```python title="src/base_teleop.py — BaseTeleop.update"
if turn != 0.0:
    self.v_local[:] = 0.0                      # 제자리 회전 시 병진 속도 즉시 0
elif fwd != 0.0 or left != 0.0:
    self.v_local += (target_local - self.v_local) * (1.0 - math.exp(-K_ACCEL * dt))
else:
    self.v_local *= math.exp(-K_BRAKE * dt)    # 키를 떼면 지수적으로 감쇠
    if np.linalg.norm(self.v_local) < VEL_ZERO_EPS:
        self.v_local[:] = 0.0
```

`K_SPEED`/`K_ACCEL`/`K_BRAKE`/`K_YAW`/`YAW_FOLLOW`/`YAW_DECAY` 같은 상수들은
이 프로젝트의 형제 저장소(`ffw-sh5-teleoperation`)의 C++ 조작감을 그대로 이식한
값이다. 마지막으로 몸체-로컬 속도를 현재 `base_yaw`만큼 회전시켜 월드 프레임
속도로 변환하는데, **yaw는 매 호출 실시간 시뮬레이션에서 읽어올 뿐 이 모듈이
따로 적분하지 않는다** — MuJoCo의 `base_yaw` 조인트 qpos가 로봇 헤딩의 유일한
출처라는 원칙을 지킨다.

## 구현: ROBOTIS식 스워브 알고리즘

핵심 입력은 ROS의 `cmd_vel`처럼 해석되는 몸체-프레임 속도
`(vx_body, vy_body, wz)`다. 각 바퀴 모듈 위치 `(x, y)`에서 필요한 접선 속도는
다음처럼 계산한다.

```python title="src/base_teleop.py — SwerveDrive.update 핵심"
wheel_vel_x = vx_body - wz * module_y
wheel_vel_y = vy_body + wz * module_x
target_robot_angle = math.atan2(wheel_vel_y, wheel_vel_x + 1e-9)
target_speed = math.hypot(wheel_vel_x, wheel_vel_y)
```

그다음 공식 컨트롤러의 처리 흐름을 그대로 따른다.

- **모듈 angle offset**: 모델별 조향 기준축 보정값을 `MODULE_ANGLE_OFFSETS`로 둔다.
  현재 모델은 세 모듈 모두 `0.0`이다.
- **180도 최적화**: 현재 조향각에서 목표각까지 90도보다 멀면 목표 조향각에 `π`를
  더하고 바퀴 회전 방향을 뒤집는다. "뒤로 굴려서 같은 벡터를 만드는" 스워브의
  표준 처리다.
- **±π 경계 처리**: `+π/-π` 근처에서 불필요하게 크게 도는 명령을 피하도록 한 번 더
  방향을 보정한다.
- **반전 FSM**: 바퀴 회전 방향이 바뀔 때 즉시 속도 부호를 뒤집지 않고
  `NORMAL → DECELERATING → STEERING → ACCELERATING` 순서로 감속, 조향, 재가속한다.
- **조향 rate limit**: 한 프레임에 조향 목표가 너무 멀리 튀지 않도록
  `STEERING_ANGULAR_VELOCITY_LIMIT`으로 제한한다.
- **alignment gating**: 조향 오차가 큰 동안에는 바퀴 구동속도를 0으로 막는다.
  세 모듈 중 하나라도 정렬 전이면 전체 구동속도를 0으로 둔다.
- **wheel speed limit**: 최종 선속도를 `WHEEL_RADIUS`로 나눠 MuJoCo `<velocity>`
  액추에이터가 받는 각속도로 바꾸고, `WHEEL_SPEED_LIMIT`으로 clamp한다.

정지 명령일 때는 현재 실제 조향각을 유지하고 구동속도만 0으로 내보낸다. 그래서 키를
떼도 조향 모듈이 원점으로 되돌아가며 바닥을 긁지 않는다.

## 이 파일이 다른 파일과 합쳐지는 방식

- **`teleop_app.py`**가 유일한 사용자다. `TeleopApp._setup_sim()`에서 하나만
  생성(`self.base_drive = base_teleop.SwerveDrive()`)하고, `_step_physics()`에서
  매 프레임 한 번 `wheel_cmds = self.base_drive.update(...)`를 호출한다.
- `drive_keys`(방향키 눌림 상태)는 `TeleopApp._read_drive_and_lift_keys()`가 만들어
  넘겨주고, `base_yaw`는 `data.qpos[self.base_yaw_qadr]`에서 직접 읽어 넘긴다.
  현재 세 바퀴의 조향 qpos와 구동 qvel도 함께 넘겨서 180도 반전 판단과 alignment
  gating이 실제 물리 상태 기준으로 동작하게 한다.
- 반환된 `wheel_cmds`는 `_step_physics()`의 물리 서브스텝 루프 안에서
  `data.ctrl[self.wheel_steer_aids[wheel]] = steer_angle` /
  `data.ctrl[self.wheel_drive_aids[wheel]] = drive_angvel`로 그대로 기록된다 —
  `base_teleop.py` 자신은 `data.ctrl`을 단 한 번도 직접 건드리지 않는다.
- `grasp.py`/`ik.py`/`arm_control.py`와는 아무 관계가 없다 — 베이스 주행과 팔/손
  제어는 완전히 독립된 액추에이터 집합이라, `teleop_app.py`의 같은 물리 스텝
  안에서 나란히 실행될 뿐이다.
