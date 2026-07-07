# `src/base_teleop.py` — 모바일 베이스 조작감 + 스워브 드라이브 기구학

## 이 파일이 하는 일

MjModel/MjData를 전혀 참조하지 않는 **순수 계산 모듈**이다. 두 개의 클래스가
단계적으로 쌓인다:

- **`BaseTeleop`** — 눌린 키를 "부드러운" 몸체-프레임(body-frame) 속도/각속도로
  바꾼다. 목표 속도로 순간이동하지 않고 지수함수로 가속/제동한다.
- **`SwerveDrive`** — `BaseTeleop`을 감싸서, 그 몸체 속도를 바퀴 3개 각각의
  (조향각, 구동각속도) 명령으로 변환한다. 실제로 바퀴를 굴리는 쪽은 이 클래스다.

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

## 구현: 스워브 드라이브가 필요한 이유

벤더 모델의 바퀴 조향 관절은 `±1.58rad`(약 ±90.5°)로 제한돼 있어서 완전 회전이
안 된다. 임의의 방향으로 가려면, 그 range 안에 들어오는 `{angle, angle±π}` 중
정확히 하나를 골라야 한다(반대 방향으로 굴리면 나머지 절반의 원을 커버) — 이게
`SwerveDrive`가 단순 차동구동이 아니라 진짜 스워브(독립 조향) 기구학을 구현하는
이유다.

```python title="src/base_teleop.py — SwerveDrive.update 핵심"
vwx = vx_body - omega * wy      # 바퀴 위치에서의 실제 속도 = 몸체속도 + omega x r
vwy = vy_body + omega * wx
angle = math.atan2(vwy, vwx)
signed_speed = speed

# 조향 range를 벗어나면 180도 반대로 돌리고 구동 방향도 반전 -- 같은 결과를
# 내는 필수 처리(선택 사항이 아니다).
if angle > STEER_RANGE[1]:
    angle -= math.pi
    signed_speed = -signed_speed
elif angle < STEER_RANGE[0]:
    angle += math.pi
    signed_speed = -signed_speed

# 뒤집지 않은 표현도 range 안에 들어온다면(경계 부근), 현재 조향각에
# 더 가까운 쪽을 선택해 불필요한 조향 움직임을 줄인다.
flipped = angle - math.pi if angle > 0 else angle + math.pi
if STEER_RANGE[0] <= flipped <= STEER_RANGE[1]:
    cur = self.steer_angle[name]
    if abs(_angle_diff(flipped, cur)) < abs(_angle_diff(angle, cur)):
        angle, signed_speed = flipped, -signed_speed
```

속도가 `STEER_ZERO_SPEED_EPS` 아래로 떨어지면 `atan2`가 잡음에 취약해지므로
조향각은 이전 값을 유지하고 구동속도만 0으로 만든다. 마지막에 선속도를 바퀴
반지름(`WHEEL_RADIUS`)으로 나눠 `<velocity>` 액추에이터가 기대하는 각속도 단위로
바꾼다.

## 이 파일이 다른 파일과 합쳐지는 방식

- **`teleop_app.py`**가 유일한 사용자다. `TeleopApp._setup_sim()`에서 하나만
  생성(`self.base_drive = base_teleop.SwerveDrive()`)하고, `_step_physics()`에서
  매 프레임 한 번 `wheel_cmds = self.base_drive.update(drive_keys, self.frame_dt,
  base_yaw)`를 호출한다.
- `drive_keys`(방향키 눌림 상태)는 `TeleopApp._read_drive_and_lift_keys()`가 만들어
  넘겨주고, `base_yaw`는 `data.qpos[self.base_yaw_qadr]`에서 직접 읽어 넘긴다 —
  즉 이 모듈은 GLFW 키보드 API도, MuJoCo API도 전혀 알지 못하고 순수하게
  `(keys, dt, yaw)` 세 값만 받아 `(steer_angle, drive_angvel)` 딕셔너리를
  돌려준다.
- 반환된 `wheel_cmds`는 `_step_physics()`의 물리 서브스텝 루프 안에서
  `data.ctrl[self.wheel_steer_aids[wheel]] = steer_angle` /
  `data.ctrl[self.wheel_drive_aids[wheel]] = drive_angvel`로 그대로 기록된다 —
  `base_teleop.py` 자신은 `data.ctrl`을 단 한 번도 직접 건드리지 않는다.
- `grasp.py`/`ik.py`/`arm_control.py`와는 아무 관계가 없다 — 베이스 주행과 팔/손
  제어는 완전히 독립된 액추에이터 집합이라, `teleop_app.py`의 같은 물리 스텝
  안에서 나란히 실행될 뿐이다.
