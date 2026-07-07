"""Phase 5 -- mobile base teleop: smoothing math ported from the sibling
ffw-sh5-teleoperation repo's InputManager/main.cpp constants (see
ffw-sh5-mobile-and-box-plan.md S1.2), re-expressed first for MuJoCo velocity actuators
directly on a virtual planar joint, then (Session 8 후속) for real per-wheel steer+drive
actuators whose ground friction is what actually propels the base -- see `SwerveDrive` below.

This module is pure math -- no MjModel/MjData, no qpos access. It holds only its own
internal smoothed-velocity/steer-angle state and returns actuator ctrl targets; the caller
writes those into `data.ctrl[...]` (never `data.qpos[...]`), exactly like every other
actuated joint in this project. Base yaw (used to rotate the local command into world frame)
is read from the live simulation each call, not integrated separately here -- MuJoCo's own
base_yaw joint qpos is the single source of truth for the robot's heading.
"""

import math

import numpy as np

K_SPEED = 0.5    # m/s, commanded local speed while a translate key is held
K_MAX = 0.55     # m/s, hard cap on the smoothed local speed
K_ACCEL = 3.0    # 1/s, exponential approach rate toward the target velocity
K_BRAKE = 6.0    # 1/s, exponential decay rate once keys are released
K_YAW = 1.2      # rad/s, commanded yaw rate while a turn key is held
YAW_FOLLOW = 4.0   # 1/s, exponential approach rate toward the target yaw rate
YAW_DECAY = 10.0   # 1/s, exponential decay rate once turn keys are released
VEL_ZERO_EPS = 0.001


class BaseTeleop:
    """키 입력을 "부드러운" 몸체-프레임(body-frame) 속도/각속도로 바꿔주는 순수 계산
    모듈. 키를 누르는 순간 목표 속도로 순간이동하지 않고 지수함수로 가속/제동하며,
    실제로 바퀴를 굴리는 건 이 클래스가 아니라 아래 SwerveDrive다."""

    def __init__(self):
        self.v_local = np.zeros(2)  # smoothed (forward, left) velocity, robot-local frame
        self.w = 0.0                # smoothed yaw rate (rad/s)

    def update(self, keys, dt, yaw):
        """keys: dict/set-like supporting `keys["w"]` truthiness for 'w','a','s','d',
        'left','right' (turn). yaw: current base_yaw joint angle (rad), read from the live
        simulation. Returns (vx_world, vy_world, vyaw) -- ctrl targets for the base's three
        velocity actuators."""
        # 눌린 키를 -1/0/+1 축 성분으로 변환 (전진-후진, 좌-우, 좌회전-우회전).
        fwd = (1.0 if keys.get("w") else 0.0) - (1.0 if keys.get("s") else 0.0)
        left = (1.0 if keys.get("a") else 0.0) - (1.0 if keys.get("d") else 0.0)
        turn = (1.0 if keys.get("left") else 0.0) - (1.0 if keys.get("right") else 0.0)

        target_local = np.array([fwd, left])
        norm = np.linalg.norm(target_local)
        if norm > 1e-9:
            target_local = target_local / norm * K_SPEED
        target_w = turn * K_YAW

        if turn != 0.0:
            # In-place rotation: kill translation immediately for a clear pivot, matching
            # the reference teleop's feel (S1.2 "제자리 회전 시 병진 속도 즉시 0").
            self.v_local[:] = 0.0
        elif fwd != 0.0 or left != 0.0:
            # 목표 속도를 향해 지수적으로 접근(가속) -- 순간 가속이 아니라 K_ACCEL로
            # 정해진 시간상수만큼 부드럽게 따라간다.
            self.v_local += (target_local - self.v_local) * (1.0 - math.exp(-K_ACCEL * dt))
        else:
            # 키를 뗐으면 지수적으로 감쇠(제동)해서 0으로 수렴.
            self.v_local *= math.exp(-K_BRAKE * dt)
            if np.linalg.norm(self.v_local) < VEL_ZERO_EPS:
                self.v_local[:] = 0.0

        speed = np.linalg.norm(self.v_local)
        if speed > K_MAX:
            self.v_local *= K_MAX / speed

        # yaw(회전) 각속도도 위와 동일한 가속/감쇠 패턴.
        if target_w != 0.0:
            self.w += (target_w - self.w) * (1.0 - math.exp(-YAW_FOLLOW * dt))
        else:
            self.w *= math.exp(-YAW_DECAY * dt)
            if abs(self.w) < VEL_ZERO_EPS:
                self.w = 0.0

        # 몸체-로컬 속도를 현재 yaw만큼 회전시켜 월드 프레임 속도로 변환 --
        # base_yaw 조인트의 실시간 qpos를 그대로 신뢰(별도로 적분하지 않음).
        cy, sy = math.cos(yaw), math.sin(yaw)
        vx_world = cy * self.v_local[0] - sy * self.v_local[1]
        vy_world = sy * self.v_local[0] + cy * self.v_local[1]
        return vx_world, vy_world, self.w


# Body-frame mounting positions (x=forward, y=left), matching models/full_scene.xml's wheel
# body `pos` attributes exactly.
# 바퀴별 몸체-프레임 장착 위치(x=전방, y=좌측) -- models/full_scene.xml의 바퀴 body
# pos 속성과 정확히 일치해야 한다.
WHEEL_POS = {
    "left_wheel": (0.1371, 0.2554),
    "right_wheel": (0.1371, -0.2554),
    "rear_wheel": (-0.2899, 0.0),
}
WHEEL_RADIUS = 0.09   # m, matches the wheel collision cylinder's size
STEER_RANGE = (-1.58, 1.58)  # rad, matches the vendored wheel_steer joint classes' range
STEER_ZERO_SPEED_EPS = 0.01  # m/s, below this hold the previous steer angle (atan2 near
                             # zero velocity is noisy/undefined, would make wheels jitter)


class SwerveDrive:
    """Converts BaseTeleop's smoothed body-frame velocity into per-wheel (steer_angle,
    drive_angular_velocity) commands -- a standard independent-steering ("swerve") drive
    kinematic, needed here (rather than simple differential drive) because the vendored
    wheel_steer joints only have +-1.58rad (~+-90.5 deg) of range, not a full continuous
    rotation: for any desired direction there is exactly one of {angle, angle+-pi} that
    falls in that range (driving the wheel "backwards" covers the other half of the circle),
    so which representation to use is not optional the way it would be with unlimited steer
    range -- picking wrong means silently clamping to the wrong direction instead of an
    equivalent flipped one.
    """

    def __init__(self):
        self.base = BaseTeleop()
        # 바퀴별 "현재 조향각" 기억 -- 속도가 0에 가까울 때 방향을 유지하거나,
        # 경계에서 더 가까운 표현을 고르는 데 쓰인다.
        self.steer_angle = {name: 0.0 for name in WHEEL_POS}

    def update(self, keys, dt, yaw):
        """Same `keys`/`dt`/`yaw` contract as BaseTeleop.update. Returns
        {wheel_name: (steer_angle_rad, drive_angvel_radps)} for all three wheels."""
        # (한글) BaseTeleop이 만든 몸체-프레임 속도를, 바퀴 3개 각각의 조향각+구동
        # 각속도로 변환한다 -- 실제로 데이터를 물리 시뮬레이션에 쓰는 건 호출부
        # (teleop_app.py)의 몫이고, 이 함수는 순수 계산만 한다.
        # 1) BaseTeleop으로 부드러운 몸체-프레임 속도(전진/좌우)+각속도를 얻는다.
        self.base.update(keys, dt, yaw)
        vx_body, vy_body = self.base.v_local
        omega = self.base.w

        commands = {}
        for name, (wx, wy) in WHEEL_POS.items():
            # 2) 각 바퀴 위치에서의 실제 속도 = 몸체 속도 + omega x r
            #    (r = 회전 중심에서 그 바퀴까지의 위치 벡터).
            # Wheel velocity = body velocity + omega x r (r = wheel position from the base's
            # own rotation center, which is base_link's origin -- the base_x/base_y joints).
            vwx = vx_body - omega * wy
            vwy = vy_body + omega * wx
            speed = math.hypot(vwx, vwy)

            if speed < STEER_ZERO_SPEED_EPS:
                # 속도가 거의 0이면 atan2가 불안정(잡음)해지므로 방향은 바꾸지 않고
                # 이전 조향각을 유지, 구동속도만 0으로.
                angle = self.steer_angle[name]
                signed_speed = 0.0
            else:
                # 3) 원하는 바퀴 진행 방향(angle)과 그 방향으로 구를 속도(signed_speed).
                angle = math.atan2(vwy, vwx)
                signed_speed = speed
                # Canonicalize into the wheel's +-90.5deg steer range: exactly one of
                # {angle, angle+pi, angle-pi} always falls inside a >90deg-wide range, so
                # this is required for correctness, not just an optimization.
                # (한글) 조향 관절은 +-90.5도까지만 돌아가므로, 그 범위를 벗어나면
                # 180도 반대로 돌리고 구동 방향(signed_speed)도 반전시켜 같은 결과를
                # 낸다 -- "다른 표현으로 바꾸는" 게 아니라 물리적으로 꼭 필요한 처리.
                if angle > STEER_RANGE[1]:
                    angle -= math.pi
                    signed_speed = -signed_speed
                elif angle < STEER_RANGE[0]:
                    angle += math.pi
                    signed_speed = -signed_speed
                # If the un-flipped representation is ALSO in range, prefer whichever is
                # closer to the wheel's current angle (minimize steering motion) -- mainly
                # matters right at the range boundary where both are valid.
                # (한글) 뒤집지 않은 각도도 범위 안에 들어온다면(경계 부근), 현재
                # 조향각에서 더 가까운 쪽을 선택해 불필요한 조향 움직임을 줄인다.
                flipped = angle - math.pi if angle > 0 else angle + math.pi
                if STEER_RANGE[0] <= flipped <= STEER_RANGE[1]:
                    cur = self.steer_angle[name]
                    if abs(_angle_diff(flipped, cur)) < abs(_angle_diff(angle, cur)):
                        angle, signed_speed = flipped, -signed_speed

            self.steer_angle[name] = angle
            # 선속도를 바퀴 반지름으로 나눠 각속도(<velocity> 액추에이터의 ctrl 단위)로 변환.
            commands[name] = (angle, signed_speed / WHEEL_RADIUS)
        return commands


def _angle_diff(a, b):
    """Smallest signed difference a-b, wrapped to [-pi, pi].
    (한글) 두 각도의 최단 signed 차이 -- 예: 179도와 -179도는 실제로 2도 차이."""
    d = (a - b) % (2 * math.pi)
    if d > math.pi:
        d -= 2 * math.pi
    return d
