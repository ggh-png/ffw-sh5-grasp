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
    def __init__(self):
        self.v_local = np.zeros(2)  # smoothed (forward, left) velocity, robot-local frame
        self.w = 0.0                # smoothed yaw rate (rad/s)

    def update(self, keys, dt, yaw):
        """keys: dict/set-like supporting `keys["w"]` truthiness for 'w','a','s','d',
        'left','right' (turn). yaw: current base_yaw joint angle (rad), read from the live
        simulation. Returns (vx_world, vy_world, vyaw) -- ctrl targets for the base's three
        velocity actuators."""
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
            self.v_local += (target_local - self.v_local) * (1.0 - math.exp(-K_ACCEL * dt))
        else:
            self.v_local *= math.exp(-K_BRAKE * dt)
            if np.linalg.norm(self.v_local) < VEL_ZERO_EPS:
                self.v_local[:] = 0.0

        speed = np.linalg.norm(self.v_local)
        if speed > K_MAX:
            self.v_local *= K_MAX / speed

        if target_w != 0.0:
            self.w += (target_w - self.w) * (1.0 - math.exp(-YAW_FOLLOW * dt))
        else:
            self.w *= math.exp(-YAW_DECAY * dt)
            if abs(self.w) < VEL_ZERO_EPS:
                self.w = 0.0

        cy, sy = math.cos(yaw), math.sin(yaw)
        vx_world = cy * self.v_local[0] - sy * self.v_local[1]
        vy_world = sy * self.v_local[0] + cy * self.v_local[1]
        return vx_world, vy_world, self.w


# Body-frame mounting positions (x=forward, y=left), matching models/full_scene.xml's wheel
# body `pos` attributes exactly.
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
        self.steer_angle = {name: 0.0 for name in WHEEL_POS}

    def update(self, keys, dt, yaw):
        """Same `keys`/`dt`/`yaw` contract as BaseTeleop.update. Returns
        {wheel_name: (steer_angle_rad, drive_angvel_radps)} for all three wheels."""
        self.base.update(keys, dt, yaw)
        vx_body, vy_body = self.base.v_local
        omega = self.base.w

        commands = {}
        for name, (wx, wy) in WHEEL_POS.items():
            # Wheel velocity = body velocity + omega x r (r = wheel position from the base's
            # own rotation center, which is base_link's origin -- the base_x/base_y joints).
            vwx = vx_body - omega * wy
            vwy = vy_body + omega * wx
            speed = math.hypot(vwx, vwy)

            if speed < STEER_ZERO_SPEED_EPS:
                angle = self.steer_angle[name]
                signed_speed = 0.0
            else:
                angle = math.atan2(vwy, vwx)
                signed_speed = speed
                # Canonicalize into the wheel's +-90.5deg steer range: exactly one of
                # {angle, angle+pi, angle-pi} always falls inside a >90deg-wide range, so
                # this is required for correctness, not just an optimization.
                if angle > STEER_RANGE[1]:
                    angle -= math.pi
                    signed_speed = -signed_speed
                elif angle < STEER_RANGE[0]:
                    angle += math.pi
                    signed_speed = -signed_speed
                # If the un-flipped representation is ALSO in range, prefer whichever is
                # closer to the wheel's current angle (minimize steering motion) -- mainly
                # matters right at the range boundary where both are valid.
                flipped = angle - math.pi if angle > 0 else angle + math.pi
                if STEER_RANGE[0] <= flipped <= STEER_RANGE[1]:
                    cur = self.steer_angle[name]
                    if abs(_angle_diff(flipped, cur)) < abs(_angle_diff(angle, cur)):
                        angle, signed_speed = flipped, -signed_speed

            self.steer_angle[name] = angle
            commands[name] = (angle, signed_speed / WHEEL_RADIUS)
        return commands


def _angle_diff(a, b):
    """Smallest signed difference a-b, wrapped to [-pi, pi]."""
    d = (a - b) % (2 * math.pi)
    if d > math.pi:
        d -= 2 * math.pi
    return d
