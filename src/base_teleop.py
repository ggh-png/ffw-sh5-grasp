"""Phase 5 -- mobile base WASD teleop: smoothing math ported from the sibling
ffw-sh5-teleoperation repo's InputManager/main.cpp constants (see
ffw-sh5-mobile-and-box-plan.md S1.2), re-expressed for MuJoCo's velocity actuators instead
of that repo's kinematic transform + convex-sweep-test base.

This module is pure math -- no MjModel/MjData, no qpos access. It holds only its own
internal smoothed-velocity state and returns world-frame velocity commands; the caller is
responsible for writing those into `data.ctrl[...]` for the base_x/base_y/base_yaw velocity
actuators (never `data.qpos[...]`), exactly like every other actuated joint in this project.
Base yaw (used to rotate the local command into world frame) is read from the live
simulation each call, not integrated separately here -- MuJoCo's own base_yaw joint qpos is
the single source of truth for the robot's heading.
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
