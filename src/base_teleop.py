"""Phase 5 -- mobile base teleop and ROBOTIS-style swerve drive control.

This module stays pure math: it reads no MuJoCo state by itself and writes no qpos.  The
caller gives key state plus, when available, the current steering/wheel state; the module
returns steering-position and wheel-velocity commands for the MuJoCo actuators.

`SwerveDrive` mirrors the structure of ROBOTIS AI Worker's
`ffw_swerve_drive_controller`: body-frame `cmd_vel`, per-module inverse kinematics,
module angle offsets, the 180 degree steering flip rule, steering velocity limiting,
alignment gating, wheel speed limits, and the DECEL -> STEERING -> ACCEL reversal sequence.
"""

import math
from enum import Enum

import numpy as np

K_SPEED = 0.5
K_MAX = 0.55
K_ACCEL = 1.0
K_BRAKE = 1.0
K_YAW = 2.0
YAW_FOLLOW = 1.0
YAW_DECAY = 1.0
VEL_ZERO_EPS = 0.001

WHEEL_POS = {
    "left_wheel": (0.1371, 0.2554),
    "right_wheel": (0.1371, -0.2554),
    "rear_wheel": (-0.2899, 0.0),
}
WHEELS = tuple(WHEEL_POS)
WHEEL_RADIUS = 0.09
STEER_RANGE = (-1.58, 1.58)
MODULE_ANGLE_OFFSETS = {name: 0.0 for name in WHEELS}
WHEEL_SPEED_LIMIT = (-50.0, 50.0)

LINEAR_VEL_DEADBAND = 0.001
ANGULAR_VEL_DEADBAND = 0.001
STEERING_ANGULAR_VELOCITY_LIMIT = 8.0
STEERING_ALIGNMENT_ANGLE_ERROR_THRESHOLD = 0.10
STEERING_ALIGNMENT_START_ANGLE_ERROR_THRESHOLD = 0.10
STEERING_ALIGNMENT_START_SPEED_ERROR_THRESHOLD = 0.10
STEERING_TOLERANCE = 0.03
REVERSAL_DECEL_RATE = 7.0
REVERSAL_ACCEL_RATE = 5.0
REVERSAL_THRESHOLD = 0.05


class BaseTeleop:
    """Convert keyboard intent into a smoothed body-frame Twist-like command."""

    def __init__(self):
        self.v_local = np.zeros(2)
        self.w = 0.0

    def update(self, keys, dt, yaw=0.0):
        fwd = (1.0 if keys.get("w") else 0.0) - (1.0 if keys.get("s") else 0.0)
        left = (1.0 if keys.get("a") else 0.0) - (1.0 if keys.get("d") else 0.0)
        turn = (1.0 if keys.get("left") else 0.0) - (1.0 if keys.get("right") else 0.0)

        target_local = np.array([fwd, left], dtype=float)
        norm = np.linalg.norm(target_local)
        if norm > 1e-9:
            target_local = target_local / norm * K_SPEED
        target_w = turn * K_YAW

        if turn != 0.0:
            self.v_local *= math.exp(-K_BRAKE * dt)
            if np.linalg.norm(self.v_local) < VEL_ZERO_EPS:
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

        if abs(self.v_local[0]) < LINEAR_VEL_DEADBAND:
            self.v_local[0] = 0.0
        if abs(self.v_local[1]) < LINEAR_VEL_DEADBAND:
            self.v_local[1] = 0.0
        if abs(self.w) < ANGULAR_VEL_DEADBAND:
            self.w = 0.0

        cy, sy = math.cos(yaw), math.sin(yaw)
        vx_world = cy * self.v_local[0] - sy * self.v_local[1]
        vy_world = sy * self.v_local[0] + cy * self.v_local[1]
        return vx_world, vy_world, self.w


class ReversalPhase(Enum):
    NORMAL = 0
    DECELERATING = 1
    STEERING = 2
    ACCELERATING = 3


class SwerveDrive:
    """ROBOTIS-style independent steering controller for this model's three modules."""

    def __init__(self):
        self.base = BaseTeleop()
        self.steer_angle = {name: 0.0 for name in WHEELS}
        self.previous_wheel_rotation_direction = {name: 1.0 for name in WHEELS}
        self.wheel_speed_scale = {name: 1.0 for name in WHEELS}
        self.reversal_phase = {name: ReversalPhase.NORMAL for name in WHEELS}
        self.reversal_target_steering_angle = {name: 0.0 for name in WHEELS}
        self.previous_commands = {name: 0.0 for name in WHEELS}

    def update(self, keys, dt, yaw=0.0, steering_positions=None, wheel_velocities=None):
        self.base.update(keys, dt, yaw)
        vx_body, vy_body = self.base.v_local
        wz = self.base.w
        cmd_zero = vx_body == 0.0 and vy_body == 0.0 and wz == 0.0

        if cmd_zero:
            for name in WHEELS:
                self.reversal_phase[name] = ReversalPhase.NORMAL
                self.wheel_speed_scale[name] = 1.0
                cur = self._current_steering(name, steering_positions)
                self.steer_angle[name] = cur
                self.previous_commands[name] = cur
            return {name: (self.steer_angle[name], 0.0) for name in WHEELS}

        module_results = {}
        all_aligned = True
        for name, (module_x, module_y) in WHEEL_POS.items():
            current_steering = self._current_steering(name, steering_positions)
            current_wheel_velocity = self._current_wheel_velocity(name, wheel_velocities)
            angle_offset = MODULE_ANGLE_OFFSETS[name]

            wheel_vel_x = vx_body - wz * module_y
            wheel_vel_y = vy_body + wz * module_x
            target_robot_angle = math.atan2(wheel_vel_y, wheel_vel_x + 1e-9)
            target_speed = math.hypot(wheel_vel_x, wheel_vel_y)
            target_joint_angle = _normalize_angle(target_robot_angle - angle_offset)

            optimized_angle = target_joint_angle
            wheel_direction = 1.0
            if abs(_shortest_angular_distance(current_steering, target_joint_angle)) > math.pi / 2:
                optimized_angle = _normalize_angle(target_joint_angle + math.pi)
                wheel_direction = -1.0

            angle_after_opt = _shortest_angular_distance(current_steering, optimized_angle)
            crosses_boundary = (
                (current_steering > 0 and optimized_angle < 0 and angle_after_opt > 0)
                or (current_steering < 0 and optimized_angle > 0 and angle_after_opt < 0)
            )
            if crosses_boundary:
                optimized_angle = _normalize_angle(optimized_angle + math.pi)
                wheel_direction *= -1.0

            limited_target = _clamp(_normalize_angle(optimized_angle), *STEER_RANGE)
            steering_target = self._update_reversal_phase(
                name, wheel_direction, limited_target, current_steering, cmd_zero, dt)
            steering_cmd = _limit_steering_rate(current_steering, steering_target, dt)

            effective_direction = (
                self.previous_wheel_rotation_direction[name]
                if self.reversal_phase[name] == ReversalPhase.DECELERATING else wheel_direction
            )
            wheel_cmd = effective_direction * target_speed * self.wheel_speed_scale[name] / WHEEL_RADIUS

            align_err = abs(_shortest_angular_distance(current_steering, limited_target))
            aligned = True
            if abs(current_wheel_velocity) >= STEERING_ALIGNMENT_START_SPEED_ERROR_THRESHOLD:
                aligned = align_err < STEERING_ALIGNMENT_ANGLE_ERROR_THRESHOLD
            else:
                aligned = align_err < STEERING_ALIGNMENT_START_ANGLE_ERROR_THRESHOLD
            if not aligned:
                wheel_cmd = 0.0
            all_aligned = all_aligned and aligned

            wheel_cmd = _clamp(wheel_cmd, *WHEEL_SPEED_LIMIT)
            module_results[name] = (steering_cmd, wheel_cmd)

        if not all_aligned:
            module_results = {name: (angle, 0.0) for name, (angle, _speed) in module_results.items()}

        for name, (angle, _speed) in module_results.items():
            self.steer_angle[name] = angle
            self.previous_commands[name] = angle
        return module_results

    def _current_steering(self, name, steering_positions):
        if steering_positions is not None and name in steering_positions:
            return float(steering_positions[name])
        return self.previous_commands[name]

    def _current_wheel_velocity(self, name, wheel_velocities):
        if wheel_velocities is not None and name in wheel_velocities:
            return float(wheel_velocities[name])
        return 0.0

    def _update_reversal_phase(self, name, wheel_direction, limited_target, current_steering,
                               cmd_zero, dt):
        if cmd_zero:
            self.wheel_speed_scale[name] = 1.0
            self.reversal_phase[name] = ReversalPhase.NORMAL
            return current_steering

        if (wheel_direction != self.previous_wheel_rotation_direction[name]
                and self.reversal_phase[name] == ReversalPhase.NORMAL):
            self.reversal_phase[name] = ReversalPhase.DECELERATING
            self.reversal_target_steering_angle[name] = limited_target

        phase = self.reversal_phase[name]
        if phase == ReversalPhase.DECELERATING:
            self.wheel_speed_scale[name] -= REVERSAL_DECEL_RATE * dt
            if self.wheel_speed_scale[name] <= REVERSAL_THRESHOLD:
                self.wheel_speed_scale[name] = 0.0
                self.reversal_target_steering_angle[name] = limited_target
                self.reversal_phase[name] = ReversalPhase.STEERING
            return current_steering

        if phase == ReversalPhase.STEERING:
            self.reversal_target_steering_angle[name] = limited_target
            self.wheel_speed_scale[name] = 0.0
            if abs(_shortest_angular_distance(current_steering, limited_target)) < STEERING_TOLERANCE:
                self.previous_wheel_rotation_direction[name] = wheel_direction
                self.reversal_phase[name] = ReversalPhase.ACCELERATING
            return limited_target

        if phase == ReversalPhase.ACCELERATING:
            self.wheel_speed_scale[name] += REVERSAL_ACCEL_RATE * dt
            if self.wheel_speed_scale[name] >= 1.0:
                self.wheel_speed_scale[name] = 1.0
                self.reversal_phase[name] = ReversalPhase.NORMAL
            return limited_target

        self.wheel_speed_scale[name] = 1.0
        return limited_target


def _limit_steering_rate(current, target, dt):
    max_change = STEERING_ANGULAR_VELOCITY_LIMIT * dt
    desired = _shortest_angular_distance(current, target)
    if abs(desired) <= max_change:
        return target
    return _clamp(_normalize_angle(current + math.copysign(max_change, desired)), *STEER_RANGE)


def _normalize_angle(angle):
    rem = (angle + math.pi) % (2.0 * math.pi)
    return rem - math.pi


def _shortest_angular_distance(start, target):
    result = (target % (2.0 * math.pi)) - (start % (2.0 * math.pi))
    if result > math.pi:
        result -= 2.0 * math.pi
    elif result < -math.pi:
        result += 2.0 * math.pi
    return result


def _clamp(value, lo, hi):
    return min(hi, max(lo, value))
