"""ROS-free AI Worker style mobile-base control.

The module has three deliberately separate layers:

``BaseTeleop``
    Turns keyboard intent into a smooth body-frame velocity command.
``SwerveKinematics``
    Converts a body twist into feasible steering/drive states (and back) using only
    geometry.  It knows nothing about keys, MuJoCo, ROS, or actuators.
``SwerveDrive``
    Applies feedback-dependent steering rate limits, alignment gating and the ROBOTIS
    ``DECELERATING -> STEERING -> ACCELERATING`` drive-direction reversal state machine.

The geometry and control flow follow ROBOTIS AI Worker's official
``ffw_swerve_drive_controller``.  This is an algorithm-only port: the public contract is
plain Python numbers and dictionaries, with no ROS dependency and no direct qpos writes.
"""

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np


# Keyboard command shaping.  These are simulation/UX limits, not hardware constants.
K_SPEED = 0.62
K_MAX = 0.70
K_ACCEL = 5.0
K_BRAKE = 8.0
K_YAW = 1.6
YAW_FOLLOW = 5.0
YAW_DECAY = 8.0
VEL_ZERO_EPS = 0.001

# FFW-SH5 three-module geometry from the official AI Worker configuration.
WHEEL_POS = {
    "left_wheel": (0.1371, 0.2554),
    "right_wheel": (0.1371, -0.2554),
    "rear_wheel": (-0.2899, 0.0),
}
WHEELS = tuple(WHEEL_POS)

# The official physical wheel is 0.0865 m.  This MuJoCo scene intentionally uses a
# 0.09 m collision cylinder, so the kinematic radius must match the contact geometry.
WHEEL_RADIUS = 0.09
# Official AI Worker runtime configuration allows approximately +/-2pi steering.  Keeping
# that range also prevents a tiny body-twist perturbation around +/-90deg from demanding an
# almost-180deg module flip.  Narrow-range behavior remains covered with an injected range.
STEER_RANGE = (-6.28, 6.28)
MODULE_ANGLE_OFFSETS = {name: 0.0 for name in WHEELS}
WHEEL_SPEED_LIMIT = (-50.0, 50.0)

LINEAR_VEL_DEADBAND = 0.001
ANGULAR_VEL_DEADBAND = 0.001
MODULE_SPEED_EPS = 1e-5
STEERING_ANGULAR_VELOCITY_LIMIT = 8.0
STEERING_ALIGNMENT_ANGLE_ERROR_THRESHOLD = 0.10
STEERING_ALIGNMENT_START_ANGLE_ERROR_THRESHOLD = 0.10
STEERING_ALIGNMENT_START_SPEED_ERROR_THRESHOLD = 0.10
STEERING_TOLERANCE = 0.03
DIRECTION_SWITCH_STEERING_HYSTERESIS = 0.08
REVERSAL_DECEL_RATE = 7.0
REVERSAL_ACCEL_RATE = 5.0
REVERSAL_THRESHOLD = 0.05
DRIVE_COMMAND_ACCEL_LIMIT = 80.0
DRIVE_COMMAND_BRAKE_LIMIT = 3.0
DRIVE_COMMAND_CREEP_THRESHOLD = 0.5
DRIVE_COMMAND_CREEP_BRAKE_LIMIT = 0.20


@dataclass(frozen=True)
class BodyTwist:
    """Planar velocity in the robot body frame."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def is_zero(self):
        return (abs(self.vx) < LINEAR_VEL_DEADBAND
                and abs(self.vy) < LINEAR_VEL_DEADBAND
                and abs(self.wz) < ANGULAR_VEL_DEADBAND)


class BaseTeleop:
    """Convert keyboard intent into a smoothed body-frame velocity command."""

    def __init__(self):
        self.v_local = np.zeros(2)
        self.w = 0.0

    def update_body(self, keys, dt, measured_twist=None):
        """Return a body-frame command; no world-frame or wheel knowledge is involved."""
        fwd = float(bool(keys.get("w"))) - float(bool(keys.get("s")))
        left = float(bool(keys.get("a"))) - float(bool(keys.get("d")))
        turn = float(bool(keys.get("left"))) - float(bool(keys.get("right")))

        target_local = np.array([fwd, left], dtype=float)
        norm = float(np.linalg.norm(target_local))
        if norm > 1e-9:
            target_local *= K_SPEED / norm
        target_w = turn * K_YAW

        # Translation and yaw are independent components of a holonomic body twist.
        # Suppressing translation whenever yaw is held made curved driving impossible.
        if fwd != 0.0 or left != 0.0:
            self.v_local += (target_local - self.v_local) * (1.0 - math.exp(-K_ACCEL * dt))
        else:
            self.v_local *= math.exp(-K_BRAKE * dt)

        speed = float(np.linalg.norm(self.v_local))
        if speed > K_MAX:
            self.v_local *= K_MAX / speed
        if float(np.linalg.norm(self.v_local)) < VEL_ZERO_EPS:
            self.v_local[:] = 0.0

        if target_w != 0.0:
            self.w += (target_w - self.w) * (1.0 - math.exp(-YAW_FOLLOW * dt))
        else:
            self.w *= math.exp(-YAW_DECAY * dt)
        if abs(self.w) < VEL_ZERO_EPS:
            self.w = 0.0

        self.v_local[np.abs(self.v_local) < LINEAR_VEL_DEADBAND] = 0.0
        if abs(self.w) < ANGULAR_VEL_DEADBAND:
            self.w = 0.0
        del measured_twist  # Kept as a compatibility hook for callers with odometry.
        return BodyTwist(float(self.v_local[0]), float(self.v_local[1]), float(self.w))

    def update(self, keys, dt, yaw=0.0):
        """Compatibility helper returning the old world ``vx, vy, wz`` tuple."""
        cmd = self.update_body(keys, dt)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return cy * cmd.vx - sy * cmd.vy, sy * cmd.vx + cy * cmd.vy, cmd.wz

    def reset_motion(self):
        """Clear residual input shaping after a feedback-confirmed physical stop."""
        self.v_local[:] = 0.0
        self.w = 0.0


class SwerveKinematics:
    """Pure inverse/forward kinematics for independently steered wheel modules."""

    def __init__(self, wheel_positions=WHEEL_POS, wheel_radius=WHEEL_RADIUS,
                 steer_range=STEER_RANGE, angle_offsets=MODULE_ANGLE_OFFSETS,
                 wheel_speed_limit=WHEEL_SPEED_LIMIT):
        self.wheel_positions = dict(wheel_positions)
        self.wheel_radius = float(wheel_radius)
        self.steer_range = tuple(float(v) for v in steer_range)
        self.angle_offsets = dict(angle_offsets)
        self.wheel_speed_limit = tuple(float(v) for v in wheel_speed_limit)

    def inverse(self, twist, steering_positions=None, preferred_directions=None):
        """Map body twist to feasible ``(steer_angle, wheel_rad_s)`` states.

        Unlike a simple ``atan2`` followed by clipping, all angle states that produce the
        same rolling direction (``angle + k*pi`` with alternating drive sign) are searched.
        This is generic for both the runtime's approximately +/-2pi steering range and an
        injected narrow-range model: feasibility is decided before selecting the shortest
        command, never by clipping an already chosen angle.
        """
        steering_positions = steering_positions or {}
        preferred_directions = preferred_directions or {}
        states = {}
        for name, (module_x, module_y) in self.wheel_positions.items():
            wheel_vx = twist.vx - twist.wz * module_y
            wheel_vy = twist.vy + twist.wz * module_x
            linear_speed = math.hypot(wheel_vx, wheel_vy)
            current = float(steering_positions.get(name, 0.0))
            if linear_speed < MODULE_SPEED_EPS:
                states[name] = (current, 0.0)
                continue

            robot_angle = math.atan2(wheel_vy, wheel_vx)
            joint_angle = _normalize_angle(robot_angle - self.angle_offsets[name])
            steer, direction = self._nearest_feasible_state(
                current, joint_angle, preferred_directions.get(name, 1.0))
            states[name] = (steer, direction * linear_speed / self.wheel_radius)

        # Preserve the requested chassis twist under saturation: one common scale keeps all
        # module speed ratios intact.  Independent clipping would bend translation/yaw.
        max_requested = max((abs(speed) for _angle, speed in states.values()), default=0.0)
        max_allowed = max(abs(self.wheel_speed_limit[0]), abs(self.wheel_speed_limit[1]))
        scale = 1.0 if max_requested <= max_allowed else max_allowed / max_requested
        if scale < 1.0:
            states = {name: (angle, speed * scale) for name, (angle, speed) in states.items()}
        return states, scale

    def forward(self, steering_positions, wheel_velocities):
        """Least-squares wheel feedback -> body ``BodyTwist`` odometry estimate."""
        rows, rhs = [], []
        for name, (module_x, module_y) in self.wheel_positions.items():
            joint_angle = float(steering_positions[name])
            robot_angle = joint_angle + self.angle_offsets[name]
            c, s = math.cos(robot_angle), math.sin(robot_angle)
            rows.append([c, s, -c * module_y + s * module_x])
            rhs.append(float(wheel_velocities[name]) * self.wheel_radius)
        # Parallel modules cannot observe velocity perpendicular to their rolling axes.
        # Tiny servo-angle noise otherwise makes the nominally rank-deficient matrix look
        # invertible and amplifies ~1e-7 rad noise into multi-m/s phantom lateral velocity.
        # Truncated SVD returns the physically meaningful minimum-norm observable twist.
        solution, *_ = np.linalg.lstsq(
            np.asarray(rows), np.asarray(rhs), rcond=1e-6)
        return BodyTwist(*(float(v) for v in solution))

    def _nearest_feasible_state(self, current, target_angle, preferred_direction=1.0):
        lo, hi = self.steer_range
        candidates = []
        for k in range(-3, 4):
            angle = target_angle + k * math.pi
            if lo - 1e-12 <= angle <= hi + 1e-12:
                direction = 1.0 if k % 2 == 0 else -1.0
                travel = abs(angle - current)
                switch_cost = (DIRECTION_SWITCH_STEERING_HYSTERESIS
                               if direction != preferred_direction else 0.0)
                candidates.append((travel + switch_cost, travel, angle, direction))
        if not candidates:
            # A steering interval narrower than pi cannot represent every direction.  Keep
            # the result safe and deterministic even for such a future model.
            clipped = _clamp(target_angle, lo, hi)
            return clipped, 1.0
        _cost, _travel, angle, direction = min(candidates, key=lambda item: item[0])
        return float(angle), float(direction)


class ReversalPhase(Enum):
    NORMAL = 0
    DECELERATING = 1
    STEERING = 2
    ACCELERATING = 3


class SwerveDrive:
    """Feedback controller that turns body twists into actuator-ready module commands."""

    def __init__(self, kinematics=None):
        self.base = BaseTeleop()
        self.kinematics = kinematics or SwerveKinematics()
        self.steer_angle = {name: 0.0 for name in WHEELS}
        self.previous_wheel_rotation_direction = {name: 1.0 for name in WHEELS}
        self.wheel_speed_scale = {name: 1.0 for name in WHEELS}
        self.reversal_phase = {name: ReversalPhase.NORMAL for name in WHEELS}
        self.reversal_target_steering_angle = {name: 0.0 for name in WHEELS}
        self.reversal_target_direction = {name: 1.0 for name in WHEELS}
        self.previous_commands = {name: 0.0 for name in WHEELS}
        self.previous_drive_commands = {name: 0.0 for name in WHEELS}
        self.wheel_saturation_scale = 1.0
        self.last_body_twist = BodyTwist()

    def update(self, keys, dt, yaw=0.0, steering_positions=None, wheel_velocities=None):
        """Keyboard compatibility path used by existing tests and callers."""
        del yaw  # Wheel inverse kinematics consumes a body-frame command.
        measured_twist = None
        if (steering_positions is not None and wheel_velocities is not None
                and all(name in steering_positions and name in wheel_velocities
                        for name in WHEELS)):
            measured_twist = self.kinematics.forward(steering_positions, wheel_velocities)
        twist = self.base.update_body(keys, dt, measured_twist)
        return self.update_twist(twist, dt, steering_positions, wheel_velocities)

    def update_twist(self, twist, dt, steering_positions=None, wheel_velocities=None):
        """Control an arbitrary body twist, enabling whole-body IK to drive the base."""
        if not isinstance(twist, BodyTwist):
            twist = BodyTwist(*(float(v) for v in twist))
        twist = BodyTwist(
            0.0 if abs(twist.vx) < LINEAR_VEL_DEADBAND else twist.vx,
            0.0 if abs(twist.vy) < LINEAR_VEL_DEADBAND else twist.vy,
            0.0 if abs(twist.wz) < ANGULAR_VEL_DEADBAND else twist.wz,
        )
        self.last_body_twist = twist
        if twist.is_zero():
            self.wheel_saturation_scale = 1.0
            self.previous_drive_commands = {name: 0.0 for name in WHEELS}
            return self._hold_zero(steering_positions)

        desired, self.wheel_saturation_scale = self.kinematics.inverse(
            twist, steering_positions or self.previous_commands,
            self.previous_wheel_rotation_direction)
        module_results = {}
        all_aligned = True
        for name, (target_angle, target_wheel_speed) in desired.items():
            steering_cmd, wheel_cmd, aligned = self._control_module(
                name, target_angle, target_wheel_speed, dt,
                steering_positions, wheel_velocities)
            module_results[name] = (steering_cmd, wheel_cmd)
            all_aligned = all_aligned and aligned

        # Match AI Worker's safety contract: do not apply propulsion until every module is
        # aligned.  This prevents a transient unintended chassis direction during steering.
        if not all_aligned:
            module_results = {name: (angle, 0.0) for name, (angle, _speed) in module_results.items()}
            self.previous_drive_commands = {name: 0.0 for name in WHEELS}
        else:
            module_results = self._rate_limit_drive_commands(module_results, dt)

        for name, (angle, _speed) in module_results.items():
            self.steer_angle[name] = angle
            self.previous_commands[name] = angle
        return module_results

    def estimate_body_twist(self, steering_positions, wheel_velocities):
        return self.kinematics.forward(steering_positions, wheel_velocities)

    def _rate_limit_drive_commands(self, commands, dt):
        """Rate-limit nonzero drive transitions; parking commands take the zero fast path."""
        result = {}
        for name, (steering, target_speed) in commands.items():
            previous = self.previous_drive_commands[name]
            braking = (previous * target_speed >= 0.0
                       and abs(target_speed) < abs(previous))
            if braking and abs(previous) < DRIVE_COMMAND_CREEP_THRESHOLD:
                rate = DRIVE_COMMAND_CREEP_BRAKE_LIMIT
            else:
                rate = DRIVE_COMMAND_BRAKE_LIMIT if braking else DRIVE_COMMAND_ACCEL_LIMIT
            max_change = rate * dt
            speed = previous + float(np.clip(target_speed - previous, -max_change, max_change))
            if abs(speed) < 1e-6:
                speed = 0.0
            self.previous_drive_commands[name] = speed
            result[name] = (steering, speed)
        return result

    def _hold_zero(self, steering_positions):
        for name in WHEELS:
            self.reversal_phase[name] = ReversalPhase.NORMAL
            self.wheel_speed_scale[name] = 1.0
            cur = self._current_steering(name, steering_positions)
            self.steer_angle[name] = cur
            self.previous_commands[name] = cur
        return {name: (self.steer_angle[name], 0.0) for name in WHEELS}

    def _control_module(self, name, target_angle, target_wheel_speed, dt,
                        steering_positions, wheel_velocities):
        current_steering = self._current_steering(name, steering_positions)
        current_wheel_velocity = self._current_wheel_velocity(name, wheel_velocities)
        direction = -1.0 if target_wheel_speed < 0.0 else 1.0
        steering_target = self._update_reversal_phase(
            name, direction, target_angle, current_steering, current_wheel_velocity, dt)
        # Rate-limit the command trajectory, not the lagging feedback position.  Using live
        # feedback as the reference held the position-servo error (and therefore torque)
        # constant; stationary tire friction could then trap steering around 20 degrees.
        steering_cmd = _limit_steering_rate(
            self.previous_commands[name], steering_target, dt)

        effective_direction = (
            self.previous_wheel_rotation_direction[name]
            if self.reversal_phase[name] == ReversalPhase.DECELERATING else direction
        )
        wheel_cmd = effective_direction * abs(target_wheel_speed) * self.wheel_speed_scale[name]
        align_err = abs(target_angle - current_steering)
        threshold = (STEERING_ALIGNMENT_ANGLE_ERROR_THRESHOLD
                     if abs(current_wheel_velocity) >= STEERING_ALIGNMENT_START_SPEED_ERROR_THRESHOLD
                     else STEERING_ALIGNMENT_START_ANGLE_ERROR_THRESHOLD)
        aligned = align_err < threshold
        if not aligned:
            wheel_cmd = 0.0
        return steering_cmd, wheel_cmd, aligned

    def _current_steering(self, name, steering_positions):
        if steering_positions is not None and name in steering_positions:
            return float(steering_positions[name])
        return self.previous_commands[name]

    @staticmethod
    def _current_wheel_velocity(name, wheel_velocities):
        if wheel_velocities is not None and name in wheel_velocities:
            return float(wheel_velocities[name])
        return 0.0

    def _update_reversal_phase(self, name, direction, target, current, wheel_velocity, dt):
        previous_direction = self.previous_wheel_rotation_direction[name]
        phase = self.reversal_phase[name]

        # If the requested direction returns to the established direction before a reversal
        # finishes, cancel it immediately instead of completing a now-stale state sequence.
        if phase in (ReversalPhase.DECELERATING, ReversalPhase.STEERING):
            if direction == previous_direction:
                self.reversal_phase[name] = ReversalPhase.NORMAL
                self.wheel_speed_scale[name] = 1.0
                return target

        if direction != previous_direction and phase in (
                ReversalPhase.NORMAL, ReversalPhase.ACCELERATING):
            self.reversal_target_steering_angle[name] = target
            self.reversal_target_direction[name] = direction
            if abs(wheel_velocity) < STEERING_ALIGNMENT_START_SPEED_ERROR_THRESHOLD:
                # A stopped wheel has no rotational energy to decelerate.  Switching its
                # sign directly removes an unnecessary control delay.
                self.previous_wheel_rotation_direction[name] = direction
                self.reversal_phase[name] = ReversalPhase.NORMAL
                self.wheel_speed_scale[name] = 1.0
                return target
            self.reversal_phase[name] = ReversalPhase.DECELERATING

        phase = self.reversal_phase[name]
        if phase == ReversalPhase.DECELERATING:
            self.reversal_target_direction[name] = direction
            self.reversal_target_steering_angle[name] = target
            self.wheel_speed_scale[name] = max(
                0.0, self.wheel_speed_scale[name] - REVERSAL_DECEL_RATE * dt)
            if self.wheel_speed_scale[name] <= REVERSAL_THRESHOLD:
                self.wheel_speed_scale[name] = 0.0
                self.reversal_phase[name] = ReversalPhase.STEERING
            return current

        if phase == ReversalPhase.STEERING:
            self.reversal_target_steering_angle[name] = target
            self.reversal_target_direction[name] = direction
            self.wheel_speed_scale[name] = 0.0
            if abs(target - current) < STEERING_TOLERANCE:
                self.previous_wheel_rotation_direction[name] = self.reversal_target_direction[name]
                self.reversal_phase[name] = ReversalPhase.ACCELERATING
            return target

        if phase == ReversalPhase.ACCELERATING:
            self.wheel_speed_scale[name] = min(
                1.0, self.wheel_speed_scale[name] + REVERSAL_ACCEL_RATE * dt)
            if self.wheel_speed_scale[name] >= 1.0:
                self.reversal_phase[name] = ReversalPhase.NORMAL
            return target

        self.wheel_speed_scale[name] = 1.0
        return target


def _limit_steering_rate(current, target, dt):
    max_change = STEERING_ANGULAR_VELOCITY_LIMIT * dt
    desired = target - current
    if abs(desired) <= max_change:
        return target
    return _clamp(current + math.copysign(max_change, desired), *STEER_RANGE)


def _normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _shortest_angular_distance(start, target):
    return _normalize_angle(target - start)


def _clamp(value, lo, hi):
    return min(hi, max(lo, value))
