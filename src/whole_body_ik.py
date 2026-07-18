"""Differential whole-body IK for the mobile FFW-SH5, using MuJoCo + NumPy only.

The controlled generalized velocity is::

    [base_x, base_y, base_yaw, lift, right_arm(7), left_arm(7)]

Both hand pose tasks are solved in one weighted least-squares problem.  Box constraints
enforce per-DOF velocity limits and one-step joint-position limits.  A small posture task
keeps the arms away from limits, allowing the holonomic base and lift to share motion instead
of treating them as unrelated manual controls.

Only the returned command is integrated.  The solver never writes live ``data.qpos``:
base velocity goes through the real swerve wheels, lift through its position actuator, and
arm positions through the existing torque controllers.  This is the ROS-free equivalent of
the weighted differential IK/QP structure used by Cyclo, specialized to this MuJoCo model.
"""

from dataclasses import dataclass, field
import math

import mujoco
import numpy as np

from base_teleop import BodyTwist


BASE_JOINTS = ("base_x", "base_y", "base_yaw")
DEFAULT_VELOCITY_LIMITS = {
    "base_x": 0.55,
    "base_y": 0.55,
    "base_yaw": 1.2,
    "lift_joint": 0.25,
}


@dataclass
class WholeBodyCommand:
    base_twist: BodyTwist = BodyTwist()
    arm_positions: dict = field(default_factory=dict)
    lift_position: float = 0.0
    position_errors: dict = field(default_factory=dict)
    orientation_errors: dict = field(default_factory=dict)
    generalized_velocity: np.ndarray = field(default_factory=lambda: np.zeros(0))


class WholeBodyIK:
    """Weighted, bounded differential IK over base, lift and both arms."""

    def __init__(self, model, site_names, arm_joint_names, *,
                 position_weight=10.0, orientation_weight=5.0,
                 position_gain=8.0, orientation_gain=7.0,
                 linear_velocity_damping=0.0, angular_velocity_damping=0.0,
                 posture_gain=1.0, joint_limit_margin=0.02):
        self.model = model
        self.site_ids = {
            side: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
            for side, name in site_names.items()
        }
        self.arm_joint_names = {side: tuple(names) for side, names in arm_joint_names.items()}
        self.joint_names = (BASE_JOINTS + ("lift_joint",)
                            + self.arm_joint_names["r"] + self.arm_joint_names["l"])
        self.joint_ids = np.array([
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.joint_names
        ], dtype=int)
        if np.any(self.joint_ids < 0) or any(site_id < 0 for site_id in self.site_ids.values()):
            raise ValueError("whole-body IK references a joint or site absent from the model")
        self.qpos_adrs = np.array([model.jnt_qposadr[jid] for jid in self.joint_ids], dtype=int)
        self.dof_ids = np.array([model.jnt_dofadr[jid] for jid in self.joint_ids], dtype=int)
        self.index = {name: i for i, name in enumerate(self.joint_names)}
        self.side_indices = {
            side: np.array([self.index[name] for name in names], dtype=int)
            for side, names in self.arm_joint_names.items()
        }

        self.position_weight = float(position_weight)
        self.orientation_weight = float(orientation_weight)
        self.position_gain = float(position_gain)
        self.orientation_gain = float(orientation_gain)
        self.linear_velocity_damping = float(linear_velocity_damping)
        self.angular_velocity_damping = float(angular_velocity_damping)
        self.posture_gain = float(posture_gain)
        self.joint_limit_margin = float(joint_limit_margin)
        self.max_task_linear_speed = 1.0
        self.max_task_angular_speed = 2.5
        self.base_linear_acceleration_limit = 8.0
        self.base_angular_acceleration_limit = 4.0
        self.base_position_fade_distance = 0.08
        self.base_orientation_fade_angle = 0.25
        self._previous_base_velocity_world = np.zeros(3)
        self._last_solve_time = None
        self._reference_base_yaw = None
        self._reference_base_xy = None
        self._reference_hand_positions = {}
        self._reference_hand_quaternions = {}
        self._reference_hand_rotations = {}

        # Regularization is expressed in task-normalized units.  The mobile/lift DOFs are
        # deliberately cheaper for common dual-hand motion; otherwise fourteen arm columns
        # absorb the shared task numerically and the physical base is barely recruited.
        self.damping_weights = np.array(
            [0.25, 0.25, 0.20, 0.12] + [0.045] * 14, dtype=float)
        self.posture_weights = np.array([0.0, 0.0, 0.0, 0.10] + [0.025] * 14, dtype=float)

        self.velocity_limits = np.array([
            DEFAULT_VELOCITY_LIMITS.get(name, 2.0) for name in self.joint_names
        ], dtype=float)
        self.position_limited = np.array([bool(model.jnt_limited[jid]) for jid in self.joint_ids])
        self.position_ranges = np.array([model.jnt_range[jid] for jid in self.joint_ids], dtype=float)

    def rebase(self, data, target_poses=None):
        """Make the current base pose the origin of subsequent common-hand motion.

        Manual driving intentionally moves the whole target frame with the chassis.  On
        handover, using the solver's startup reference would interpret that movement as a
        new task and command the base back to its old pose.  Rebasing to the carried target
        poses makes zero target delta mean "stay here" while preserving later WBIK motion.
        """
        current_q = np.asarray(data.qpos[self.qpos_adrs], dtype=float)
        self._reference_base_yaw = float(current_q[self.index["base_yaw"]])
        self._reference_base_xy = current_q[:2].copy()
        target_poses = target_poses or {}
        for side, site_id in self.site_ids.items():
            if side in target_poses:
                position, quaternion = target_poses[side]
                position = np.asarray(position, dtype=float).copy()
                quaternion = np.asarray(quaternion, dtype=float).copy()
                rotation = np.zeros(9)
                mujoco.mju_quat2Mat(rotation, quaternion)
                rotation = rotation.reshape(3, 3)
            else:
                position = data.site_xpos[site_id].copy()
                quaternion = np.zeros(4)
                mujoco.mju_mat2Quat(quaternion, data.site_xmat[site_id])
                rotation = data.site_xmat[site_id].reshape(3, 3).copy()
            self._reference_hand_positions[side] = position
            self._reference_hand_quaternions[side] = quaternion
            self._reference_hand_rotations[side] = rotation
        self._previous_base_velocity_world[:] = 0.0
        self._last_solve_time = None

    def solve(self, data, target_poses, dt, *, active_sides=("r", "l"),
              arm_nominal=None, lift_nominal=None):
        """Return actuator-level goals for one control frame.

        ``target_poses`` maps ``"r"``/``"l"`` to world ``(position, quaternion)``.
        ``active_sides`` lets FK-mode arms opt out without removing base/lift from the IK of
        the remaining hand.
        """
        dt = max(float(dt), 1e-5)
        active_sides = tuple(side for side in active_sides if side in self.site_ids)
        current_q = np.asarray(data.qpos[self.qpos_adrs], dtype=float).copy()
        if self._reference_base_yaw is None:
            self.rebase(data)
        rows, rhs = [], []
        dual_base_request = None
        position_errors, orientation_errors = {}, {}

        for side in active_sides:
            target_pos, target_quat = target_poses[side]
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, data, jacp, jacr, self.site_ids[side])
            jac = np.vstack((jacp[:, self.dof_ids], jacr[:, self.dof_ids]))
            site_linear_velocity = jacp @ data.qvel
            site_angular_velocity = jacr @ data.qvel

            current_pos = data.site_xpos[self.site_ids[side]]
            pos_error = np.asarray(target_pos, dtype=float) - current_pos
            current_quat = np.zeros(4)
            mujoco.mju_mat2Quat(current_quat, data.site_xmat[self.site_ids[side]])
            ori_error_local = np.zeros(3)
            mujoco.mju_subQuat(ori_error_local, np.asarray(target_quat), current_quat)
            current_rotation = data.site_xmat[self.site_ids[side]].reshape(3, 3)
            ori_error_world = current_rotation @ ori_error_local

            desired = np.concatenate((
                _clip_norm(self.position_gain * pos_error
                           - self.linear_velocity_damping * site_linear_velocity,
                           self.max_task_linear_speed),
                _clip_norm(self.orientation_gain * ori_error_world
                           - self.angular_velocity_damping * site_angular_velocity,
                           self.max_task_angular_speed),
            ))
            weights = np.sqrt(np.array(
                [self.position_weight] * 3 + [self.orientation_weight] * 3))
            rows.append(weights[:, None] * jac)
            rhs.append(weights * desired)
            position_errors[side] = float(np.linalg.norm(pos_error))
            orientation_errors[side] = float(np.linalg.norm(ori_error_local))

        # A stable hierarchy for dual-hand common motion: explicitly servo the base from
        # the average task error, then let the same least-squares system use lift/arms for
        # the remaining individual-hand residual.  Relying only on minimum norm allowed
        # fourteen arm columns to change the common error's sign while the swerve base was
        # still steering, producing repeated chassis reversals.
        if all(side in position_errors for side in ("r", "l")):
            reference_centroid = 0.5 * (
                self._reference_hand_positions["r"] + self._reference_hand_positions["l"])
            target_centroid = 0.5 * (
                np.asarray(target_poses["r"][0]) + np.asarray(target_poses["l"][0]))
            desired_base_xy = self._reference_base_xy + (target_centroid - reference_centroid)[:2]
            base_position_error = desired_base_xy - current_q[:2]
            target_yaw_deltas = []
            for side in ("r", "l"):
                target_quaternion = np.asarray(target_poses[side][1], dtype=float)
                delta_local = np.zeros(3)
                mujoco.mju_subQuat(
                    delta_local, target_quaternion,
                    self._reference_hand_quaternions[side])
                delta_world = self._reference_hand_rotations[side] @ delta_local
                target_yaw_deltas.append(delta_world[2])
            desired_base_yaw = self._reference_base_yaw + float(np.mean(target_yaw_deltas))
            base_yaw_error = _wrap_angle(
                desired_base_yaw - current_q[self.index["base_yaw"]])
            yaw_deadband = 0.05
            yaw_control_error = (0.0 if abs(base_yaw_error) <= yaw_deadband else
                                 math.copysign(abs(base_yaw_error) - yaw_deadband,
                                               base_yaw_error))
            desired_base = np.array([
                5.0 * base_position_error[0],
                5.0 * base_position_error[1],
                np.clip(3.0 * yaw_control_error, -0.8, 0.8),
            ])
            desired_base = np.clip(
                desired_base, -self.velocity_limits[:3], self.velocity_limits[:3])
            dual_base_request = desired_base.copy()
            base_selector = np.zeros((3, len(self.joint_names)))
            base_selector[:, :3] = np.eye(3)
            common_base_weights = np.sqrt(np.array([30.0, 30.0, 100.0]))
            rows.append(common_base_weights[:, None] * base_selector)
            rhs.append(common_base_weights * desired_base)

        # Tikhonov damping makes the least-squares system well-conditioned at singularities.
        rows.append(np.diag(np.sqrt(self.damping_weights)))
        rhs.append(np.zeros(len(self.joint_names)))

        nominal = current_q.copy()
        if lift_nominal is not None:
            nominal[self.index["lift_joint"]] = float(lift_nominal)
        if arm_nominal is not None:
            for side in ("r", "l"):
                if side in arm_nominal:
                    nominal[self.side_indices[side]] = np.asarray(arm_nominal[side], dtype=float)
        posture_velocity = self.posture_gain * (nominal - current_q)
        rows.append(np.diag(np.sqrt(self.posture_weights)))
        rhs.append(np.sqrt(self.posture_weights) * posture_velocity)

        matrix = np.vstack(rows)
        vector = np.concatenate(rhs)
        lower, upper = self._velocity_bounds(current_q, dt)

        # A joint in FK mode must remain under the FK controller.  Pin its differential IK
        # velocity to zero while still allowing the other arm, lift and base to cooperate.
        for side in ("r", "l"):
            if side not in active_sides:
                lower[self.side_indices[side]] = 0.0
                upper[self.side_indices[side]] = 0.0

        qdot = _bounded_least_squares(matrix, vector, lower, upper)
        if dual_base_request is not None:
            # Enforce the hierarchy exactly.  The weighted rows above let lift/arms solve
            # the residual consistently; copying the three base components prevents small
            # numerical compromises from becoming large swerve heading changes.
            qdot[:3] = dual_base_request
        qdot = self._shape_base_velocity(
            qdot, position_errors, orientation_errors, dt, float(data.time))
        next_q = current_q + qdot * dt
        next_q = self._clip_positions(next_q)

        yaw = float(data.qpos[self.qpos_adrs[self.index["base_yaw"]]])
        vx_world, vy_world = qdot[self.index["base_x"]], qdot[self.index["base_y"]]
        cy, sy = math.cos(yaw), math.sin(yaw)
        base_twist = BodyTwist(
            float(cy * vx_world + sy * vy_world),
            float(-sy * vx_world + cy * vy_world),
            float(qdot[self.index["base_yaw"]]),
        )
        arm_positions = {
            side: next_q[self.side_indices[side]].copy() for side in active_sides
        }
        return WholeBodyCommand(
            base_twist=base_twist,
            arm_positions=arm_positions,
            lift_position=float(next_q[self.index["lift_joint"]]),
            position_errors=position_errors,
            orientation_errors=orientation_errors,
            generalized_velocity=qdot,
        )

    def _shape_base_velocity(self, qdot, position_errors, orientation_errors, dt, data_time):
        """Fade and acceleration-limit the physical base part of the differential solution.

        Arms track a new position target almost immediately, while swerve steering and the
        heavy chassis have real lag.  Feeding every frame's unconstrained sign change into
        the modules caused repeated direction reversals near a goal.  The task remains fast
        at large error, then hands fine positioning back to lift/arms as error approaches
        zero; a short acceleration ramp suppresses one-frame chassis reversals.
        """
        result = qdot.copy()
        if self._last_solve_time is None or data_time < self._last_solve_time:
            self._previous_base_velocity_world[:] = 0.0
        self._last_solve_time = data_time

        position_ratio = max(position_errors.values(), default=0.0) / self.base_position_fade_distance
        orientation_ratio = (max(orientation_errors.values(), default=0.0)
                             / self.base_orientation_fade_angle)
        task_scale = float(np.clip(max(position_ratio, orientation_ratio), 0.0, 1.0))
        requested = result[:3] * task_scale
        max_delta = np.array([
            self.base_linear_acceleration_limit * dt,
            self.base_linear_acceleration_limit * dt,
            self.base_angular_acceleration_limit * dt,
        ])
        shaped = self._previous_base_velocity_world + np.clip(
            requested - self._previous_base_velocity_world, -max_delta, max_delta)
        shaped = np.clip(shaped, -self.velocity_limits[:3], self.velocity_limits[:3])
        result[:3] = shaped
        self._previous_base_velocity_world = shaped.copy()
        return result

    def _velocity_bounds(self, current_q, dt):
        lower = -self.velocity_limits.copy()
        upper = self.velocity_limits.copy()
        for i, limited in enumerate(self.position_limited):
            if not limited:
                continue
            lo, hi = self.position_ranges[i]
            margin = min(self.joint_limit_margin, max(0.0, 0.25 * (hi - lo)))
            lower[i] = max(lower[i], (lo + margin - current_q[i]) / dt)
            upper[i] = min(upper[i], (hi - margin - current_q[i]) / dt)
            if lower[i] > upper[i]:
                # The state is farther outside the margin than one frame can recover.  Use
                # the fastest valid recovery velocity without violating the velocity box.
                recovery = (self.velocity_limits[i]
                            if current_q[i] < lo + margin else -self.velocity_limits[i])
                lower[i] = upper[i] = recovery
        return lower, upper

    def _clip_positions(self, q):
        result = q.copy()
        for i, limited in enumerate(self.position_limited):
            if limited:
                result[i] = np.clip(result[i], *self.position_ranges[i])
        return result


def _bounded_least_squares(matrix, vector, lower, upper):
    """Small active-set box-constrained least-squares solver.

    The FFW problem has only 18 variables.  Solving the free set, pinning the largest bound
    violation, and repeating is deterministic, dependency-free and sufficient for the
    velocity/one-step position boxes used here.
    """
    matrix = np.asarray(matrix, dtype=float)
    vector = np.asarray(vector, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    x = np.zeros(matrix.shape[1], dtype=float)
    free = upper - lower > 1e-12
    fixed = ~free
    x[fixed] = 0.5 * (lower[fixed] + upper[fixed])

    for _ in range(matrix.shape[1] + 1):
        if np.any(free):
            residual = vector - matrix[:, fixed] @ x[fixed]
            solution, *_ = np.linalg.lstsq(matrix[:, free], residual, rcond=None)
            x[free] = solution
        low_violation = lower - x
        high_violation = x - upper
        violation = np.maximum(low_violation, high_violation)
        violation[~free] = -np.inf
        index = int(np.argmax(violation))
        if violation[index] <= 1e-10:
            break
        x[index] = lower[index] if low_violation[index] > high_violation[index] else upper[index]
        free[index] = False
        fixed[index] = True
    return np.clip(x, lower, upper)


def _clip_norm(vector, limit):
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    return vector if norm <= limit or norm < 1e-12 else vector * (limit / norm)


def _wrap_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi
