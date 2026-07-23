"""Differential whole-body IK for the mobile FFW-SH5, using MuJoCo + NumPy only.

The controlled generalized velocity is::

    [base_x, base_y, base_yaw, lift, right_arm(7), left_arm(7)]

Both hand pose tasks are solved in one weighted least-squares problem.  Box constraints
enforce per-DOF velocity limits and one-step joint-position limits, while signed-distance
control barriers reactively avoid self/workspace collision.  A small posture task keeps the
arms away from limits, allowing the holonomic base and lift to share motion instead of
treating them as unrelated manual controls.

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
import kinematics


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
    minimum_collision_distance: float = math.inf
    active_collision_pairs: tuple = ()
    collision_constraint_violation: float = 0.0


class WholeBodyIK:
    """Weighted, bounded differential IK over base, lift and both arms."""

    def __init__(self, model, site_names, arm_joint_names, *,
                 position_weight=10.0, orientation_weight=5.0,
                 position_gain=8.0, orientation_gain=7.0,
                 linear_velocity_damping=0.0, angular_velocity_damping=0.0,
                 posture_gain=1.0, joint_limit_margin=0.02,
                 joint_limit_gain=5.0, rigid_grasp_weight=250.0,
                 collision_avoidance=True, collision_pairs=None,
                 collision_buffer=0.03, collision_safe_distance=0.01,
                 collision_barrier_gain=50.0, collision_slack_weight=1000.0):
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
        # Parse the compiled MJCF topology once, then share the immutable tree between
        # both end-effector solvers.  FK/Jacobian evaluation below is independent of the
        # live MjData and does not call a MuJoCo forward/Jacobian solver.
        self.kinematic_tree = kinematics.KinematicTree(model)
        self.kinematics_solvers = {
            side: kinematics.KinematicsSolver(
                model, site_names[side], self.joint_names, tree=self.kinematic_tree)
            for side in self.site_ids
        }

        self.position_weight = float(position_weight)
        self.orientation_weight = float(orientation_weight)
        self.position_gain = float(position_gain)
        self.orientation_gain = float(orientation_gain)
        self.linear_velocity_damping = float(linear_velocity_damping)
        self.angular_velocity_damping = float(angular_velocity_damping)
        self.posture_gain = float(posture_gain)
        self.joint_limit_margin = float(joint_limit_margin)
        self.joint_limit_gain = float(joint_limit_gain)
        self.rigid_grasp_weight = float(rigid_grasp_weight)
        self.collision_buffer = float(collision_buffer)
        self.collision_safe_distance = float(collision_safe_distance)
        self.collision_barrier_gain = float(collision_barrier_gain)
        self.collision_slack_weight = float(collision_slack_weight)
        if self.collision_safe_distance < 0.0:
            raise ValueError("collision_safe_distance must be non-negative")
        if self.collision_buffer <= self.collision_safe_distance:
            raise ValueError("collision_buffer must exceed collision_safe_distance")
        if self.collision_barrier_gain <= 0.0 or self.collision_slack_weight <= 0.0:
            raise ValueError("collision barrier gain/weight must be positive")
        if collision_avoidance:
            self.collision_pairs = tuple(
                kinematics.default_collision_pairs(model)
                if collision_pairs is None else collision_pairs)
        else:
            self.collision_pairs = ()
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
        self._rigid_grasp_reference = None

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
        for side in self.site_ids:
            if side in target_poses:
                position, quaternion = target_poses[side]
                position = np.asarray(position, dtype=float).copy()
                quaternion = kinematics.normalize_quaternion(quaternion)
            else:
                state = self.site_state(data, side, current_q)
                position = state.position
                quaternion = state.quaternion
            self._reference_hand_positions[side] = position
            self._reference_hand_quaternions[side] = quaternion
        self._previous_base_velocity_world[:] = 0.0
        self._last_solve_time = None

    def set_rigid_grasp(self, data, active):
        """Capture or clear the current left-hand pose in the right-hand frame.

        Cyclo's bimanual MoveL controller adds a six-dimensional relative-pose equality to
        its QP.  We retain the same velocity-level geometry as a strong least-squares task,
        which fits this small NumPy solver without adding OSQP or ROS dependencies.
        """
        if not active:
            self._rigid_grasp_reference = None
            return
        current_q = np.asarray(data.qpos[self.qpos_adrs], dtype=float)
        right = self.site_state(data, "r", current_q)
        left = self.site_state(data, "l", current_q)
        right_rotation = _quaternion_matrix(right.quaternion)
        left_rotation = _quaternion_matrix(left.quaternion)
        self._rigid_grasp_reference = {
            "position_right": right_rotation.T @ (left.position - right.position),
            "rotation_right": right_rotation.T @ left_rotation,
        }

    def solve(self, data, target_poses, dt, *, active_sides=("r", "l"),
              arm_nominal=None, lift_nominal=None, rigid_grasp=False,
              whole_body_enabled=True):
        """Return actuator-level goals for one control frame.

        ``target_poses`` maps ``"r"``/``"l"`` to world ``(position, quaternion)``.
        ``active_sides`` lets FK-mode arms opt out.  With ``whole_body_enabled=False`` the
        planar base and lift are pinned at zero differential velocity, leaving an arm-only
        solve that retains the same joint-limit and collision constraints.
        """
        dt = max(float(dt), 1e-5)
        active_sides = tuple(side for side in active_sides if side in self.site_ids)
        current_q = np.asarray(data.qpos[self.qpos_adrs], dtype=float).copy()
        if self._reference_base_yaw is None:
            self.rebase(data)
        rows, rhs = [], []
        dual_base_request = None
        position_errors, orientation_errors = {}, {}

        site_states = {}
        for side in active_sides:
            target_pos, target_quat = target_poses[side]
            state = self.site_state(data, side, current_q)
            site_states[side] = state
            jac = state.jacobian
            site_velocity = jac @ data.qvel[self.dof_ids]

            current_pos = state.position
            pos_error = np.asarray(target_pos, dtype=float) - current_pos
            ori_error_world = kinematics.shortest_orientation_error(
                target_quat, state.quaternion)

            desired = np.concatenate((
                _clip_norm(self.position_gain * pos_error
                           - self.linear_velocity_damping * site_velocity[:3],
                           self.max_task_linear_speed),
                _clip_norm(self.orientation_gain * ori_error_world
                           - self.angular_velocity_damping * site_velocity[3:],
                           self.max_task_angular_speed),
            ))
            weights = np.sqrt(np.array(
                [self.position_weight] * 3 + [self.orientation_weight] * 3))
            rows.append(weights[:, None] * jac)
            rhs.append(weights * desired)
            position_errors[side] = float(np.linalg.norm(pos_error))
            orientation_errors[side] = float(np.linalg.norm(ori_error_world))

        if rigid_grasp and all(side in site_states for side in ("r", "l")):
            if self._rigid_grasp_reference is None:
                self.set_rigid_grasp(data, True)
            grasp_jacobian, grasp_velocity = self._rigid_grasp_task(site_states, dt)
            weight = math.sqrt(self.rigid_grasp_weight)
            rows.append(weight * grasp_jacobian)
            rhs.append(weight * grasp_velocity)

        # A stable hierarchy for dual-hand common motion: explicitly servo the base from
        # the average task error, then let the same least-squares system use lift/arms for
        # the remaining individual-hand residual.  Relying only on minimum norm allowed
        # fourteen arm columns to change the common error's sign while the swerve base was
        # still steering, producing repeated chassis reversals.
        if whole_body_enabled and all(side in position_errors for side in ("r", "l")):
            reference_centroid = 0.5 * (
                self._reference_hand_positions["r"] + self._reference_hand_positions["l"])
            target_centroid = 0.5 * (
                np.asarray(target_poses["r"][0]) + np.asarray(target_poses["l"][0]))
            desired_base_xy = self._reference_base_xy + (target_centroid - reference_centroid)[:2]
            base_position_error = desired_base_xy - current_q[:2]
            target_yaw_deltas = []
            for side in ("r", "l"):
                target_quaternion = np.asarray(target_poses[side][1], dtype=float)
                delta_world = kinematics.shortest_orientation_error(
                    target_quaternion, self._reference_hand_quaternions[side])
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

        # The UI whole-body switch is a hard participation gate, not merely a small task
        # weight.  Pinning all four body DOFs guarantees that numerical compromise,
        # posture bias or a collision barrier cannot leave a residual chassis/lift command.
        if not whole_body_enabled:
            lower[:4] = 0.0
            upper[:4] = 0.0

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
        if whole_body_enabled:
            qdot = self._shape_base_velocity(
                qdot, position_errors, orientation_errors, dt, float(data.time))
        else:
            qdot[:3] = 0.0
            self._previous_base_velocity_world[:] = 0.0
            self._last_solve_time = None
        collision_constraints = self._collision_constraints(data, dt)
        if collision_constraints:
            barrier_matrix = np.vstack([
                constraint.gradient for constraint, _bound in collision_constraints])
            barrier_lower = np.array([
                bound for _constraint, bound in collision_constraints])
            # This is Cyclo's collision CBF with a quadratic slack penalty, reduced to a
            # squared hinge loss and solved by a tiny active set.  Applying it after base
            # velocity shaping ensures acceleration limiting cannot re-introduce an unsafe
            # approach velocity.
            qdot = _bounded_least_squares_with_barriers(
                np.eye(len(qdot)), qdot, lower, upper,
                barrier_matrix, barrier_lower, self.collision_slack_weight)
            # The next acceleration ramp must start from the command actually returned,
            # including any safety override of the shaped base velocity.
            self._previous_base_velocity_world = qdot[:3].copy()
            collision_violation = float(np.max(np.maximum(
                barrier_lower - barrier_matrix @ qdot, 0.0)))
            collision_names = tuple(
                constraint.name for constraint, _bound in collision_constraints)
            minimum_collision_distance = min(
                constraint.distance for constraint, _bound in collision_constraints)
        else:
            collision_violation = 0.0
            collision_names = ()
            minimum_collision_distance = math.inf
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
            minimum_collision_distance=minimum_collision_distance,
            active_collision_pairs=collision_names,
            collision_constraint_violation=collision_violation,
        )

    def _collision_constraints(self, data, dt):
        """Return active ``grad(distance) @ qdot >= lower`` CBF rows."""
        barrier_gain = min(
            self.collision_barrier_gain, 1.0 / max(float(dt), 1e-5))
        constraints = []
        for result in self.collision_distances(data):
            if np.linalg.norm(result.gradient) < 1e-10:
                continue
            lower = -barrier_gain * (result.distance - self.collision_safe_distance)
            constraints.append((result, float(lower)))
        return constraints

    def site_state(self, data, side, current_q=None):
        """Evaluate one hand through the custom tree FK/Jacobian implementation."""
        if side not in self.kinematics_solvers:
            raise ValueError(f"unknown hand side: {side!r}")
        if current_q is None:
            current_q = np.asarray(data.qpos[self.qpos_adrs], dtype=float)
        return self.kinematics_solvers[side].forward(
            current_q, context_qpos=data.qpos)

    def collision_distances(self, data, max_distance=None):
        """Return monitored pair distances for control diagnostics/visualization.

        The query is read-only and uses the same geometry/gradient code as the CBF, so a
        rendered closest-point line cannot silently disagree with the safety controller.
        """
        distance_limit = (self.collision_buffer if max_distance is None
                          else max(float(max_distance), 0.0))
        results = []
        for pair in self.collision_pairs:
            result = kinematics.collision_distance_gradient(
                self.model, data, pair, self.kinematic_tree,
                self.joint_ids, distance_limit)
            if result is not None:
                results.append(result)
        return tuple(results)

    def _rigid_grasp_task(self, site_states, dt):
        """Build Cyclo-style relative hand Jacobian and drift-correction velocity."""
        right, left = site_states["r"], site_states["l"]
        reference = self._rigid_grasp_reference
        right_rotation = _quaternion_matrix(right.quaternion)
        right_to_left_world = right_rotation @ reference["position_right"]
        transform = np.eye(6)
        transform[:3, 3:] = -_skew(right_to_left_world)
        grasp_jacobian = left.jacobian - transform @ right.jacobian

        desired_left_position = right.position + right_to_left_world
        desired_left_rotation = right_rotation @ reference["rotation_right"]
        desired_left_quaternion = _matrix_quaternion(desired_left_rotation)
        # Exact 1/dt correction, as in Cyclo, can exceed this model's velocity box after a
        # contact disturbance. Norm clipping preserves direction while keeping the QP
        # feasible enough for the strong soft equality to recover over multiple frames.
        correction_dt = max(float(dt), 1e-5)
        linear = _clip_norm(
            (desired_left_position - left.position) / correction_dt,
            self.max_task_linear_speed)
        angular = _clip_norm(
            kinematics.shortest_orientation_error(
                desired_left_quaternion, left.quaternion) / correction_dt,
            self.max_task_angular_speed)
        return grasp_jacobian, np.concatenate((linear, angular))

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
        barrier_gain = min(self.joint_limit_gain, 1.0 / max(float(dt), 1e-5))
        for i, limited in enumerate(self.position_limited):
            if not limited:
                continue
            lo, hi = self.position_ranges[i]
            margin = min(self.joint_limit_margin, max(0.0, 0.25 * (hi - lo)))
            safe_lo, safe_hi = lo + margin, hi - margin
            # Cyclo's joint-limit control-barrier constraint bounds approach velocity by
            # distance to each limit. Unlike a one-frame hard clamp, this decelerates before
            # the margin and gives smooth, exponentially safe recovery if contact pushes a
            # joint outside it.
            lower[i] = max(lower[i], -barrier_gain * (current_q[i] - safe_lo))
            upper[i] = min(upper[i], barrier_gain * (safe_hi - current_q[i]))
            if lower[i] > upper[i]:
                # Degenerate/narrow ranges or a badly out-of-range imported state: choose a
                # bounded recovery direction instead of passing an infeasible box onward.
                recovery = (self.velocity_limits[i]
                            if current_q[i] < safe_lo else -self.velocity_limits[i])
                lower[i] = upper[i] = recovery
        return lower, upper

    def _clip_positions(self, q):
        result = q.copy()
        for i, limited in enumerate(self.position_limited):
            if limited:
                result[i] = np.clip(result[i], *self.position_ranges[i])
        return result


def _bounded_least_squares(matrix, vector, lower, upper):
    """Solve a small box-constrained least-squares problem with a BVLS active set.

    The former one-way active set pinned a variable after its first unconstrained bound
    violation and could never release it, so coupled Jacobian columns sometimes produced a
    feasible but non-optimal velocity. Cyclo uses a full QP solver; bounded-variable least
    squares adds the missing KKT release step and reaches the same box-QP optimum without
    OSQP. It solves at most a few 18-column least-squares systems per control frame.
    """
    matrix = np.asarray(matrix, dtype=float)
    vector = np.asarray(vector, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    if matrix.ndim != 2 or vector.shape != (matrix.shape[0],):
        raise ValueError("incompatible least-squares matrix/vector shapes")
    if lower.shape != (matrix.shape[1],) or upper.shape != lower.shape:
        raise ValueError("incompatible least-squares bound shapes")
    if np.any(lower > upper):
        raise ValueError("lower bound exceeds upper bound")

    tolerance = 1e-10
    fixed = upper - lower <= tolerance
    movable = ~fixed
    x = np.zeros(matrix.shape[1], dtype=float)
    x[fixed] = 0.5 * (lower[fixed] + upper[fixed])
    if np.any(movable):
        reduced_rhs = vector - matrix[:, fixed] @ x[fixed]
        solution, *_ = np.linalg.lstsq(matrix[:, movable], reduced_rhs, rcond=None)
        x[movable] = np.clip(solution, lower[movable], upper[movable])
    active_lower = movable & (x <= lower + tolerance)
    active_upper = movable & ~active_lower & (x >= upper - tolerance)

    for _ in range(4 * matrix.shape[1] + 4):
        active = fixed | active_lower | active_upper
        free = ~active
        candidate = x.copy()
        if np.any(free):
            reduced_rhs = vector - matrix[:, active] @ x[active]
            candidate[free], *_ = np.linalg.lstsq(
                matrix[:, free], reduced_rhs, rcond=None)

        direction = candidate - x
        step = 1.0
        for index in np.flatnonzero(free):
            if candidate[index] < lower[index] - tolerance:
                step = min(step, (lower[index] - x[index]) / direction[index])
            elif candidate[index] > upper[index] + tolerance:
                step = min(step, (upper[index] - x[index]) / direction[index])
        if step < 1.0 - tolerance:
            x += max(0.0, step) * direction
            x = np.clip(x, lower, upper)
            active_lower |= free & (direction < 0.0) & (x <= lower + tolerance)
            active_upper |= free & (direction > 0.0) & (x >= upper - tolerance)
            continue

        x = np.clip(candidate, lower, upper)
        residual = matrix @ x - vector
        gradient = matrix.T @ residual
        lower_violation = np.where(active_lower, -gradient, -np.inf)
        upper_violation = np.where(active_upper, gradient, -np.inf)
        lower_index = int(np.argmax(lower_violation))
        upper_index = int(np.argmax(upper_violation))
        worst_lower = lower_violation[lower_index]
        worst_upper = upper_violation[upper_index]
        if max(worst_lower, worst_upper) <= tolerance:
            break
        if worst_lower >= worst_upper:
            active_lower[lower_index] = False
        else:
            active_upper[upper_index] = False
    return np.clip(x, lower, upper)


def _bounded_least_squares_with_barriers(matrix, vector, lower, upper,
                                         barrier_matrix, barrier_lower, slack_weight):
    """Solve box least squares plus soft one-sided linear barrier constraints.

    Eliminating non-negative slack from ``G x + s >= h`` adds the convex penalty
    ``weight * max(0, h - G x)^2``.  On each smooth region this is ordinary bounded least
    squares, so updating the violated-row active set reaches its piecewise-quadratic
    optimum without bringing ROS, OSQP, or another optimizer into the runtime.
    """
    barrier_matrix = np.asarray(barrier_matrix, dtype=float)
    barrier_lower = np.asarray(barrier_lower, dtype=float)
    if barrier_matrix.ndim != 2 or barrier_matrix.shape[1] != np.shape(matrix)[1]:
        raise ValueError("incompatible collision barrier matrix shape")
    if barrier_lower.shape != (barrier_matrix.shape[0],):
        raise ValueError("incompatible collision barrier lower-bound shape")
    if barrier_matrix.shape[0] == 0:
        return _bounded_least_squares(matrix, vector, lower, upper)

    root_weight = math.sqrt(float(slack_weight))
    solution = _bounded_least_squares(matrix, vector, lower, upper)
    active = barrier_matrix @ solution < barrier_lower
    for _ in range(2 * barrier_matrix.shape[0] + 4):
        if not np.any(active):
            return solution
        augmented_matrix = np.vstack((matrix, root_weight * barrier_matrix[active]))
        augmented_vector = np.concatenate((vector, root_weight * barrier_lower[active]))
        candidate = _bounded_least_squares(
            augmented_matrix, augmented_vector, lower, upper)
        next_active = barrier_matrix @ candidate < barrier_lower - 1e-10
        if np.array_equal(next_active, active):
            return candidate
        solution, active = candidate, next_active
    return solution


def _clip_norm(vector, limit):
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    return vector if norm <= limit or norm < 1e-12 else vector * (limit / norm)


def _wrap_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _skew(vector):
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _quaternion_matrix(quaternion):
    matrix = np.zeros(9)
    mujoco.mju_quat2Mat(matrix, kinematics.normalize_quaternion(quaternion))
    return matrix.reshape(3, 3)


def _matrix_quaternion(matrix):
    quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(quaternion, np.asarray(matrix, dtype=float).reshape(9))
    return kinematics.normalize_quaternion(quaternion)
