"""ROS-free mobile kinematics and whole-body IK regression gates.

Run headless: ``python3 tests/test_whole_body.py``.
"""

import ast
import itertools
import pathlib
import sys
import time

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "full_scene.xml"

import arm_control  # noqa: E402
import base_teleop  # noqa: E402
import grasp  # noqa: E402
import kinematics  # noqa: E402
import teleop_app  # noqa: E402
import teleop_targets  # noqa: E402
import whole_body_ik  # noqa: E402

ARMS = {side: [f"arm_{side}_joint{i}" for i in range(1, 8)] for side in ("r", "l")}
HOME = np.array([0.0, 0.0, 0.0, -np.pi / 2, 0.0, 0.0, 0.0])
ORIENTATION_ERROR_LENGTH = 0.20


def run_ros_free_dependency_gate():
    """Prevent an accidental ROS/MoveIt dependency from entering the runtime path."""
    runtime_files = (
        "base_teleop.py", "kinematics.py", "whole_body_ik.py",
        "teleop_targets.py", "teleop_app.py")
    forbidden = {
        "rclpy", "rospy", "geometry_msgs", "nav_msgs", "sensor_msgs", "tf2_ros",
        "moveit", "moveit_commander", "ament_index_python", "controller_manager",
    }
    imported = set()
    for filename in runtime_files:
        source = (REPO_ROOT / "src" / filename).read_text(encoding="utf-8")
        for node in ast.walk(ast.parse(source, filename=filename)):
            if isinstance(node, ast.Import):
                imported.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module.split(".")[0])
    violations = sorted(imported & forbidden)
    ok = not violations
    print(f"ROS-free dependency gate: forbidden imports={violations}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def _reset(model, data):
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)


def _sites(model):
    return {
        side: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"grasp_target_{side}")
        for side in ("r", "l")
    }


def _target_poses(data, sites, delta):
    targets = {}
    for side, site_id in sites.items():
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, data.site_xmat[site_id])
        targets[side] = (data.site_xpos[site_id].copy() + np.asarray(delta), quat)
    return targets


def _pose_error_metric(data, targets, sites):
    """Position error plus orientation error mapped to an equivalent 20cm lever arm."""
    total = 0.0
    for side, site in sites.items():
        target_pos, target_quat = targets[side]
        current_quat = np.zeros(4)
        mujoco.mju_mat2Quat(current_quat, data.site_xmat[site])
        orientation_error = np.zeros(3)
        mujoco.mju_subQuat(orientation_error, target_quat, current_quat)
        total += np.linalg.norm(target_pos - data.site_xpos[site])
        total += ORIENTATION_ERROR_LENGTH * np.linalg.norm(orientation_error)
    return float(total)


def run_swerve_kinematics_gate():
    kin = base_teleop.SwerveKinematics()
    limited_kin = base_teleop.SwerveKinematics(steer_range=(-1.58, 1.58))
    rng = np.random.default_rng(42)
    max_roundtrip = 0.0
    feasible = True
    for _ in range(100):
        twist = base_teleop.BodyTwist(*rng.uniform([-0.4, -0.4, -0.7], [0.4, 0.4, 0.7]))
        current = {name: rng.uniform(*base_teleop.STEER_RANGE) for name in base_teleop.WHEELS}
        states, scale = kin.inverse(twist, current)
        estimate = kin.forward(
            {name: state[0] for name, state in states.items()},
            {name: state[1] for name, state in states.items()},
        )
        error = np.linalg.norm(
            np.array([estimate.vx, estimate.vy, estimate.wz])
            - scale * np.array([twist.vx, twist.vy, twist.wz]))
        max_roundtrip = max(max_roundtrip, float(error))
        feasible &= all(base_teleop.STEER_RANGE[0] <= angle <= base_teleop.STEER_RANGE[1]
                        for angle, _speed in states.values())

    fast = base_teleop.BodyTwist(8.0, -3.0, 5.0)
    saturated, saturation_scale = kin.inverse(fast)
    saturation_ok = (
        0.0 < saturation_scale < 1.0
        and max(abs(speed) for _angle, speed in saturated.values())
        <= base_teleop.WHEEL_SPEED_LIMIT[1] + 1e-12
    )
    # 100 degrees is outside this model's +90 degree steering limit, but the same rolling
    # direction is representable as -80 degrees with reversed wheel rotation.
    angle, direction = limited_kin._nearest_feasible_state(0.0, np.radians(100.0))
    equivalent_ok = abs(np.degrees(angle) + 80.0) < 1e-9 and direction == -1.0
    ok = feasible and max_roundtrip < 1e-10 and saturation_ok and equivalent_ok
    print(f"Swerve kinematics gate: feasible={feasible} roundtrip={max_roundtrip:.2e} "
          f"global_saturation={saturation_ok} +/-90_equivalent={equivalent_ok}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def _brute_box_least_squares(matrix, vector, lower, upper):
    """Enumerate the 3-variable active sets used to validate the NumPy box-QP solver."""
    best = np.inf
    for state in itertools.product((-1, 0, 1), repeat=matrix.shape[1]):
        fixed = np.array([value != 0 for value in state])
        free = ~fixed
        candidate = np.zeros(matrix.shape[1])
        for index, value in enumerate(state):
            if value < 0:
                candidate[index] = lower[index]
            elif value > 0:
                candidate[index] = upper[index]
        if np.any(free):
            residual = vector - matrix[:, fixed] @ candidate[fixed]
            candidate[free], *_ = np.linalg.lstsq(matrix[:, free], residual, rcond=None)
        if np.any(candidate < lower - 1e-9) or np.any(candidate > upper + 1e-9):
            continue
        best = min(best, float(np.linalg.norm(matrix @ candidate - vector) ** 2))
    return best


def run_box_qp_gate():
    """The dependency-free bounded solver must reach the true convex box optimum."""
    rng = np.random.default_rng(20260719)
    worst_gap = 0.0
    for _ in range(25):
        matrix = rng.normal(size=(8, 3))
        vector = rng.normal(size=8)
        lower = rng.uniform(-1.2, -0.1, size=3)
        upper = rng.uniform(0.1, 1.2, size=3)
        solution = whole_body_ik._bounded_least_squares(
            matrix, vector, lower, upper)
        objective = float(np.linalg.norm(matrix @ solution - vector) ** 2)
        optimum = _brute_box_least_squares(matrix, vector, lower, upper)
        worst_gap = max(worst_gap, objective - optimum)
    ok = worst_gap < 1e-9
    print(f"Box-QP gate: worst_objective_gap={worst_gap:.2e}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_joint_limit_cbf_gate(model):
    """Cyclo-style barrier bounds must slow approach and recover outside the margin."""
    data = mujoco.MjData(model)
    _reset(model, data)
    solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    index = solver.index["arm_r_joint4"]
    safe_high = solver.position_ranges[index, 1] - solver.joint_limit_margin
    current = data.qpos[solver.qpos_adrs].copy()
    current[index] = safe_high - 0.005
    lower, upper = solver._velocity_bounds(current, 0.04)
    expected_upper = solver.joint_limit_gain * 0.005
    slowed = 0.0 <= upper[index] <= expected_upper + 1e-12
    stays_safe = current[index] + 0.04 * upper[index] <= safe_high + 1e-12

    current[index] = safe_high + 0.01
    recovery_lower, recovery_upper = solver._velocity_bounds(current, 0.04)
    recovers = recovery_lower[index] <= recovery_upper[index] < 0.0
    ok = slowed and stays_safe and recovers
    print(f"Joint-limit CBF gate: approach_bound={upper[index]:.4f} "
          f"safe={stays_safe} recovery={recovery_upper[index]:.4f}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


SELF_COLLISION_Q = np.array([
    -2.36395827, -0.77470818, 0.09261652, -2.40170025,
    1.90799215, -0.47054658, 0.01506766,
    0.88489933, 0.57009304, -1.24600523, 0.60312349,
    0.13156373, 0.24918186, 0.61419769,
])
HAND_COLLISION_Q = np.array([
    -0.09212671, 0.0, 0.38088724, -1.46555720,
    -0.03419333, -0.02012158, -0.37858088,
    -0.09212671, 0.0, -0.38088724, -1.46555720,
    0.03419333, -0.02012158, 0.37858088,
])


def _self_collision_fixture(model):
    data = mujoco.MjData(model)
    _reset(model, data)
    solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    arm_indices = np.r_[solver.side_indices["r"], solver.side_indices["l"]]
    data.qpos[solver.qpos_adrs[arm_indices]] = SELF_COLLISION_Q
    mujoco.mj_forward(model, data)
    pair = next(pair for pair in solver.collision_pairs
                if pair.name == "body:hx5_r_base/lift_link")
    return data, solver, pair


def run_collision_gradient_gate(model):
    """Closest-point Jacobian must match a numerical signed-distance derivative."""
    data, solver, pair = _self_collision_fixture(model)
    result = kinematics.collision_distance_gradient(
        model, data, pair, solver.dof_ids, 0.10)
    step = 1e-6
    numerical = np.zeros(len(solver.dof_ids))
    for index, qpos_adr in enumerate(solver.qpos_adrs):
        distances = []
        for sign in (-1.0, 1.0):
            scratch = mujoco.MjData(model)
            scratch.qpos[:] = data.qpos
            scratch.qpos[qpos_adr] += sign * step
            mujoco.mj_forward(model, scratch)
            perturbed = kinematics.collision_distance_gradient(
                model, scratch, pair, solver.dof_ids, 0.10)
            distances.append(perturbed.distance)
        numerical[index] = (distances[1] - distances[0]) / (2.0 * step)
    error = float(np.max(np.abs(result.gradient - numerical)))
    # Palm boxes trigger unstable GJK feature switching in MuJoCo 3.10, so their dedicated
    # conservative sphere proxy must remain continuous under the same finite-difference test.
    arm_indices = np.r_[solver.side_indices["r"], solver.side_indices["l"]]
    data.qpos[solver.qpos_adrs[arm_indices]] = HAND_COLLISION_Q
    mujoco.mj_forward(model, data)
    hand_pair = next(pair for pair in solver.collision_pairs
                     if "hx5_r_base/hx5_l_base" in pair.name)
    hand_result = kinematics.collision_distance_gradient(
        model, data, hand_pair, solver.dof_ids, 0.10)
    hand_numerical = np.zeros(len(solver.dof_ids))
    for index, qpos_adr in enumerate(solver.qpos_adrs):
        distances = []
        for sign in (-1.0, 1.0):
            scratch = mujoco.MjData(model)
            scratch.qpos[:] = data.qpos
            scratch.qpos[qpos_adr] += sign * step
            mujoco.mj_forward(model, scratch)
            perturbed = kinematics.collision_distance_gradient(
                model, scratch, hand_pair, solver.dof_ids, 0.10)
            distances.append(perturbed.distance)
        hand_numerical[index] = (distances[1] - distances[0]) / (2.0 * step)
    hand_error = float(np.max(np.abs(
        hand_result.gradient - hand_numerical)))
    pair_scope_ok = not any(
        token in pair.name for pair in solver.collision_pairs
        for token in ("wheel", "floor", "finger", "can"))
    ok = (result.distance < solver.collision_buffer and error < 1e-6
          and hand_pair.mode == "bounding_sphere"
          and hand_result.distance < solver.collision_safe_distance
          and hand_error < 1e-6 and pair_scope_ok)
    print(f"Collision gradient gate: pairs={len(solver.collision_pairs)} "
          f"body/hand={result.distance*1000:.1f}/{hand_result.distance*1000:.1f}mm "
          f"max_error={max(error, hand_error):.2e} "
          f"scope={pair_scope_ok}: {'OK' if ok else 'FAIL'}")
    return ok


def run_self_collision_cbf_gate(model):
    """An inward hand request inside 10 mm must become a separating velocity."""
    data = mujoco.MjData(model)
    _reset(model, data)
    probe_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    arm_indices = np.r_[probe_solver.side_indices["r"], probe_solver.side_indices["l"]]
    data.qpos[probe_solver.qpos_adrs[arm_indices]] = HAND_COLLISION_Q
    mujoco.mj_forward(model, data)
    pair = next(pair for pair in probe_solver.collision_pairs
                if "hx5_r_base/hx5_l_base" in pair.name)
    result = kinematics.collision_distance_gradient(
        model, data, pair, probe_solver.dof_ids, 0.10)
    normal = result.point_b - result.point_a
    normal /= np.linalg.norm(normal)
    sites = _sites(model)
    targets = _target_poses(data, sites, [0.0, 0.0, 0.0])
    targets["r"] = (targets["r"][0] - 0.08 * normal, targets["r"][1])

    free_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS,
        collision_avoidance=False)
    safe_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    qpos_before = data.qpos.copy()
    free = free_solver.solve(data, targets, 0.04, active_sides=("r",))
    safe = safe_solver.solve(data, targets, 0.04, active_sides=("r",))
    constraints = safe_solver._collision_constraints(data, 0.04)
    constraint, lower = next(
        item for item in constraints if item[0].name == pair.name)
    free_rate = float(constraint.gradient @ free.generalized_velocity)
    safe_rate = float(constraint.gradient @ safe.generalized_velocity)
    ok = (np.array_equal(data.qpos, qpos_before)
          and constraint.distance < safe_solver.collision_safe_distance
          and free_rate < lower - 0.01
          and safe_rate >= lower - 1e-3
          and pair.name in safe.active_collision_pairs
          and safe.collision_constraint_violation < 1e-3)
    print(f"Self-collision CBF gate: d={constraint.distance*1000:.1f}mm "
          f"distance_rate={free_rate:+.3f}->{safe_rate:+.3f} "
          f"required>={lower:+.3f} violation={safe.collision_constraint_violation:.2e}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_table_collision_cbf_gate(model):
    """A downward dual-hand request must not cross the relaxed 10 mm margin."""
    data = mujoco.MjData(model)
    _reset(model, data)
    probe = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    data.qpos[probe.qpos_adrs[probe.index["lift_joint"]]] -= 0.035
    mujoco.mj_forward(model, data)
    sites = _sites(model)
    targets = _target_poses(data, sites, [0.0, 0.0, -0.15])
    free_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS,
        collision_avoidance=False)
    safe_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    free = free_solver.solve(data, targets, 0.04)
    safe = safe_solver.solve(data, targets, 0.04)
    constraints = safe_solver._collision_constraints(data, 0.04)
    matrix = np.vstack([constraint.gradient for constraint, _lower in constraints])
    lower = np.array([bound for _constraint, bound in constraints])
    free_violation = float(np.max(lower - matrix @ free.generalized_velocity))
    safe_violation = float(np.max(np.maximum(
        lower - matrix @ safe.generalized_velocity, 0.0)))
    predicted_distances = np.array([
        constraint.distance for constraint, _bound in constraints
    ]) + 0.04 * (matrix @ safe.generalized_velocity)

    # Collision checking is still comfortably inside the 25 Hz UI's 40 ms frame budget.
    for _ in range(10):
        safe_solver.solve(data, targets, 0.04)
    start = time.perf_counter()
    for _ in range(100):
        safe_solver.solve(data, targets, 0.04)
    milliseconds = 1000.0 * (time.perf_counter() - start) / 100.0
    workspace_only = all(
        constraint.name.startswith("workspace:") for constraint, _lower in constraints)
    ok = (len(constraints) >= 2 and workspace_only
          and free_violation > 0.10 and safe_violation < 1e-3
          and np.min(predicted_distances) >= safe_solver.collision_safe_distance - 1e-4
          and safe.generalized_velocity[probe.index["lift_joint"]]
          > free.generalized_velocity[probe.index["lift_joint"]] + 0.10
          and milliseconds < 5.0)
    print(f"Table collision CBF gate: active={len(constraints)} "
          f"violation={free_violation:.3f}->{safe_violation:.2e} "
          f"lift={free.generalized_velocity[3]:+.3f}->"
          f"{safe.generalized_velocity[3]:+.3f} {milliseconds:.2f}ms: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_collision_inactive_regression_gate(model):
    """Far from obstacles, enabling collision avoidance must be output-neutral."""
    data = mujoco.MjData(model)
    _reset(model, data)
    sites = _sites(model)
    targets = _target_poses(data, sites, [0.0, 0.0, 0.0])
    free_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS,
        collision_avoidance=False)
    safe_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    free = free_solver.solve(data, targets, 0.04)
    safe = safe_solver.solve(data, targets, 0.04)
    difference = float(np.max(np.abs(
        free.generalized_velocity - safe.generalized_velocity)))
    ok = (not safe.active_collision_pairs and difference < 1e-12
          and np.isinf(safe.minimum_collision_distance))
    print(f"Inactive collision regression gate: active={len(safe.active_collision_pairs)} "
          f"max_command_delta={difference:.2e}: {'OK' if ok else 'FAIL'}")
    return ok


def run_rigid_grasp_gate(model):
    """Captured hand relation must override conflicting independent hand requests."""
    data = mujoco.MjData(model)
    _reset(model, data)
    sites = _sites(model)
    targets = _target_poses(data, sites, [0.0, 0.0, 0.0])
    targets["r"] = (targets["r"][0] + np.array([0.08, 0.0, 0.0]), targets["r"][1])
    targets["l"] = (targets["l"][0] - np.array([0.08, 0.0, 0.0]), targets["l"][1])

    free_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    rigid_solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    rigid_solver.set_rigid_grasp(data, True)
    free = free_solver.solve(data, targets, 0.04)
    rigid = rigid_solver.solve(data, targets, 0.04, rigid_grasp=True)

    states = {
        side: kinematics.evaluate_site(model, data, sites[side])
        for side in ("r", "l")
    }
    grasp_jacobian, desired_velocity = rigid_solver._rigid_grasp_task(states, 0.04)
    free_residual = float(np.linalg.norm(
        grasp_jacobian @ free.generalized_velocity - desired_velocity))
    rigid_residual = float(np.linalg.norm(
        grasp_jacobian @ rigid.generalized_velocity - desired_velocity))
    ok = free_residual > 0.05 and rigid_residual < 0.25 * free_residual
    print(f"Rigid-grasp gate: relative_twist={free_residual:.3f}->{rigid_residual:.3f}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_rigid_grasp_physical_gate():
    """Virtual-object MoveL must preserve the captured relation through real dynamics."""
    app = teleop_app.TeleopApp.__new__(teleop_app.TeleopApp)
    app._setup_sim()
    app.q_des_r = teleop_app.HOME_Q_R.copy()
    app.q_des_l = teleop_app.HOME_Q_L.copy()
    app.arm_mode = {"r": "ik", "l": "ik"}
    app.fk_q_deg = {
        "r": list(np.degrees(app.q_des_r)),
        "l": list(np.degrees(app.q_des_l)),
    }
    app.frame_dt = 0.04
    app.steps_per_frame = round(app.frame_dt / app.model.opt.timestep)
    app.ik_err_mm = {"r": 0.0, "l": 0.0}
    no_keys = {key: False for key in ("w", "a", "s", "d", "left", "right")}

    app.capture_grasp()
    reference = app.whole_body_solver._rigid_grasp_reference
    start_midpoint = 0.5 * (
        app.data.site_xpos[app.site_r] + app.data.site_xpos[app.site_l])
    app.targets["virtual_object_pos"][0] -= 0.08
    app.targets["virtual_object_rpy"][2] += 12.0
    for _ in range(50):
        app._step_physics(no_keys)

    right = kinematics.evaluate_site(app.model, app.data, app.site_r)
    left = kinematics.evaluate_site(app.model, app.data, app.site_l)
    right_rotation = whole_body_ik._quaternion_matrix(right.quaternion)
    position_right = right_rotation.T @ (left.position - right.position)
    rotation_right = right_rotation.T @ whole_body_ik._quaternion_matrix(left.quaternion)
    current_relative_quaternion = whole_body_ik._matrix_quaternion(rotation_right)
    reference_relative_quaternion = whole_body_ik._matrix_quaternion(
        reference["rotation_right"])
    position_drift = float(np.linalg.norm(
        position_right - reference["position_right"]))
    orientation_drift = float(np.linalg.norm(
        kinematics.shortest_orientation_error(
            reference_relative_quaternion, current_relative_quaternion)))
    final_midpoint = 0.5 * (right.position + left.position)
    moved = float(np.linalg.norm(final_midpoint - start_midpoint))
    ok = position_drift < 0.012 and orientation_drift < np.radians(3.0) and moved > 0.03
    print(f"Rigid-grasp physical gate: drift={position_drift*1000:.1f}mm/"
          f"{np.degrees(orientation_drift):.2f}deg moved={moved*1000:.1f}mm: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_world_anchor_gate():
    app = teleop_app.TeleopApp.__new__(teleop_app.TeleopApp)
    app._setup_sim()
    app.targets["pos_r"] = [0.08, -0.03, 0.04]
    app.targets["rpy_r"] = [5.0, -7.0, 11.0]
    hand_before = app._target_world_pose("r")
    virtual_before = app._virtual_object_world_pose()

    app.data.qpos[app.base_x_qadr] += 0.25
    app.data.qpos[app.base_y_qadr] -= 0.12
    app.data.qpos[app.base_yaw_qadr] += np.radians(35.0)
    mujoco.mj_forward(app.model, app.data)
    hand_after = app._target_world_pose("r")
    virtual_after = app._virtual_object_world_pose()

    hand_fixed = (np.linalg.norm(hand_after[0] - hand_before[0]) < 1e-12
                  and abs(abs(np.dot(hand_after[1], hand_before[1])) - 1.0) < 1e-12)
    virtual_fixed = (np.linalg.norm(virtual_after[0] - virtual_before[0]) < 1e-12
                     and abs(abs(np.dot(virtual_after[1], virtual_before[1])) - 1.0) < 1e-12)
    actual_hand_moved = np.linalg.norm(app.data.site_xpos[app.site_r] - hand_before[0]) > 0.1
    ok = hand_fixed and virtual_fixed and actual_hand_moved
    print(f"World target anchor gate: hand_fixed={hand_fixed} virtual_fixed={virtual_fixed} "
          f"actual_robot_moved={actual_hand_moved}: {'OK' if ok else 'FAIL'}")
    return ok


def run_manual_handover_gate():
    """Manual base motion must carry targets and release without a return command."""
    app = teleop_app.TeleopApp.__new__(teleop_app.TeleopApp)
    app._setup_sim()
    targets_before = {side: app._target_world_pose(side) for side in ("r", "l")}
    virtual_before = app._virtual_object_world_pose()
    app.whole_body_solver.solve(app.data, targets_before, 0.04)

    previous_base = np.array([
        app.data.qpos[app.base_x_qadr], app.data.qpos[app.base_y_qadr],
        app.data.qpos[app.base_yaw_qadr],
    ])
    app.data.qpos[app.base_x_qadr] += 0.25
    app.data.qpos[app.base_y_qadr] -= 0.10
    app.data.qpos[app.base_yaw_qadr] += np.radians(20.0)
    mujoco.mj_forward(app.model, app.data)
    current_base = np.array([
        app.data.qpos[app.base_x_qadr], app.data.qpos[app.base_y_qadr],
        app.data.qpos[app.base_yaw_qadr],
    ])
    teleop_targets.carry_world_targets_with_base(app, previous_base, current_base)
    targets_after = {side: app._target_world_pose(side) for side in ("r", "l")}
    virtual_after = app._virtual_object_world_pose()

    delta_yaw = current_base[2] - previous_base[2]
    c, s = np.cos(delta_yaw), np.sin(delta_yaw)
    rotation = np.array([[c, -s], [s, c]])

    def expected_position(position):
        expected = np.asarray(position).copy()
        expected[:2] = current_base[:2] + rotation @ (expected[:2] - previous_base[:2])
        return expected

    carried_positions = all(
        np.linalg.norm(targets_after[side][0] - expected_position(targets_before[side][0]))
        < 1e-12 for side in ("r", "l"))
    carried_virtual = np.linalg.norm(
        virtual_after[0] - expected_position(virtual_before[0])) < 1e-12
    delta_quat = np.array([np.cos(delta_yaw / 2), 0.0, 0.0, np.sin(delta_yaw / 2)])
    carried_orientations = True
    for side in ("r", "l"):
        expected_quat = np.zeros(4)
        mujoco.mju_mulQuat(expected_quat, delta_quat, targets_before[side][1])
        carried_orientations &= abs(abs(np.dot(expected_quat, targets_after[side][1])) - 1.0) < 1e-12

    app.whole_body_solver.rebase(app.data, targets_after)
    handover = app.whole_body_solver.solve(app.data, targets_after, 0.04)
    handover_speed = np.linalg.norm([
        handover.base_twist.vx, handover.base_twist.vy, handover.base_twist.wz])
    no_return = handover_speed < 1e-10

    shifted_targets = {
        side: (pose[0] + np.array([0.05, 0.0, 0.0]), pose[1])
        for side, pose in targets_after.items()
    }
    resumed = app.whole_body_solver.solve(app.data, shifted_targets, 0.04)
    resumed_speed = np.linalg.norm([resumed.base_twist.vx, resumed.base_twist.vy])
    wbik_resumes = resumed_speed > 0.05
    ok = (carried_positions and carried_virtual and carried_orientations
          and no_return and wbik_resumes)
    print(f"Manual handover gate: carried_pos={carried_positions} "
          f"carried_ori={carried_orientations} virtual={carried_virtual} "
          f"return_twist={handover_speed:.2e} resumed={resumed_speed:.3f}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_manual_release_physical_gate():
    """End-to-end release must stop chassis and wheel rotation without a rebound."""
    app = teleop_app.TeleopApp.__new__(teleop_app.TeleopApp)
    app._setup_sim()
    app.q_des_r = teleop_app.HOME_Q_R.copy()
    app.q_des_l = teleop_app.HOME_Q_L.copy()
    app.arm_mode = {"r": "ik", "l": "ik"}
    app.fk_q_deg = {
        "r": list(np.degrees(app.q_des_r)),
        "l": list(np.degrees(app.q_des_l)),
    }
    app.frame_dt = 0.04
    app.steps_per_frame = round(app.frame_dt / app.model.opt.timestep)
    app.ik_err_mm = {"r": 0.0, "l": 0.0}
    no_keys = {key: False for key in ("w", "a", "s", "d", "left", "right")}
    backward = dict(no_keys, s=True)

    for _ in range(25):
        app._step_physics(backward)
    release_x = float(app.data.qpos[app.base_x_qadr])
    positions = [release_x]
    stop_time = None
    wheel_stop_time = None
    for frame in range(75):
        app._step_physics(no_keys)
        positions.append(float(app.data.qpos[app.base_x_qadr]))
        max_wheel_speed = max(
            abs(app.data.qvel[dof]) for dof in app.wheel_drive_dofs.values())
        if wheel_stop_time is None and max_wheel_speed < 0.01:
            wheel_stop_time = (frame + 1) * app.frame_dt
        if (stop_time is None and abs(app.data.qvel[app.base_x_dof]) < 0.01
                and not app._manual_override_active):
            stop_time = (frame + 1) * app.frame_dt

    minimum_x = min(positions)
    return_distance = positions[-1] - minimum_x
    braking_excursion = release_x - minimum_x
    ok = (return_distance < 0.005
          and stop_time is not None and stop_time < 0.5
          and wheel_stop_time is not None and wheel_stop_time < 0.5)
    print(f"Manual release physical gate: brake_excursion={braking_excursion*1000:.1f}mm "
          f"return={return_distance*1000:.3f}mm base_stop={stop_time}s "
          f"wheel_stop={wheel_stop_time}s: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_whole_body_solver_gate(model):
    data = mujoco.MjData(model)
    _reset(model, data)
    sites = _sites(model)
    solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    targets = _target_poses(data, sites, [0.12, 0.0, 0.08])
    qpos_before = data.qpos.copy()
    lift_qadr = model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")]
    command = solver.solve(
        data, targets, 0.04,
        arm_nominal={"r": HOME, "l": HOME},
        lift_nominal=float(data.qpos[lift_qadr]),
    )
    read_only = np.array_equal(data.qpos, qpos_before)
    qdot = command.generalized_velocity
    base_used = np.linalg.norm(qdot[:3]) > 1e-3
    lift_used = abs(qdot[3]) > 1e-3
    both_arms_used = all(np.linalg.norm(qdot[solver.side_indices[s]]) > 1e-3 for s in ("r", "l"))
    within_velocity_limits = np.all(np.abs(qdot) <= solver.velocity_limits + 1e-12)

    scratch = mujoco.MjData(model)
    scratch.qpos[:] = data.qpos
    full_velocity = np.zeros(model.nv)
    full_velocity[solver.dof_ids] = qdot
    before_error = sum(np.linalg.norm(targets[s][0] - data.site_xpos[sites[s]]) for s in sites)
    mujoco.mj_integratePos(model, scratch.qpos, full_velocity, 0.04)
    mujoco.mj_forward(model, scratch)
    after_error = sum(np.linalg.norm(targets[s][0] - scratch.site_xpos[sites[s]]) for s in sites)
    descent = after_error < before_error
    ok = read_only and base_used and lift_used and both_arms_used and within_velocity_limits and descent
    print(f"Whole-body solver gate: read_only={read_only} base={base_used} lift={lift_used} "
          f"both_arms={both_arms_used} limits={within_velocity_limits} "
          f"error={before_error*1000:.1f}->{after_error*1000:.1f}mm: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_solver_latency_gate(model, solve_count=200):
    """Keep the bounded QP comfortably below the 25 Hz application's frame budget."""
    data = mujoco.MjData(model)
    _reset(model, data)
    sites = _sites(model)
    solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    targets = _target_poses(data, sites, [0.12, 0.08, 0.04])
    for _ in range(10):
        solver.solve(data, targets, 0.04)
    start = time.perf_counter()
    for _ in range(solve_count):
        solver.solve(data, targets, 0.04)
    milliseconds = 1000.0 * (time.perf_counter() - start) / solve_count
    ok = milliseconds < 5.0
    print(f"Whole-body latency gate: {milliseconds:.3f}ms/solve (<5ms): "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_randomized_whole_body_gate(model, trial_count=40):
    """Repeated one-step descent/read-only/bound checks across XYZ and yaw targets."""
    rng = np.random.default_rng(20260718)
    successes = 0
    worst_ratio = 0.0
    for _ in range(trial_count):
        data = mujoco.MjData(model)
        _reset(model, data)
        sites = _sites(model)
        solver = whole_body_ik.WholeBodyIK(
            model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
        delta = rng.uniform([-0.14, -0.14, -0.08], [0.14, 0.14, 0.12])
        yaw = rng.uniform(-np.radians(18.0), np.radians(18.0))
        targets = _target_poses(data, sites, delta)
        yaw_quat = np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])
        for side in targets:
            position, quaternion = targets[side]
            target_quaternion = np.zeros(4)
            mujoco.mju_mulQuat(target_quaternion, yaw_quat, quaternion)
            targets[side] = (position, target_quaternion)

        qpos_before = data.qpos.copy()
        command = solver.solve(
            data, targets, 0.04,
            arm_nominal={"r": HOME, "l": HOME},
            lift_nominal=float(data.qpos[solver.qpos_adrs[3]]),
        )
        scratch = mujoco.MjData(model)
        scratch.qpos[:] = data.qpos
        velocity = np.zeros(model.nv)
        velocity[solver.dof_ids] = command.generalized_velocity
        before = _pose_error_metric(data, targets, sites)
        mujoco.mj_integratePos(model, scratch.qpos, velocity, 0.04)
        mujoco.mj_forward(model, scratch)
        after = _pose_error_metric(scratch, targets, sites)
        ratio = after / max(before, 1e-9)
        worst_ratio = max(worst_ratio, ratio)
        bounded = np.all(
            np.abs(command.generalized_velocity) <= solver.velocity_limits + 1e-12)
        if (np.array_equal(data.qpos, qpos_before) and bounded and after < before
                and np.isfinite(command.generalized_velocity).all()):
            successes += 1
    ok = successes >= trial_count - 1
    print(f"Randomized WBIK gate: {successes}/{trial_count} descent+read-only+bounded "
          f"worst_ratio={worst_ratio:.3f}: {'OK' if ok else 'FAIL'}")
    return ok


def _physical_whole_body_trial(model, delta, duration=1.5, yaw_delta=0.0,
                               rotate_positions=False):
    """Run one dual-hand target through arm/lift and real wheel-ground contact."""
    data = mujoco.MjData(model)
    _reset(model, data)
    sites = _sites(model)
    solver = whole_body_ik.WholeBodyIK(
        model, {side: f"grasp_target_{side}" for side in ("r", "l")}, ARMS)
    start_targets = _target_poses(data, sites, [0.0, 0.0, 0.0])
    targets = _target_poses(data, sites, delta)
    if yaw_delta:
        center = np.mean([data.site_xpos[site] for site in sites.values()], axis=0)
        cy, sy = np.cos(yaw_delta), np.sin(yaw_delta)
        rotation = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        rotation_quat = np.array([np.cos(yaw_delta / 2.0), 0.0, 0.0,
                                  np.sin(yaw_delta / 2.0)])
        for side, site in sites.items():
            position, quaternion = targets[side]
            if rotate_positions:
                position = center + rotation @ (position - center)
            rotated_quaternion = np.zeros(4)
            mujoco.mju_mulQuat(rotated_quaternion, rotation_quat, quaternion)
            targets[side] = (position, rotated_quaternion)
    controllers = {side: arm_control.ArmTorqueController(model, ARMS[side]) for side in ("r", "l")}
    drive = base_teleop.SwerveDrive()

    steer_qadrs = {
        wheel: model.jnt_qposadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{wheel}_steer_joint")]
        for wheel in base_teleop.WHEELS
    }
    drive_dofs = {
        wheel: model.jnt_dofadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{wheel}_drive_joint")]
        for wheel in base_teleop.WHEELS
    }
    steer_aids = {wheel: mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{wheel}_steer") for wheel in base_teleop.WHEELS}
    drive_aids = {wheel: mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{wheel}_drive") for wheel in base_teleop.WHEELS}
    lift_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")
    lift_qadr = model.jnt_qposadr[lift_jid]
    lift_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lift_joint")
    base_qadrs = np.array([
        model.jnt_qposadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        for name in ("base_x", "base_y", "base_yaw")
    ])
    initial_base = data.qpos[base_qadrs].copy()
    initial_error = _pose_error_metric(data, targets, sites)
    position_ramp_frames = int(np.ceil(
        max(np.max(np.abs(targets[s][0] - start_targets[s][0])) for s in sites) / 0.03))
    orientation_ramp_frames = int(np.ceil(abs(yaw_delta) / np.radians(8.0)))
    ramp_frames = max(1, position_ramp_frames, orientation_ramp_frames)

    frame_dt = 0.04
    max_qacc = 0.0
    reach_time = None
    for frame in range(round(duration / frame_dt)):
        progress = min(1.0, (frame + 1) / ramp_frames)
        command_targets = {}
        for side in sites:
            start_pos, start_quat = start_targets[side]
            final_pos, final_quat = targets[side]
            command_pos = start_pos + progress * (final_pos - start_pos)
            command_quat = (1.0 - progress) * start_quat + progress * final_quat
            if np.dot(start_quat, final_quat) < 0.0:
                command_quat = (1.0 - progress) * start_quat - progress * final_quat
            command_quat /= np.linalg.norm(command_quat)
            command_targets[side] = (command_pos, command_quat)
        command = solver.solve(
            data, command_targets, frame_dt,
            arm_nominal={"r": HOME, "l": HOME},
            lift_nominal=float(data.qpos[lift_qadr]),
        )
        steering = {wheel: float(data.qpos[qadr]) for wheel, qadr in steer_qadrs.items()}
        wheel_velocity = {wheel: float(data.qvel[dof]) for wheel, dof in drive_dofs.items()}
        wheel_commands = drive.update_twist(
            command.base_twist, frame_dt, steering, wheel_velocity)
        for _ in range(round(frame_dt / model.opt.timestep)):
            for side in ("r", "l"):
                controllers[side].apply(data, command.arm_positions[side])
            data.ctrl[lift_aid] = command.lift_position
            for wheel, (angle, speed) in wheel_commands.items():
                data.ctrl[steer_aids[wheel]] = angle
                data.ctrl[drive_aids[wheel]] = speed
            grasp.apply_grasp(model, data, 0.0, 0.0, side="r")
            grasp.apply_grasp(model, data, 0.0, 0.0, side="l")
            mujoco.mj_step(model, data)
            max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))
        current_error = _pose_error_metric(data, targets, sites)
        if reach_time is None and current_error < max(0.02, 0.10 * initial_error):
            reach_time = (frame + 1) * frame_dt

    final_error = _pose_error_metric(data, targets, sites)
    final_position_error = sum(
        np.linalg.norm(targets[s][0] - data.site_xpos[sites[s]]) for s in sites)
    final_orientation_error = 0.0
    for side, site in sites.items():
        current_quat = np.zeros(4)
        mujoco.mju_mat2Quat(current_quat, data.site_xmat[site])
        orientation_error = np.zeros(3)
        mujoco.mju_subQuat(orientation_error, targets[side][1], current_quat)
        final_orientation_error += np.linalg.norm(orientation_error)
    return {
        "initial_error": float(initial_error),
        "final_error": float(final_error),
        "final_position_error": float(final_position_error),
        "final_orientation_error": float(final_orientation_error),
        "base_delta": data.qpos[base_qadrs].copy() - initial_base,
        "reach_time": reach_time,
        "max_qacc": max_qacc,
        "finite": bool(np.isfinite(data.qpos).all()),
    }


def run_physical_whole_body_gate(model):
    """Exercise common translation, lateral, vertical and yaw whole-body targets."""
    trials = {
        "longitudinal": _physical_whole_body_trial(
            model, [-0.25, 0.0, 0.08], duration=2.0),
        "lateral": _physical_whole_body_trial(model, [0.0, 0.22, 0.0], duration=2.0),
        "vertical": _physical_whole_body_trial(model, [0.0, 0.0, 0.18]),
        "yaw": _physical_whole_body_trial(
            model, [0.0, 0.0, 0.0], duration=2.0, yaw_delta=np.radians(25.0)),
    }
    ok = True
    for name, result in trials.items():
        ratio = result["final_error"] / max(result["initial_error"], 1e-9)
        stable = result["finite"] and result["max_qacc"] < 1e5
        tracked = ratio < (0.30 if name == "yaw" else 0.18)
        recruited_expected_base = {
            "longitudinal": result["base_delta"][0] < -0.12,
            "lateral": result["base_delta"][1] > 0.08,
            "vertical": True,
            "yaw": abs(result["base_delta"][2]) > 0.10,
        }[name]
        trial_ok = stable and tracked and recruited_expected_base
        ok &= trial_ok
        base = result["base_delta"]
        print(f"Physical WBIK {name}: error={result['initial_error']*1000:.1f}->"
              f"{result['final_error']*1000:.1f}mm ratio={ratio:.3f} "
              f"(pos={result['final_position_error']*1000:.1f}mm "
              f"ori={np.degrees(result['final_orientation_error']):.1f}deg) "
              f"base=({base[0]:+.3f},{base[1]:+.3f},{np.degrees(base[2]):+.1f}deg) "
              f"reach={result['reach_time']}s max|qacc|={result['max_qacc']:.0f}: "
              f"{'OK' if trial_ok else 'FAIL'}")
    return ok


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    ok = (run_ros_free_dependency_gate()
          and run_swerve_kinematics_gate()
          and run_box_qp_gate()
          and run_joint_limit_cbf_gate(model)
          and run_collision_gradient_gate(model)
          and run_self_collision_cbf_gate(model)
          and run_table_collision_cbf_gate(model)
          and run_collision_inactive_regression_gate(model)
          and run_rigid_grasp_gate(model)
          and run_rigid_grasp_physical_gate()
          and run_world_anchor_gate()
          and run_manual_handover_gate()
          and run_manual_release_physical_gate()
          and run_whole_body_solver_gate(model)
          and run_solver_latency_gate(model)
          and run_randomized_whole_body_gate(model)
          and run_physical_whole_body_gate(model))
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
