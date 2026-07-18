"""Phase 6 -- can-only Cyclo marker/UI integration gate.

The live teleop app now keeps one object workflow: can grasping plus Cyclo-style hand target
control.  This test covers that contract: numeric X/Y/Z plus Roll/Pitch/Yaw targets are the
control surface, visible markers are synced from those targets, and Bimanual MoveL can still
capture both hand targets and move them through the virtual object marker.

Run headless: `python3 tests/test_phase_6.py`
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "full_scene.xml"

import teleop_app  # noqa: E402
import teleop_render  # noqa: E402
import teleop_ui  # noqa: E402

ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])


def _rpy_deg_to_quat(rpy_deg):
    r, p, y = np.radians(rpy_deg)
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def _make_sim_only_app():
    app = teleop_app.TeleopApp.__new__(teleop_app.TeleopApp)
    app._setup_sim()
    return app


def _set_hand_base_target(app, side, base_pos):
    app.targets[f"pos_{side}"] = (np.array(base_pos) - app.home_pos_local[side]).tolist()


def run_model_gate(model):
    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    virtual_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "virtual_object_marker")
    ok = can_jid != -1 and virtual_body != -1 and model.nq == len(model.key_qpos[0])
    print(f"Model gate: can_free={can_jid} nq={model.nq} key_qpos={len(model.key_qpos[0])} "
          f"virtual_marker={virtual_body}: {'OK' if ok else 'FAIL'}")
    return ok


def run_cyclo_marker_jog_gate():
    class FakeApp:
        arm_mode = {"l": "ik", "r": "ik"}
        cyclo_controller = "movel"
        cyclo_grasp_captured = False
        targets = {
            "pos_l": [0.0, 0.0, 0.0],
            "pos_r": [0.0, 0.0, 0.0],
            "rpy_l": [0.0, 0.0, 0.0],
            "rpy_r": [0.0, 0.0, 0.0],
            "virtual_object_pos": [0.30, 0.0, 0.85],
            "virtual_object_rpy": [0.0, 0.0, 0.0],
        }

        def apply_virtual_object_target(self):
            return None

    app = FakeApp()
    teleop_ui._apply_cartesian_jog(
        app, "both", pos_delta=(0.005, -0.010, 0.015), rpy_delta=(1.0, -2.0, 3.0))
    both_ok = (
        np.allclose(app.targets["pos_l"], [0.005, -0.010, 0.015])
        and np.allclose(app.targets["pos_r"], [0.005, -0.010, 0.015])
        and np.allclose(app.targets["rpy_l"], [1.0, -2.0, 3.0])
        and np.allclose(app.targets["rpy_r"], [1.0, -2.0, 3.0])
    )

    app.arm_mode["l"] = "fk"
    teleop_ui._apply_cartesian_jog(app, "both", pos_delta=(1.0, 1.0, 1.0),
                                   rpy_delta=(100.0, 100.0, 100.0))
    fk_skip_and_clamp_ok = (
        np.allclose(app.targets["pos_l"], [0.005, -0.010, 0.015])
        and np.allclose(app.targets["rpy_l"], [1.0, -2.0, 3.0])
        and np.allclose(app.targets["pos_r"], [0.35, 0.35, 0.35])
        and np.allclose(app.targets["rpy_r"], [90.0, 90.0, 90.0])
    )
    move_marker_ok = (
        teleop_ui._active_marker_choices(app) == (("r", "Right goal"), ("l", "Left goal"))
        and teleop_ui._selected_marker_label(app) == "Right goal"
    )
    app.cyclo_controller = "bimanual_movel"
    app.cyclo_grasp_captured = True
    app.jog_side = "r"
    virtual_marker_ok = (
        teleop_ui._active_marker_choices(app) == (("virtual", "Virtual object"),)
        and teleop_ui._selected_marker_label(app) == "Virtual object"
    )
    ok = both_ok and fk_skip_and_clamp_ok and move_marker_ok and virtual_marker_ok
    print(f"Cyclo marker jog gate: both_updates={both_ok} "
          f"fk_skip_and_clamp={fk_skip_and_clamp_ok} marker_choices={move_marker_ok and virtual_marker_ok}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_initial_ik_target_origin_gate():
    app = _make_sim_only_app()
    offsets_zero = (
        np.allclose(app.targets["pos_r"], [0.0, 0.0, 0.0])
        and np.allclose(app.targets["pos_l"], [0.0, 0.0, 0.0])
    )
    pose_matches = True
    reports = []
    for side, site_id in (("r", app.site_r), ("l", app.site_l)):
        pos, quat = app._target_world_pose(side)
        site_quat = np.zeros(4)
        mujoco.mju_mat2Quat(site_quat, app.data.site_xmat[site_id])
        pos_err = float(np.linalg.norm(pos - app.data.site_xpos[site_id]))
        quat_dot = abs(float(np.dot(quat, site_quat)))
        case_ok = pos_err < 1e-12 and (1.0 - quat_dot) < 1e-12
        pose_matches = pose_matches and case_ok
        reports.append(f"{side}: pos_err={pos_err*1000:.6f}mm quat_dot={quat_dot:.12f}")
    ok = offsets_zero and pose_matches
    print(f"Initial IK target origin gate: offsets_zero={offsets_zero} "
          f"{'; '.join(reports)}: {'OK' if ok else 'FAIL'}")
    return ok


def run_cyclo_bimanual_virtual_object_gate():
    app = _make_sim_only_app()
    _set_hand_base_target(app, "r", [0.34, -0.08, 0.88])
    _set_hand_base_target(app, "l", [0.34, 0.08, 0.88])
    app.targets["rpy_r"] = [0.0, 0.0, 0.0]
    app.targets["rpy_l"] = [0.0, 0.0, 0.0]
    r0 = app._target_world_pose("r")[0]
    l0 = app._target_world_pose("l")[0]

    app.capture_grasp()
    capture_ok = app.cyclo_grasp_captured and app.cyclo_controller == "bimanual_movel"
    rel0 = l0 - r0
    app.targets["virtual_object_pos"][0] += 0.025
    app.targets["virtual_object_pos"][2] += 0.060
    app.targets["virtual_object_rpy"][2] += 12.0
    app.apply_virtual_object_target()
    r1 = app._target_world_pose("r")[0]
    l1 = app._target_world_pose("l")[0]
    rel1 = l1 - r1
    rel_len_ok = abs(np.linalg.norm(rel1) - np.linalg.norm(rel0)) < 1e-9
    moved_ok = np.linalg.norm(0.5 * (r1 + l1) - 0.5 * (r0 + l0)) > 0.05

    app._sync_ik_mocaps_from_targets()
    vo_pos = app._local_to_world_pos(app.targets["virtual_object_pos"])
    marker_err = float(np.linalg.norm(app.data.mocap_pos[app.virtual_object_mocap_id] - vo_pos))

    app.release_grasp()
    release_ok = not app.cyclo_grasp_captured and app.cyclo_capture_offsets is None
    ok = capture_ok and rel_len_ok and moved_ok and marker_err < 1e-9 and release_ok
    print(f"Cyclo bimanual virtual object gate: capture={capture_ok} "
          f"rel_len={rel_len_ok} moved={moved_ok} marker_err={marker_err*1000:.6f}mm "
          f"release={release_ok}: {'OK' if ok else 'FAIL'}")
    return ok


def run_cyclo_3d_gizmo_pose_gate():
    app = _make_sim_only_app()
    world_pos = np.array([0.42, -0.11, 0.94])
    world_quat = _rpy_deg_to_quat([13.0, -8.0, 21.0])
    matrix = app._pose_to_imguizmo_matrix(world_pos, world_quat)
    round_pos, round_quat = app._imguizmo_matrix_to_pose(matrix)
    roundtrip_ok = (
        np.linalg.norm(round_pos - world_pos) < 1e-7
        and abs(abs(float(np.dot(round_quat, world_quat))) - 1.0) < 1e-7
    )

    app._set_gizmo_target_world_pose("r", world_pos, world_quat)
    hand_pos = app._target_world_pose("r")[0]
    hand_quat = app._target_world_quat("r")
    hand_ok = (
        np.linalg.norm(hand_pos - world_pos) < 1e-9
        and abs(abs(float(np.dot(hand_quat, world_quat))) - 1.0) < 1e-9
    )

    app.capture_grasp()
    vo_pos = np.array([0.43, 0.02, 0.98])
    vo_quat = _rpy_deg_to_quat([0.0, 0.0, 16.0])
    app._set_gizmo_target_world_pose("virtual", vo_pos, vo_quat)
    new_vo_pos, new_vo_quat = app._virtual_object_world_pose()
    virtual_ok = (
        np.linalg.norm(new_vo_pos - vo_pos) < 1e-9
        and abs(abs(float(np.dot(new_vo_quat, vo_quat))) - 1.0) < 1e-9
        and app.cyclo_grasp_captured
    )
    ok = roundtrip_ok and hand_ok and virtual_ok
    print(f"Cyclo 3D gizmo pose gate: roundtrip={roundtrip_ok} "
          f"hand_target={hand_ok} virtual_target={virtual_ok}: {'OK' if ok else 'FAIL'}")
    return ok


def run_bimanual_marker_visibility_gate():
    app = _make_sim_only_app()
    geom_id = app.virtual_object_marker_geom_id
    site_id = app.virtual_object_marker_site_id
    geom_alpha0 = float(app.model.geom_rgba[geom_id][3])
    site_alpha0 = float(app.model.site_rgba[site_id][3])

    _set_hand_base_target(app, "r", [0.34, -0.08, 0.88])
    _set_hand_base_target(app, "l", [0.34, 0.08, 0.88])
    app.capture_grasp()
    app._sync_ik_mocaps_from_targets()
    geom_alpha_capture = float(app.model.geom_rgba[geom_id][3])
    site_alpha_capture = float(app.model.site_rgba[site_id][3])

    app.release_grasp()
    app._sync_ik_mocaps_from_targets()
    geom_alpha_release = float(app.model.geom_rgba[geom_id][3])
    site_alpha_release = float(app.model.site_rgba[site_id][3])

    hidden_initial = geom_alpha0 == 0.0 and site_alpha0 == 0.0
    visible_capture = (
        abs(geom_alpha_capture - app.virtual_object_marker_rgba["geom"][3]) < 1e-12
        and abs(site_alpha_capture - app.virtual_object_marker_rgba["site"][3]) < 1e-12
    )
    hidden_release = geom_alpha_release == 0.0 and site_alpha_release == 0.0
    ok = hidden_initial and visible_capture and hidden_release
    print(f"Bimanual marker visibility gate: initial_hidden={hidden_initial} "
          f"capture_visible={visible_capture} release_hidden={hidden_release}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_numeric_target_marker_sync_gate():
    app = _make_sim_only_app()
    app.data.qpos[app.base_x_qadr] = 0.12
    app.data.qpos[app.base_y_qadr] = -0.04
    app.data.qpos[app.base_yaw_qadr] = np.radians(17.0)
    mujoco.mj_forward(app.model, app.data)

    app.targets["pos_r"] = [0.04, -0.03, 0.05]
    app.targets["rpy_r"] = [11.0, -7.0, 5.0]
    app.targets["pos_l"] = [-0.02, 0.04, 0.06]
    app.targets["rpy_l"] = [-9.0, -6.0, -4.0]

    for side, mocap_id in app.ik_target_mocap_ids.items():
        app.data.mocap_pos[mocap_id] = [9.0, 9.0, 9.0]
        app.data.mocap_quat[mocap_id] = [0.0, 1.0, 0.0, 0.0]

    app._sync_ik_mocaps_from_targets()

    ok = True
    reports = []
    for side, mocap_id in app.ik_target_mocap_ids.items():
        expected_pos = app._target_world_pose(side)[0]
        expected_quat = app._target_world_quat(side)
        pos_err = float(np.linalg.norm(app.data.mocap_pos[mocap_id] - expected_pos))
        quat_dot = abs(float(np.dot(app.data.mocap_quat[mocap_id], expected_quat)))
        case_ok = pos_err < 1e-9 and (1.0 - quat_dot) < 1e-9
        ok = ok and case_ok
        reports.append(f"{side}: pos_err={pos_err*1000:.6f}mm quat_dot={quat_dot:.12f}")

    print(f"Numeric target -> marker sync gate: {'; '.join(reports)}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_whole_body_toggle_gate():
    """Mode changes preserve world goals and clear stale chassis commands."""
    app = _make_sim_only_app()
    app.targets["pos_r"] = [0.025, -0.018, 0.012]
    app.targets["rpy_r"] = [7.0, -4.0, 5.0]
    app.targets["pos_l"] = [-0.020, 0.014, 0.018]
    app.targets["rpy_l"] = [-6.0, 3.0, -4.0]
    app.targets["virtual_object_pos"] = [0.31, -0.02, 0.87]
    app.targets["virtual_object_rpy"] = [2.0, -3.0, 8.0]
    hand_before = {side: app._target_world_pose(side) for side in ("r", "l")}
    virtual_before = app._virtual_object_world_pose()
    app.whole_body_base_twist = teleop_app.base_teleop.BodyTwist(0.2, -0.1, 0.3)
    app.commanded_base_twist = teleop_app.base_teleop.BodyTwist(0.2, -0.1, 0.3)

    app.toggle_whole_body_control()
    off_mode = not app.whole_body_enabled
    hand_off = {side: app._target_world_pose(side) for side in ("r", "l")}
    virtual_off = app._virtual_object_world_pose()
    off_pose_preserved = all(
        np.allclose(hand_before[side][0], hand_off[side][0], atol=1e-10)
        and abs(np.dot(hand_before[side][1], hand_off[side][1])) > 1.0 - 1e-10
        for side in ("r", "l"))
    off_pose_preserved &= (
        np.allclose(virtual_before[0], virtual_off[0], atol=1e-10)
        and abs(np.dot(virtual_before[1], virtual_off[1])) > 1.0 - 1e-10)
    stale_command_cleared = (
        app.commanded_base_twist == teleop_app.base_teleop.BodyTwist()
        and app.whole_body_base_twist == teleop_app.base_teleop.BodyTwist())
    smoothed_synced = all(
        np.allclose(app.smoothed_pos[side], app.targets[f"pos_{side}"])
        and np.allclose(app.smoothed_rpy[side], app.targets[f"rpy_{side}"])
        for side in ("r", "l"))

    app.toggle_whole_body_control()
    hand_on = {side: app._target_world_pose(side) for side in ("r", "l")}
    virtual_on = app._virtual_object_world_pose()
    round_trip = app.whole_body_enabled and all(
        np.allclose(hand_before[side][0], hand_on[side][0], atol=1e-10)
        and abs(np.dot(hand_before[side][1], hand_on[side][1])) > 1.0 - 1e-10
        for side in ("r", "l"))
    round_trip &= (
        np.allclose(virtual_before[0], virtual_on[0], atol=1e-10)
        and abs(np.dot(virtual_before[1], virtual_on[1])) > 1.0 - 1e-10)

    captured_app = _make_sim_only_app()
    captured_app.capture_grasp()
    captured_app.targets["virtual_object_pos"][0] += 0.035
    captured_app.targets["virtual_object_rpy"][2] = 6.0
    captured_app.apply_virtual_object_target()
    captured_before = {
        side: captured_app._target_world_pose(side) for side in ("r", "l")}
    captured_virtual_before = captured_app._virtual_object_world_pose()
    captured_app.toggle_whole_body_control()
    captured_app.toggle_whole_body_control()
    captured_round_trip = all(
        np.allclose(captured_before[side][0], captured_app._target_world_pose(side)[0],
                    atol=1e-10)
        and abs(np.dot(captured_before[side][1],
                       captured_app._target_world_pose(side)[1])) > 1.0 - 1e-10
        for side in ("r", "l"))
    captured_virtual_after = captured_app._virtual_object_world_pose()
    captured_round_trip &= (
        np.allclose(captured_virtual_before[0], captured_virtual_after[0], atol=1e-10)
        and abs(np.dot(captured_virtual_before[1], captured_virtual_after[1])) > 1.0 - 1e-10)

    integration_app = _make_sim_only_app()
    integration_app.q_des_r = teleop_app.HOME_Q_R.copy()
    integration_app.q_des_l = teleop_app.HOME_Q_L.copy()
    integration_app.arm_mode = {"r": "ik", "l": "ik"}
    integration_app.fk_q_deg = {
        "r": np.degrees(integration_app.q_des_r).tolist(),
        "l": np.degrees(integration_app.q_des_l).tolist(),
    }
    integration_app.frame_dt = 1.0 / teleop_app.LOOP_HZ
    integration_app.steps_per_frame = max(
        1, round(integration_app.frame_dt / integration_app.model.opt.timestep))
    integration_app.ik_err_mm = {"r": 0.0, "l": 0.0}
    integration_app.toggle_whole_body_control()
    integration_app.targets["lift"] += 0.02
    integration_app._step_physics(
        {key: False for key in ("w", "a", "s", "d", "left", "right")})
    off_integration = (
        integration_app.lift_cmd == integration_app.targets["lift"]
        and integration_app.commanded_base_twist == teleop_app.base_teleop.BodyTwist())

    ok = (off_mode and off_pose_preserved and stale_command_cleared and smoothed_synced
          and round_trip and captured_round_trip and off_integration)
    print(f"Whole-body toggle gate: off={off_mode} off_pose={off_pose_preserved} "
          f"stale_zero={stale_command_cleared} smoothing={smoothed_synced} "
          f"round_trip={round_trip} captured={captured_round_trip} "
          f"integration={off_integration}: {'OK' if ok else 'FAIL'}")
    return ok


def run_collision_visualization_gate():
    """The V toggle must expose active CBF points without requiring a GL window."""
    app = _make_sim_only_app()
    initially_off = (
        not app.collision_viz
        and teleop_render.collision_visualization_data(app) == ())
    app.toggle_collision_visualization()

    lift_index = app.whole_body_solver.index["lift_joint"]
    lift_qadr = app.whole_body_solver.qpos_adrs[lift_index]
    app.data.qpos[lift_qadr] -= 0.035
    mujoco.mj_forward(app.model, app.data)
    constraints = teleop_render.collision_visualization_data(app)
    data_ok = (
        len(constraints) >= 2
        and all(constraint.distance <= app.whole_body_solver.collision_buffer + 1e-12
                for constraint in constraints)
        and all(np.isfinite(constraint.point_a).all()
                and np.isfinite(constraint.point_b).all()
                for constraint in constraints)
    )

    app.scene = mujoco.MjvScene(app.model, maxgeom=100)
    teleop_render._append_collision_overlay(app, constraints)
    overlay_ok = (
        app.scene.ngeom == 3 * len(constraints)
        and sum(int(app.scene.geoms[i].type) == int(mujoco.mjtGeom.mjGEOM_LINE)
                for i in range(app.scene.ngeom)) == len(constraints)
        and sum(int(app.scene.geoms[i].type) == int(mujoco.mjtGeom.mjGEOM_SPHERE)
                for i in range(app.scene.ngeom)) == 2 * len(constraints)
    )
    safe_distance = app.whole_body_solver.collision_safe_distance
    buffer_distance = app.whole_body_solver.collision_buffer
    colors_distinct = (
        not np.array_equal(teleop_render._collision_color(-0.001, safe_distance),
                           teleop_render._collision_color(0.5 * safe_distance, safe_distance))
        and not np.array_equal(
            teleop_render._collision_color(0.5 * safe_distance, safe_distance),
            teleop_render._collision_color(
                0.5 * (safe_distance + buffer_distance), safe_distance))
    )
    app.toggle_collision_visualization()
    toggles_off = not app.collision_viz and not teleop_render.collision_visualization_data(app)
    ok = initially_off and data_ok and overlay_ok and colors_distinct and toggles_off
    print(f"Collision visualization gate: initial_off={initially_off} "
          f"active={len(constraints)} overlay_geoms={app.scene.ngeom} "
          f"colors={colors_distinct} toggles_off={toggles_off}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_manual_xyz_rpy_ik_gate(model):
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    solver_r = teleop_app.ik.InverseKinematics(model, "grasp_target_r", ARM_R)
    solver_l = teleop_app.ik.InverseKinematics(model, "grasp_target_l", ARM_L)
    site_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r")
    site_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l")
    cases = (
        ("r", solver_r, site_r, HOME_Q_R, np.array([-0.035, -0.015, 0.025]), np.array([8.0, -4.0, 6.0])),
        ("l", solver_l, site_l, HOME_Q_L, np.array([-0.035, 0.015, 0.025]), np.array([-8.0, -4.0, -6.0])),
    )
    ok = True
    reports = []
    for side, solver, site_id, q_seed, pos_delta, rpy_delta in cases:
        target_pos = data.site_xpos[site_id].copy() + pos_delta
        home_quat = np.zeros(4)
        target_quat = np.zeros(4)
        mujoco.mju_mat2Quat(home_quat, data.site_xmat[site_id])
        mujoco.mju_mulQuat(target_quat, home_quat, _rpy_deg_to_quat(rpy_delta))
        q_sol, pos_err, ori_err, converged = solver.solve_pose_multistart(
            q_seed, target_pos, target_quat, np.random.default_rng(100 + (side == "l")),
            context_qpos=data.qpos, success_pos_tol=0.005, success_ori_tol=np.radians(5.0))
        scratch = mujoco.MjData(model)
        scratch.qpos[:] = data.qpos
        for qadr, val in zip(solver.qpos_adrs, q_sol):
            scratch.qpos[qadr] = val
        mujoco.mj_forward(model, scratch)
        reached_delta = scratch.site_xpos[site_id] - data.site_xpos[site_id]
        pos_delta_ok = np.linalg.norm(reached_delta - pos_delta) < 0.006
        case_ok = converged and pos_delta_ok and pos_err < 0.005 and ori_err < np.radians(5.0)
        ok = ok and case_ok
        reports.append(
            f"{side}: converged={converged} pos_err={pos_err*1000:.2f}mm "
            f"ori_err={np.degrees(ori_err):.2f}deg delta_ok={pos_delta_ok}")
    print(f"Manual XYZ/RPY IK gate: {'; '.join(reports)}: {'OK' if ok else 'FAIL'}")
    return ok


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    ok = (run_model_gate(model)
          and run_initial_ik_target_origin_gate()
          and run_cyclo_marker_jog_gate()
          and run_cyclo_bimanual_virtual_object_gate()
          and run_cyclo_3d_gizmo_pose_gate()
          and run_bimanual_marker_visibility_gate()
          and run_numeric_target_marker_sync_gate()
          and run_whole_body_toggle_gate()
          and run_collision_visualization_gate()
          and run_manual_xyz_rpy_ik_gate(model))
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
