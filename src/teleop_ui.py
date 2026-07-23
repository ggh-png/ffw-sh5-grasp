"""Compact ImGui control and diagnostics workspaces for TeleopApp.

Split out of teleop_app.py because it's the one genuinely independent piece of that
file: this module only reads/writes the app's already-public state (`app.targets`,
`app.contact_viz`, ...) and never touches physics or the 3D render -- it doesn't need to
know how IK, grasp synergy, or mj_step work, only what a slider's current value is. This
module doesn't import teleop_app (avoids a circular import); it just duck-types on
whatever `app` object `draw_panel` is called with. Related controls are grouped into tabs
inside one Control Center, while joint/tree inspection shares one Diagnostics window. This
keeps native multi-viewport support without creating a separate OS window for every feature.
"""

import math
import time

from imgui_bundle import imgui

JOG_POS_STEP_DEFAULT = 0.005
JOG_RPY_STEP_DEFAULT = 2.0
HAND_POS_OFFSET_RANGE = (-0.35, 0.35)
POS_AXES = ("X", "Y", "Z")
RPY_AXES = ("Roll", "Pitch", "Yaw")
UI_WINDOW_SPECS = {
    "control": {
        "title": "FFW-SH5 Control Center",
        "position": (575, 10),
        "size": (470, 720),
        "visible": True,
    },
    "diagnostics": {
        "title": "FFW-SH5 Diagnostics",
        "position": (10, 295),
        "size": (650, 390),
        "visible": True,
    },
}


def _begin_expanded(title, flags=0):
    """imgui.begin's return type varies (plain bool vs. (expanded, opened) tuple)
    depending on binding version -- normalize to just the "should I draw contents" bool."""
    result = imgui.begin(title, None, flags) if flags else imgui.begin(title)
    if isinstance(result, tuple):
        return result[0]
    return result


def _ensure_window_state(app):
    """Initialize persistent visibility/filter state without requiring render setup."""
    if not hasattr(app, "ui_windows") or set(app.ui_windows) != set(UI_WINDOW_SPECS):
        previous = getattr(app, "ui_windows", {})
        app.ui_windows = {
            "control": any(previous.get(key, False)
                           for key in ("control", "marker", "right_arm",
                                       "left_arm", "robot")),
            "diagnostics": any(previous.get(key, False)
                               for key in ("diagnostics", "joints", "tree")),
        }
        if not previous:
            app.ui_windows = {
                key: spec["visible"] for key, spec in UI_WINDOW_SPECS.items()
            }
    if not hasattr(app, "kinematic_tree_scope"):
        app.kinematic_tree_scope = "both"
    if not hasattr(app, "kinematic_tree_show_full"):
        app.kinematic_tree_show_full = False
    if not hasattr(app, "ui_layout_request"):
        # The status window remains in the MuJoCo viewport; all tools start as native
        # platform windows just beyond its right edge.  This one-shot request also
        # overrides an older imgui.ini layout that kept them trapped in the main window.
        app.ui_layout_request = "detach"
    return app.ui_windows


def _begin_tool_window(app, key):
    """Open one movable/resizable tool window and persist its close-button state."""
    spec = UI_WINDOW_SPECS[key]
    layout_request = app.ui_layout_request
    main_viewport = imgui.get_main_viewport()
    if layout_request == "detach":
        index = tuple(UI_WINDOW_SPECS).index(key)
        # Cascade title bars so every detached OS window remains individually reachable.
        position = (
            main_viewport.pos.x + main_viewport.size.x + 24.0 + 36.0 * index,
            main_viewport.pos.y + 24.0 + 36.0 * index,
        )
        imgui.set_next_window_pos(position, imgui.Cond_.always)
    elif layout_request == "main":
        imgui.set_next_window_pos(
            (main_viewport.pos.x + spec["position"][0],
             main_viewport.pos.y + spec["position"][1]),
            imgui.Cond_.always)
    else:
        imgui.set_next_window_pos(
            (main_viewport.pos.x + spec["position"][0],
             main_viewport.pos.y + spec["position"][1]),
            imgui.Cond_.first_use_ever)
    imgui.set_next_window_size(spec["size"], imgui.Cond_.first_use_ever)
    expanded, opened = imgui.begin(spec["title"], app.ui_windows[key])
    app.ui_windows[key] = bool(opened)
    return expanded


def _ik_err_text(app, side):
    """FK 모드인 손은 IK를 아예 안 풀므로 mm 오차 자체가 의미 없다 -- 지난 IK
    모드 시절의 값을 그대로 보여주는 대신 "FK"라고 명시한다."""
    if app.arm_mode[side] == "ik":
        return f"{app.ik_err_mm[side]:.2f}mm"
    return "FK"


def _note_manual_pose_edit(app):
    return None


def _clamp(value, lo, hi):
    return min(hi, max(lo, value))


def _slider_float_clamped(label, value, lo, hi, fmt):
    changed, value = imgui.slider_float(label, value, lo, hi, fmt)
    if changed:
        value = _clamp(value, lo, hi)
    return changed, value


def _draw_vector_sliders(prefix, values, axes, lo, hi, fmt, on_change=None):
    changed_any = False
    for i, axis in enumerate(axes):
        changed, values[i] = _slider_float_clamped(f"{axis}##{prefix}_{axis}", values[i], lo, hi, fmt)
        if changed:
            changed_any = True
            if on_change is not None:
                on_change()
    return changed_any


def _ensure_jog_state(app):
    if not hasattr(app, "jog_side"):
        app.jog_side = "virtual" if getattr(app, "cyclo_grasp_captured", False) else "r"
    if not hasattr(app, "jog_pos_step_m"):
        app.jog_pos_step_m = JOG_POS_STEP_DEFAULT
    if not hasattr(app, "jog_rpy_step_deg"):
        app.jog_rpy_step_deg = JOG_RPY_STEP_DEFAULT


def _clamp_pose_targets(targets, side):
    pos = targets[f"pos_{side}"]
    rpy = targets[f"rpy_{side}"]
    for i in range(3):
        pos[i] = _clamp(pos[i], HAND_POS_OFFSET_RANGE[0], HAND_POS_OFFSET_RANGE[1])
        rpy[i] = _clamp(rpy[i], -90.0, 90.0)


def _apply_cartesian_jog(app, side, pos_delta=(0.0, 0.0, 0.0), rpy_delta=(0.0, 0.0, 0.0)):
    if side == "virtual":
        pos = app.targets["virtual_object_pos"]
        rpy = app.targets["virtual_object_rpy"]
        for i in range(3):
            pos[i] = _clamp(pos[i] + pos_delta[i], -0.2, 1.2)
            rpy[i] = _clamp(rpy[i] + rpy_delta[i], -90.0, 90.0)
        app.apply_virtual_object_target()
        _note_manual_pose_edit(app)
        return

    sides = ("l", "r") if side == "both" else (side,)
    for s in sides:
        if app.arm_mode[s] != "ik":
            continue
        pos = app.targets[f"pos_{s}"]
        rpy = app.targets[f"rpy_{s}"]
        for i in range(3):
            pos[i] += pos_delta[i]
            rpy[i] += rpy_delta[i]
        _clamp_pose_targets(app.targets, s)
    _note_manual_pose_edit(app)


def _repeat_button(label):
    pressed = imgui.button(label)
    active = imgui.is_item_active()
    return pressed or active


def _draw_jog_row(app, title, axis_labels, step, is_rotation=False):
    imgui.text(f"{title} jog")
    for i, axis in enumerate(axis_labels):
        if i:
            imgui.same_line()
        neg = f"{axis}-##jog_{title}_{axis}_neg"
        pos = f"{axis}+##jog_{title}_{axis}_pos"
        if _repeat_button(neg):
            delta = [0.0, 0.0, 0.0]
            delta[i] = -step
            if is_rotation:
                _apply_cartesian_jog(app, app.jog_side, rpy_delta=delta)
            else:
                _apply_cartesian_jog(app, app.jog_side, pos_delta=delta)
        imgui.same_line()
        if _repeat_button(pos):
            delta = [0.0, 0.0, 0.0]
            delta[i] = step
            if is_rotation:
                _apply_cartesian_jog(app, app.jog_side, rpy_delta=delta)
            else:
                _apply_cartesian_jog(app, app.jog_side, pos_delta=delta)


def _active_marker_choices(app):
    if app.cyclo_controller == "bimanual_movel" and app.cyclo_grasp_captured:
        return (("virtual", "Virtual object"),)
    return (("r", "Right goal"), ("l", "Left goal"))


def _selected_marker_label(app):
    choices = dict(_active_marker_choices(app))
    jog_side = getattr(app, "jog_side", None)
    if jog_side in choices:
        return choices[jog_side]
    return next(iter(choices.values()))


def _draw_cyclo_control_panel(app):
    _ensure_jog_state(app)
    imgui.text("Controller")
    for controller, label in (("movel", "MoveL"), ("bimanual_movel", "Bimanual MoveL")):
        if controller != "movel":
            imgui.same_line()
        if imgui.radio_button(f"{label}##cyclo{controller}", app.cyclo_controller == controller):
            if controller == "movel" and app.cyclo_grasp_captured:
                app.release_grasp()
            app.cyclo_controller = controller

    _, app.cyclo_move_time = _slider_float_clamped("Move time", app.cyclo_move_time, 0.2, 8.0, "%.1f s")

    if app.cyclo_controller == "bimanual_movel":
        if imgui.button("Release Grasp" if app.cyclo_grasp_captured else "Capture Grasp"):
            if app.cyclo_grasp_captured:
                app.release_grasp()
                app.jog_side = "r"
            else:
                app.capture_grasp()
                app.jog_side = "virtual"
        imgui.text(f"Grasp: {'captured' if app.cyclo_grasp_captured else 'free'}")
        imgui.text(f"Status: {app.cyclo_status}")

    imgui.text("Active marker")
    choices = _active_marker_choices(app)
    if app.jog_side not in {choice[0] for choice in choices}:
        app.jog_side = choices[0][0]
    for i, (side, label) in enumerate(choices):
        if i:
            imgui.same_line()
        if imgui.radio_button(f"{label}##jogside{side}", app.jog_side == side):
            app.jog_side = side
    imgui.text("3D gizmo: arrows = XYZ, rings = Roll/Pitch/Yaw")

    _, app.jog_pos_step_m = _slider_float_clamped("Position step", app.jog_pos_step_m,
                                                  0.001, 0.050, "%.3f m")
    _, app.jog_rpy_step_deg = _slider_float_clamped("RPY step", app.jog_rpy_step_deg,
                                                    0.5, 15.0, "%.1f deg")

    _draw_jog_row(app, "Position", POS_AXES, app.jog_pos_step_m, is_rotation=False)
    _draw_jog_row(app, "RPY", RPY_AXES, app.jog_rpy_step_deg, is_rotation=True)
    if imgui.button("Reset selected RPY##jog_reset_rpy"):
        if app.jog_side == "virtual":
            app.targets["virtual_object_rpy"] = [0.0, 0.0, 0.0]
            app.apply_virtual_object_target()
        else:
            for side in (("l", "r") if app.jog_side == "both" else (app.jog_side,)):
                if app.arm_mode[side] == "ik":
                    app.targets[f"rpy_{side}"][0] = 0.0
                    app.targets[f"rpy_{side}"][1] = 0.0
                    app.targets[f"rpy_{side}"][2] = 0.0
        _note_manual_pose_edit(app)

    if app.cyclo_controller == "bimanual_movel" and app.cyclo_grasp_captured:
        imgui.separator()
        imgui.text("Virtual object target")
        pos = app.targets["virtual_object_pos"]
        rpy = app.targets["virtual_object_rpy"]
        def apply_virtual_edit():
            app.apply_virtual_object_target()
            _note_manual_pose_edit(app)
        _draw_vector_sliders("virtual_object_pos", pos, POS_AXES, -0.2, 1.2, "%.3f m", apply_virtual_edit)
        _draw_vector_sliders("virtual_object_rpy", rpy, RPY_AXES, -90.0, 90.0, "%.1f deg", apply_virtual_edit)


def _draw_status_panel(app, data):
    imgui.text(f"CAN  |  {app.cyclo_controller}  |  marker: {_selected_marker_label(app)}")
    imgui.text(f"sim {data.time:6.1f}s  wall {time.perf_counter()-app.wall_start:6.1f}s  "
               f"{app.freq_ema:4.1f} Hz")
    imgui.text(f"IK err  L: {_ik_err_text(app, 'l')}   R: {_ik_err_text(app, 'r')}")
    imgui.text(f"Base x={data.qpos[app.base_x_qadr]:+.2f}m y={data.qpos[app.base_y_qadr]:+.2f}m "
               f"yaw={math.degrees(data.qpos[app.base_yaw_qadr]):+.1f}deg")
    body_cmd = getattr(app, "commanded_base_twist", None)
    if body_cmd is not None:
        whole_body_state = "ON" if getattr(app, "whole_body_enabled", True) else "OFF (arm-only)"
        imgui.text(f"Whole-body IK {whole_body_state}  |  body cmd vx={body_cmd.vx:+.2f} "
                   f"vy={body_cmd.vy:+.2f} wz={body_cmd.wz:+.2f}")
    if getattr(app, "collision_viz", False):
        active = len(getattr(app, "collision_active_pairs", ()))
        distance = getattr(app, "collision_min_distance", math.inf)
        buffer_mm = 1000.0 * app.whole_body_solver.collision_buffer
        distance_text = (f"min {distance*1000:.1f}mm" if math.isfinite(distance)
                         else f"clear >{buffer_mm:.0f}mm")
        violation = getattr(app, "collision_constraint_violation", 0.0)
        imgui.text(f"Collision CBF viz ON  |  active {active}  |  {distance_text}  |  "
                   f"slack {violation:.4f}m/s")
    imgui.text("Keys: arrows drive/yaw, [/] strafe, Q/E lift, R reset, G contacts, V collision, C camera")


def _draw_ik_pose_controls(app, targets, side):
    pos = targets[f"pos_{side}"]
    rpy = targets[f"rpy_{side}"]
    imgui.text("Position offset from home (startup/world anchor)")
    _draw_vector_sliders(f"{side}_pos", pos, POS_AXES,
                         HAND_POS_OFFSET_RANGE[0], HAND_POS_OFFSET_RANGE[1], "%.3f m",
                         lambda: _note_manual_pose_edit(app))
    imgui.text("Orientation RPY (home-relative)")
    _draw_vector_sliders(f"{side}_rpy", rpy, RPY_AXES, -90.0, 90.0, "%.1f deg",
                         lambda: _note_manual_pose_edit(app))
    if imgui.button(f"Reset RPY##{side}"):
        rpy[0], rpy[1], rpy[2] = 0.0, 0.0, 0.0
        _note_manual_pose_edit(app)


def _draw_fk_joint_controls(app, side):
    imgui.text("Joint angles (deg)")
    fk_deg = app.fk_q_deg[side]
    for i, (lo, hi) in enumerate(app.arm_joint_ranges_deg[side]):
        _, fk_deg[i] = _slider_float_clamped(f"J{i+1}##{side}fk", fk_deg[i], lo, hi, "%.1f deg")


def _draw_arm_panel(app, targets, side):
    mode = app.arm_mode[side]
    imgui.text(f"Mode: {'IK pose' if mode == 'ik' else 'FK joints'}")
    imgui.same_line()
    if imgui.button(f"Switch to {'FK' if mode == 'ik' else 'IK'}##{side}mode"):
        app.set_arm_mode(side, "fk" if mode == "ik" else "ik")

    if mode == "ik":
        _draw_ik_pose_controls(app, targets, side)
    else:
        _draw_fk_joint_controls(app, side)


def _draw_can_grasp_panel(app, targets):
    for side, label in (("r", "Right"), ("l", "Left")):
        if side == "l":
            imgui.separator()
        if imgui.button(f"{'Release' if app.grab_state[side] else 'Grab'} {label}##grab{side}"):
            app.grab_state[side] = not bool(app.grab_state[side])
        changed, targets[f"grasp_{side}"] = imgui.slider_float(
            f"{label} grasp##{side}", targets[f"grasp_{side}"], 0.0, 1.0)
        if changed:
            app.grab_state[side] = None
        changed, targets[f"thumb_{side}"] = imgui.slider_float(
            f"{label} thumb##{side}", targets[f"thumb_{side}"], 0.0, 1.0)
        if changed:
            app.grab_state[side] = None


def _draw_lift_utils_panel(app, targets):
    whole_body_enabled = getattr(app, "whole_body_enabled", True)
    button_label = ("Whole-body Control: ON##wholebody"
                    if whole_body_enabled else "Whole-body Control: OFF (arm-only)##wholebody")
    if imgui.button(button_label):
        app.toggle_whole_body_control()
    imgui.same_line()
    imgui.text("base + lift join IK" if whole_body_enabled else "base + lift excluded from IK")
    _, targets["lift"] = _slider_float_clamped(
        "Lift target", targets["lift"], app.lift_range[0], app.lift_range[1], "%.3f m")
    if imgui.button("Reset Can (R)"):
        app.reset_active_object()
    imgui.same_line()
    if imgui.button("Contact Viz (G)"):
        app.contact_viz = not app.contact_viz
    imgui.same_line()
    changed, collision_viz = imgui.checkbox(
        "Collision CBF Viz (V)", getattr(app, "collision_viz", False))
    if changed:
        app.collision_viz = collision_viz
    imgui.same_line()
    if imgui.button("Camera (C)"):
        app.cycle_camera()


def _draw_joint_monitor(app, data):
    imgui.begin_child("joint_monitor", (0, 0), True)
    for name in app.monitor_qposadr:
        val = float(data.qpos[app.monitor_qposadr[name]])
        lo, hi = app.monitor_ranges[name]
        frac = (val - lo) / (hi - lo) if hi > lo else 0.0
        frac = _clamp(frac, 0.0, 1.0)
        imgui.progress_bar(frac, (200, 0), f"{name} {math.degrees(val):+.1f}deg")
    imgui.end_child()


def kinematic_tree_body_ids(app, scope=None, show_full=None):
    """Return body ids visible in the tree window for a side/full-tree selection."""
    _ensure_window_state(app)
    tree = app.whole_body_solver.kinematic_tree
    scope = app.kinematic_tree_scope if scope is None else scope
    show_full = app.kinematic_tree_show_full if show_full is None else show_full
    if scope not in {"both", "r", "l"}:
        raise ValueError(f"invalid kinematic tree scope: {scope!r}")
    if show_full:
        return frozenset(range(len(tree.bodies)))

    visible = {0}
    sides = ("r", "l") if scope == "both" else (scope,)
    for side in sides:
        site_id = app.whole_body_solver.kinematics_solvers[side].site_id
        visible.update(tree.site_paths[site_id])
    return frozenset(visible)


def _joint_state_text(app, joint):
    value = float(app.data.qpos[joint.qpos_adr])
    if joint.kind_name == "hinge":
        return f"{math.degrees(value):+.1f} deg"
    if joint.kind_name == "slide":
        return f"{value:+.3f} m"
    return "multi-DOF state"


def _draw_kinematic_body(app, body_id, visible_body_ids, controlled_joint_ids,
                         target_site_ids):
    tree = app.whole_body_solver.kinematic_tree
    body = tree.bodies[body_id]
    body_name = body.name or "world"
    flags = (imgui.TreeNodeFlags_.span_avail_width
             | imgui.TreeNodeFlags_.draw_lines_to_nodes)
    if not app.kinematic_tree_show_full or body_id == 0:
        flags |= imgui.TreeNodeFlags_.default_open
    expanded = imgui.tree_node_ex(f"{body_name}  [body {body_id}]##kinbody{body_id}", flags)
    if not expanded:
        return

    for joint_id in body.joint_ids:
        joint = tree.joints[joint_id]
        marker = "[controlled] " if joint_id in controlled_joint_ids else ""
        name = joint.name or f"joint {joint_id}"
        imgui.bullet_text(
            f"{marker}{name} <{joint.kind_name}>  {_joint_state_text(app, joint)}")
    for site_id in tree.sites_by_body[body_id]:
        site = tree.sites[site_id]
        marker = "[IK target] " if site_id in target_site_ids else ""
        name = site.name or f"site {site_id}"
        imgui.bullet_text(f"{marker}{name} <site>")
    for child_id in tree.children_by_body[body_id]:
        if child_id in visible_body_ids:
            _draw_kinematic_body(
                app, child_id, visible_body_ids, controlled_joint_ids, target_site_ids)
    imgui.tree_pop()


def _draw_kinematic_tree(app):
    tree = app.whole_body_solver.kinematic_tree
    imgui.text("Scope")
    for index, (scope, label) in enumerate(
            (("both", "Both arms"), ("r", "Right"), ("l", "Left"))):
        if index:
            imgui.same_line()
        if imgui.radio_button(f"{label}##tree_scope_{scope}",
                              app.kinematic_tree_scope == scope):
            app.kinematic_tree_scope = scope
    changed, show_full = imgui.checkbox(
        "Show full MJCF tree", app.kinematic_tree_show_full)
    if changed:
        app.kinematic_tree_show_full = show_full

    visible = kinematic_tree_body_ids(app)
    controlled_joint_ids = set(map(int, app.whole_body_solver.joint_ids))
    target_site_ids = {
        solver.site_id for solver in app.whole_body_solver.kinematics_solvers.values()
    }
    imgui.text(
        f"Showing {len(visible)}/{len(tree.bodies)} bodies  |  "
        f"{len(controlled_joint_ids)} controlled joints")
    imgui.text("[controlled] solver column   [IK target] grasp site")
    imgui.separator()
    imgui.begin_child("kinematic_tree_scroll", (0, 0), True)
    _draw_kinematic_body(
        app, 0, visible, controlled_joint_ids, target_site_ids)
    imgui.end_child()


def _draw_window_visibility(app):
    imgui.separator_text("Workspaces")
    if imgui.button("Detach tools outside"):
        app.ui_layout_request = "detach"
    imgui.same_line()
    if imgui.button("Return tools to main"):
        app.ui_layout_request = "main"

    if imgui.button("Show all"):
        for key in app.ui_windows:
            app.ui_windows[key] = True
    imgui.same_line()
    if imgui.button("Control only"):
        for key in app.ui_windows:
            app.ui_windows[key] = key == "control"
    imgui.same_line()
    if imgui.button("Hide all"):
        for key in app.ui_windows:
            app.ui_windows[key] = False

    for index, (key, spec) in enumerate(UI_WINDOW_SPECS.items()):
        if index % 2:
            imgui.same_line()
        changed, visible = imgui.checkbox(
            f"{spec['title']}##window_{key}", app.ui_windows[key])
        if changed:
            app.ui_windows[key] = visible


def _draw_status_window(app, data):
    main_viewport = imgui.get_main_viewport()
    imgui.set_next_window_pos(
        (main_viewport.pos.x + 10.0, main_viewport.pos.y + 10.0),
        imgui.Cond_.always)
    imgui.set_next_window_size((550, 275), imgui.Cond_.first_use_ever)
    if _begin_expanded("FFW-SH5 Status & Windows"):
        _draw_status_panel(app, data)
        _draw_window_visibility(app)
    imgui.end()


def _draw_if_visible(app, key, draw_contents):
    if not app.ui_windows[key]:
        return
    expanded = _begin_tool_window(app, key)
    if expanded:
        draw_contents()
    imgui.end()


def _draw_tab(label, draw_contents):
    """Draw one selected tab while keeping binding-specific tuple handling local."""
    selected, _ = imgui.begin_tab_item(label)
    if selected:
        draw_contents()
        imgui.end_tab_item()


def _draw_control_center(app, targets):
    """Group the normal operator workflow into one tabbed native window."""
    if not imgui.begin_tab_bar("control_center_tabs"):
        return
    _draw_tab("Target", lambda: _draw_cyclo_control_panel(app))
    _draw_tab(
        f"Right Arm ({app.arm_mode['r'].upper()})###right_arm_tab",
        lambda: _draw_arm_panel(app, targets, "r"))
    _draw_tab(
        f"Left Arm ({app.arm_mode['l'].upper()})###left_arm_tab",
        lambda: _draw_arm_panel(app, targets, "l"))

    def draw_robot_controls():
        imgui.separator_text("Lift / Utilities")
        _draw_lift_utils_panel(app, targets)
        imgui.separator_text("Can Grasp")
        _draw_can_grasp_panel(app, targets)

    _draw_tab("Robot / Grasp", draw_robot_controls)
    imgui.end_tab_bar()


def _draw_diagnostics(app, data):
    """Keep infrequent, scroll-heavy inspection tools out of the control workflow."""
    if not imgui.begin_tab_bar("diagnostics_tabs"):
        return
    _draw_tab("Kinematic Tree", lambda: _draw_kinematic_tree(app))
    _draw_tab("Joint Monitor", lambda: _draw_joint_monitor(app, data))
    imgui.end_tab_bar()


def draw_panel(app):
    """Draw two tabbed workspaces and write UI changes to app state."""
    targets = app.targets
    data = app.data
    _ensure_window_state(app)
    _draw_status_window(app, data)
    _draw_if_visible(app, "control", lambda: _draw_control_center(app, targets))
    _draw_if_visible(app, "diagnostics", lambda: _draw_diagnostics(app, data))
    app.ui_layout_request = None
