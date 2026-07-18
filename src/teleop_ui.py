"""Phase 4 -- the ImGui slider panel for teleop_app.TeleopApp.

Split out of teleop_app.py because it's the one genuinely independent piece of that
file: this module only reads/writes the app's already-public state (`app.targets`,
`app.contact_viz`, ...) and never touches physics or the 3D render -- it doesn't need to
know how IK, grasp synergy, or mj_step work, only what a slider's current value is. This
module doesn't import teleop_app (avoids a circular import); it just duck-types on
whatever `app` object `draw_panel` is called with, so the only supported way to add a
new slider/button is to look here, not in teleop_app.py's physics/render code.
"""

import math
import time

from imgui_bundle import imgui

JOG_POS_STEP_DEFAULT = 0.005
JOG_RPY_STEP_DEFAULT = 2.0
HAND_POS_OFFSET_RANGE = (-0.35, 0.35)
POS_AXES = ("X", "Y", "Z")
RPY_AXES = ("Roll", "Pitch", "Yaw")
SIDE_LABELS = {"r": "Right", "l": "Left", "virtual": "Virtual object"}


def _begin_expanded(title, flags=0):
    """imgui.begin's return type varies (plain bool vs. (expanded, opened) tuple)
    depending on binding version -- normalize to just the "should I draw contents" bool."""
    result = imgui.begin(title, None, flags) if flags else imgui.begin(title)
    if isinstance(result, tuple):
        return result[0]
    return result


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


def _section(title, default_open=True):
    flags = imgui.TreeNodeFlags_.default_open if default_open else 0
    return imgui.collapsing_header(title, flags)


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
    if not _section("Cyclo / Marker Control"):
        return

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
        imgui.text(f"Whole-body IK ON  |  body cmd vx={body_cmd.vx:+.2f} "
                   f"vy={body_cmd.vy:+.2f} wz={body_cmd.wz:+.2f}")
    imgui.text("Keys: arrows drive/yaw, [/] strafe, Q/E lift, R reset, G contacts, C camera")
    imgui.separator()


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
    label = f"{SIDE_LABELS[side]} Arm"
    if not _section(label):
        return
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
    if not _section("Can Grasp"):
        return
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
    if not _section("Lift / Utilities"):
        return
    _, targets["lift"] = _slider_float_clamped(
        "Lift target (whole-body)", targets["lift"], app.lift_range[0], app.lift_range[1], "%.3f m")
    if imgui.button("Reset Can (R)"):
        app.reset_active_object()
    imgui.same_line()
    if imgui.button("Contact Viz (G)"):
        app.contact_viz = not app.contact_viz
    imgui.same_line()
    if imgui.button("Camera (C)"):
        app.cycle_camera()


def _draw_joint_monitor(app, data):
    if not _section("Joint Monitor", default_open=False):
        return
    imgui.begin_child("joint_monitor", (0, 260), True)
    for name in app.monitor_qposadr:
        val = float(data.qpos[app.monitor_qposadr[name]])
        lo, hi = app.monitor_ranges[name]
        frac = (val - lo) / (hi - lo) if hi > lo else 0.0
        frac = _clamp(frac, 0.0, 1.0)
        imgui.progress_bar(frac, (200, 0), f"{name} {math.degrees(val):+.1f}deg")
    imgui.end_child()


def draw_panel(app):
    """Draw the whole "FFW-SH5 Teleop" panel for this frame and write any slider/button
    interaction straight back into `app`'s state (same one-way-data-flow contract as the
    rest of the app: this panel writes targets, the physics step reads them)."""
    targets = app.targets
    data = app.data

    imgui.set_next_window_pos((10, 10), imgui.Cond_.first_use_ever)
    imgui.set_next_window_size((460, app.window_h - 20), imgui.Cond_.first_use_ever)
    if not _begin_expanded("FFW-SH5 Teleop"):
        imgui.end()
        return

    _draw_status_panel(app, data)
    _draw_cyclo_control_panel(app)
    _draw_arm_panel(app, targets, "r")
    _draw_arm_panel(app, targets, "l")

    _draw_can_grasp_panel(app, targets)
    _draw_lift_utils_panel(app, targets)
    _draw_joint_monitor(app, data)
    imgui.end()
