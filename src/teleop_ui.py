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
    if app.scenario == "box" and app.box_tracking:
        app.box_tracking = False


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
        pos[i] = min(1.2, max(-0.2, pos[i]))
        rpy[i] = min(90.0, max(-90.0, rpy[i]))


def _apply_cartesian_jog(app, side, pos_delta=(0.0, 0.0, 0.0), rpy_delta=(0.0, 0.0, 0.0)):
    if side == "virtual":
        pos = app.targets["virtual_object_pos"]
        rpy = app.targets["virtual_object_rpy"]
        for i in range(3):
            pos[i] = min(1.2, max(-0.2, pos[i] + pos_delta[i]))
            rpy[i] = min(90.0, max(-90.0, rpy[i] + rpy_delta[i]))
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
    imgui.text(title)
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


def _draw_cyclo_control_panel(app):
    _ensure_jog_state(app)
    if not imgui.collapsing_header("Cyclo Control", imgui.TreeNodeFlags_.default_open):
        return

    imgui.text("Controller")
    for controller, label in (("movel", "MoveL"), ("bimanual_movel", "Bimanual MoveL")):
        if controller != "movel":
            imgui.same_line()
        if imgui.radio_button(f"{label}##cyclo{controller}", app.cyclo_controller == controller):
            if controller == "movel" and app.cyclo_grasp_captured:
                app.release_grasp()
            app.cyclo_controller = controller

    changed, app.cyclo_move_time = imgui.slider_float(
        "Move time", app.cyclo_move_time, 0.2, 8.0, "%.1f s")
    if changed:
        app.cyclo_move_time = min(8.0, max(0.2, app.cyclo_move_time))

    if app.cyclo_controller == "bimanual_movel":
        if imgui.button("Release Grasp (/capture_grasp false)"
                        if app.cyclo_grasp_captured
                        else "Capture Grasp (/capture_grasp true)"):
            if app.cyclo_grasp_captured:
                app.release_grasp()
                app.jog_side = "r"
            else:
                app.capture_grasp()
                app.jog_side = "virtual"
        imgui.text(f"status: {app.cyclo_status}")

    imgui.text("Interactive marker")
    choices = (("virtual", "virtual_object_marker"),) if (
        app.cyclo_controller == "bimanual_movel" and app.cyclo_grasp_captured
    ) else (("r", "right_goal_marker"), ("l", "left_goal_marker"))
    if app.jog_side not in {choice[0] for choice in choices}:
        app.jog_side = choices[0][0]
    for i, (side, label) in enumerate(choices):
        if i:
            imgui.same_line()
        if imgui.radio_button(f"{label}##jogside{side}", app.jog_side == side):
            app.jog_side = side
    imgui.text("Drag the 3D arrows for XYZ and colored rings for Roll/Pitch/Yaw.")

    changed, app.jog_pos_step_m = imgui.slider_float(
        "Position step", app.jog_pos_step_m, 0.001, 0.050, "%.3f m")
    if changed:
        app.jog_pos_step_m = min(0.050, max(0.001, app.jog_pos_step_m))
    changed, app.jog_rpy_step_deg = imgui.slider_float(
        "RPY step", app.jog_rpy_step_deg, 0.5, 15.0, "%.1f deg")
    if changed:
        app.jog_rpy_step_deg = min(15.0, max(0.5, app.jog_rpy_step_deg))

    _draw_jog_row(app, "Translate", ("X", "Y", "Z"), app.jog_pos_step_m, is_rotation=False)
    _draw_jog_row(app, "Rotate", ("Roll", "Pitch", "Yaw"), app.jog_rpy_step_deg, is_rotation=True)
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
        imgui.text("virtual_object_goal_move")
        pos = app.targets["virtual_object_pos"]
        rpy = app.targets["virtual_object_rpy"]
        for i, axis in enumerate(("X", "Y", "Z")):
            changed, pos[i] = imgui.slider_float(
                f"VO {axis}##virtual_object_pos_{axis}", pos[i], -0.2, 1.2, "%.3f m")
            if changed:
                app.apply_virtual_object_target()
                _note_manual_pose_edit(app)
        for i, axis in enumerate(("Roll", "Pitch", "Yaw")):
            changed, rpy[i] = imgui.slider_float(
                f"VO {axis}##virtual_object_rpy_{axis}", rpy[i], -90.0, 90.0, "%.1f deg")
            if changed:
                app.apply_virtual_object_target()
                _note_manual_pose_edit(app)


def draw_panel(app):
    """Draw the whole "FFW-SH5 Teleop" panel for this frame and write any slider/button
    interaction straight back into `app`'s state (same one-way-data-flow contract as the
    rest of the app: this panel writes targets, the physics step reads them)."""
    targets = app.targets
    data = app.data

    imgui.set_next_window_pos((10, 10), imgui.Cond_.first_use_ever)
    imgui.set_next_window_size((380, app.window_h - 20), imgui.Cond_.first_use_ever)
    if not _begin_expanded("FFW-SH5 Teleop"):
        imgui.end()
        return

    imgui.text(f"sim {data.time:6.1f}s  wall {time.perf_counter()-app.wall_start:6.1f}s  "
               f"{app.freq_ema:4.1f} Hz")
    imgui.text(f"IK err  L: {_ik_err_text(app, 'l')}   R: {_ik_err_text(app, 'r')}")
    imgui.text(f"Base  x={data.qpos[app.base_x_qadr]:+.2f}m y={data.qpos[app.base_y_qadr]:+.2f}m "
               f"yaw={math.degrees(data.qpos[app.base_yaw_qadr]):+.1f}deg  "
               f"(Up/Down drive, Left/Right yaw, [/] strafe, Q/E lift)")
    imgui.text(f"Scenario: {app.scenario.upper()}  (set at launch with --scenario)")
    imgui.separator()

    _draw_cyclo_control_panel(app)

    for side, label in (("r", "Right hand control target"), ("l", "Left hand control target")):
        if imgui.collapsing_header(label, imgui.TreeNodeFlags_.default_open):
            mode = app.arm_mode[side]
            imgui.text(f"Mode: {'IK (pose)' if mode == 'ik' else 'FK (joint)'}")
            imgui.same_line()
            if imgui.button(f"Switch to {'FK' if mode == 'ik' else 'IK'}##{side}mode"):
                app.set_arm_mode(side, "fk" if mode == "ik" else "ik")

            if mode == "ik":
                pos = targets[f"pos_{side}"]
                rpy = targets[f"rpy_{side}"]
                for i, axis in enumerate(("X", "Y", "Z")):
                    changed, pos[i] = imgui.slider_float(
                        f"{axis}##{side}pos", pos[i], -0.2, 1.2, "%.3f m")
                    if changed:
                        _note_manual_pose_edit(app)
                imgui.text("Roll/Pitch/Yaw (relative to home pose, hand-local axes)")
                for i, axis in enumerate(("Roll", "Pitch", "Yaw")):
                    changed, rpy[i] = imgui.slider_float(
                        f"{axis}##{side}rpy", rpy[i], -90.0, 90.0, "%.1f deg")
                    if changed:
                        _note_manual_pose_edit(app)
                if imgui.button(f"Reset orientation##{side}"):
                    rpy[0], rpy[1], rpy[2] = 0.0, 0.0, 0.0
                    _note_manual_pose_edit(app)
            else:
                # FK: IK를 아예 거치지 않고 관절각을 직접 토크 제어기 목표로 쓴다 --
                # 리프트를 움직이는 동안 이 손을 FK로 두면 팔이 리프트에 강체로
                # 붙어서 오르내리기만 하므로, 어깨 높이가 프레임 사이에 바뀌어도
                # IK가 그걸 뒤늦게 쫓아가며 생기는 출렁임 자체가 없다.
                imgui.text("Joint angles (deg)")
                fk_deg = app.fk_q_deg[side]
                for i, (lo, hi) in enumerate(app.arm_joint_ranges_deg[side]):
                    _, fk_deg[i] = imgui.slider_float(f"J{i+1}##{side}fk", fk_deg[i], lo, hi, "%.1f deg")

    if app.scenario == "can":
        if imgui.collapsing_header("Hand grasp targets", imgui.TreeNodeFlags_.default_open):
            for side, label in (("r", "Right"), ("l", "Left")):
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
    else:
        if imgui.collapsing_header("Box squeeze grasp", imgui.TreeNodeFlags_.default_open):
            forces = app.box_contact_forces
            held = app.box_held
            imgui.text(f"Box contact L={forces['l']:.2f}N R={forces['r']:.2f}N "
                       f"{'HELD' if held else 'not held'}")
            imgui.text(f"constraint: {'ACTIVE' if app.constraint_active else 'off'}  "
                       f"relative drift={app.constraint_err_mm:.2f}mm")
            if imgui.button("Release Box" if app.box_grab else "Grab Box"):
                app.box_grab = not app.box_grab
                if not app.box_grab:
                    app.gap_locked = False
                    app.constraint_active = False
                    app.rigid_relative_pose = None
                    app.box_tracking = True
            _, app.box_tracking = imgui.checkbox("Auto-align XYZ targets to box", app.box_tracking)
            if app.gap_locked:
                imgui.text(f"Squeeze locked at {targets['squeeze_gap']*1000:.1f}mm")
            else:
                _, targets["squeeze_gap"] = imgui.slider_float(
                    "Squeeze gap", targets["squeeze_gap"], -0.015, 0.05, "%.3f m")
            _, app.box_squeeze_kp_scale = imgui.slider_float(
                "Arm kp scale while grabbing", app.box_squeeze_kp_scale, 0.05, 1.0, "%.2f")

    if imgui.collapsing_header("Lift / Utils", imgui.TreeNodeFlags_.default_open):
        _, targets["lift"] = imgui.slider_float(
            "Lift", targets["lift"], app.lift_range[0], app.lift_range[1], "%.3f m")
        if imgui.button(f"Reset {app.scenario.title()} (R)"):
            app.reset_active_object()
        imgui.same_line()
        if imgui.button("Toggle Contact Viz (G)"):
            app.contact_viz = not app.contact_viz
        imgui.same_line()
        if imgui.button("Cycle Camera (C)"):
            app.cycle_camera()

    if imgui.collapsing_header("Joint position monitor"):
        imgui.begin_child("joint_monitor", (0, 260), True)
        for name in app.monitor_qposadr:
            val = float(data.qpos[app.monitor_qposadr[name]])
            lo, hi = app.monitor_ranges[name]
            frac = (val - lo) / (hi - lo) if hi > lo else 0.0
            frac = min(1.0, max(0.0, frac))
            imgui.progress_bar(frac, (200, 0), f"{name} {math.degrees(val):+.1f}deg")
        imgui.end_child()
    imgui.end()
