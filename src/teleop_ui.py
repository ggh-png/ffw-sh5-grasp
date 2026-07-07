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


def _begin_expanded(title, flags=0):
    """imgui.begin's return type varies (plain bool vs. (expanded, opened) tuple)
    depending on binding version -- normalize to just the "should I draw contents" bool."""
    result = imgui.begin(title, None, flags) if flags else imgui.begin(title)
    if isinstance(result, tuple):
        return result[0]
    return result


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
    imgui.text(f"IK err  L: {app.ik_err_mm['l']:.2f}mm   R: {app.ik_err_mm['r']:.2f}mm")
    imgui.text(f"Base  x={data.qpos[app.base_x_qadr]:+.2f}m y={data.qpos[app.base_y_qadr]:+.2f}m "
               f"yaw={math.degrees(data.qpos[app.base_yaw_qadr]):+.1f}deg  "
               f"(Up/Down drive, Left/Right yaw, Shift+Left/Right strafe, Q/E lift)")
    imgui.separator()

    for side, label in (("r", "Right hand control target"), ("l", "Left hand control target")):
        if imgui.collapsing_header(label, imgui.TreeNodeFlags_.default_open):
            pos = targets[f"pos_{side}"]
            rpy = targets[f"rpy_{side}"]
            for i, axis in enumerate(("X", "Y", "Z")):
                _, pos[i] = imgui.slider_float(f"{axis}##{side}pos", pos[i], -0.2, 1.2, "%.3f m")
            imgui.text("Roll/Pitch/Yaw (relative to home pose, hand-local axes)")
            for i, axis in enumerate(("Roll", "Pitch", "Yaw")):
                _, rpy[i] = imgui.slider_float(f"{axis}##{side}rpy", rpy[i], -90.0, 90.0, "%.1f deg")
            if imgui.button(f"Reset orientation##{side}"):
                rpy[0], rpy[1], rpy[2] = 0.0, 0.0, 0.0

    if imgui.collapsing_header("Hand grasp targets", imgui.TreeNodeFlags_.default_open):
        for side, label in (("r", "Right"), ("l", "Left")):
            _, targets[f"grasp_{side}"] = imgui.slider_float(
                f"{label} grasp##{side}", targets[f"grasp_{side}"], 0.0, 1.0)
            _, targets[f"thumb_{side}"] = imgui.slider_float(
                f"{label} thumb##{side}", targets[f"thumb_{side}"], 0.0, 1.0)

    if imgui.collapsing_header("Lift / Utils", imgui.TreeNodeFlags_.default_open):
        _, targets["lift"] = imgui.slider_float(
            "Lift", targets["lift"], app.lift_range[0], app.lift_range[1], "%.3f m")
        if imgui.button("Reset Can (R)"):
            app.reset_can()
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
