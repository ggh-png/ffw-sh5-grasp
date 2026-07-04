"""Phase 4 -- slider teleop app for models/full_scene.xml, single native window.

Reproduces the reference video's interface directly in the same window as the 3D view:
EE pose target sliders (X/Y/Z/Roll/Pitch/Yaw) per hand driving the Phase 3 IK
(src/ik.py) + Phase 3 arm torque control (src/arm_control.py), grasp/thumb sliders per
hand driving the Phase 2 synergy (src/grasp.py), a joint position monitor, and an HUD
(ik_err, sim/wall time, loop freq).

**Rendering note**: this does NOT use mujoco.viewer.launch_passive, because that owns its
own window with no hook for drawing custom widgets inside it. Instead it opens one GLFW
window and drives MuJoCo's own low-level render API (MjrContext/MjvScene/mjr_render)
directly, with Dear ImGui (via imgui-bundle) drawn on top in the same framebuffer each
frame -- the same approach MuJoCo's own C++ "simulate" app uses internally. Earlier
attempts at this in this sandboxed environment (Python 3.14, Wayland session) hit
`OpenGL.error.Error: Attempt to retrieve context when no valid context` from imgui-bundle's
PyOpenGL-based renderer; the fix is `glfw.init_hint(glfw.PLATFORM, glfw.PLATFORM_X11)`
before `glfw.init()`, which makes GLFW create an XWayland (GLX) window instead of a native
Wayland (EGL) one, matching the platform PyOpenGL's context tracking expects (see
NOTES.md "Phase 4" for the diagnosis; imgui_bundle's own glfw_backend.py has a related
PYOPENGL_PLATFORM workaround, but it only takes effect if applied before `OpenGL` is
imported anywhere in the process -- forcing GLFW's own platform sidesteps that ordering
requirement entirely).

Since everything now runs in one thread/one loop (no more GUI-thread-writes-targets /
physics-thread-reads split), the one-way-data-flow constraint PLAN.md asks for is trivially
true: there's only one flow, target sliders read by the physics update each frame, no
concurrent access at all.

Run: `python3 src/teleop_app.py`.
Mouse: left-drag orbit, right-drag pan, scroll zoom (standard MuJoCo camera controls).
Keyboard: R = reset can (+-2cm random), G = toggle contact force/point visualization,
C = cycle camera preset (overview / right-hand close-up). Same three actions also have
buttons in the on-screen panel.
"""

import math
import pathlib
import sys
import time

import glfw
# Must precede glfw.init() -- see module docstring.
glfw.init_hint(glfw.PLATFORM, glfw.PLATFORM_X11)

from imgui_bundle import imgui
from imgui_bundle.python_backends.glfw_backend import GlfwRenderer
import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "full_scene.xml"

import arm_control  # noqa: E402
import grasp  # noqa: E402
import ik  # noqa: E402

ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
# Reused from tests/test_phase_3.py / tests/test_phase_4.py -- see NOTES.md "Phase 4" for
# why these carry over unchanged onto full_scene.xml (pure rigid translation, no rotation,
# between arm_hand.xml's fixed arm_base and this scene's lift-jointed one).
HOME_Q_R = np.array([-0.225, -0.394, 0.682, -2.613, -0.704, 0.843, -1.218])
HOME_Q_L = np.array([-0.2222, 0.3763, -0.4512, -1.2252, 0.8006, 0.9576, 0.0270])
LIFT_RANGE = (-0.5, 0.0)
MONITOR_JOINTS = (
    [f"arm_r_joint{i}" for i in range(1, 8)] + [f"arm_l_joint{i}" for i in range(1, 8)]
    + [f"finger_r_joint{i}" for i in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)]
    + [f"finger_l_joint{i}" for i in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)]
    + ["lift_joint", "head_joint1", "head_joint2"]
)

WINDOW_W, WINDOW_H = 1440, 900
LOOP_HZ = 25.0
IK_MAX_ITER = 30


def rpy_deg_to_quat(rpy_deg):
    r, p, y = (math.radians(v) for v in rpy_deg)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_rpy_deg(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(v) for v in (roll, pitch, yaw)]


def _begin_expanded(title, flags=0):
    """imgui.begin's return type varies (plain bool vs. (expanded, opened) tuple)
    depending on binding version -- normalize to just the "should I draw contents" bool."""
    result = imgui.begin(title, None, flags) if flags else imgui.begin(title)
    if isinstance(result, tuple):
        return result[0]
    return result


def _set_camera_preset(cam, preset):
    if preset == 0:  # overview
        cam.lookat[:] = [0.3, 0.0, 1.0]
        cam.distance = 2.2
        cam.azimuth = 120
        cam.elevation = -20
    else:  # right-hand close-up
        cam.lookat[:] = [0.5055, 0.0, 0.85]
        cam.distance = 0.5
        cam.azimuth = 90
        cam.elevation = -15


def _reset_can_random(model, data, rng):
    """The one qpos write in this file outside of the initial keyframe reset -- resetting a
    freely-placed object's spawn pose is the explicit exception PLAN.md's rule 1 carves out,
    not a kinematic override of the robot itself."""
    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    qadr = model.jnt_qposadr[can_jid]
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    home_can_pos = model.key_qpos[key_id][qadr:qadr + 3].copy()
    data.qpos[qadr:qadr + 3] = home_can_pos + rng.uniform(-0.02, 0.02, size=3)
    data.qpos[qadr + 3:qadr + 7] = [1, 0, 0, 0]
    dof = model.jnt_dofadr[can_jid]
    data.qvel[dof:dof + 6] = 0.0


class KeyEdge:
    """Edge-triggered key check (fires once per press, not once per frame held)."""

    def __init__(self):
        self._prev = set()

    def pressed(self, window, key):
        down = glfw.get_key(window, key) == glfw.PRESS
        was_down = key in self._prev
        if down:
            self._prev.add(key)
        else:
            self._prev.discard(key)
        return down and not was_down


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    solver_r = ik.InverseKinematics(model, "grasp_target_r", ARM_R)
    solver_l = ik.InverseKinematics(model, "grasp_target_l", ARM_L)
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
    lift_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lift_joint")
    monitor_qposadr = {n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                        for n in MONITOR_JOINTS}
    monitor_ranges = {n: model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                       for n in MONITOR_JOINTS}
    rng = np.random.default_rng()

    site_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r")
    site_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l")
    init_quat_r = np.zeros(4)
    mujoco.mju_mat2Quat(init_quat_r, data.site_xmat[site_r])
    init_quat_l = np.zeros(4)
    mujoco.mju_mat2Quat(init_quat_l, data.site_xmat[site_l])

    targets = {
        "pos_r": data.site_xpos[site_r].tolist(), "rpy_r": quat_to_rpy_deg(init_quat_r),
        "pos_l": data.site_xpos[site_l].tolist(), "rpy_l": quat_to_rpy_deg(init_quat_l),
        "grasp_r": 0.0, "thumb_r": 0.0, "grasp_l": 0.0, "thumb_l": 0.0,
        "lift": float(data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")]]),
    }
    contact_viz = [False]
    camera_preset = [0]

    if not glfw.init():
        raise RuntimeError("glfw.init() failed")
    window = glfw.create_window(WINDOW_W, WINDOW_H, "FFW-SH5 Teleop", None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("glfw.create_window() failed")
    glfw.make_context_current(window)
    glfw.swap_interval(0)

    imgui.create_context()
    impl = GlfwRenderer(window)

    scene = mujoco.MjvScene(model, maxgeom=10000)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    _set_camera_preset(cam, 0)
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)
    pert = mujoco.MjvPerturb()
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

    q_des_r = HOME_Q_R.copy()
    q_des_l = HOME_Q_L.copy()
    frame_dt = 1.0 / LOOP_HZ
    steps_per_frame = max(1, round(frame_dt / model.opt.timestep))
    freq_ema = LOOP_HZ
    wall_start = time.perf_counter()
    last_mouse = list(glfw.get_cursor_pos(window))
    keys = KeyEdge()
    ik_err_mm = {"l": 0.0, "r": 0.0}

    while not glfw.window_should_close(window):
        t0 = time.perf_counter()
        glfw.poll_events()
        impl.process_inputs()
        imgui.new_frame()

        io = imgui.get_io()

        # --- camera mouse interaction (skip while ImGui wants the mouse, e.g. dragging a
        # slider) ---
        cur_mouse = list(glfw.get_cursor_pos(window))
        dx, dy = cur_mouse[0] - last_mouse[0], cur_mouse[1] - last_mouse[1]
        last_mouse = cur_mouse
        if not io.want_capture_mouse:
            left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
            right = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
            middle = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
            if left or right or middle:
                _, win_h = glfw.get_window_size(window)
                mod_shift = (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                             or glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
                if right:
                    action = mujoco.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_MOVE_V
                elif left:
                    action = mujoco.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_ROTATE_V
                else:
                    action = mujoco.mjtMouse.mjMOUSE_ZOOM
                mujoco.mjv_moveCamera(model, action, dx / win_h, dy / win_h, scene, cam)
            if io.mouse_wheel != 0:
                mujoco.mjv_moveCamera(model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * io.mouse_wheel, scene, cam)

        if not io.want_capture_keyboard:
            if keys.pressed(window, glfw.KEY_R):
                _reset_can_random(model, data, rng)
                mujoco.mj_forward(model, data)
            if keys.pressed(window, glfw.KEY_G):
                contact_viz[0] = not contact_viz[0]
            if keys.pressed(window, glfw.KEY_C):
                camera_preset[0] = 1 - camera_preset[0]
                _set_camera_preset(cam, camera_preset[0])

        # --- UI panel ---
        imgui.set_next_window_pos((10, 10), imgui.Cond_.first_use_ever)
        imgui.set_next_window_size((380, WINDOW_H - 20), imgui.Cond_.first_use_ever)
        if _begin_expanded("FFW-SH5 Teleop"):
            imgui.text(f"sim {data.time:6.1f}s  wall {time.perf_counter()-wall_start:6.1f}s  "
                       f"{freq_ema:4.1f} Hz")
            imgui.text(f"IK err  L: {ik_err_mm['l']:.2f}mm   R: {ik_err_mm['r']:.2f}mm")
            imgui.separator()

            for side, label in (("r", "Right hand control target"), ("l", "Left hand control target")):
                if imgui.collapsing_header(label, imgui.TreeNodeFlags_.default_open):
                    pos = targets[f"pos_{side}"]
                    rpy = targets[f"rpy_{side}"]
                    for i, axis in enumerate(("X", "Y", "Z")):
                        _, pos[i] = imgui.slider_float(f"{axis}##{side}pos", pos[i], -0.2, 1.2, "%.3f m")
                    for i, axis in enumerate(("Roll", "Pitch", "Yaw")):
                        _, rpy[i] = imgui.slider_float(f"{axis}##{side}rpy", rpy[i], -180.0, 180.0, "%.1f deg")

            if imgui.collapsing_header("Hand grasp targets", imgui.TreeNodeFlags_.default_open):
                for side, label in (("r", "Right"), ("l", "Left")):
                    _, targets[f"grasp_{side}"] = imgui.slider_float(
                        f"{label} grasp##{side}", targets[f"grasp_{side}"], 0.0, 1.0)
                    _, targets[f"thumb_{side}"] = imgui.slider_float(
                        f"{label} thumb##{side}", targets[f"thumb_{side}"], 0.0, 1.0)

            if imgui.collapsing_header("Lift / Utils", imgui.TreeNodeFlags_.default_open):
                _, targets["lift"] = imgui.slider_float("Lift", targets["lift"], LIFT_RANGE[0], LIFT_RANGE[1], "%.3f m")
                if imgui.button("Reset Can (R)"):
                    _reset_can_random(model, data, rng)
                    mujoco.mj_forward(model, data)
                imgui.same_line()
                if imgui.button("Toggle Contact Viz (G)"):
                    contact_viz[0] = not contact_viz[0]
                imgui.same_line()
                if imgui.button("Cycle Camera (C)"):
                    camera_preset[0] = 1 - camera_preset[0]
                    _set_camera_preset(cam, camera_preset[0])

            if imgui.collapsing_header("Joint position monitor"):
                imgui.begin_child("joint_monitor", (0, 260), True)
                for name in MONITOR_JOINTS:
                    val = float(data.qpos[monitor_qposadr[name]])
                    lo, hi = monitor_ranges[name]
                    frac = (val - lo) / (hi - lo) if hi > lo else 0.0
                    frac = min(1.0, max(0.0, frac))
                    imgui.progress_bar(frac, (200, 0), f"{name} {math.degrees(val):+.1f}deg")
                imgui.end_child()
        imgui.end()

        # --- physics step ---
        ctx_qpos = data.qpos.copy()
        quat_r = rpy_deg_to_quat(targets["rpy_r"])
        quat_l = rpy_deg_to_quat(targets["rpy_l"])
        q_des_r, perr_r, _ = solver_r.solve_pose(q_des_r, np.array(targets["pos_r"]), quat_r,
                                                  max_iter=IK_MAX_ITER, context_qpos=ctx_qpos)
        q_des_l, perr_l, _ = solver_l.solve_pose(q_des_l, np.array(targets["pos_l"]), quat_l,
                                                  max_iter=IK_MAX_ITER, context_qpos=ctx_qpos)
        ik_err_mm["r"] = perr_r * 1000.0
        ik_err_mm["l"] = perr_l * 1000.0

        for _ in range(steps_per_frame):
            ctrl_r.apply(data, q_des_r)
            ctrl_l.apply(data, q_des_l)
            data.ctrl[lift_aid] = targets["lift"]
            grasp.apply_grasp(model, data, grasp=targets["grasp_r"], thumb=targets["thumb_r"], side="r")
            grasp.apply_grasp(model, data, grasp=targets["grasp_l"], thumb=targets["thumb_l"], side="l")
            mujoco.mj_step(model, data)

        # --- render 3D scene ---
        opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = contact_viz[0]
        opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = contact_viz[0]
        fb_w, fb_h = glfw.get_framebuffer_size(window)
        viewport = mujoco.MjrRect(0, 0, fb_w, fb_h)
        mujoco.mjv_updateScene(model, data, opt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
        mujoco.mjr_render(viewport, scene, context)

        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

        elapsed = time.perf_counter() - t0
        freq_ema = 0.9 * freq_ema + 0.1 * (1.0 / max(elapsed, 1e-6))
        sleep_time = frame_dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    impl.shutdown()
    glfw.terminate()


if __name__ == "__main__":
    main()
