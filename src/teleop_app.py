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

**RPY control note (Session 8)**: the Roll/Pitch/Yaw sliders are a rotation *relative to
each hand's home-pose orientation* (`home_quat_r`/`home_quat_l`, captured once at startup),
composed in the hand's own local frame (`quat_mul(home, rpy_delta)`), not raw absolute
world-frame Euler angles. Originally they were absolute (slider shown value = world-Euler
decomposition of the actual site quat), which meant the sliders started at the oddly large
values (90, 0, 90) -- the home pose isn't at identity -- and, because Tait-Bryan Euler angles
compose about progressively-rotated axes, "Roll" and "Pitch" at that operating point actually
rotated the hand about world Y and world -X respectively (verified numerically, not just
felt), not the world/local X/Y one would guess from the labels; only "Yaw" (the outermost
term) ever stayed clean. Composing a local delta onto a fixed home reference instead makes
(0,0,0) the natural pose and pins each slider to its own hand-local axis exactly at that
point (verified: Roll/Pitch/Yaw deltas from home rotate about local X/Y/Z exactly), which is
what most of a teleop session's small-to-moderate adjustments stay near. This does not (and
mathematically cannot, for any 3-parameter Euler triple) remove *all* axis coupling -- push
two sliders far from zero simultaneously and some residual coupling reappears -- but it fixes
the coupling that was previously present even at the resting/default slider position.

**Mobile base (Phase 5, src/base_teleop.py)**: EE pose sliders are base-local, not world-
fixed -- every frame the current pos_r/pos_l/rpy_r/rpy_l target is rotated+translated by the
base's live `(base_x, base_y, base_yaw)` qpos before being handed to IK, so the arm doesn't
have to fight the base moving/turning under it (ffw-sh5-mobile-and-box-plan.md S3.3).
Driving itself never touches qpos -- arrow keys go through `base_teleop.SwerveDrive`
(accel/brake smoothing ported from the sibling ffw-sh5-teleoperation repo's reference feel,
then converted to per-wheel steer angle + drive speed) into the three wheels' real steer/
drive actuators. Motion is genuinely wheel-friction-driven now (Session 8 후속): the
base_x/base_y/base_yaw joints are still there so base_link can't tip over, but nothing
actuates them directly any more -- ground contact under the spinning wheels is what pushes
base_link along them. Deliberately arrow-keys-only, not WASD -- WASD collides with the
keybindings a MuJoCo user already expects from other tools (see NOTES.md "Phase 5 후속").

Run: `python3 src/teleop_app.py`.
Mouse: left-drag orbit, right-drag pan, scroll zoom (standard MuJoCo camera controls).
Keyboard: Up/Down = base forward/back (robot-local), Left/Right = base yaw,
Shift+Left/Right = base strafe, Q/E = lift down/up, R = reset can (+-2cm random),
G = toggle contact force/point
visualization, C = cycle camera preset (overview / right-hand close-up). R/G/C also have
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
import base_teleop  # noqa: E402
import grasp  # noqa: E402
import ik  # noqa: E402

ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
# Matches models/full_scene.xml's "home" keyframe, which as of Session 8 (Phase 5 follow-up)
# matches the sibling ffw-sh5-mujoco repo's rest pose (only the elbow/joint4 bent -90 deg,
# everything else 0) -- see NOTES.md "Phase 5 후속".
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
LIFT_RANGE = (-0.5, 0.0)
MONITOR_JOINTS = (
    [f"arm_r_joint{i}" for i in range(1, 8)] + [f"arm_l_joint{i}" for i in range(1, 8)]
    + [f"finger_r_joint{i}" for i in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)]
    + [f"finger_l_joint{i}" for i in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)]
    + ["lift_joint", "head_joint1", "head_joint2"]
)

WINDOW_W, WINDOW_H = 1440, 900
LOOP_HZ = 25.0
# Session 8 (Phase 5 후속): each solve_pose outer iteration costs several mj_forward calls
# on this full-robot model (~71us each, measured directly) via its backtracking line
# search. Once an RPY target goes past the reachable orientation workspace from the new
# (folded-elbow) home pose, position converges but orientation never does, so the loop
# spends every iteration up to max_iter re-discovering the same non-improving result --
# measured directly: dragging Pitch from 0 to 90 deg (warm-started frame to frame, as the
# live UI does) cost ~20ms/frame once stuck past ~63 deg at max_iter=30, vs ~9.6ms at
# max_iter=15 with *no measurable convergence-quality loss* in the reachable range (checked
# at 30/45/60/63 deg -- pos/ori error identical to 2 decimal places). Two hands stuck at
# once could otherwise burn ~40ms alone, the entire frame budget at LOOP_HZ=25.
IK_MAX_ITER = 15
# See "IK target rate limiting" note near `smoothed_pos`/`smoothed_rpy` below: caps how far
# the *effective* IK target can move in one frame, independent of how far the raw slider
# jumped. 0.02m/frame at LOOP_HZ=25 is 0.5 m/s -- a brisk but trackable teleop speed; 5
# deg/frame is 125 deg/s, generous but bounded.
MAX_POS_STEP_PER_FRAME = 0.02
MAX_RPY_STEP_PER_FRAME_DEG = 5.0


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
    base_x_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_x")]
    base_y_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_y")]
    base_yaw_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_yaw")]
    # Real per-wheel steer (position) + drive (velocity) actuators -- see
    # src/base_teleop.py's SwerveDrive and the module docstring's "Mobile base" note. Base
    # motion is now a *consequence* of wheel-ground friction, not a direct actuator on
    # base_x/base_y/base_yaw (those joints still exist so base_link can't tip, but nothing
    # actuates them directly any more).
    wheel_steer_aids = {w: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{w}_steer")
                         for w in ("left_wheel", "right_wheel", "rear_wheel")}
    wheel_drive_aids = {w: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{w}_drive")
                         for w in ("left_wheel", "right_wheel", "rear_wheel")}
    base_drive = base_teleop.SwerveDrive()
    monitor_qposadr = {n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                        for n in MONITOR_JOINTS}
    monitor_ranges = {n: model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                       for n in MONITOR_JOINTS}
    rng = np.random.default_rng()

    site_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r")
    site_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l")
    # Reference orientation each hand's RPY sliders are relative to (see module docstring's
    # "RPY control" note): the home-pose site orientation, fixed for the whole session.
    home_quat_r = np.zeros(4)
    mujoco.mju_mat2Quat(home_quat_r, data.site_xmat[site_r])
    home_quat_l = np.zeros(4)
    mujoco.mju_mat2Quat(home_quat_l, data.site_xmat[site_l])

    targets = {
        "pos_r": data.site_xpos[site_r].tolist(), "rpy_r": [0.0, 0.0, 0.0],
        "pos_l": data.site_xpos[site_l].tolist(), "rpy_l": [0.0, 0.0, 0.0],
        "grasp_r": 0.0, "thumb_r": 0.0, "grasp_l": 0.0, "thumb_l": 0.0,
        "lift": float(data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")]]),
    }
    # IK actually chases these smoothed copies, not the raw slider values directly (see
    # module docstring's "IK target rate limiting" note) -- a slider yanked/typed to a
    # distant value in one frame otherwise sends solve_pose a target it can't reach within
    # max_joint_delta's per-iteration step size, burning the full IK_MAX_ITER budget on a
    # jump that will take several frames to close anyway (measured directly: ~11ms/frame for
    # a 0.3m single-frame position jump). Capping how far the effective target itself can
    # move per frame keeps every solve_pose call cheap AND produces smoother motion than a
    # teleport-style jump.
    smoothed_pos = {"r": np.array(targets["pos_r"]), "l": np.array(targets["pos_l"])}
    smoothed_rpy = {"r": np.array(targets["rpy_r"]), "l": np.array(targets["rpy_l"])}
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

        # --- continuous drive/lift keys (level-triggered, not edge-triggered like R/G/C
        # above -- driving should keep responding for as long as the key is held) ---
        # Arrow keys only (no WASD): Up/Down = forward/back, plain Left/Right = yaw,
        # Shift+Left/Right = strafe -- WASD was dropped since it collides with the
        # keybindings a MuJoCo user already expects from other tools; Shift-as-modifier
        # matches the existing mouse-camera convention just above (mod_shift).
        drive_keys = {"w": False, "a": False, "s": False, "d": False, "left": False, "right": False}
        lift_dir = 0.0
        if not io.want_capture_keyboard:
            shift_held = (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                          or glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
            drive_keys["w"] = glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS
            drive_keys["s"] = glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS
            if shift_held:
                drive_keys["a"] = glfw.get_key(window, glfw.KEY_LEFT) == glfw.PRESS
                drive_keys["d"] = glfw.get_key(window, glfw.KEY_RIGHT) == glfw.PRESS
            else:
                drive_keys["left"] = glfw.get_key(window, glfw.KEY_LEFT) == glfw.PRESS
                drive_keys["right"] = glfw.get_key(window, glfw.KEY_RIGHT) == glfw.PRESS
            if glfw.get_key(window, glfw.KEY_E) == glfw.PRESS:
                lift_dir += 1.0
            if glfw.get_key(window, glfw.KEY_Q) == glfw.PRESS:
                lift_dir -= 1.0
        if lift_dir != 0.0:
            targets["lift"] = float(np.clip(targets["lift"] + lift_dir * 0.3 * frame_dt,
                                             LIFT_RANGE[0], LIFT_RANGE[1]))

        # --- UI panel ---
        imgui.set_next_window_pos((10, 10), imgui.Cond_.first_use_ever)
        imgui.set_next_window_size((380, WINDOW_H - 20), imgui.Cond_.first_use_ever)
        if _begin_expanded("FFW-SH5 Teleop"):
            imgui.text(f"sim {data.time:6.1f}s  wall {time.perf_counter()-wall_start:6.1f}s  "
                       f"{freq_ema:4.1f} Hz")
            imgui.text(f"IK err  L: {ik_err_mm['l']:.2f}mm   R: {ik_err_mm['r']:.2f}mm")
            imgui.text(f"Base  x={data.qpos[base_x_qadr]:+.2f}m y={data.qpos[base_y_qadr]:+.2f}m "
                       f"yaw={math.degrees(data.qpos[base_yaw_qadr]):+.1f}deg  "
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

        # Rate-limit the raw slider targets into the smoothed copies IK actually chases (see
        # "IK target rate limiting" note near their declaration above).
        for side in ("r", "l"):
            raw_pos = np.array(targets[f"pos_{side}"])
            delta = np.clip(raw_pos - smoothed_pos[side], -MAX_POS_STEP_PER_FRAME, MAX_POS_STEP_PER_FRAME)
            smoothed_pos[side] = smoothed_pos[side] + delta
            raw_rpy = np.array(targets[f"rpy_{side}"])
            delta_rpy = np.clip(raw_rpy - smoothed_rpy[side], -MAX_RPY_STEP_PER_FRAME_DEG, MAX_RPY_STEP_PER_FRAME_DEG)
            smoothed_rpy[side] = smoothed_rpy[side] + delta_rpy

        # EE pose targets are base-local (see module docstring's "Mobile base" note): rotate
        # +translate by the base's CURRENT pose so driving/turning the base carries the
        # target along with it instead of the arm having to chase a world-fixed point.
        base_x, base_y, base_yaw = (data.qpos[base_x_qadr], data.qpos[base_y_qadr],
                                     data.qpos[base_yaw_qadr])
        base_quat = np.array([math.cos(base_yaw / 2), 0.0, 0.0, math.sin(base_yaw / 2)])
        cy, sy = math.cos(base_yaw), math.sin(base_yaw)

        def local_to_world_pos(p_local):
            x, y, z = p_local
            return np.array([base_x + cy * x - sy * y, base_y + sy * x + cy * y, z])

        # RPY sliders are a LOCAL rotation on top of the hand's own home orientation (see
        # module docstring), not raw world-frame Euler angles -- composing quat_mul(home,
        # delta) keeps roll/pitch/yaw = 0 at the natural grasp pose and each axis meaning
        # "rotate about the hand's own current X/Y/Z" near that pose, instead of the
        # scrambled axes an absolute-Euler encoding gives once home is far from identity.
        # base_quat is then applied on the left so the whole hand-relative-to-base target
        # turns together with the base, same reasoning as the position transform above.
        quat_r = np.zeros(4)
        mujoco.mju_mulQuat(quat_r, home_quat_r, rpy_deg_to_quat(smoothed_rpy["r"]))
        mujoco.mju_mulQuat(quat_r, base_quat, quat_r)
        quat_l = np.zeros(4)
        mujoco.mju_mulQuat(quat_l, home_quat_l, rpy_deg_to_quat(smoothed_rpy["l"]))
        mujoco.mju_mulQuat(quat_l, base_quat, quat_l)
        pos_r_world = local_to_world_pos(smoothed_pos["r"])
        pos_l_world = local_to_world_pos(smoothed_pos["l"])
        q_des_r, perr_r, _ = solver_r.solve_pose(q_des_r, pos_r_world, quat_r,
                                                  max_iter=IK_MAX_ITER, context_qpos=ctx_qpos)
        q_des_l, perr_l, _ = solver_l.solve_pose(q_des_l, pos_l_world, quat_l,
                                                  max_iter=IK_MAX_ITER, context_qpos=ctx_qpos)
        ik_err_mm["r"] = perr_r * 1000.0
        ik_err_mm["l"] = perr_l * 1000.0

        wheel_cmds = base_drive.update(drive_keys, frame_dt, base_yaw)

        for _ in range(steps_per_frame):
            ctrl_r.apply(data, q_des_r)
            ctrl_l.apply(data, q_des_l)
            data.ctrl[lift_aid] = targets["lift"]
            for wheel, (steer_angle, drive_angvel) in wheel_cmds.items():
                data.ctrl[wheel_steer_aids[wheel]] = steer_angle
                data.ctrl[wheel_drive_aids[wheel]] = drive_angvel
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
