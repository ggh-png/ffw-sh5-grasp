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

**코드 구조 (가독성 정리)**: 이 파일의 실행 로직은 전부 `TeleopApp` 클래스 하나에
모여 있다. `__init__`이 (1) 모델/솔버/제어기 (2) 창/렌더링 (3) 루프 타이밍 상태를
순서대로 구성하고, `run()`의 메인 루프는 매 프레임 아래 메서드들을 정확히 이 순서로
호출한다: 마우스 카메라 → 엣지 트리거 키(R/G/C) → 연속 입력 키(주행/리프트) → UI
패널 → 물리 스텝(IK+제어+mj_step) → 렌더링 → 프레임 타이밍. 각 메서드는 딱 그
단계만 담당하고, 상태는 전부 self.* 속성으로 공유한다 -- 원래 하나의 거대한 main()
함수 안에서 지역 변수로 얽혀 있던 것을 단계별로 나눈 것뿐, 동작은 전혀 바뀌지 않았다.
ImGui 슬라이더 패널 자체(위젯 배치)는 물리/렌더링과 무관하게 독립적으로 바뀔 일이
많아 `src/teleop_ui.py`로 따로 뺐다 -- 슬라이더를 추가/수정할 땐 그 파일만 보면 된다.
그 밖의(더 안 쪼갠) 부분들은 실제로 나눠봐도 서로 상태를 너무 많이 공유하거나
(물리 스텝 vs 렌더링), 주석이 바로 옆 코드에 있어야 의미가 있어서(튜닝 상수들)
파일을 나누는 게 오히려 왔다갔다하며 읽게 만들 것 같아 그대로 뒀다.
"""

import argparse
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
import bimanual_constraint  # noqa: E402
import grasp  # noqa: E402
import ik  # noqa: E402
import teleop_ui  # noqa: E402

# 양팔 관절 이름 목록 (IK solver / 토크 제어기에 그대로 넘겨진다).
ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
# Matches models/full_scene.xml's "home" keyframe, which as of Session 8 (Phase 5 follow-up)
# matches the sibling ffw-sh5-mujoco repo's rest pose (only the elbow/joint4 bent -90 deg,
# everything else 0) -- see NOTES.md "Phase 5 후속".
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
LIFT_RANGE = (-0.5, 0.0)
BOX_HALF_EXTENTS = np.array([0.15, 0.15, 0.20])
BOX_PREGRASP_GAP = 0.03
BOX_SQUEEZE_GAP = -0.005
BOX_GRAB_RAMP_TIME = 1.0
# 패널의 "Joint position monitor"에 진행률 막대로 표시할 관절 전체 목록.
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
# Session 8 후속 3 -- rate-limiting the target (above) fixed *literally unreachable* jumps
# (e.g. a slider yanked to a corner of its range) but not every real case: some in-range
# position+RPY combinations land solve_pose in a genuine local-minimum/joint-limit lockup
# where the per-frame error stops shrinking and plateaus tens of cm off target -- measured
# directly (pos+rpy target home+(0.2,0.15,-0.1)m / (30,20,15)deg plateaus at ~130mm error
# even at the full IK_MAX_ITER=15, forever, ~11ms/frame/hand the whole time it's held).
# That's the same cost profile as the fixed corner-case bug, just reached by an ordinary
# slider drag instead of an extreme one, so rate-limiting alone doesn't catch it. Fix: track
# each hand's per-frame solve error; once it's stayed above STUCK_POS_TOL for
# STUCK_FRAMES_THRESHOLD consecutive frames (a real lockup, not a normal multi-frame
# convergence -- legitimate reachable targets converge under 5mm within 1-2 frames given the
# warm start, per direct measurement), drop that hand's iteration budget to STUCK_MAX_ITER
# until it recovers. This changes nothing about *whether* a target converges (a target stuck
# at max_iter=15 was never going to converge at 15 either -- confirmed the same plateau value
# is reached either way) or the accuracy of any target that does converge -- convergent
# tracking always still gets the full IK_MAX_ITER budget every frame. It only stops paying
# full price to keep re-discovering the same non-improving result once that's already been
# established.
STUCK_POS_TOL = 0.03
STUCK_FRAMES_THRESHOLD = 5
STUCK_MAX_ITER = 4


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
    """rpy_deg_to_quat의 역변환 -- FK 모드에서 IK 모드로 되돌아갈 때, 현재 실제
    방향(쿼터니언)을 RPY 슬라이더 값(도)으로 되짚어 오기 위해 필요하다."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


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


class TeleopApp:
    """단일 네이티브 창 텔레옵 앱. `run()`의 메인 루프가 매 프레임 아래 단계를
    순서대로 실행한다: 마우스 카메라 -> 엣지 키(R/G/C) -> 연속 키(주행/리프트) ->
    UI 패널 -> 물리 스텝 -> 렌더링. 상태는 전부 인스턴스 속성(self.*)에 있다."""

    def __init__(self, initial_scenario="can"):
        self.initial_scenario = initial_scenario
        self._setup_sim()
        self._setup_render()
        self._setup_loop_state()

    # ------------------------------------------------------------------
    # 초기화
    # ------------------------------------------------------------------

    def _setup_sim(self):
        """모델 로드, 홈 자세 리셋, IK 솔버/토크 제어기, actuator/joint id 조회,
        슬라이더 목표값(targets) 초기화까지 -- 렌더링/윈도우와 무관한 부분 전부."""
        model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        data = mujoco.MjData(model)
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        mujoco.mj_forward(model, data)
        self.model = model
        self.data = data

        # 손별 IK 솔버 + 팔 토크 제어기 (양팔 각각 독립).
        self.solver_r = ik.InverseKinematics(model, "grasp_target_r", ARM_R)
        self.solver_l = ik.InverseKinematics(model, "grasp_target_l", ARM_L)
        self.ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
        self.ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
        # FK(관절각 직접 제어) 모드에서 패널 슬라이더의 최소/최대값으로 쓸, 각 팔
        # 관절의 range를 도(degree) 단위로 미리 계산해둔다.
        self.arm_joint_ranges_deg = {
            side: [tuple(math.degrees(v) for v in model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)])
                   for n in joints]
            for side, joints in (("r", ARM_R), ("l", ARM_L))
        }
        self.lift_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lift_joint")
        self.base_x_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_x")]
        self.base_y_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_y")]
        self.base_yaw_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_yaw")]
        # Real per-wheel steer (position) + drive (velocity) actuators -- see
        # src/base_teleop.py's SwerveDrive and the module docstring's "Mobile base" note. Base
        # motion is now a *consequence* of wheel-ground friction, not a direct actuator on
        # base_x/base_y/base_yaw (those joints still exist so base_link can't tip, but nothing
        # actuates them directly any more).
        self.wheel_steer_aids = {w: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{w}_steer")
                                  for w in ("left_wheel", "right_wheel", "rear_wheel")}
        self.wheel_drive_aids = {w: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{w}_drive")
                                  for w in ("left_wheel", "right_wheel", "rear_wheel")}
        # 모바일 베이스: SwerveDrive가 키 입력을 바퀴별 조향각+구동속도로 변환.
        self.base_drive = base_teleop.SwerveDrive()
        self.monitor_qposadr = {n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                                 for n in MONITOR_JOINTS}
        self.monitor_ranges = {n: model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                                for n in MONITOR_JOINTS}
        self.rng = np.random.default_rng()

        self.site_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r")
        self.site_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l")
        self.can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
        self.box_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_free")
        self.can_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "can_geom")
        self.box_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
        self.object_jids = {"can": self.can_jid, "box": self.box_jid}
        self.object_geom_ids = {"can": self.can_geom_id, "box": self.box_geom_id}
        self.object_home_qpos = {
            name: model.key_qpos[key_id][model.jnt_qposadr[jid]:model.jnt_qposadr[jid] + 7].copy()
            for name, jid in self.object_jids.items()
        }
        self.object_geom_rgba = {
            name: model.geom_rgba[gid].copy() for name, gid in self.object_geom_ids.items()
        }
        # Reference orientation each hand's RPY sliders are relative to (see module docstring's
        # "RPY control" note): the home-pose site orientation, fixed for the whole session.
        self.home_quat_r = np.zeros(4)
        mujoco.mju_mat2Quat(self.home_quat_r, data.site_xmat[self.site_r])
        self.home_quat_l = np.zeros(4)
        mujoco.mju_mat2Quat(self.home_quat_l, data.site_xmat[self.site_l])

        # 슬라이더가 직접 쓰는 "목표값" 딕셔너리 -- 물리 루프는 이 값만 읽고, 이 값을
        # 쓰는 건 GUI 슬라이더뿐이다(단방향 데이터 흐름, 모듈 docstring 참고).
        self.targets = {
            "pos_r": data.site_xpos[self.site_r].tolist(), "rpy_r": [0.0, 0.0, 0.0],
            "pos_l": data.site_xpos[self.site_l].tolist(), "rpy_l": [0.0, 0.0, 0.0],
            "grasp_r": 0.0, "thumb_r": 0.0, "grasp_l": 0.0, "thumb_l": 0.0,
            "squeeze_gap": BOX_PREGRASP_GAP,
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
        self.smoothed_pos = {"r": np.array(self.targets["pos_r"]), "l": np.array(self.targets["pos_l"])}
        self.smoothed_rpy = {"r": np.array(self.targets["rpy_r"]), "l": np.array(self.targets["rpy_l"])}
        # See STUCK_POS_TOL note above -- counts consecutive frames each hand's solve_pose has
        # failed to converge, so a genuine lockup can be throttled to STUCK_MAX_ITER.
        self.stuck_counter = {"r": 0, "l": 0}
        self.contact_viz = False
        self.camera_preset = 0
        self.scenario = None
        self.grab_state = {"r": None, "l": None}
        self.box_tracking = True
        self.box_grab = False
        self.gap_locked = False
        self.constraint_active = False
        self.rigid_relative_pose = None
        self.box_contact_forces = {"l": 0.0, "r": 0.0}
        self.box_held = False
        self.constraint_err_mm = 0.0
        self.box_squeeze_kp_scale = 0.2
        # teleop_ui.draw_panel reads these two off `app` directly (see that module's
        # docstring for why it doesn't import teleop_app's constants instead).
        self.lift_range = LIFT_RANGE
        self.switch_scenario(self.initial_scenario, force=True)

    def _setup_render(self):
        """창/렌더링 초기화: GLFW 창 하나 + 그 위에 ImGui, MuJoCo 저수준 렌더 API로
        3D 장면을 직접 그린다(모듈 docstring의 "Rendering note" 참고)."""
        if not glfw.init():
            raise RuntimeError("glfw.init() failed")
        window = glfw.create_window(WINDOW_W, WINDOW_H, "FFW-SH5 Teleop", None, None)
        if not window:
            glfw.terminate()
            raise RuntimeError("glfw.create_window() failed")
        glfw.make_context_current(window)
        glfw.swap_interval(0)
        self.window = window
        self.window_h = WINDOW_H

        imgui.create_context()
        self.impl = GlfwRenderer(window)

        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.cam)
        _set_camera_preset(self.cam, 0)
        self.opt = mujoco.MjvOption()
        mujoco.mjv_defaultOption(self.opt)
        self.pert = mujoco.MjvPerturb()
        self.context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

    def _setup_loop_state(self):
        """메인 루프에서만 쓰는 상태(IK 웜스타트 값, 타이밍, 입력 헬퍼)."""
        self.q_des_r = HOME_Q_R.copy()
        self.q_des_l = HOME_Q_L.copy()
        # 손별 제어 모드: "ik"(EE 포즈 슬라이더 -> solve_pose, 기존 동작) 또는
        # "fk"(관절각 슬라이더를 그대로 토크 제어기 목표로 사용, IK 자체를 건너뜀).
        # 리프트를 움직이는 동안 IK가 매 프레임에서만 어깨 높이를 다시 읽어들여서
        # 생기는 출렁임(리프트가 프레임 사이에도 계속 움직이는데 IK는 프레임당
        # 한 번만 풀림)을 피하고 싶을 때 FK로 전환해 관절각을 고정해두면, 팔 전체가
        # 리프트에 강체로 붙어 그대로 오르내리기만 해서 흔들림이 아예 없다.
        self.arm_mode = {"r": "ik", "l": "ik"}
        self.fk_q_deg = {"r": [math.degrees(v) for v in self.q_des_r],
                          "l": [math.degrees(v) for v in self.q_des_l]}
        self.frame_dt = 1.0 / LOOP_HZ
        self.steps_per_frame = max(1, round(self.frame_dt / self.model.opt.timestep))
        self.freq_ema = LOOP_HZ
        self.wall_start = time.perf_counter()
        self.last_mouse = list(glfw.get_cursor_pos(self.window))
        self.keys = KeyEdge()
        self.ik_err_mm = {"l": 0.0, "r": 0.0}

    # ------------------------------------------------------------------
    # R/G/C 세 가지 동작 -- 키보드(_handle_edge_keys)와 패널 버튼
    # (teleop_ui.draw_panel) 양쪽에서 똑같이 호출하는 공용 메서드.
    # ------------------------------------------------------------------

    def reset_can(self):
        _reset_can_random(self.model, self.data, self.rng)
        mujoco.mj_forward(self.model, self.data)

    def reset_box(self):
        self._reset_object("box")
        mujoco.mj_forward(self.model, self.data)

    def reset_active_object(self):
        if self.scenario == "box":
            self.reset_box()
        else:
            self.reset_can()

    def switch_scenario(self, scenario, force=False):
        if scenario not in ("can", "box"):
            raise ValueError(f"unknown scenario {scenario!r}")
        if scenario == self.scenario and not force:
            return

        active = scenario
        inactive = "box" if scenario == "can" else "can"
        self._reset_object(active)
        self._park_object(inactive)
        self._set_object_collision(active, True)
        self._set_object_collision(inactive, False)
        self.scenario = scenario
        self.grab_state = {"r": None, "l": None}
        self.box_grab = False
        self.gap_locked = False
        self.constraint_active = False
        self.rigid_relative_pose = None
        self.box_tracking = scenario == "box"
        self.targets["squeeze_gap"] = BOX_PREGRASP_GAP
        mujoco.mj_forward(self.model, self.data)

    def _reset_object(self, name):
        jid = self.object_jids[name]
        qadr = self.model.jnt_qposadr[jid]
        dof = self.model.jnt_dofadr[jid]
        self.data.qpos[qadr:qadr + 7] = self.object_home_qpos[name]
        self.data.qvel[dof:dof + 6] = 0.0

    def _park_object(self, name):
        jid = self.object_jids[name]
        qadr = self.model.jnt_qposadr[jid]
        dof = self.model.jnt_dofadr[jid]
        self.data.qpos[qadr:qadr + 3] = [2.0, 2.0, 0.1]
        self.data.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[dof:dof + 6] = 0.0

    def _set_object_collision(self, name, active):
        gid = self.object_geom_ids[name]
        self.model.geom_contype[gid] = 1 if active else 0
        self.model.geom_conaffinity[gid] = 1 if active else 0
        self.model.geom_rgba[gid][3] = self.object_geom_rgba[name][3] if active else 0.0

    def cycle_camera(self):
        self.camera_preset = 1 - self.camera_preset
        _set_camera_preset(self.cam, self.camera_preset)

    def set_arm_mode(self, side, mode):
        """이 손을 "ik"(EE 포즈 슬라이더) <-> "fk"(관절각 슬라이더)로 전환한다.
        전환 순간 팔이 튀지 않도록 방향에 따라 다르게 동기화한다:

        - ik -> fk: 지금 토크 제어기가 추종 중이던 관절각(q_des)을 그대로 FK
          슬라이더 값으로 복사 -- 전환 직후 첫 스텝의 목표 관절각이 전환 직전과
          정확히 같아서 포즈 점프가 없다.
        - fk -> ik: 반대로 EE 포즈 슬라이더 쪽이 전환 전 값(어쩌면 한참 전에 마지막
          으로 IK 모드였을 때의 낡은 목표)을 그대로 들고 있으므로, 그걸 쓰는 대신
          "지금 실제 site가 있는 월드 포즈"를 베이스-로컬 좌표/RPY로 역산해 targets/
          smoothed_pos/smoothed_rpy에 다시 채워 넣는다 -- 그래야 IK가 방금 있던
          자리에서부터 이어서 풀리지, 옛 목표로 갑자기 끌려가지 않는다.
        """
        if mode == self.arm_mode[side]:
            return
        q_des = self.q_des_r if side == "r" else self.q_des_l

        if mode == "fk":
            self.fk_q_deg[side] = [math.degrees(v) for v in q_des]
        else:
            site_id = self.site_r if side == "r" else self.site_l
            home_quat = self.home_quat_r if side == "r" else self.home_quat_l
            base_x = self.data.qpos[self.base_x_qadr]
            base_y = self.data.qpos[self.base_y_qadr]
            base_yaw = self.data.qpos[self.base_yaw_qadr]
            cy, sy = math.cos(base_yaw), math.sin(base_yaw)

            # 월드 위치 -> 베이스-로컬 위치 (local_to_world_pos의 역변환, _step_physics 참고).
            world_pos = self.data.site_xpos[site_id]
            dx, dy = world_pos[0] - base_x, world_pos[1] - base_y
            local_pos = [cy * dx + sy * dy, -sy * dx + cy * dy, float(world_pos[2])]

            # 월드 방향 -> "홈 포즈 기준 로컬 회전" RPY (quat_r = base_quat * home_quat * rpy_delta
            # 조립의 역산, _step_physics 참고).
            base_quat = np.array([math.cos(base_yaw / 2), 0.0, 0.0, math.sin(base_yaw / 2)])
            base_quat_inv, home_quat_inv = np.zeros(4), np.zeros(4)
            mujoco.mju_negQuat(base_quat_inv, base_quat)
            mujoco.mju_negQuat(home_quat_inv, home_quat)
            world_quat = np.zeros(4)
            mujoco.mju_mat2Quat(world_quat, self.data.site_xmat[site_id])
            tmp = np.zeros(4)
            mujoco.mju_mulQuat(tmp, base_quat_inv, world_quat)
            rpy_delta_quat = np.zeros(4)
            mujoco.mju_mulQuat(rpy_delta_quat, home_quat_inv, tmp)
            rpy_deg = quat_to_rpy_deg(rpy_delta_quat)

            self.targets[f"pos_{side}"] = local_pos
            self.targets[f"rpy_{side}"] = rpy_deg
            self.smoothed_pos[side] = np.array(local_pos)
            self.smoothed_rpy[side] = np.array(rpy_deg)
            self.stuck_counter[side] = 0

        self.arm_mode[side] = mode

    # ------------------------------------------------------------------
    # 메인 루프
    # ------------------------------------------------------------------

    def run(self):
        # 매 프레임 (1) 입력 처리 (2) IK 풀기 (3) 물리 스텝 (4) 렌더링을 전부 한
        # 스레드/한 루프 안에서 순서대로 실행한다.
        while not glfw.window_should_close(self.window):
            t0 = time.perf_counter()
            glfw.poll_events()
            self.impl.process_inputs()
            imgui.new_frame()
            io = imgui.get_io()

            self._handle_camera_mouse(io)
            self._handle_edge_keys(io)
            drive_keys = self._read_drive_and_lift_keys(io)
            self._draw_ui_panel()
            self._step_physics(drive_keys)
            self._render_scene()
            self._end_frame(t0)

        self.impl.shutdown()
        glfw.terminate()

    def _handle_camera_mouse(self, io):
        """마우스 카메라 조작 -- 슬라이더를 드래그하는 중이면(want_capture_mouse)
        카메라가 같이 돌지 않도록 건너뛴다."""
        cur_mouse = list(glfw.get_cursor_pos(self.window))
        dx, dy = cur_mouse[0] - self.last_mouse[0], cur_mouse[1] - self.last_mouse[1]
        self.last_mouse = cur_mouse
        if io.want_capture_mouse:
            return
        left = glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        right = glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        middle = glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
        if left or right or middle:
            _, win_h = glfw.get_window_size(self.window)
            mod_shift = (glfw.get_key(self.window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                         or glfw.get_key(self.window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
            if right:
                action = mujoco.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_MOVE_V
            elif left:
                action = mujoco.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_ROTATE_V
            else:
                action = mujoco.mjtMouse.mjMOUSE_ZOOM
            mujoco.mjv_moveCamera(self.model, action, dx / win_h, dy / win_h, self.scene, self.cam)
        if io.mouse_wheel != 0:
            mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0,
                                   -0.05 * io.mouse_wheel, self.scene, self.cam)

    def _handle_edge_keys(self, io):
        """눌렀다 뗄 때 한 번만 반응하는 키(R=캔 리셋, G=접촉 시각화 토글,
        C=카메라 전환) -- 계속 누르고 있어도 반복 발동하지 않는다."""
        if io.want_capture_keyboard:
            return
        if self.keys.pressed(self.window, glfw.KEY_R):
            self.reset_active_object()
        if self.keys.pressed(self.window, glfw.KEY_G):
            self.contact_viz = not self.contact_viz
        if self.keys.pressed(self.window, glfw.KEY_C):
            self.cycle_camera()

    def _read_drive_and_lift_keys(self, io):
        """continuous drive/lift keys (level-triggered, not edge-triggered like R/G/C
        above -- driving should keep responding for as long as the key is held).
        (한글) 주행/리프트 키는 누르고 있는 동안 계속 반응해야 하므로(edge-triggered
        아님) 매 프레임 눌림 상태를 그대로 읽는다.

        Arrow keys only (no WASD): Up/Down = forward/back, plain Left/Right = yaw,
        Shift+Left/Right = strafe -- WASD was dropped since it collides with the
        keybindings a MuJoCo user already expects from other tools; Shift-as-modifier
        matches the existing mouse-camera convention (mod_shift).
        """
        drive_keys = {"w": False, "a": False, "s": False, "d": False, "left": False, "right": False}
        lift_dir = 0.0
        if not io.want_capture_keyboard:
            shift_held = (glfw.get_key(self.window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                          or glfw.get_key(self.window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
            drive_keys["w"] = glfw.get_key(self.window, glfw.KEY_UP) == glfw.PRESS
            drive_keys["s"] = glfw.get_key(self.window, glfw.KEY_DOWN) == glfw.PRESS
            if shift_held:
                drive_keys["a"] = glfw.get_key(self.window, glfw.KEY_LEFT) == glfw.PRESS
                drive_keys["d"] = glfw.get_key(self.window, glfw.KEY_RIGHT) == glfw.PRESS
            else:
                drive_keys["left"] = glfw.get_key(self.window, glfw.KEY_LEFT) == glfw.PRESS
                drive_keys["right"] = glfw.get_key(self.window, glfw.KEY_RIGHT) == glfw.PRESS
            if glfw.get_key(self.window, glfw.KEY_E) == glfw.PRESS:
                lift_dir += 1.0
            if glfw.get_key(self.window, glfw.KEY_Q) == glfw.PRESS:
                lift_dir -= 1.0
        if lift_dir != 0.0:
            self.targets["lift"] = float(np.clip(
                self.targets["lift"] + lift_dir * 0.3 * self.frame_dt, LIFT_RANGE[0], LIFT_RANGE[1]))
        return drive_keys

    def _draw_ui_panel(self):
        """슬라이더 패널 -- HUD, 양손 EE 포즈, grasp/thumb, 리프트, 관절 모니터.
        실제 위젯 배치는 teleop_ui.draw_panel로 옮겼다(그 모듈 docstring 참고) --
        여기서 바뀌는 값은 전부 self.targets로만 들어가고, 물리 반영은
        _step_physics에서 이뤄진다."""
        teleop_ui.draw_panel(self)

    def _box_qpos_slice(self):
        qadr = self.model.jnt_qposadr[self.box_jid]
        return self.data.qpos[qadr:qadr + 7]

    def _update_box_tracking_targets(self, world_to_base_pos):
        box_qpos = self._box_qpos_slice()
        box_pos = box_qpos[:3]
        box_quat = box_qpos[3:7]
        box_mat = np.zeros(9)
        mujoco.mju_quat2Mat(box_mat, box_quat)
        box_R = box_mat.reshape(3, 3)
        gap = self.targets["squeeze_gap"]
        offset = BOX_HALF_EXTENTS[1] + gap
        pos_r = box_pos + box_R @ np.array([0.0, -offset, 0.0])
        pos_l = box_pos + box_R @ np.array([0.0, offset, 0.0])
        self.targets["pos_r"] = world_to_base_pos(pos_r).tolist()
        self.targets["pos_l"] = world_to_base_pos(pos_l).tolist()

    def _update_box_grasp_state(self):
        self.box_contact_forces = grasp.get_box_hand_contacts(self.model, self.data)
        self.box_held = (self.box_contact_forces["l"] >= 1.0
                         and self.box_contact_forces["r"] >= 1.0)
        if self.scenario != "box":
            self.constraint_err_mm = 0.0
            return
        if not self.box_grab:
            self.gap_locked = False
            self.constraint_active = False
            self.rigid_relative_pose = None
            self.constraint_err_mm = 0.0
            return
        if self.box_held and not self.constraint_active:
            self.rigid_relative_pose = bimanual_constraint.snapshot_relative_pose(
                self.data, self.site_r, self.site_l)
            self.constraint_active = True
            self.gap_locked = True
            self.box_tracking = False
        if self.constraint_active and self.rigid_relative_pose is not None:
            pos_err, _ = bimanual_constraint.relative_pose_error(
                self.data, self.site_r, self.site_l, self.rigid_relative_pose)
            self.constraint_err_mm = float(np.linalg.norm(pos_err) * 1000.0)

    def _step_physics(self, drive_keys):
        """(한글) 여기서부터 실제 물리 반영: 슬라이더 목표값을 rate-limit -> 베이스
        로컬->월드 변환 -> IK로 관절각 계산 -> 토크 제어기/grasp/바퀴 ctrl 기록 ->
        mj_step. context_qpos는 IK가 담당하지 않는 다른 관절(리프트 등)을 지금
        상태로 시드하기 위함(ik.py 참고). 여기 말고는 qpos에 손대는 곳이 없다
        (캔 리셋의 명시적 예외 제외)."""
        model, data = self.model, self.data
        ctx_qpos = data.qpos.copy()

        base_x, base_y, base_yaw = (data.qpos[self.base_x_qadr], data.qpos[self.base_y_qadr],
                                     data.qpos[self.base_yaw_qadr])
        base_quat = np.array([math.cos(base_yaw / 2), 0.0, 0.0, math.sin(base_yaw / 2)])
        cy, sy = math.cos(base_yaw), math.sin(base_yaw)

        def local_to_world_pos(p_local):
            x, y, z = p_local
            return np.array([base_x + cy * x - sy * y, base_y + sy * x + cy * y, z])

        def world_to_base_pos(p_world):
            dx, dy = p_world[0] - base_x, p_world[1] - base_y
            return np.array([cy * dx + sy * dy, -sy * dx + cy * dy, p_world[2]])

        self._update_box_grasp_state()
        if self.scenario == "box":
            if not self.gap_locked:
                desired_gap = BOX_SQUEEZE_GAP if self.box_grab else BOX_PREGRASP_GAP
                max_gap_step = ((BOX_PREGRASP_GAP - BOX_SQUEEZE_GAP)
                                * self.frame_dt / BOX_GRAB_RAMP_TIME)
                gap_delta = np.clip(desired_gap - self.targets["squeeze_gap"],
                                    -max_gap_step, max_gap_step)
                self.targets["squeeze_gap"] += float(gap_delta)
            if self.box_tracking and not self.constraint_active:
                self._update_box_tracking_targets(world_to_base_pos)
        else:
            for side in ("r", "l"):
                if self.grab_state[side] is None:
                    continue
                desired = 1.0 if self.grab_state[side] else 0.0
                step = self.frame_dt / BOX_GRAB_RAMP_TIME
                for name in (f"grasp_{side}", f"thumb_{side}"):
                    delta = np.clip(desired - self.targets[name], -step, step)
                    self.targets[name] += float(delta)

        # Rate-limit the raw slider targets into the smoothed copies IK actually chases (see
        # "IK target rate limiting" note near their declaration in _setup_sim).
        for side in ("r", "l"):
            raw_pos = np.array(self.targets[f"pos_{side}"])
            delta = np.clip(raw_pos - self.smoothed_pos[side], -MAX_POS_STEP_PER_FRAME, MAX_POS_STEP_PER_FRAME)
            self.smoothed_pos[side] = self.smoothed_pos[side] + delta
            raw_rpy = np.array(self.targets[f"rpy_{side}"])
            delta_rpy = np.clip(raw_rpy - self.smoothed_rpy[side],
                                 -MAX_RPY_STEP_PER_FRAME_DEG, MAX_RPY_STEP_PER_FRAME_DEG)
            self.smoothed_rpy[side] = self.smoothed_rpy[side] + delta_rpy

        # EE pose targets are base-local (see module docstring's "Mobile base" note): rotate
        # +translate by the base's CURRENT pose so driving/turning the base carries the
        # target along with it instead of the arm having to chase a world-fixed point.
        # RPY sliders are a LOCAL rotation on top of the hand's own home orientation (see
        # module docstring), not raw world-frame Euler angles -- composing quat_mul(home,
        # delta) keeps roll/pitch/yaw = 0 at the natural grasp pose and each axis meaning
        # "rotate about the hand's own current X/Y/Z" near that pose, instead of the
        # scrambled axes an absolute-Euler encoding gives once home is far from identity.
        # base_quat is then applied on the left so the whole hand-relative-to-base target
        # turns together with the base, same reasoning as the position transform above.
        #
        # (한글) 이 IK 계산은 arm_mode가 "ik"인 손에만 수행한다 -- "fk"인 손은 관절각
        # 슬라이더 값을 그대로 목표로 쓰고 IK를 아예 건너뛴다. 관절각으로 직접 고정해
        # 두면 팔이 리프트(어깨 높이)와 강체로 함께 움직일 뿐이라, 리프트가 프레임 중간
        # 에도 계속 움직이는데 IK는 프레임당 한 번만 그 순간의 어깨 높이를 반영해서
        # 생기는 출렁임 자체가 원천적으로 없다.
        q_des_r_prev = self.q_des_r.copy()
        q_des_l_prev = self.q_des_l.copy()
        if self.arm_mode["r"] == "ik":
            quat_r = np.zeros(4)
            mujoco.mju_mulQuat(quat_r, self.home_quat_r, rpy_deg_to_quat(self.smoothed_rpy["r"]))
            mujoco.mju_mulQuat(quat_r, base_quat, quat_r)
            pos_r_world = local_to_world_pos(self.smoothed_pos["r"])
            # See STUCK_POS_TOL note above -- a hand that's been stuck (unconverged) for
            # several frames in a row gets thrown into a cheap iteration budget instead of
            # repeatedly paying full price to re-discover the same non-improving result.
            iter_r = STUCK_MAX_ITER if self.stuck_counter["r"] >= STUCK_FRAMES_THRESHOLD else IK_MAX_ITER
            self.q_des_r, perr_r, _ = self.solver_r.solve_pose(self.q_des_r, pos_r_world, quat_r,
                                                                max_iter=iter_r, context_qpos=ctx_qpos)
            self.stuck_counter["r"] = self.stuck_counter["r"] + 1 if perr_r > STUCK_POS_TOL else 0
            self.ik_err_mm["r"] = perr_r * 1000.0
        else:
            self.q_des_r = np.radians(self.fk_q_deg["r"])
            self.stuck_counter["r"] = 0

        if self.arm_mode["l"] == "ik":
            quat_l = np.zeros(4)
            mujoco.mju_mulQuat(quat_l, self.home_quat_l, rpy_deg_to_quat(self.smoothed_rpy["l"]))
            mujoco.mju_mulQuat(quat_l, base_quat, quat_l)
            pos_l_world = local_to_world_pos(self.smoothed_pos["l"])
            iter_l = STUCK_MAX_ITER if self.stuck_counter["l"] >= STUCK_FRAMES_THRESHOLD else IK_MAX_ITER
            self.q_des_l, perr_l, _ = self.solver_l.solve_pose(self.q_des_l, pos_l_world, quat_l,
                                                                max_iter=iter_l, context_qpos=ctx_qpos)
            self.stuck_counter["l"] = self.stuck_counter["l"] + 1 if perr_l > STUCK_POS_TOL else 0
            self.ik_err_mm["l"] = perr_l * 1000.0
        else:
            self.q_des_l = np.radians(self.fk_q_deg["l"])
            self.stuck_counter["l"] = 0

        if (self.constraint_active and self.rigid_relative_pose is not None
                and self.arm_mode["r"] == "ik" and self.arm_mode["l"] == "ik"):
            dq_r = self.q_des_r - q_des_r_prev
            dq_l = self.q_des_l - q_des_l_prev
            dq_r, dq_l = bimanual_constraint.project_desired_delta(
                model, data, self.site_r, self.site_l, self.ctrl_r.dof_ids, self.ctrl_l.dof_ids,
                dq_r, dq_l, self.frame_dt, reference=self.rigid_relative_pose)
            self.q_des_r = q_des_r_prev + dq_r
            self.q_des_l = q_des_l_prev + dq_l

        wheel_cmds = self.base_drive.update(drive_keys, self.frame_dt, base_yaw)
        arm_kp_scale = self.box_squeeze_kp_scale if self.scenario == "box" and self.box_grab else 1.0

        # 렌더 프레임 하나(frame_dt)당 물리 스텝을 여러 번(steps_per_frame) 돌린다 --
        # 렌더링은 25Hz면 충분하지만 물리는 훨씬 촘촘한 timestep이 필요하기 때문.
        # IK로 구한 목표 관절각(q_des_r/l)과 grasp/thumb/lift/바퀴 ctrl을 매 서브스텝
        # 다시 적용한 뒤 mj_step 한 번.
        for _ in range(self.steps_per_frame):
            self.ctrl_r.apply(data, self.q_des_r, kp_scale=arm_kp_scale)
            self.ctrl_l.apply(data, self.q_des_l, kp_scale=arm_kp_scale)
            data.ctrl[self.lift_aid] = self.targets["lift"]
            for wheel, (steer_angle, drive_angvel) in wheel_cmds.items():
                data.ctrl[self.wheel_steer_aids[wheel]] = steer_angle
                data.ctrl[self.wheel_drive_aids[wheel]] = drive_angvel
            grasp.apply_grasp(model, data, grasp=self.targets["grasp_r"], thumb=self.targets["thumb_r"], side="r")
            grasp.apply_grasp(model, data, grasp=self.targets["grasp_l"], thumb=self.targets["thumb_l"], side="l")
            mujoco.mj_step(model, data)
        self._update_box_grasp_state()

    def _render_scene(self):
        """(한글) 갱신된 물리 상태(data)를 MuJoCo 저수준 API로 화면에 그리고,
        그 위에 ImGui 패널을 합성한다."""
        self.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = self.contact_viz
        self.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = self.contact_viz
        fb_w, fb_h = glfw.get_framebuffer_size(self.window)
        viewport = mujoco.MjrRect(0, 0, fb_w, fb_h)
        mujoco.mjv_updateScene(self.model, self.data, self.opt, self.pert, self.cam,
                                mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(viewport, self.scene, self.context)

        imgui.render()
        self.impl.render(imgui.get_draw_data())
        glfw.swap_buffers(self.window)

    def _end_frame(self, t0):
        """(한글) 목표 루프 주기(LOOP_HZ)를 맞추기 위해 남는 시간만큼 잠깐 대기하고,
        HUD에 표시할 실측 주파수(freq_ema)를 지수이동평균으로 갱신."""
        elapsed = time.perf_counter() - t0
        self.freq_ema = 0.9 * self.freq_ema + 0.1 * (1.0 / max(elapsed, 1e-6))
        sleep_time = self.frame_dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


def _parse_args(argv):
    parser = argparse.ArgumentParser(description="FFW-SH5 teleop app")
    parser.add_argument("scenario_pos", nargs="?", choices=("can", "box"),
                        help="initial scenario: can or box")
    parser.add_argument("--scenario", choices=("can", "box"),
                        help="initial scenario: can or box")
    args = parser.parse_args(argv)
    return args.scenario if args.scenario is not None else (args.scenario_pos or "can")


def main(argv=None):
    scenario = _parse_args(sys.argv[1:] if argv is None else argv)
    TeleopApp(initial_scenario=scenario).run()


if __name__ == "__main__":
    main()
