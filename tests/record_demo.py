"""Dev utility (not a Phase test): render the Phase 4 scripted pick-and-lift sequence on
models/full_scene.xml to an animated GIF, contact force/point visualization on, camera
following the right hand. This is the offline stand-in for the "촬영" deliverable in
PLAN.md's Phase 4 -- the interactive src/teleop_app.py is for a human at the sliders, but
this script drives the identical validated sequence from tests/test_phase_4.py
headlessly so the demo doesn't depend on someone being at a keyboard/mouse.

Usage: python3 tests/record_demo.py [out.gif]
"""

import pathlib
import sys

import mujoco
import numpy as np
from PIL import Image

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "full_scene.xml"

import arm_control  # noqa: E402
import grasp  # noqa: E402
import ik  # noqa: E402

ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
# Matches models/full_scene.xml's "home" keyframe (Session 8 Phase 5 follow-up) -- see
# NOTES.md "Phase 5 후속".
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])

PRE_GRASP_OFFSET = np.array([0.0, 0.0, 0.10])
RAMP_TIME = 1.0
SETTLE_TIME = 1.0
LIFT_HEIGHT = 0.10
LIFT_SPEED = 0.02
POST_LIFT_HOLD = 2.0
APPROACH_SPEED = 0.03

FRAME_EVERY_S = 0.15
GIF_FRAME_MS = 90  # ~11fps playback


def _read_arm_q(model, data, joint_names):
    return np.array([data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]]
                      for n in joint_names])


class FrameGrabber:
    def __init__(self, model):
        self.renderer = mujoco.Renderer(model, height=360, width=480)
        self.cam = mujoco.MjvCamera()
        self.cam.lookat[:] = [0.5055, 0.0, 0.85]
        self.cam.distance = 0.55
        self.cam.azimuth = 100
        self.cam.elevation = -18
        self.opt = mujoco.MjvOption()
        self.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        self.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        self.frames = []
        self._next_t = 0.0

    def maybe_capture(self, data):
        if data.time < self._next_t:
            return
        self._next_t += FRAME_EVERY_S
        self.renderer.update_scene(data, camera=self.cam, scene_option=self.opt)
        self.frames.append(Image.fromarray(self.renderer.render()))

    def save(self, path):
        # Adaptive palette quantization -- this content (flat robot-arm shading, checker
        # floor, few colors) loses essentially nothing visually but shrinks the GIF several
        # times over vs. PIL's default per-frame full-color GIF encoding.
        quantized = [f.quantize(colors=128, method=Image.MEDIANCUT) for f in self.frames]
        quantized[0].save(path, save_all=True, append_images=quantized[1:],
                           duration=GIF_FRAME_MS, loop=0, optimize=False)
        print(f"wrote {len(self.frames)} frames to {path}")


def _move(model, data, ctrl_r, ctrl_l, grabber, q_from, q_to, duration, dt, grasp_frac=None, thumb_frac=None):
    n = int(duration / dt)
    for i in range(n):
        frac = i / n
        ctrl_r.apply(data, q_from + frac * (q_to - q_from))
        ctrl_l.apply(data, HOME_Q_L)
        if grasp_frac is not None:
            grasp.apply_grasp(model, data, grasp=grasp_frac, thumb=thumb_frac, side="r")
        mujoco.mj_step(model, data)
        grabber.maybe_capture(data)


def _hold(model, data, ctrl_r, ctrl_l, grabber, q_des, duration, dt, grasp_frac=None, thumb_frac=None):
    n = int(duration / dt)
    for _ in range(n):
        ctrl_r.apply(data, q_des)
        ctrl_l.apply(data, HOME_Q_L)
        if grasp_frac is not None:
            grasp.apply_grasp(model, data, grasp=grasp_frac, thumb=thumb_frac, side="r")
        mujoco.mj_step(model, data)
        grabber.maybe_capture(data)


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else str(REPO_ROOT / "docs" / "assets" / "demo.gif")
    pathlib.Path(out_path).parent.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    solver = ik.InverseKinematics(model, "grasp_target_r", ARM_R)
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
    grabber = FrameGrabber(model)
    rng = np.random.default_rng(0)
    dt = model.opt.timestep

    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    can_qadr = model.jnt_qposadr[can_jid]
    can_pos0 = data.qpos[can_qadr:can_qadr + 3].copy()
    target_quat = np.array([0.5, 0.5, 0.5, 0.5])

    ctx = data.qpos.copy()
    pregrasp_pos = can_pos0 + PRE_GRASP_OFFSET
    q_pregrasp, _, _, ok1 = solver.solve_pose_multistart(HOME_Q_R, pregrasp_pos, target_quat, rng, context_qpos=ctx)
    q_grasp, _, _, ok2 = solver.solve_pose_multistart(q_pregrasp, can_pos0, target_quat, rng, context_qpos=ctx)
    assert ok1 and ok2, "IK failed to set up demo -- check models/full_scene.xml"

    q_home = _read_arm_q(model, data, ARM_R)
    grabber.maybe_capture(data)

    print("move -> pre-grasp")
    _move(model, data, ctrl_r, ctrl_l, grabber, q_home, q_pregrasp, 3.0, dt, grasp_frac=0.0, thumb_frac=0.0)
    _hold(model, data, ctrl_r, ctrl_l, grabber, q_pregrasp, 0.5, dt, grasp_frac=0.0, thumb_frac=0.0)

    print("approach")
    approach_time = np.linalg.norm(PRE_GRASP_OFFSET) / APPROACH_SPEED
    _move(model, data, ctrl_r, ctrl_l, grabber, q_pregrasp, q_grasp, approach_time, dt, grasp_frac=0.0, thumb_frac=0.0)
    _hold(model, data, ctrl_r, ctrl_l, grabber, q_grasp, 0.5, dt, grasp_frac=0.0, thumb_frac=0.0)

    print("grasp")
    n = int(RAMP_TIME / dt)
    for i in range(n):
        frac = i / n
        ctrl_r.apply(data, q_grasp)
        ctrl_l.apply(data, HOME_Q_L)
        grasp.apply_grasp(model, data, grasp=frac, thumb=frac, side="r")
        mujoco.mj_step(model, data)
        grabber.maybe_capture(data)
    _hold(model, data, ctrl_r, ctrl_l, grabber, q_grasp, SETTLE_TIME, dt, grasp_frac=1.0, thumb_frac=1.0)

    print("lift")
    lift_target_pos = can_pos0 + np.array([0, 0, LIFT_HEIGHT])
    q_lift, _, _, _ = solver.solve_pose_multistart(q_grasp, lift_target_pos, target_quat, rng, context_qpos=ctx)
    lift_time = LIFT_HEIGHT / LIFT_SPEED
    _move(model, data, ctrl_r, ctrl_l, grabber, q_grasp, q_lift, lift_time, dt, grasp_frac=1.0, thumb_frac=1.0)
    _hold(model, data, ctrl_r, ctrl_l, grabber, q_lift, POST_LIFT_HOLD, dt, grasp_frac=1.0, thumb_frac=1.0)

    net_lift = data.qpos[can_qadr + 2] - can_pos0[2]
    print(f"net_lift={net_lift*100:.2f}cm grasped={grasp.is_grasped(model, data, side='r')}")
    grabber.save(out_path)


if __name__ == "__main__":
    main()
