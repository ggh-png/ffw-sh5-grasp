"""Phase 4 -- full_scene (whole robot, fixed base) regression.

models/full_scene.xml embeds the exact Phase 1-3 right-arm+hand physics (capsule
collision, grasp synergy, IK site offset, HOME_Q) inside the full FFW-SH5 body: fixed
base (freejoint removed), visual-only wheels (joints removed), lift_joint + head kept as
real actuators, both arms as motor+feedforward torque control (src/arm_control.py, same
fix as Phase 3), both hands with mirrored capsule collision. The arm_base translation
(mobile-base+lift height vs. arm_hand.xml's fixed 1.0m) was folded
into the table/can placement so every Phase 1-3 validated number (HOME_Q, grasp_target site
offset, thumb pre-shape, capsule sizes) carries over unchanged for the right arm -- this is
verified directly by part 1 below rather than assumed.

Part 1 (hold regression): apply the "home" keyframe (both arms via arm_control's
feedforward+PD torque law, lift/head/fingers via their own position actuators) and hold for
5s. Asserts no divergence (max|qacc| bound, matching Phase 0's check) and that both arms'
grasp_target site drifts less than 2mm from where the keyframe placed it -- this is the
direct regression check for the arm_base translation/HOME_Q reuse reasoning above.

Part 2 (integrated pick, script-driven, no teleop): identical sequence to
tests/test_phase_3.py (home -> pre-grasp -> approach -> grasp -> lift) run on the right
hand/can here, with the left arm, lift and head all held at their keyframe pose throughout
via the same per-step torque/position control (proving the rest of the body doesn't
interfere with the validated right-hand pipeline). Success rate must stay >= 7/10, matching
Phase 3's bar.

The left hand's grasp synergy is mirrored geometry (see src/grasp.py), not independently
regression-tested against its own can -- Part 1's hold check covers it (no divergence, small
site drift) but Part 2 only exercises the right hand, honestly matching what's actually been
validated.

Run headless: `python3 tests/test_phase_4.py`
"""

import pathlib
import sys

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

# Session 8 (Phase 5 follow-up): matches the rest pose used by the sibling ffw-sh5-mujoco
# repo's Controller.reset() (only joint4/elbow set to -90 deg, everything else 0)
# rather than the old arm_hand.xml-derived
# "already reaching for the can" pose. This is now just a generic ready/rest seed for IK
# multistart, not tied to any particular can geometry -- solve_pose_multistart's random
# restarts (not this seed) do the real work of finding the pregrasp/grasp configuration.
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
LIFT_HOME = -0.39
THUMB_PRESHAPE_R = {"finger_r_joint1": 0.131, "finger_r_joint2": -1.309}
THUMB_PRESHAPE_L = {"finger_l_joint1": 0.131, "finger_l_joint2": 1.309}

HOLD_DURATION = 5.0
HOLD_QACC_LIMIT = 1e5
HOLD_SITE_DRIFT_LIMIT = 0.002  # 2mm

N_IK_SAMPLES = 100
IK_TEST_SPREAD = 0.2
POS_TOL = 0.005
ORI_TOL_DEG = 5.0
IK_SUCCESS_RATE_TARGET = 0.95

N_PICK_TRIALS = 10
PICK_SUCCESS_RATE_TARGET = 0.7
APPROACH_SPEED = 0.03
PRE_GRASP_OFFSET = np.array([0.0, 0.0, 0.10])
GRASP_TARGET_OFFSET = np.array([0.0, 0.0, 0.0])
RAMP_TIME = 1.0
SETTLE_TIME = 1.0
LIFT_HEIGHT = 0.10
LIFT_SPEED = 0.02
POST_LIFT_HOLD = 3.0
MIN_NET_LIFT = 0.08
CAN_NOISE = 0.005


def _reset_home(model, data):
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)


def _hold_whole_body(model, data, ctrl_r, ctrl_l, q_r, q_l, grasp_r=0.0, thumb_r=0.0):
    """Every step: right arm torque-controlled to q_r, left arm to q_l (both via
    arm_control's feedforward+PD), lift/head/fingers via their own position actuators
    (ctrl already set once is enough for those, but left/right hand grasp needs
    grasp.apply_grasp reapplied since it's the thing under test)."""
    ctrl_r.apply(data, q_r)
    ctrl_l.apply(data, q_l)
    grasp.apply_grasp(model, data, grasp=grasp_r, thumb=thumb_r, side="r")


def run_hold_test(model):
    data = mujoco.MjData(model)
    _reset_home(model, data)
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)

    site_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r")
    site_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l")
    p0_r = data.site_xpos[site_r].copy()
    p0_l = data.site_xpos[site_l].copy()

    dt = model.opt.timestep
    n = int(HOLD_DURATION / dt)
    max_qacc = 0.0
    for _ in range(n):
        _hold_whole_body(model, data, ctrl_r, ctrl_l, HOME_Q_R, HOME_Q_L,
                          grasp_r=0.0, thumb_r=0.0)
        mujoco.mj_step(model, data)
        max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))

    drift_r = float(np.linalg.norm(data.site_xpos[site_r] - p0_r))
    drift_l = float(np.linalg.norm(data.site_xpos[site_l] - p0_l))
    print(f"Hold test: max|qacc|={max_qacc:.3f} (limit {HOLD_QACC_LIMIT:.0e}), "
          f"site_r drift={drift_r*1000:.3f}mm site_l drift={drift_l*1000:.3f}mm "
          f"(limit {HOLD_SITE_DRIFT_LIMIT*1000:.0f}mm)")
    ok = (max_qacc < HOLD_QACC_LIMIT and drift_r < HOLD_SITE_DRIFT_LIMIT
          and drift_l < HOLD_SITE_DRIFT_LIMIT)
    return ok


def run_ik_unit_test(model, solver, rng):
    joint_ranges = np.array([model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                              for n in ARM_R])
    scratch = mujoco.MjData(model)
    _reset_home(model, scratch)  # seeds lift_joint and other upstream context joints

    successes = 0
    pos_errs, ori_errs = [], []
    for _ in range(N_IK_SAMPLES):
        q_target = np.clip(HOME_Q_R + rng.uniform(-IK_TEST_SPREAD, IK_TEST_SPREAD, size=7),
                            joint_ranges[:, 0], joint_ranges[:, 1])
        for qadr, val in zip(solver.qpos_adrs, q_target):
            scratch.qpos[qadr] = val
        mujoco.mj_forward(model, scratch)
        target_pos = scratch.site_xpos[solver.site_id].copy()
        target_quat = np.zeros(4)
        mujoco.mju_mat2Quat(target_quat, scratch.site_xmat[solver.site_id])

        _, pos_err, ori_err, converged = solver.solve_pose_multistart(
            HOME_Q_R, target_pos, target_quat, rng,
            success_pos_tol=POS_TOL, success_ori_tol=np.radians(ORI_TOL_DEG),
            context_qpos=scratch.qpos)
        pos_errs.append(pos_err)
        ori_errs.append(np.degrees(ori_err))
        if converged:
            successes += 1

    rate = successes / N_IK_SAMPLES
    print(f"IK unit test: {successes}/{N_IK_SAMPLES} converged ({rate*100:.0f}%), "
          f"target >= {IK_SUCCESS_RATE_TARGET*100:.0f}%")
    print(f"  pos_err: median={np.median(pos_errs)*1000:.3f}mm max={np.max(pos_errs)*1000:.3f}mm")
    print(f"  ori_err: median={np.median(ori_errs):.3f}deg max={np.max(ori_errs):.3f}deg")
    return rate >= IK_SUCCESS_RATE_TARGET


def _read_arm_q(model, data, joint_names):
    return np.array([data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]]
                      for n in joint_names])


def _hold(model, data, ctrl_r, ctrl_l, q_r_des, duration, dt, grasp_frac=None, thumb_frac=None):
    n = int(duration / dt)
    for _ in range(n):
        ctrl_r.apply(data, q_r_des)
        ctrl_l.apply(data, HOME_Q_L)
        if grasp_frac is not None:
            grasp.apply_grasp(model, data, grasp=grasp_frac, thumb=thumb_frac, side="r")
        mujoco.mj_step(model, data)


def _move(model, data, ctrl_r, ctrl_l, q_from, q_to, duration, dt, grasp_frac=None, thumb_frac=None):
    n = int(duration / dt)
    for i in range(n):
        frac = i / n
        ctrl_r.apply(data, q_from + frac * (q_to - q_from))
        ctrl_l.apply(data, HOME_Q_L)
        if grasp_frac is not None:
            grasp.apply_grasp(model, data, grasp=grasp_frac, thumb=thumb_frac, side="r")
        mujoco.mj_step(model, data)


def run_pick_trial(model, data, solver, ctrl_r, ctrl_l, rng):
    _reset_home(model, data)
    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    can_qadr = model.jnt_qposadr[can_jid]
    can_pos0 = data.qpos[can_qadr:can_qadr + 3].copy()
    can_pos0[:3] += rng.uniform(-CAN_NOISE, CAN_NOISE, size=3)
    data.qpos[can_qadr:can_qadr + 3] = can_pos0
    mujoco.mj_forward(model, data)

    target_quat = np.array([0.5, 0.5, 0.5, 0.5])
    dt = model.opt.timestep

    grasp_target_pos = can_pos0 + GRASP_TARGET_OFFSET
    pregrasp_pos = grasp_target_pos + PRE_GRASP_OFFSET
    ctx = data.qpos.copy()  # lift_joint etc. -- see ik.py's context_qpos note
    q_pregrasp, perr, oerr, ok1 = solver.solve_pose_multistart(HOME_Q_R, pregrasp_pos, target_quat, rng, context_qpos=ctx)
    q_grasp, perr2, oerr2, ok2 = solver.solve_pose_multistart(q_pregrasp, grasp_target_pos, target_quat, rng, context_qpos=ctx)
    if not (ok1 and ok2):
        return {"success": False, "reason": "ik_failed", "net_lift": 0.0}

    q_home = _read_arm_q(model, data, ARM_R)

    _move(model, data, ctrl_r, ctrl_l, q_home, q_pregrasp, 3.0, dt, grasp_frac=0.0, thumb_frac=0.0)
    _hold(model, data, ctrl_r, ctrl_l, q_pregrasp, 1.0, dt, grasp_frac=0.0, thumb_frac=0.0)

    approach_dist = np.linalg.norm(PRE_GRASP_OFFSET)
    approach_time = approach_dist / APPROACH_SPEED
    _move(model, data, ctrl_r, ctrl_l, q_pregrasp, q_grasp, approach_time, dt, grasp_frac=0.0, thumb_frac=0.0)
    _hold(model, data, ctrl_r, ctrl_l, q_grasp, 1.0, dt, grasp_frac=0.0, thumb_frac=0.0)

    n = int(RAMP_TIME / dt)
    for i in range(n):
        frac = i / n
        ctrl_r.apply(data, q_grasp)
        ctrl_l.apply(data, HOME_Q_L)
        grasp.apply_grasp(model, data, grasp=frac, thumb=frac, side="r")
        mujoco.mj_step(model, data)
    _hold(model, data, ctrl_r, ctrl_l, q_grasp, SETTLE_TIME, dt, grasp_frac=1.0, thumb_frac=1.0)

    grasped = grasp.is_grasped(model, data, side="r")
    can_z_before_lift = data.qpos[can_qadr + 2]

    lift_target_pos = grasp_target_pos + np.array([0, 0, LIFT_HEIGHT])
    q_lift, _, _, _ = solver.solve_pose_multistart(q_grasp, lift_target_pos, target_quat, rng, context_qpos=ctx)
    lift_time = LIFT_HEIGHT / LIFT_SPEED
    _move(model, data, ctrl_r, ctrl_l, q_grasp, q_lift, lift_time, dt, grasp_frac=1.0, thumb_frac=1.0)
    _hold(model, data, ctrl_r, ctrl_l, q_lift, POST_LIFT_HOLD, dt, grasp_frac=1.0, thumb_frac=1.0)

    net_lift = data.qpos[can_qadr + 2] - can_z_before_lift
    return {
        "success": grasped and net_lift >= MIN_NET_LIFT,
        "reason": "ok",
        "net_lift": net_lift,
    }


def run_pick_test(model, solver, ctrl_r, ctrl_l, rng):
    data = mujoco.MjData(model)
    results = []
    for trial in range(N_PICK_TRIALS):
        r = run_pick_trial(model, data, solver, ctrl_r, ctrl_l, rng)
        results.append(r)
        print(f"  pick trial {trial}: success={r['success']} net_lift={r['net_lift']*100:.2f}cm reason={r['reason']}")
    n_success = sum(r["success"] for r in results)
    rate = n_success / N_PICK_TRIALS
    print(f"Pick test: {n_success}/{N_PICK_TRIALS} ({rate*100:.0f}%), target >= {PICK_SUCCESS_RATE_TARGET*100:.0f}%")
    return rate >= PICK_SUCCESS_RATE_TARGET


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    solver = ik.InverseKinematics(model, "grasp_target_r", ARM_R)
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
    rng = np.random.default_rng(0)

    hold_ok = run_hold_test(model)
    ik_ok = run_ik_unit_test(model, solver, rng)
    pick_ok = run_pick_test(model, solver, ctrl_r, ctrl_l, rng)

    ok = hold_ok and ik_ok and pick_ok
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
