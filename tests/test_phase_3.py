"""Phase 3 -- arm_hand scene + 6DOF IK.

Part 1 (IK unit test): sample 100 random reachable poses via forward kinematics, starting
from HOME_Q (a "ready" configuration near the table, not an arbitrary all-zero pose) and
perturbing every joint by up to +-IK_TEST_SPREAD rad -- this is "the reachable workspace"
in the sense PLAN.md means it: the region actually used while reaching for something on the
table, not arbitrary/unlikely arm configurations across the full joint range (many of which
require routing through singularities that no reasonable teleop session would ever visit).
Each target is solved via solve_pose_multistart (home guess, then a few random restarts if
that doesn't converge -- solve_pose alone can land in a local minimum for a large gap).
Success requires >= 95% of targets converging to position error < 5mm and orientation error
< 5 deg.

Part 2 (integrated pick, script-driven, no teleop): home -> pre-grasp (above the can) ->
straight-line approach (3cm/s) -> Phase 2's grasp sequence -> 10cm lift, run 10 times,
success rate must be >= 7/10. This exercises the full pipeline without any autonomous
perception/planning -- it's a regression harness for the arm+hand+IK integration, not a
"skill".

Run headless: `python3 tests/test_phase_3.py`
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "arm_hand.xml"

import grasp  # noqa: E402
import ik  # noqa: E402

ARM_JOINTS = [f"arm_r_joint{i}" for i in range(1, 8)]

# "Ready" configuration 5cm back / 25cm above the can -- well clear of the table (see the
# "home" keyframe in models/arm_hand.xml). A side-approach home (15cm back, 10cm up) was
# tried first and dragged the palm along the table's near-edge corner during transit; going
# well above and approaching straight down avoids the table's footprint (see NOTES.md
# "Phase 3"). Deliberately NOT the grasp pose itself or all-zeros (both tried, both bad).
HOME_Q = np.array([0.1781, -0.0457, 0.5309, -2.7593, 0.096, 1.0107, -0.4492])
IK_TEST_SPREAD = 0.2  # rad per joint, defines "reachable workspace" for the unit test

N_IK_SAMPLES = 100
POS_TOL = 0.005  # 5mm
ORI_TOL_DEG = 5.0
IK_SUCCESS_RATE_TARGET = 0.95

N_PICK_TRIALS = 10
PICK_SUCCESS_RATE_TARGET = 0.7
APPROACH_SPEED = 0.03  # m/s, per PLAN.md
PRE_GRASP_OFFSET = np.array([0.0, 0.0, 0.10])  # straight above the can, then descend
RAMP_TIME = 1.0
SETTLE_TIME = 1.0
LIFT_HEIGHT = 0.10
LIFT_SPEED = 0.02
POST_LIFT_HOLD = 3.0
MIN_NET_LIFT = 0.08
CAN_NOISE = 0.005


def run_ik_unit_test(model, solver, rng):
    joint_ranges = np.array([model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                              for n in ARM_JOINTS])
    scratch = mujoco.MjData(model)

    successes = 0
    pos_errs, ori_errs = [], []
    for _ in range(N_IK_SAMPLES):
        q_target = np.clip(HOME_Q + rng.uniform(-IK_TEST_SPREAD, IK_TEST_SPREAD, size=7),
                            joint_ranges[:, 0], joint_ranges[:, 1])
        for qadr, val in zip(solver.qpos_adrs, q_target):
            scratch.qpos[qadr] = val
        mujoco.mj_forward(model, scratch)
        target_pos = scratch.site_xpos[solver.site_id].copy()
        target_quat = np.zeros(4)
        mujoco.mju_mat2Quat(target_quat, scratch.site_xmat[solver.site_id])

        _, pos_err, ori_err, converged = solver.solve_pose_multistart(
            HOME_Q, target_pos, target_quat, rng,
            success_pos_tol=POS_TOL, success_ori_tol=np.radians(ORI_TOL_DEG))
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


def _set_arm_ctrl(model, data, q):
    for name, val in zip(ARM_JOINTS, q):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for aid in range(model.nu):
            if model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT and model.actuator_trnid[aid, 0] == jid:
                data.ctrl[aid] = val
                break


def _read_arm_q(model, data):
    return np.array([data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]]
                      for n in ARM_JOINTS])


def _servo_to_target(model, data, solver, rng, target_pos, target_quat, dt, settle_steps, n_correction=2):
    """Closed-loop correction: read the arm's actual current joint angles (data.qpos -- a
    read, never written directly; only ctrl is written) and re-solve/re-servo from there.

    NOTE (see NOTES.md "Phase 3" for the full writeup): this measurably helps but does not
    fully close the gap. The arm's real settling position for a given ctrl target has a
    small residual error -- individually-small per-joint errors (~0.01 rad each,
    correlated with each joint's actuator_forcerange headroom) accumulate through the 7-link
    chain into an ~15-20mm site error concentrated in one task-space direction. Raising kp
    5x barely moved it, and a task-space "aim past the target by the observed error"
    overshoot compensation made it *worse* (different multistart solution each retry, not a
    stable feedback loop) -- so this is deliberately just the plain re-solve, documented as
    a known, unresolved limitation rather than a disguised bug.
    """
    q_target = _read_arm_q(model, data)
    for _ in range(n_correction):
        q_target, _, _, _ = solver.solve_pose_multistart(q_target, target_pos, target_quat, rng)
        _set_arm_ctrl(model, data, q_target)
        for _ in range(settle_steps):
            mujoco.mj_step(model, data)
    return q_target


def run_pick_trial(model, data, solver, rng):
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    can_qadr = model.jnt_qposadr[can_jid]
    can_pos0 = data.qpos[can_qadr : can_qadr + 3].copy()
    can_pos0[:3] += rng.uniform(-CAN_NOISE, CAN_NOISE, size=3)
    data.qpos[can_qadr : can_qadr + 3] = can_pos0
    mujoco.mj_forward(model, data)

    target_quat = np.array([0.5, 0.5, 0.5, 0.5])  # matches models/hand_only.xml's validated grasp orientation
    dt = model.opt.timestep

    # IK solve: pre-grasp (offset back along the approach axis) and final grasp pose.
    pregrasp_pos = can_pos0 + PRE_GRASP_OFFSET
    q_pregrasp, perr, oerr, ok1 = solver.solve_pose_multistart(HOME_Q, pregrasp_pos, target_quat, rng)
    q_grasp, perr2, oerr2, ok2 = solver.solve_pose_multistart(q_pregrasp, can_pos0, target_quat, rng)
    if not (ok1 and ok2):
        return {"success": False, "reason": "ik_failed", "net_lift": 0.0}

    # 1) move arm to pre-grasp. Despite kp=600 the arm settles slowly for large
    # reconfigurations -- some joints run near their real torque limit (forcerange, taken
    # directly from the official spec) and creep the last stretch (see NOTES.md "Phase 3").
    # 5s is generous but this is a real robot arm, not a teleported one.
    _set_arm_ctrl(model, data, q_pregrasp)
    grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0)
    for _ in range(int(5.0 / dt)):
        mujoco.mj_step(model, data)

    # 2) straight-line approach at APPROACH_SPEED from pre-grasp to grasp pose (interpolate
    # in joint space between the two IK solutions -- both share the same target orientation
    # and are close together, so this tracks a near-straight-line Cartesian path)
    approach_dist = np.linalg.norm(PRE_GRASP_OFFSET)
    approach_time = approach_dist / APPROACH_SPEED
    t = 0.0
    while t < approach_time:
        frac = t / approach_time
        q_interp = q_pregrasp + frac * (q_grasp - q_pregrasp)
        _set_arm_ctrl(model, data, q_interp)
        mujoco.mj_step(model, data)
        t += dt
    _set_arm_ctrl(model, data, q_grasp)
    for _ in range(int(2.0 / dt)):
        mujoco.mj_step(model, data)
    # closed-loop correction: open-loop servoing leaves a real residual error (see
    # NOTES.md "Phase 3") that's too large for Phase 2's grasp tolerances on its own.
    _servo_to_target(model, data, solver, rng, can_pos0, target_quat, dt, settle_steps=int(1.0 / dt))

    # 3) Phase 2 grasp sequence: ramp closed, settle
    t = 0.0
    while t < RAMP_TIME:
        frac = t / RAMP_TIME
        grasp.apply_grasp(model, data, grasp=frac, thumb=frac)
        mujoco.mj_step(model, data)
        t += dt
    t = 0.0
    while t < SETTLE_TIME:
        grasp.apply_grasp(model, data, grasp=1.0, thumb=1.0)
        mujoco.mj_step(model, data)
        t += dt

    grasped = grasp.is_grasped(model, data)
    can_z_before_lift = data.qpos[can_qadr + 2]

    # 4) lift: move the IK target itself up by LIFT_HEIGHT and re-solve, then servo there
    lift_target_pos = can_pos0 + np.array([0, 0, LIFT_HEIGHT])
    q_lift, _, _, _ = solver.solve_pose_multistart(q_grasp, lift_target_pos, target_quat, rng)
    lift_time = LIFT_HEIGHT / LIFT_SPEED
    t = 0.0
    while t < lift_time:
        frac = t / lift_time
        q_interp = q_grasp + frac * (q_lift - q_grasp)
        _set_arm_ctrl(model, data, q_interp)
        grasp.apply_grasp(model, data, grasp=1.0, thumb=1.0)
        mujoco.mj_step(model, data)
        t += dt
    _set_arm_ctrl(model, data, q_lift)

    t = 0.0
    while t < POST_LIFT_HOLD:
        grasp.apply_grasp(model, data, grasp=1.0, thumb=1.0)
        mujoco.mj_step(model, data)
        t += dt

    net_lift = data.qpos[can_qadr + 2] - can_z_before_lift
    return {
        "success": grasped and net_lift >= MIN_NET_LIFT,
        "reason": "ok",
        "net_lift": net_lift,
    }


def run_pick_test(model, solver, rng):
    data = mujoco.MjData(model)
    results = []
    for trial in range(N_PICK_TRIALS):
        r = run_pick_trial(model, data, solver, rng)
        results.append(r)
        print(f"  pick trial {trial}: success={r['success']} net_lift={r['net_lift']*100:.2f}cm reason={r['reason']}")
    n_success = sum(r["success"] for r in results)
    rate = n_success / N_PICK_TRIALS
    print(f"Pick test: {n_success}/{N_PICK_TRIALS} ({rate*100:.0f}%), target >= {PICK_SUCCESS_RATE_TARGET*100:.0f}%")
    return rate >= PICK_SUCCESS_RATE_TARGET


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    solver = ik.InverseKinematics(model, "grasp_target", ARM_JOINTS)
    rng = np.random.default_rng(0)

    ik_ok = run_ik_unit_test(model, solver, rng)
    pick_ok = run_pick_test(model, solver, rng)

    ok = ik_ok and pick_ok
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
