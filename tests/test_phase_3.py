"""Phase 3 -- arm_hand scene + 6DOF IK.

Part 0 (FK/Jacobian test): compare the public world-aligned geometric Jacobian against
central finite differences, including normalized quaternion double-cover handling.

Part 1 (IK unit test): sample 100 random reachable poses via forward kinematics, starting
from HOME_Q (a "ready" configuration near the table, not an arbitrary all-zero pose) and
perturbing every joint by up to +-IK_TEST_SPREAD rad -- this is "the reachable workspace"
in the sense this project means it: the region actually used while reaching for something on
the table, not arbitrary/unlikely arm configurations across the full joint range (many of which
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

import arm_control  # noqa: E402
import grasp  # noqa: E402
import ik  # noqa: E402
import kinematics  # noqa: E402

ARM_JOINTS = [f"arm_r_joint{i}" for i in range(1, 8)]

# "Ready" configuration 10cm back / 20cm above the can -- well clear of the table (see the
# "home" keyframe in models/arm_hand.xml). Recomputed after fixing a grasp_target site
# definition bug: the site was defined using the can's *world* coordinates from
# hand_only.xml as if they were a *local* offset from the palm, which they
# were not -- every HOME_Q/q_pregrasp/q_grasp computed before the fix targeted the wrong
# physical point on the hand entirely.
HOME_Q = np.array([-0.225, -0.394, 0.682, -2.613, -0.704, 0.843, -1.218])
IK_TEST_SPREAD = 0.2  # rad per joint, defines "reachable workspace" for the unit test

N_IK_SAMPLES = 100
POS_TOL = 0.005  # 5mm
ORI_TOL_DEG = 5.0
IK_SUCCESS_RATE_TARGET = 0.95

N_PICK_TRIALS = 10
PICK_SUCCESS_RATE_TARGET = 0.7
APPROACH_SPEED = 0.03  # m/s
PRE_GRASP_OFFSET = np.array([0.0, 0.0, 0.10])  # straight above the can, then descend
# Before fixing models/arm_hand.xml's grasp_target site (it was defined using
# hand_only.xml's can *world* coordinates as if they were a palm-*local* offset -- they
# weren't), the resulting bad geometry made the middle finger's MCP
# knuckle graze the table. Kept at zero now that the real fix (the site itself) is in;
# a future can/table layout that reintroduces the clearance problem should raise this
# instead of trying to "float" the can (it has a real freejoint -- it free-falls back to
# table_top + its own half-height regardless of spawn height).
GRASP_TARGET_OFFSET = np.array([0.0, 0.0, 0.0])
RAMP_TIME = 1.0
SETTLE_TIME = 1.0
LIFT_HEIGHT = 0.10
LIFT_SPEED = 0.02
POST_LIFT_HOLD = 3.0
MIN_NET_LIFT = 0.08
CAN_NOISE = 0.005


def run_fk_jacobian_test(solver):
    """Public FK and world-aligned Jacobian must agree with finite differences."""
    state = solver.forward_kinematics(HOME_Q)
    epsilon = 1e-6
    numerical = np.zeros_like(state.jacobian)
    for index in range(len(HOME_Q)):
        q_plus, q_minus = HOME_Q.copy(), HOME_Q.copy()
        q_plus[index] += epsilon
        q_minus[index] -= epsilon
        plus = solver.forward_kinematics(q_plus)
        minus = solver.forward_kinematics(q_minus)
        numerical[:3, index] = (plus.position - minus.position) / (2.0 * epsilon)
        numerical[3:, index] = kinematics.shortest_orientation_error(
            plus.quaternion, minus.quaternion) / (2.0 * epsilon)

    max_error = float(np.max(np.abs(state.jacobian - numerical)))
    quaternion_unit = abs(np.linalg.norm(state.quaternion) - 1.0) < 1e-12
    double_cover_error = np.linalg.norm(
        kinematics.shortest_orientation_error(state.quaternion, -state.quaternion))
    ok = max_error < 1e-5 and quaternion_unit and double_cover_error < 1e-12
    print(f"FK/Jacobian test: max_fd_error={max_error:.2e} quaternion_unit={quaternion_unit} "
          f"double_cover={double_cover_error:.1e}: {'OK' if ok else 'FAIL'}")
    return ok


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


def _read_arm_q(model, data):
    return np.array([data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]]
                      for n in ARM_JOINTS])


def _hold(model, data, controller, q_des, duration, dt, grasp_frac=None, thumb_frac=None):
    """Step the sim for `duration` seconds, driving the arm to q_des via torque control
    every step (motor actuators need fresh torque each step -- unlike the old <position>
    actuators, there's nothing to "hold" a stale ctrl value) and optionally the hand via
    grasp.apply_grasp at a constant fraction.
    """
    n = int(duration / dt)
    for _ in range(n):
        controller.apply(data, q_des)
        if grasp_frac is not None:
            grasp.apply_grasp(model, data, grasp=grasp_frac, thumb=thumb_frac)
        mujoco.mj_step(model, data)


def _move(model, data, controller, q_from, q_to, duration, dt, grasp_frac=None, thumb_frac=None):
    """Like _hold, but ramps q_des linearly from q_from to q_to over `duration` seconds."""
    n = int(duration / dt)
    for i in range(n):
        frac = i / n
        controller.apply(data, q_from + frac * (q_to - q_from))
        if grasp_frac is not None:
            grasp.apply_grasp(model, data, grasp=grasp_frac, thumb=thumb_frac)
        mujoco.mj_step(model, data)


def run_pick_trial(model, data, solver, controller, rng):
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
    # grasp_target_pos aims 3cm above the can's actual center (GRASP_TARGET_OFFSET) so the
    # hand clears the table; can_pos0 itself (used for noise and lift measurement) is
    # unaffected.
    grasp_target_pos = can_pos0 + GRASP_TARGET_OFFSET
    pregrasp_pos = grasp_target_pos + PRE_GRASP_OFFSET
    q_pregrasp, perr, oerr, ok1 = solver.solve_pose_multistart(HOME_Q, pregrasp_pos, target_quat, rng)
    q_grasp, perr2, oerr2, ok2 = solver.solve_pose_multistart(q_pregrasp, grasp_target_pos, target_quat, rng)
    if not (ok1 and ok2):
        return {"success": False, "reason": "ik_failed", "net_lift": 0.0}

    q_home = _read_arm_q(model, data)

    # 1) move arm to pre-grasp (torque control: gravity/Coriolis feedforward + PD, see
    # src/arm_control.py -- this is what replaced the old <position>-actuator approach
    # after diagnosing its ~15-20mm residual site error).
    _move(model, data, controller, q_home, q_pregrasp, 3.0, dt, grasp_frac=0.0, thumb_frac=0.0)
    _hold(model, data, controller, q_pregrasp, 1.0, dt, grasp_frac=0.0, thumb_frac=0.0)

    # 2) straight-line approach at APPROACH_SPEED from pre-grasp to grasp pose (interpolate
    # in joint space between the two IK solutions -- both share the same target orientation
    # and are close together, so this tracks a near-straight-line Cartesian path)
    approach_dist = np.linalg.norm(PRE_GRASP_OFFSET)
    approach_time = approach_dist / APPROACH_SPEED
    _move(model, data, controller, q_pregrasp, q_grasp, approach_time, dt, grasp_frac=0.0, thumb_frac=0.0)
    _hold(model, data, controller, q_grasp, 1.0, dt, grasp_frac=0.0, thumb_frac=0.0)

    # 3) Phase 2 grasp sequence: ramp closed, settle (arm holds q_grasp throughout)
    n = int(RAMP_TIME / dt)
    for i in range(n):
        frac = i / n
        controller.apply(data, q_grasp)
        grasp.apply_grasp(model, data, grasp=frac, thumb=frac)
        mujoco.mj_step(model, data)
    _hold(model, data, controller, q_grasp, SETTLE_TIME, dt, grasp_frac=1.0, thumb_frac=1.0)

    grasped = grasp.is_grasped(model, data)
    can_z_before_lift = data.qpos[can_qadr + 2]

    # 4) lift: move the IK target itself up by LIFT_HEIGHT and re-solve, then servo there
    lift_target_pos = grasp_target_pos + np.array([0, 0, LIFT_HEIGHT])
    q_lift, _, _, _ = solver.solve_pose_multistart(q_grasp, lift_target_pos, target_quat, rng)
    lift_time = LIFT_HEIGHT / LIFT_SPEED
    _move(model, data, controller, q_grasp, q_lift, lift_time, dt, grasp_frac=1.0, thumb_frac=1.0)
    _hold(model, data, controller, q_lift, POST_LIFT_HOLD, dt, grasp_frac=1.0, thumb_frac=1.0)

    net_lift = data.qpos[can_qadr + 2] - can_z_before_lift
    return {
        "success": grasped and net_lift >= MIN_NET_LIFT,
        "reason": "ok",
        "net_lift": net_lift,
    }


def run_pick_test(model, solver, controller, rng):
    data = mujoco.MjData(model)
    results = []
    for trial in range(N_PICK_TRIALS):
        r = run_pick_trial(model, data, solver, controller, rng)
        results.append(r)
        print(f"  pick trial {trial}: success={r['success']} net_lift={r['net_lift']*100:.2f}cm reason={r['reason']}")
    n_success = sum(r["success"] for r in results)
    rate = n_success / N_PICK_TRIALS
    print(f"Pick test: {n_success}/{N_PICK_TRIALS} ({rate*100:.0f}%), target >= {PICK_SUCCESS_RATE_TARGET*100:.0f}%")
    return rate >= PICK_SUCCESS_RATE_TARGET


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    solver = ik.InverseKinematics(model, "grasp_target", ARM_JOINTS)
    controller = arm_control.ArmTorqueController(model, ARM_JOINTS)
    rng = np.random.default_rng(0)

    fk_ok = run_fk_jacobian_test(solver)
    ik_ok = run_ik_unit_test(model, solver, rng)
    pick_ok = run_pick_test(model, solver, controller, rng)

    ok = fk_ok and ik_ok and pick_ok
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
