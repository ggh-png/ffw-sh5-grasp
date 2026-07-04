"""Phase 2 -- fixed-hand grasp + lift (the core of this project).

For each of N_TRIALS: reset to the "pregrasp" keyframe with the can perturbed by +-5mm
in x/y/z, ramp grasp+thumb from 0->1 (rate-limited, not a step function), confirm a
force-based grasp (src/grasp.py:is_grasped), then raise the mocap anchor (and therefore the
welded hand) 10cm at 2cm/s and hold for 5s. Success = the can rose with the hand (>= 8cm net
lift, since some settle/compliance is expected) and slipped < 1cm in the hand's frame during
the final hold.

Run headless: `python3 tests/test_phase_2.py`
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "hand_only.xml"

import grasp  # noqa: E402

N_TRIALS = 10
NOISE = 0.005  # +-5mm, per PLAN.md
RAMP_TIME = 1.0  # s, grasp/thumb 0->1
SETTLE_TIME = 1.0  # s, hold closed grasp before lifting
LIFT_HEIGHT = 0.10  # m
LIFT_SPEED = 0.02  # m/s (2cm/s, per PLAN.md)
POST_LIFT_HOLD = 5.0  # s
MIN_NET_LIFT = 0.08  # m -- some sag under load is expected, still counts as a real lift
MAX_SLIP = 0.01  # m, measured in the hand's own frame over the post-lift hold

SUCCESS_RATE_TARGET = 0.8  # 8/10


def hand_frame_offset(model, data, can_qadr):
    """Can position relative to the (moving) hand base -- isolates slip from the lift itself."""
    hand_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hx5_r_base")
    return data.qpos[can_qadr : can_qadr + 3] - data.xpos[hand_bid]


def run_trial(model, data, rng):
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "pregrasp")
    mujoco.mj_resetDataKeyframe(model, data, key_id)

    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    qadr = model.jnt_qposadr[can_jid]
    data.qpos[qadr : qadr + 3] += rng.uniform(-NOISE, NOISE, size=3)
    mujoco.mj_forward(model, data)

    mocap_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand_mocap")
    mocap_id = model.body_mocapid[mocap_bid]
    mocap_start = data.mocap_pos[mocap_id].copy()

    dt = model.opt.timestep

    # 1) ramp grasp+thumb closed
    t = 0.0
    while t < RAMP_TIME:
        frac = t / RAMP_TIME
        grasp.apply_grasp(model, data, grasp=frac, thumb=frac)
        mujoco.mj_step(model, data)
        t += dt

    # 2) settle
    t = 0.0
    while t < SETTLE_TIME:
        grasp.apply_grasp(model, data, grasp=1.0, thumb=1.0)
        mujoco.mj_step(model, data)
        t += dt

    grasped_before_lift = grasp.is_grasped(model, data)
    can_z_before_lift = data.qpos[qadr + 2]

    # 3) lift the mocap anchor at LIFT_SPEED -- the weld constraint drags hx5_r_base (and
    # whatever it's holding) along; this is a mocap target update, not a qpos override of a
    # dynamic body.
    lift_duration = LIFT_HEIGHT / LIFT_SPEED
    t = 0.0
    while t < lift_duration:
        data.mocap_pos[mocap_id] = mocap_start + np.array([0, 0, min(t, lift_duration) * LIFT_SPEED])
        grasp.apply_grasp(model, data, grasp=1.0, thumb=1.0)
        mujoco.mj_step(model, data)
        t += dt
    data.mocap_pos[mocap_id] = mocap_start + np.array([0, 0, LIFT_HEIGHT])

    # 4) hold at full lift height, track slip in the hand's frame
    offsets = []
    t = 0.0
    while t < POST_LIFT_HOLD:
        grasp.apply_grasp(model, data, grasp=1.0, thumb=1.0)
        mujoco.mj_step(model, data)
        offsets.append(hand_frame_offset(model, data, qadr).copy())
        t += dt

    can_z_after_hold = data.qpos[qadr + 2]
    net_lift = can_z_after_hold - can_z_before_lift
    offsets = np.array(offsets)
    slip = np.linalg.norm(offsets[-1] - offsets[0])

    return {
        "grasped_before_lift": grasped_before_lift,
        "net_lift": net_lift,
        "slip": slip,
        "success": grasped_before_lift and net_lift >= MIN_NET_LIFT and slip <= MAX_SLIP,
    }


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    rng = np.random.default_rng(0)

    results = []
    for trial in range(N_TRIALS):
        r = run_trial(model, data, rng)
        results.append(r)
        print(
            f"trial {trial}: grasped={r['grasped_before_lift']} "
            f"net_lift={r['net_lift']*100:.2f}cm slip={r['slip']*1000:.2f}mm "
            f"success={r['success']}"
        )

    n_success = sum(r["success"] for r in results)
    rate = n_success / N_TRIALS
    print(f"\nSuccess rate: {n_success}/{N_TRIALS} ({rate*100:.0f}%), target >= {SUCCESS_RATE_TARGET*100:.0f}%")

    ok = rate >= SUCCESS_RATE_TARGET
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
