"""Phase 1 — hand_only scene + collision validation.

Runs the penetration test from PLAN.md: close all finger curl joints at max
actuator authority toward the can, 20 times, and record the worst
finger-vs-can contact penetration depth (`contact.dist`, negative = overlap).

Also reports the achieved real-time factor for the same rollout.

Run headless: `python3 tests/test_phase_1.py`
"""

import pathlib
import time
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
MODEL_PATH = REPO_ROOT / "models" / "hand_only.xml"

N_TRIALS = 20
SIM_SECONDS = 1.5
PENETRATION_LIMIT = 0.002  # 2mm
RTF_TARGET = 0.5

# Curl joints only: thumb (mcp_pitch, ip) + each finger's (pip, dip, tip).
CURL_JOINTS = {"finger_r_joint3", "finger_r_joint4"}
for base in (5, 9, 13, 17):
    CURL_JOINTS.update({f"finger_r_joint{base+1}", f"finger_r_joint{base+2}", f"finger_r_joint{base+3}"})

CAN_INIT_POS = np.array([0.105, 0.065, 0.16])  # matches models/hand_only.xml can body pos (Phase 2)
CAN_INIT_QUAT = np.array([1.0, 0.0, 0.0, 0.0])


def actuator_for_joint(model, jid):
    for aid in range(model.nu):
        if (
            model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT
            and model.actuator_trnid[aid, 0] == jid
        ):
            return aid
    return None


def reset_trial(model, data):
    # Phase 1 tests the closing motion itself, not placement robustness (that's Phase 2's
    # +-5mm randomized grasp+lift test) -- deterministic can pose, repeated for consistency.
    mujoco.mj_resetData(model, data)
    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    qadr = model.jnt_qposadr[can_jid]
    data.qpos[qadr : qadr + 3] = CAN_INIT_POS
    data.qpos[qadr + 3 : qadr + 7] = CAN_INIT_QUAT
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def close_hand(model, data):
    for jid in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        if name in CURL_JOINTS:
            aid = actuator_for_joint(model, jid)
            if aid is None:
                continue  # locked joint (range=0), no actuator -- see Phase 2 NOTES
            hi = model.jnt_range[jid][1]
            data.ctrl[aid] = hi


def worst_finger_can_penetration(model, data):
    can_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "can_geom")
    worst = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        if can_gid not in (c.geom1, c.geom2):
            continue
        other = c.geom1 if c.geom2 == can_gid else c.geom2
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[other]) or ""
        if not (bname.startswith("finger_r_") or bname == "hx5_r_base"):
            continue  # ignore can-vs-floor once it falls; only hand-can contacts matter here
        if c.dist < worst:
            worst = c.dist
    return worst


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    n_steps = int(SIM_SECONDS / model.opt.timestep)
    max_penetration = 0.0
    total_wall = 0.0

    for trial in range(N_TRIALS):
        reset_trial(model, data)
        close_hand(model, data)

        t0 = time.perf_counter()
        for _ in range(n_steps):
            mujoco.mj_step(model, data)
            pen = worst_finger_can_penetration(model, data)
            if -pen > max_penetration:
                max_penetration = -pen
        total_wall += time.perf_counter() - t0

    sim_seconds_total = N_TRIALS * SIM_SECONDS
    rtf = sim_seconds_total / total_wall

    print(f"Trials: {N_TRIALS}, {n_steps} steps each ({SIM_SECONDS}s sim)")
    print(f"Max finger-can penetration depth: {max_penetration * 1000:.3f} mm (limit {PENETRATION_LIMIT*1000:.1f} mm)")
    print(f"Real-time factor: {rtf:.2f} (target >= {RTF_TARGET})")

    ok = max_penetration < PENETRATION_LIMIT
    print("PASS" if ok else "FAIL: penetration exceeds limit")
    if rtf < RTF_TARGET:
        print(f"WARNING: real-time factor {rtf:.2f} below target {RTF_TARGET}")

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
