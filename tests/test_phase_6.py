"""Phase 6 -- bimanual box lift groundwork.

This test is intentionally a gatekeeper before tuning a full scripted lift sequence:
models/full_scene.xml must load with the added box freejoint/keyframe, the can and box
scenarios must be physically separable by parking the inactive object, the box must sit and
drop onto the table without numerical blow-up, and the bimanual projection must reduce the
relative-motion residual between the two grasp sites.

Run headless: `python3 tests/test_phase_6.py`
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "full_scene.xml"

import arm_control  # noqa: E402
import bimanual_constraint  # noqa: E402
import grasp  # noqa: E402

ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
QACC_LIMIT = 1e5
BOX_IDLE_DRIFT_LIMIT = 0.002
BOX_HOME_Z = 0.9316
BOX_HOME_QPOS = np.array([0.3055, 0.0, BOX_HOME_Z, 1.0, 0.0, 0.0, 0.0])


def _reset_home(model, data):
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)


def _set_geom_active(model, geom_name, active):
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    model.geom_contype[gid] = 1 if active else 0
    model.geom_conaffinity[gid] = 1 if active else 0
    return gid


def _park_freejoint(model, data, joint_name):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qadr = model.jnt_qposadr[jid]
    dof = model.jnt_dofadr[jid]
    data.qpos[qadr:qadr + 3] = [2.0, 2.0, 0.1]
    data.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[dof:dof + 6] = 0.0


def _box_setup(model, data):
    _reset_home(model, data)
    _park_freejoint(model, data, "can_free")
    box_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_free")
    box_qadr = model.jnt_qposadr[box_jid]
    box_dof = model.jnt_dofadr[box_jid]
    data.qpos[box_qadr:box_qadr + 7] = BOX_HOME_QPOS
    data.qvel[box_dof:box_dof + 6] = 0.0
    _set_geom_active(model, "can_geom", False)
    _set_geom_active(model, "box_geom", True)
    _open_hands(model, data)
    mujoco.mj_forward(model, data)


def _open_hands(model, data):
    for side in ("l", "r"):
        for i in range(1, 21):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"finger_{side}_joint{i}")
            if jid == -1:
                continue
            qadr = model.jnt_qposadr[jid]
            dof = model.jnt_dofadr[jid]
            data.qpos[qadr] = 0.0
            data.qvel[dof] = 0.0


def _make_rig(model):
    lift_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lift_joint")
    lift_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")
    return {
        "ctrl_r": arm_control.ArmTorqueController(model, ARM_R),
        "ctrl_l": arm_control.ArmTorqueController(model, ARM_L),
        "lift_aid": lift_aid,
        "lift_qadr": model.jnt_qposadr[lift_jid],
    }


def _step_hold(model, data, rig):
    rig["ctrl_r"].apply(data, HOME_Q_R)
    rig["ctrl_l"].apply(data, HOME_Q_L)
    data.ctrl[rig["lift_aid"]] = data.qpos[rig["lift_qadr"]]
    grasp.apply_open_hand(model, data, side="r")
    grasp.apply_open_hand(model, data, side="l")
    mujoco.mj_step(model, data)
    return float(np.max(np.abs(data.qacc)))


def run_model_gate(model):
    box_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_free")
    box_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    ok = box_jid != -1 and box_gid != -1 and model.nq == len(model.key_qpos[0])
    print(f"Model gate: box_free={box_jid} box_geom={box_gid} nq={model.nq} "
          f"key_qpos={len(model.key_qpos[0])}: {'OK' if ok else 'FAIL'}")
    return ok


def run_box_idle(model):
    data = mujoco.MjData(model)
    _box_setup(model, data)
    rig = _make_rig(model)
    box_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_free")
    box_qadr = model.jnt_qposadr[box_jid]
    p0 = data.qpos[box_qadr:box_qadr + 3].copy()
    max_qacc = 0.0
    for _ in range(int(5.0 / model.opt.timestep)):
        max_qacc = max(max_qacc, _step_hold(model, data, rig))
    drift = float(np.linalg.norm(data.qpos[box_qadr:box_qadr + 3] - p0))
    print(f"Box idle: drift={drift*1000:.3f}mm max|qacc|={max_qacc:.3f}")
    return drift < BOX_IDLE_DRIFT_LIMIT and max_qacc < QACC_LIMIT


def run_box_drop(model):
    data = mujoco.MjData(model)
    _box_setup(model, data)
    rig = _make_rig(model)
    box_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_free")
    box_qadr = model.jnt_qposadr[box_jid]
    data.qpos[box_qadr + 2] += 0.10
    mujoco.mj_forward(model, data)
    max_qacc = 0.0
    for _ in range(int(3.0 / model.opt.timestep)):
        max_qacc = max(max_qacc, _step_hold(model, data, rig))
    z = float(data.qpos[box_qadr + 2])
    min_dist = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1),
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2),
        }
        if names == {"box_geom", "table"}:
            min_dist = min(min_dist, c.dist)
    ok = abs(z - BOX_HOME_Z) < 0.01 and min_dist >= -0.001 and max_qacc < QACC_LIMIT
    print(f"Box drop: z={z:.4f}m min_box_table_dist={min_dist*1000:.3f}mm "
          f"max|qacc|={max_qacc:.3f}: {'OK' if ok else 'FAIL'}")
    return ok


def run_constraint_projection(model):
    data = mujoco.MjData(model)
    _box_setup(model, data)
    site_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r")
    site_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l")
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
    rng = np.random.default_rng(0)
    dq_r = rng.normal(scale=0.02, size=7)
    dq_l = rng.normal(scale=0.02, size=7)

    jacp_r = np.zeros((3, model.nv))
    jacr_r = np.zeros((3, model.nv))
    jacp_l = np.zeros((3, model.nv))
    jacr_l = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp_r, jacr_r, site_r)
    mujoco.mj_jacSite(model, data, jacp_l, jacr_l, site_l)
    r_to_l = data.site_xpos[site_l] - data.site_xpos[site_r]
    skew = np.array([[0, -r_to_l[2], r_to_l[1]], [r_to_l[2], 0, -r_to_l[0]], [-r_to_l[1], r_to_l[0], 0]])
    transform = np.eye(6)
    transform[0:3, 3:6] = -skew
    j_r = np.vstack([jacp_r[:, ctrl_r.dof_ids], jacr_r[:, ctrl_r.dof_ids]])
    j_l = np.vstack([jacp_l[:, ctrl_l.dof_ids], jacr_l[:, ctrl_l.dof_ids]])
    j_grasp = np.hstack([-transform @ j_r, j_l])
    before = float(np.linalg.norm(j_grasp @ np.concatenate([dq_r, dq_l])))
    pr, pl = bimanual_constraint.project_desired_delta(
        model, data, site_r, site_l, ctrl_r.dof_ids, ctrl_l.dof_ids,
        dq_r, dq_l, model.opt.timestep)
    after = float(np.linalg.norm(j_grasp @ np.concatenate([pr, pl])))
    ok = after < before * 0.05
    print(f"Constraint projection: residual before={before:.6f} after={after:.6f}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    ok = (run_model_gate(model) and run_box_idle(model) and run_box_drop(model)
          and run_constraint_projection(model))
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
