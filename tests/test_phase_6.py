"""Phase 6 -- bimanual box lift integration gate.

This test keeps the box task tied to the requested control contract: numeric X/Y/Z plus
Roll/Pitch/Yaw targets are the control surface, visible IK target markers are display-only,
and the box can be squeezed, lifted, and carried by the mobile base without regressing the
underlying model/keyframe/constraint assumptions.

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
import base_teleop  # noqa: E402
import bimanual_constraint  # noqa: E402
import grasp  # noqa: E402
import ik  # noqa: E402
import teleop_app  # noqa: E402
import teleop_ui  # noqa: E402

ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
QACC_LIMIT = 1e5
BOX_IDLE_DRIFT_LIMIT = 0.002
BOX_HOME_Z = 0.8716
BOX_HOME_QPOS = np.array([0.4055, 0.0, BOX_HOME_Z, 1.0, 0.0, 0.0, 0.0])
BOX_HALF_EXTENTS = np.array([0.10, 0.10, 0.14])
BOX_PREGRASP_GAP = 0.03
BOX_SQUEEZE_GAP = -0.005
BOX_TARGET_SITE_TO_PALM_MARGIN = 0.060
BOX_SQUEEZE_KP_SCALE = 0.08
CONTROL_FRAME_DT = 0.04
CONSTRAINT_DRIFT_GAIN = 0.2
BOX_FORCE_RANGE = (0.5, 10.0)
BOX_LIFT_HEIGHT = 0.10
BOX_MIN_NET_LIFT = 0.08
BOX_MAX_REL_DRIFT = 0.005
BOX_DRIVE_TIME = 1.5
BOX_MIN_DRIVE_DISTANCE = 0.30


def _rpy_deg_to_quat(rpy_deg):
    r, p, y = np.radians(rpy_deg)
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


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
        "solver_r": ik.InverseKinematics(model, "grasp_target_r", ARM_R),
        "solver_l": ik.InverseKinematics(model, "grasp_target_l", ARM_L),
        "site_r": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r"),
        "site_l": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l"),
        "lift_aid": lift_aid,
        "lift_qadr": model.jnt_qposadr[lift_jid],
        "base_x_qadr": model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_x")],
        "base_yaw_qadr": model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_yaw")],
        "wheel_steer_aids": {
            w: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{w}_steer")
            for w in ("left_wheel", "right_wheel", "rear_wheel")
        },
        "wheel_drive_aids": {
            w: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{w}_drive")
            for w in ("left_wheel", "right_wheel", "rear_wheel")
        },
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
    virtual_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "virtual_object_marker")
    ok = box_jid != -1 and box_gid != -1 and virtual_body != -1 and model.nq == len(model.key_qpos[0])
    print(f"Model gate: box_free={box_jid} box_geom={box_gid} nq={model.nq} "
          f"key_qpos={len(model.key_qpos[0])} virtual_marker={virtual_body}: "
          f"{'OK' if ok else 'FAIL'}")
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


def _box_target_positions_from_qpos(box_qpos, gap, z_offset=0.0):
    box_pos = box_qpos[:3].copy()
    box_pos[2] += z_offset
    box_quat = box_qpos[3:7]
    box_mat = np.zeros(9)
    mujoco.mju_quat2Mat(box_mat, box_quat)
    box_R = box_mat.reshape(3, 3)
    offset = BOX_HALF_EXTENTS[1] + gap - BOX_TARGET_SITE_TO_PALM_MARGIN
    return (box_pos + box_R @ np.array([0.0, -offset, 0.0]),
            box_pos + box_R @ np.array([0.0, offset, 0.0]))


def _box_target_positions(model, data, gap, z_offset=0.0):
    box_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_free")
    box_qadr = model.jnt_qposadr[box_jid]
    return _box_target_positions_from_qpos(data.qpos[box_qadr:box_qadr + 7], gap, z_offset)


def _site_home_quats(data, rig):
    quat_r = np.zeros(4)
    quat_l = np.zeros(4)
    mujoco.mju_mat2Quat(quat_r, data.site_xmat[rig["site_r"]])
    mujoco.mju_mat2Quat(quat_l, data.site_xmat[rig["site_l"]])
    return quat_r, quat_l


def _solve_box_pair(model, data, rig, q_r_seed, q_l_seed, gap, rng, z_offset=0.0,
                    n_restarts=8):
    quat_r = rig.get("target_quat_r")
    quat_l = rig.get("target_quat_l")
    if quat_r is None or quat_l is None:
        quat_r, quat_l = _site_home_quats(data, rig)
    pos_r, pos_l = _box_target_positions(model, data, gap, z_offset=z_offset)
    q_r, perr_r, oerr_r, ok_r = rig["solver_r"].solve_pose_multistart(
        q_r_seed, pos_r, quat_r, rng, n_restarts=n_restarts, context_qpos=data.qpos,
        success_pos_tol=0.01, success_ori_tol=np.radians(10.0))
    q_l, perr_l, oerr_l, ok_l = rig["solver_l"].solve_pose_multistart(
        q_l_seed, pos_l, quat_l, rng, n_restarts=n_restarts, context_qpos=data.qpos,
        success_pos_tol=0.01, success_ori_tol=np.radians(10.0))
    return {
        "q_r": q_r, "q_l": q_l, "ok": ok_r and ok_l,
        "pos_err_r": perr_r, "pos_err_l": perr_l,
        "ori_err_r": oerr_r, "ori_err_l": oerr_l,
    }


def _apply_box_controls(model, data, rig, q_r, q_l, wheel_cmds=None):
    rig["ctrl_r"].apply(data, q_r, kp_scale=BOX_SQUEEZE_KP_SCALE)
    rig["ctrl_l"].apply(data, q_l, kp_scale=BOX_SQUEEZE_KP_SCALE)
    data.ctrl[rig["lift_aid"]] = data.qpos[rig["lift_qadr"]]
    grasp.apply_open_hand(model, data, side="r")
    grasp.apply_open_hand(model, data, side="l")
    if wheel_cmds is not None:
        for wheel, (steer_angle, drive_angvel) in wheel_cmds.items():
            data.ctrl[rig["wheel_steer_aids"][wheel]] = steer_angle
            data.ctrl[rig["wheel_drive_aids"][wheel]] = drive_angvel
    else:
        for aid in rig["wheel_drive_aids"].values():
            data.ctrl[aid] = 0.0


def _frame_move_pair(model, data, rig, q_r_from, q_l_from, q_r_to, q_l_to,
                     duration, reference=None):
    frames = max(1, int(round(duration / CONTROL_FRAME_DT)))
    steps_per_frame = max(1, int(round(CONTROL_FRAME_DT / model.opt.timestep)))
    q_r = q_r_from.copy()
    q_l = q_l_from.copy()
    max_qacc = 0.0
    for frame in range(frames):
        remaining = max(1, frames - frame)
        dq_r = (q_r_to - q_r) / remaining
        dq_l = (q_l_to - q_l) / remaining
        if reference is not None:
            dq_r, dq_l = bimanual_constraint.project_desired_delta(
                model, data, rig["site_r"], rig["site_l"],
                rig["ctrl_r"].dof_ids, rig["ctrl_l"].dof_ids,
                dq_r, dq_l, CONTROL_FRAME_DT, reference=reference,
                drift_gain=CONSTRAINT_DRIFT_GAIN)
        q_r += dq_r
        q_l += dq_l
        for _ in range(steps_per_frame):
            _apply_box_controls(model, data, rig, q_r, q_l)
            mujoco.mj_step(model, data)
            max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))
    return q_r, q_l, max_qacc


def _frame_hold_pair(model, data, rig, q_r, q_l, duration, reference=None,
                     drive=None, drive_keys=None):
    frames = max(1, int(round(duration / CONTROL_FRAME_DT)))
    steps_per_frame = max(1, int(round(CONTROL_FRAME_DT / model.opt.timestep)))
    max_qacc = 0.0
    q_r = q_r.copy()
    q_l = q_l.copy()
    for _ in range(frames):
        wheel_cmds = None
        if drive is not None:
            wheel_cmds = drive.update(drive_keys or {}, CONTROL_FRAME_DT,
                                      float(data.qpos[rig["base_yaw_qadr"]]))
        if reference is not None:
            dq_r, dq_l = bimanual_constraint.project_desired_delta(
                model, data, rig["site_r"], rig["site_l"],
                rig["ctrl_r"].dof_ids, rig["ctrl_l"].dof_ids,
                np.zeros(7), np.zeros(7), CONTROL_FRAME_DT, reference=reference,
                drift_gain=CONSTRAINT_DRIFT_GAIN)
            q_r += dq_r
            q_l += dq_l
        for _ in range(steps_per_frame):
            _apply_box_controls(model, data, rig, q_r, q_l, wheel_cmds=wheel_cmds)
            mujoco.mj_step(model, data)
            max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))
    return q_r, q_l, max_qacc


def _frame_lift_box(model, data, rig, q_r, q_l, box_qpos_ref, lift_height, duration, reference):
    frames = max(1, int(round(duration / CONTROL_FRAME_DT)))
    steps_per_frame = max(1, int(round(CONTROL_FRAME_DT / model.opt.timestep)))
    quat_r = rig["target_quat_r"]
    quat_l = rig["target_quat_l"]
    q_r = q_r.copy()
    q_l = q_l.copy()
    max_qacc = 0.0
    for frame in range(frames):
        z_offset = lift_height * (frame + 1) / frames
        pos_r, pos_l = _box_target_positions_from_qpos(
            box_qpos_ref, BOX_SQUEEZE_GAP, z_offset=z_offset)
        next_r, _, _ = rig["solver_r"].solve_pose(
            q_r, pos_r, quat_r, max_iter=15, context_qpos=data.qpos)
        next_l, _, _ = rig["solver_l"].solve_pose(
            q_l, pos_l, quat_l, max_iter=15, context_qpos=data.qpos)
        dq_r, dq_l = bimanual_constraint.project_desired_delta(
            model, data, rig["site_r"], rig["site_l"],
            rig["ctrl_r"].dof_ids, rig["ctrl_l"].dof_ids,
            next_r - q_r, next_l - q_l, CONTROL_FRAME_DT, reference=reference,
            drift_gain=CONSTRAINT_DRIFT_GAIN)
        q_r += dq_r
        q_l += dq_l
        for _ in range(steps_per_frame):
            _apply_box_controls(model, data, rig, q_r, q_l)
            mujoco.mj_step(model, data)
            max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))
    return q_r, q_l, max_qacc


def _prepare_squeezed_box(model, seed=10):
    data = mujoco.MjData(model)
    _box_setup(model, data)
    rig = _make_rig(model)
    rig["target_quat_r"], rig["target_quat_l"] = _site_home_quats(data, rig)
    rng = np.random.default_rng(seed)
    pre = _solve_box_pair(model, data, rig, HOME_Q_R, HOME_Q_L, BOX_PREGRASP_GAP, rng)
    squeeze = _solve_box_pair(model, data, rig, pre["q_r"], pre["q_l"], BOX_SQUEEZE_GAP, rng)
    if not (pre["ok"] and squeeze["ok"]):
        return data, rig, squeeze, None, None, False, np.inf
    q_r, q_l, max_qacc = _frame_move_pair(
        model, data, rig, HOME_Q_R, HOME_Q_L, pre["q_r"], pre["q_l"], 3.0)
    q_r, q_l, qacc = _frame_move_pair(
        model, data, rig, q_r, q_l, squeeze["q_r"], squeeze["q_l"], 1.0)
    max_qacc = max(max_qacc, qacc)
    q_r, q_l, qacc = _frame_hold_pair(model, data, rig, q_r, q_l, 1.0)
    max_qacc = max(max_qacc, qacc)
    return data, rig, squeeze, q_r, q_l, True, max_qacc


def _prepare_lifted_box(model, seed=20):
    data, rig, squeeze, q_r, q_l, ok, max_qacc = _prepare_squeezed_box(model, seed=seed)
    if not ok or not grasp.is_box_held(model, data, min_force_per_hand=BOX_FORCE_RANGE[0]):
        return data, rig, q_r, q_l, None, False, 0.0, np.inf
    reference = bimanual_constraint.snapshot_relative_pose(data, rig["site_r"], rig["site_l"])
    box_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_free")
    box_qadr = model.jnt_qposadr[box_jid]
    box_qpos_ref = data.qpos[box_qadr:box_qadr + 7].copy()
    z0 = float(data.qpos[box_qadr + 2])
    q_r, q_l, qacc = _frame_lift_box(
        model, data, rig, q_r, q_l, box_qpos_ref, BOX_LIFT_HEIGHT, 5.0, reference)
    max_qacc = max(max_qacc, qacc)
    q_r, q_l, qacc = _frame_hold_pair(model, data, rig, q_r, q_l, 1.0, reference=reference)
    max_qacc = max(max_qacc, qacc)
    net_lift = float(data.qpos[box_qadr + 2] - z0)
    return data, rig, q_r, q_l, reference, True, net_lift, max_qacc


def run_box_pregrasp_ik(model):
    data = mujoco.MjData(model)
    _box_setup(model, data)
    rig = _make_rig(model)
    rig["target_quat_r"], rig["target_quat_l"] = _site_home_quats(data, rig)
    rng = np.random.default_rng(30)
    successes = 0
    pos_errs = []
    for _ in range(20):
        result = _solve_box_pair(model, data, rig, HOME_Q_R, HOME_Q_L, BOX_PREGRASP_GAP, rng)
        pos_errs.extend([result["pos_err_r"], result["pos_err_l"]])
        if result["ok"]:
            successes += 1
    ok = successes >= 18 and max(pos_errs) < 0.01
    print(f"Box pregrasp IK: {successes}/20 pair solves, "
          f"max_pos_err={max(pos_errs)*1000:.3f}mm: {'OK' if ok else 'FAIL'}")
    return ok


def run_squeeze_stability(model):
    data, _, _, _, _, ok, max_qacc = _prepare_squeezed_box(model)
    forces = grasp.get_box_hand_contacts(model, data)
    held = grasp.is_box_held(model, data, min_force_per_hand=BOX_FORCE_RANGE[0])
    force_ok = all(BOX_FORCE_RANGE[0] <= forces[s] <= BOX_FORCE_RANGE[1] for s in ("l", "r"))
    ok = ok and held and force_ok and max_qacc < QACC_LIMIT
    print(f"Box squeeze: L={forces['l']:.3f}N R={forces['r']:.3f}N "
          f"max|qacc|={max_qacc:.3f}: {'OK' if ok else 'FAIL'}")
    return ok


def run_scripted_lift(model):
    data, rig, _, _, reference, ok, net_lift, max_qacc = _prepare_lifted_box(model)
    forces = grasp.get_box_hand_contacts(model, data)
    pos_err, _ = bimanual_constraint.relative_pose_error(data, rig["site_r"], rig["site_l"], reference)
    drift = float(np.linalg.norm(pos_err))
    held = grasp.is_box_held(model, data, min_force_per_hand=BOX_FORCE_RANGE[0])
    ok = (ok and held and net_lift >= BOX_MIN_NET_LIFT and drift < BOX_MAX_REL_DRIFT
          and max_qacc < QACC_LIMIT)
    print(f"Box lift: net_lift={net_lift*1000:.1f}mm drift={drift*1000:.3f}mm "
          f"L={forces['l']:.3f}N R={forces['r']:.3f}N max|qacc|={max_qacc:.3f}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_drive_while_held(model):
    data, rig, q_r, q_l, reference, ok, net_lift, max_qacc = _prepare_lifted_box(model, seed=40)
    if not ok:
        print("Box drive: setup failed")
        return False
    drive = base_teleop.SwerveDrive()
    x0 = float(data.qpos[rig["base_x_qadr"]])
    q_r, q_l, qacc = _frame_hold_pair(
        model, data, rig, q_r, q_l, BOX_DRIVE_TIME, reference=reference,
        drive=drive, drive_keys={"s": True})
    max_qacc = max(max_qacc, qacc)
    q_r, q_l, qacc = _frame_hold_pair(
        model, data, rig, q_r, q_l, 0.5, reference=reference, drive=drive, drive_keys={})
    max_qacc = max(max_qacc, qacc)
    dx = float(data.qpos[rig["base_x_qadr"]] - x0)
    pos_err, _ = bimanual_constraint.relative_pose_error(data, rig["site_r"], rig["site_l"], reference)
    drift = float(np.linalg.norm(pos_err))
    held = grasp.is_box_held(model, data, min_force_per_hand=BOX_FORCE_RANGE[0])
    ok = (held and abs(dx) >= BOX_MIN_DRIVE_DISTANCE and net_lift >= BOX_MIN_NET_LIFT
          and drift < BOX_MAX_REL_DRIFT and max_qacc < QACC_LIMIT)
    print(f"Box drive: base_dx={dx:.3f}m drift={drift*1000:.3f}mm "
          f"held={held} max|qacc|={max_qacc:.3f}: {'OK' if ok else 'FAIL'}")
    return ok


def run_manual_pose_edit_gate():
    class FakeApp:
        scenario = "box"
        box_tracking = True

    app = FakeApp()
    teleop_ui._note_manual_pose_edit(app)
    box_ok = not app.box_tracking

    app.scenario = "can"
    app.box_tracking = True
    teleop_ui._note_manual_pose_edit(app)
    can_ok = app.box_tracking
    ok = box_ok and can_ok
    print(f"Manual pose edit gate: box_auto_align_disabled={box_ok} "
          f"can_unchanged={can_ok}: {'OK' if ok else 'FAIL'}")
    return ok


def run_cyclo_marker_jog_gate():
    class FakeApp:
        scenario = "box"
        box_tracking = True
        arm_mode = {"l": "ik", "r": "ik"}
        cyclo_grasp_captured = False
        targets = {
            "pos_l": [0.30, 0.10, 0.80],
            "pos_r": [0.30, -0.10, 0.80],
            "rpy_l": [0.0, 0.0, 0.0],
            "rpy_r": [0.0, 0.0, 0.0],
            "virtual_object_pos": [0.30, 0.0, 0.85],
            "virtual_object_rpy": [0.0, 0.0, 0.0],
        }

        def apply_virtual_object_target(self):
            return None

    app = FakeApp()
    teleop_ui._apply_cartesian_jog(
        app, "both", pos_delta=(0.005, -0.010, 0.015), rpy_delta=(1.0, -2.0, 3.0))
    both_ok = (
        np.allclose(app.targets["pos_l"], [0.305, 0.090, 0.815])
        and np.allclose(app.targets["pos_r"], [0.305, -0.110, 0.815])
        and np.allclose(app.targets["rpy_l"], [1.0, -2.0, 3.0])
        and np.allclose(app.targets["rpy_r"], [1.0, -2.0, 3.0])
        and not app.box_tracking
    )

    app.arm_mode["l"] = "fk"
    teleop_ui._apply_cartesian_jog(app, "both", pos_delta=(1.0, 1.0, 1.0),
                                   rpy_delta=(100.0, 100.0, 100.0))
    fk_skip_and_clamp_ok = (
        np.allclose(app.targets["pos_l"], [0.305, 0.090, 0.815])
        and np.allclose(app.targets["rpy_l"], [1.0, -2.0, 3.0])
        and np.allclose(app.targets["pos_r"], [1.2, 0.890, 1.2])
        and np.allclose(app.targets["rpy_r"], [90.0, 90.0, 90.0])
    )
    ok = both_ok and fk_skip_and_clamp_ok
    print(f"Cyclo marker jog gate: both_updates={both_ok} "
          f"fk_skip_and_clamp={fk_skip_and_clamp_ok}: {'OK' if ok else 'FAIL'}")
    return ok


def run_cyclo_bimanual_virtual_object_gate():
    app = _make_sim_only_app("box")
    app.targets["pos_r"] = [0.34, -0.08, 0.88]
    app.targets["pos_l"] = [0.34, 0.08, 0.88]
    app.targets["rpy_r"] = [0.0, 0.0, 0.0]
    app.targets["rpy_l"] = [0.0, 0.0, 0.0]
    r0 = app._local_to_world_pos(app.targets["pos_r"])
    l0 = app._local_to_world_pos(app.targets["pos_l"])

    app.capture_grasp()
    capture_ok = (app.cyclo_grasp_captured and app.cyclo_controller == "bimanual_movel"
                  and app.box_grab and not app.box_tracking)
    rel0 = l0 - r0
    app.targets["virtual_object_pos"][0] += 0.025
    app.targets["virtual_object_pos"][2] += 0.060
    app.targets["virtual_object_rpy"][2] += 12.0
    app.apply_virtual_object_target()
    r1 = app._local_to_world_pos(app.targets["pos_r"])
    l1 = app._local_to_world_pos(app.targets["pos_l"])
    rel1 = l1 - r1
    rel_len_ok = abs(np.linalg.norm(rel1) - np.linalg.norm(rel0)) < 1e-9
    moved_ok = np.linalg.norm(0.5 * (r1 + l1) - 0.5 * (r0 + l0)) > 0.05

    app._sync_ik_mocaps_from_targets()
    vo_pos = app._local_to_world_pos(app.targets["virtual_object_pos"])
    marker_err = float(np.linalg.norm(app.data.mocap_pos[app.virtual_object_mocap_id] - vo_pos))

    app.release_grasp()
    release_ok = (not app.cyclo_grasp_captured and app.cyclo_capture_offsets is None
                  and not app.constraint_active and not app.box_grab and app.box_tracking)
    ok = capture_ok and rel_len_ok and moved_ok and marker_err < 1e-9 and release_ok
    print(f"Cyclo bimanual virtual object gate: capture={capture_ok} "
          f"rel_len={rel_len_ok} moved={moved_ok} marker_err={marker_err*1000:.6f}mm "
          f"release={release_ok}: {'OK' if ok else 'FAIL'}")
    return ok


def run_cyclo_3d_gizmo_pose_gate():
    app = _make_sim_only_app("box")
    world_pos = np.array([0.42, -0.11, 0.94])
    world_quat = _rpy_deg_to_quat([13.0, -8.0, 21.0])
    matrix = app._pose_to_imguizmo_matrix(world_pos, world_quat)
    round_pos, round_quat = app._imguizmo_matrix_to_pose(matrix)
    roundtrip_ok = (
        np.linalg.norm(round_pos - world_pos) < 1e-7
        and abs(abs(float(np.dot(round_quat, world_quat))) - 1.0) < 1e-7
    )

    app._set_gizmo_target_world_pose("r", world_pos, world_quat)
    hand_pos = app._local_to_world_pos(app.targets["pos_r"])
    hand_quat = app._target_world_quat("r")
    hand_ok = (
        np.linalg.norm(hand_pos - world_pos) < 1e-9
        and abs(abs(float(np.dot(hand_quat, world_quat))) - 1.0) < 1e-9
        and not app.box_tracking
    )

    app.capture_grasp()
    vo_pos = np.array([0.43, 0.02, 0.98])
    vo_quat = _rpy_deg_to_quat([0.0, 0.0, 16.0])
    app._set_gizmo_target_world_pose("virtual", vo_pos, vo_quat)
    new_vo_pos, new_vo_quat = app._virtual_object_world_pose()
    virtual_ok = (
        np.linalg.norm(new_vo_pos - vo_pos) < 1e-9
        and abs(abs(float(np.dot(new_vo_quat, vo_quat))) - 1.0) < 1e-9
        and app.cyclo_grasp_captured
    )
    ok = roundtrip_ok and hand_ok and virtual_ok
    print(f"Cyclo 3D gizmo pose gate: roundtrip={roundtrip_ok} "
          f"hand_target={hand_ok} virtual_target={virtual_ok}: {'OK' if ok else 'FAIL'}")
    return ok


def run_marker_control_removed_gate():
    app_source = pathlib.Path(teleop_app.__file__).read_text(encoding="utf-8")
    forbidden = (
        "_handle_ik_target_mouse",
        "_select_ik_target",
        "_set_target_from_world_pose",
        "ik_dragging_side",
        "ik_selected_side",
        "mjv_" + "movePerturb",
        "mjv_" + "select",
    )
    found = [token for token in forbidden if token in app_source]
    ok = not found
    print(f"Display marker gate: removed_mouse_target_control={ok} "
          f"forbidden_found={found}: {'OK' if ok else 'FAIL'}")
    return ok


def _make_sim_only_app(initial_scenario="box"):
    app = teleop_app.TeleopApp.__new__(teleop_app.TeleopApp)
    app.initial_scenario = initial_scenario
    app._setup_sim()
    return app


def run_numeric_target_marker_sync_gate():
    app = _make_sim_only_app("box")
    app.data.qpos[app.base_x_qadr] = 0.12
    app.data.qpos[app.base_y_qadr] = -0.04
    app.data.qpos[app.base_yaw_qadr] = np.radians(17.0)
    mujoco.mj_forward(app.model, app.data)

    app.targets["pos_r"] = [0.34, -0.13, 0.92]
    app.targets["rpy_r"] = [11.0, -7.0, 5.0]
    app.targets["pos_l"] = [0.31, 0.14, 0.91]
    app.targets["rpy_l"] = [-9.0, -6.0, -4.0]

    for side, mocap_id in app.ik_target_mocap_ids.items():
        app.data.mocap_pos[mocap_id] = [9.0, 9.0, 9.0]
        app.data.mocap_quat[mocap_id] = [0.0, 1.0, 0.0, 0.0]

    app._sync_ik_mocaps_from_targets()

    ok = True
    reports = []
    for side, mocap_id in app.ik_target_mocap_ids.items():
        expected_pos = app._local_to_world_pos(app.targets[f"pos_{side}"])
        expected_quat = app._target_world_quat(side)
        pos_err = float(np.linalg.norm(app.data.mocap_pos[mocap_id] - expected_pos))
        quat_dot = abs(float(np.dot(app.data.mocap_quat[mocap_id], expected_quat)))
        case_ok = pos_err < 1e-9 and (1.0 - quat_dot) < 1e-9
        ok = ok and case_ok
        reports.append(f"{side}: pos_err={pos_err*1000:.6f}mm quat_dot={quat_dot:.12f}")

    print(f"Numeric target -> marker sync gate: {'; '.join(reports)}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_box_tracking_target_math_gate():
    app = _make_sim_only_app("box")
    box_qadr = app.model.jnt_qposadr[app.box_jid]
    yaw = np.radians(23.0)
    app.data.qpos[box_qadr:box_qadr + 7] = [
        BOX_HOME_QPOS[0], BOX_HOME_QPOS[1], BOX_HOME_QPOS[2],
        np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2),
    ]
    app.targets["squeeze_gap"] = BOX_SQUEEZE_GAP
    mujoco.mj_forward(app.model, app.data)

    app._update_box_tracking_targets(app._world_to_base_pos)
    expected_r, expected_l = _box_target_positions_from_qpos(
        app.data.qpos[box_qadr:box_qadr + 7], BOX_SQUEEZE_GAP)
    actual_r = app._local_to_world_pos(app.targets["pos_r"])
    actual_l = app._local_to_world_pos(app.targets["pos_l"])
    err_r = float(np.linalg.norm(actual_r - expected_r))
    err_l = float(np.linalg.norm(actual_l - expected_l))
    ok = err_r < 1e-9 and err_l < 1e-9
    print(f"Box auto-align target math gate: R_err={err_r*1000:.6f}mm "
          f"L_err={err_l*1000:.6f}mm: {'OK' if ok else 'FAIL'}")
    return ok


def run_manual_xyz_rpy_ik_gate(model):
    data = mujoco.MjData(model)
    _box_setup(model, data)
    rig = _make_rig(model)
    cases = (
        ("r", rig["solver_r"], rig["site_r"], HOME_Q_R,
         np.array([-0.035, -0.015, 0.025]), np.array([8.0, -4.0, 6.0])),
        ("l", rig["solver_l"], rig["site_l"], HOME_Q_L,
         np.array([-0.035, 0.015, 0.025]), np.array([-8.0, -4.0, -6.0])),
    )
    ok = True
    reports = []
    for side, solver, site_id, q_seed, pos_delta, rpy_delta in cases:
        target_pos = data.site_xpos[site_id].copy() + pos_delta
        home_quat = np.zeros(4)
        target_quat = np.zeros(4)
        mujoco.mju_mat2Quat(home_quat, data.site_xmat[site_id])
        mujoco.mju_mulQuat(target_quat, home_quat, _rpy_deg_to_quat(rpy_delta))
        q_sol, pos_err, ori_err, converged = solver.solve_pose_multistart(
            q_seed, target_pos, target_quat, np.random.default_rng(100 + (side == "l")),
            context_qpos=data.qpos, success_pos_tol=0.005, success_ori_tol=np.radians(5.0))
        scratch = mujoco.MjData(model)
        scratch.qpos[:] = data.qpos
        for qadr, val in zip(solver.qpos_adrs, q_sol):
            scratch.qpos[qadr] = val
        mujoco.mj_forward(model, scratch)
        reached_delta = scratch.site_xpos[site_id] - data.site_xpos[site_id]
        pos_delta_ok = np.linalg.norm(reached_delta - pos_delta) < 0.006
        case_ok = converged and pos_delta_ok and pos_err < 0.005 and ori_err < np.radians(5.0)
        ok = ok and case_ok
        reports.append(
            f"{side}: converged={converged} pos_err={pos_err*1000:.2f}mm "
            f"ori_err={np.degrees(ori_err):.2f}deg delta_ok={pos_delta_ok}")
    print(f"Manual XYZ/RPY IK gate: {'; '.join(reports)}: {'OK' if ok else 'FAIL'}")
    return ok


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    ok = (run_model_gate(model) and run_box_idle(model) and run_box_drop(model)
          and run_constraint_projection(model) and run_box_pregrasp_ik(model)
          and run_squeeze_stability(model) and run_scripted_lift(model)
          and run_drive_while_held(model) and run_manual_pose_edit_gate()
          and run_cyclo_marker_jog_gate()
          and run_cyclo_bimanual_virtual_object_gate()
          and run_cyclo_3d_gizmo_pose_gate()
          and run_marker_control_removed_gate() and run_numeric_target_marker_sync_gate()
          and run_box_tracking_target_math_gate()
          and run_manual_xyz_rpy_ik_gate(model))
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
