"""Bimanual rigid-grasp projection for Phase 6 box lifting.

The teleop app still solves each arm with the existing IK module. This file only projects
the two arms' desired joint deltas onto the manifold that keeps the left grasp site rigidly
attached to the right grasp site, then adds a small drift correction toward the relative
pose captured at the instant the box is first held.
"""

import mujoco
import numpy as np


def _skew(v):
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _site_quat(data, site_id):
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, data.site_xmat[site_id])
    return quat


def snapshot_relative_pose(data, right_site_id, left_site_id):
    """Capture left site's pose in the right site's current frame."""
    r_pos = data.site_xpos[right_site_id].copy()
    l_pos = data.site_xpos[left_site_id].copy()
    r_mat = data.site_xmat[right_site_id].reshape(3, 3).copy()
    l_mat = data.site_xmat[left_site_id].reshape(3, 3).copy()
    return {
        "pos_r": r_mat.T @ (l_pos - r_pos),
        "mat_r": r_mat.T @ l_mat,
    }


def relative_pose_error(data, right_site_id, left_site_id, reference):
    """Return (position_error_world, orientation_error_world) from current to reference."""
    r_pos = data.site_xpos[right_site_id]
    l_pos = data.site_xpos[left_site_id]
    r_mat = data.site_xmat[right_site_id].reshape(3, 3)
    desired_l_pos = r_pos + r_mat @ reference["pos_r"]
    desired_l_mat = r_mat @ reference["mat_r"]

    desired_l_quat = np.zeros(4)
    mujoco.mju_mat2Quat(desired_l_quat, desired_l_mat.reshape(9))
    current_l_quat = _site_quat(data, left_site_id)
    ori_err_local = np.zeros(3)
    mujoco.mju_subQuat(ori_err_local, desired_l_quat, current_l_quat)
    l_mat = data.site_xmat[left_site_id].reshape(3, 3)
    return l_pos - desired_l_pos, l_mat @ ori_err_local


def project_desired_delta(model, data, right_site_id, left_site_id, right_dof_ids,
                          left_dof_ids, dq_r, dq_l, dt, reference=None,
                          drift_gain=0.2, damping=1e-8):
    """Project desired arm deltas so the two grasp sites keep a rigid relative pose.

    The input and return convention is `(dq_r, dq_l)`, each 7 elements. Jacobians are built
    from the live MuJoCo state; the caller decides how those projected deltas are folded
    back into its IK target state.
    """
    jacp_r = np.zeros((3, model.nv))
    jacr_r = np.zeros((3, model.nv))
    jacp_l = np.zeros((3, model.nv))
    jacr_l = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp_r, jacr_r, right_site_id)
    mujoco.mj_jacSite(model, data, jacp_l, jacr_l, left_site_id)

    j_r = np.vstack([jacp_r[:, right_dof_ids], jacr_r[:, right_dof_ids]])
    j_l = np.vstack([jacp_l[:, left_dof_ids], jacr_l[:, left_dof_ids]])
    r_to_l = data.site_xpos[left_site_id] - data.site_xpos[right_site_id]
    transform = np.eye(6)
    transform[0:3, 3:6] = -_skew(r_to_l)
    j_grasp = np.hstack([-transform @ j_r, j_l])

    dq = np.concatenate([dq_r, dq_l])
    desired_rel_vel = np.zeros(6)
    if reference is not None and dt > 0.0:
        pos_err, ori_err = relative_pose_error(data, right_site_id, left_site_id, reference)
        desired_rel_vel[:3] = -drift_gain * pos_err / dt
        desired_rel_vel[3:] = drift_gain * ori_err / dt

    residual = j_grasp @ dq - desired_rel_vel
    gram = j_grasp @ j_grasp.T + damping * np.eye(6)
    correction = j_grasp.T @ np.linalg.solve(gram, residual)
    projected = dq - correction
    return projected[:len(dq_r)], projected[len(dq_r):]
