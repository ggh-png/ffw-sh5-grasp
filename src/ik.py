"""Phase 3 -- 6DOF damped least-squares (DLS) IK for the right arm.

Targets a MuJoCo *site* (`grasp_target`, defined on hx5_r_base in models/arm_hand.xml at
the exact palm-relative offset validated in Phase 1/2) rather than a body origin, so
`mj_jacSite` gives the position+rotation Jacobian of that offset point directly -- no manual
offset correction needed the way a raw `mj_jacBody` would require.

Developed in the two stages PLAN.md asks for:
  1. `solve_position` -- position-only, 3DOF, verify convergence first.
  2. `solve_pose` -- adds orientation on top of the same DLS core.

**Kinematic override boundary**: this solver never touches the live simulation's `data`. It
iterates on its own scratch `mujoco.MjData` (kinematics only -- `mj_forward`, no contacts,
no dynamics), seeded from a starting joint configuration, and returns the solved joint
angles as a plain array. The caller is responsible for feeding that array into
`data.ctrl[...]` (position actuator targets) on the real simulation, never `data.qpos[...]`
-- the actual arm motion is then produced by the physics-driven position actuators, exactly
like every other actuated joint in this project.
"""

import mujoco
import numpy as np

DEFAULT_DAMPING = 0.05
DEFAULT_MAX_JOINT_DELTA = 0.05  # rad per solver iteration
DEFAULT_MAX_ITER = 50
POS_TOL = 1e-4
ORI_TOL = 1e-3


class InverseKinematics:
    def __init__(self, model, site_name, joint_names, damping=DEFAULT_DAMPING,
                 max_joint_delta=DEFAULT_MAX_JOINT_DELTA):
        self.model = model
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        self.joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in joint_names]
        self.dof_ids = [model.jnt_dofadr[j] for j in self.joint_ids]
        self.qpos_adrs = [model.jnt_qposadr[j] for j in self.joint_ids]
        self.joint_ranges = np.array([model.jnt_range[j] for j in self.joint_ids])
        self.damping = damping
        self.max_joint_delta = max_joint_delta
        self.n = len(joint_names)
        # Scratch kinematics-only buffer -- never the live simulation's MjData.
        self._scratch = mujoco.MjData(model)

    def _read_q(self, scratch):
        return np.array([scratch.qpos[a] for a in self.qpos_adrs])

    def _write_q(self, scratch, q):
        for qadr, val in zip(self.qpos_adrs, q):
            scratch.qpos[qadr] = val

    def _clamp_to_limits(self, q):
        return np.clip(q, self.joint_ranges[:, 0], self.joint_ranges[:, 1])

    def _jac(self, scratch):
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, scratch, jacp, jacr, self.site_id)
        cols = self.dof_ids
        return jacp[:, cols], jacr[:, cols]

    def _dls_step(self, J, err):
        lam2 = self.damping ** 2
        JJt = J @ J.T + lam2 * np.eye(J.shape[0])
        return J.T @ np.linalg.solve(JJt, err)

    def solve_position(self, q_init, target_pos, max_iter=DEFAULT_MAX_ITER, tol=POS_TOL):
        """Position-only 3DOF DLS starting from q_init. Returns (q_solution, pos_err_norm)."""
        scratch = self._scratch
        mujoco.mj_resetData(self.model, scratch)
        q = np.array(q_init, dtype=float)
        self._write_q(scratch, q)
        mujoco.mj_forward(self.model, scratch)

        err_norm = np.inf
        for _ in range(max_iter):
            cur_pos = scratch.site_xpos[self.site_id]
            err = target_pos - cur_pos
            err_norm = float(np.linalg.norm(err))
            if err_norm < tol:
                break
            jacp, _ = self._jac(scratch)
            dq = np.clip(self._dls_step(jacp, err), -self.max_joint_delta, self.max_joint_delta)
            q = self._clamp_to_limits(q + dq)
            self._write_q(scratch, q)
            mujoco.mj_forward(self.model, scratch)
        return q, err_norm

    def _pose_error(self, scratch, target_pos, target_quat):
        cur_pos = scratch.site_xpos[self.site_id]
        pos_err = target_pos - cur_pos
        cur_quat = np.zeros(4)
        mujoco.mju_mat2Quat(cur_quat, scratch.site_xmat[self.site_id])
        ori_err = np.zeros(3)
        mujoco.mju_subQuat(ori_err, target_quat, cur_quat)
        return pos_err, ori_err

    def solve_pose(self, q_init, target_pos, target_quat, max_iter=DEFAULT_MAX_ITER,
                   pos_tol=POS_TOL, ori_tol=ORI_TOL, ori_weight=0.3):
        """Hierarchical 6DOF DLS: position (3) task-priority over orientation (3), with a
        backtracking line search so every accepted step actually reduces error.

        A naive stacked-Jacobian 6D DLS solve (position+orientation errors in one
        least-squares problem) diverges whenever the orientation gap is large relative to
        the damping -- fixing orientation yanks position off target and vice versa. The
        hierarchical fix alone (position DLS + orientation corrected only through the
        position Jacobian's null space) still oscillated on larger gaps: running it *longer*
        made results worse, the signature of overshoot rather than slow convergence. So each
        iteration:
          1. dq_pos  = Jp^T (Jp Jp^T + lam^2 I)^-1 e_pos
          2. g_ori   = Jr^T e_ori_world   (e_ori rotated site-local -> world first)
          3. dq_ori  = g_ori - Jp^T (Jp Jp^T + lam^2 I)^-1 (Jp g_ori)   (null-space projected)
          4. dq = dq_pos + dq_ori, clamped
          5. backtrack: halve dq (up to 6x) until pos_err + ori_weight*ori_err actually drops;
             if it never does, take the smallest tried step (guarantees monotonic progress)
        Returns (q_solution, pos_err_norm, ori_err_norm_rad).
        """
        scratch = self._scratch
        mujoco.mj_resetData(self.model, scratch)
        q = np.array(q_init, dtype=float)
        self._write_q(scratch, q)
        mujoco.mj_forward(self.model, scratch)

        pos_err, ori_err = self._pose_error(scratch, target_pos, target_quat)
        pos_err_norm = float(np.linalg.norm(pos_err))
        ori_err_norm = float(np.linalg.norm(ori_err))
        lam2 = self.damping ** 2

        for _ in range(max_iter):
            if pos_err_norm < pos_tol and ori_err_norm < ori_tol:
                break

            site_R = scratch.site_xmat[self.site_id].reshape(3, 3)
            ori_err_world = site_R @ ori_err

            jacp, jacr = self._jac(scratch)
            JJt_inv = np.linalg.inv(jacp @ jacp.T + lam2 * np.eye(3))

            dq_pos = jacp.T @ (JJt_inv @ pos_err)
            g_ori = jacr.T @ ori_err_world
            dq_ori = g_ori - jacp.T @ (JJt_inv @ (jacp @ g_ori))
            dq_full = np.clip(dq_pos + dq_ori, -self.max_joint_delta, self.max_joint_delta)

            cur_cost = pos_err_norm + ori_weight * ori_err_norm
            best = None
            step = 1.0
            for _ in range(6):
                q_try = self._clamp_to_limits(q + step * dq_full)
                self._write_q(scratch, q_try)
                mujoco.mj_forward(self.model, scratch)
                pe, oe = self._pose_error(scratch, target_pos, target_quat)
                pen, oen = float(np.linalg.norm(pe)), float(np.linalg.norm(oe))
                cost = pen + ori_weight * oen
                if best is None or cost < best[0]:
                    best = (cost, q_try, pe, oe, pen, oen)
                if cost < cur_cost:
                    break
                step *= 0.5

            _, q, pos_err, ori_err, pos_err_norm, ori_err_norm = best
            self._write_q(scratch, q)
            mujoco.mj_forward(self.model, scratch)
        return q, pos_err_norm, ori_err_norm

    def solve_pose_multistart(self, q_init, target_pos, target_quat, rng, n_restarts=8,
                               max_iter=250, success_pos_tol=0.005, success_ori_tol=np.radians(5)):
        """solve_pose can still land in a local minimum/joint-limit lockup for a large gap.
        Try q_init first (cheap, and it's usually a good guess -- e.g. the previous frame's
        solution during continuous teleop tracking); if that doesn't converge, retry from a
        few random joint configurations and keep whichever converges (or the least-bad
        attempt, if none do). success_pos_tol/success_ori_tol are the caller's acceptance
        thresholds (e.g. the IK unit test's 5mm/5deg), independent of the solver's own
        internal step-termination tolerances. Returns (q_solution, pos_err_norm,
        ori_err_norm_rad, converged).
        """
        best = None
        candidates = [np.array(q_init, dtype=float)]
        candidates += [rng.uniform(self.joint_ranges[:, 0], self.joint_ranges[:, 1]) for _ in range(n_restarts)]
        for q0 in candidates:
            q, pe, oe = self.solve_pose(q0, target_pos, target_quat, max_iter=max_iter)
            if pe < success_pos_tol and oe < success_ori_tol:
                return q, pe, oe, True
            if best is None or pe + oe < best[1] + best[2]:
                best = (q, pe, oe)
        return best[0], best[1], best[2], False
