"""Damped least-squares inverse kinematics for a MuJoCo site.

The solver evaluates position and orientation in a private scratch ``MjData`` and returns
joint targets.  It never mutates live simulation state; callers apply the result through
the robot's actuators.
"""

import mujoco
import numpy as np

import kinematics

DEFAULT_DAMPING = 0.05
DEFAULT_MAX_JOINT_DELTA = 0.05  # rad per solver iteration
DEFAULT_MAX_ITER = 50
POS_TOL = 1e-4
ORI_TOL = 1e-3


class InverseKinematics:
    """Find joint angles that satisfy a target site position or pose."""

    def __init__(self, model, site_name, joint_names, damping=DEFAULT_DAMPING,
                 max_joint_delta=DEFAULT_MAX_JOINT_DELTA):
        self.model = model
        self.kinematics = kinematics.KinematicsSolver(model, site_name, joint_names)
        self.site_id = self.kinematics.site_id
        self.joint_ids = self.kinematics.joint_ids.tolist()
        # dof_ids: qvel/Jacobian 열 인덱스용. qpos_adrs: qpos 배열 인덱스용 --
        # 힌지 관절은 둘이 같지만, 자유도(quaternion 등)가 섞인 관절에서는 다를 수 있어
        # 이 프로젝트에서는 항상 명시적으로 구분해서 쓴다.
        self.dof_ids = self.kinematics.dof_ids.tolist()
        self.qpos_adrs = self.kinematics.qpos_adrs.tolist()
        self.joint_ranges = self.kinematics.joint_ranges.copy()
        self.damping = damping
        self.max_joint_delta = max_joint_delta
        self.n = len(joint_names)
        # Scratch kinematics-only buffer -- never the live simulation's MjData.
        # (한글) 이 solver 전용 임시 데이터 -- 실시간 시뮬레이션의 data와는 완전히 별개.
        self._scratch = self.kinematics.data

    def forward_kinematics(self, q, context_qpos=None):
        """Return normalized world pose + world-aligned Jacobian for ``q``.

        This mirrors Cyclo Control's separate ``computePose``/``computeJacobian`` boundary,
        while returning both from one MuJoCo forward pass.  The evaluation uses the private
        scratch state and therefore cannot kinematically override the live robot.
        """
        return self.kinematics.forward(q, context_qpos)

    def _initialize_scratch(self, q, context_qpos=None):
        """Seed and forward the shared scratch kinematics state."""
        self.kinematics.update(q, context_qpos)
        return self._scratch

    def _write_q(self, scratch, q):
        """관절각 배열을 scratch의 해당 qpos 슬롯에 써넣는다(역시 scratch에만 씀)."""
        scratch.qpos[self.qpos_adrs] = q

    def _clamp_to_limits(self, q):
        """관절 range를 벗어나지 않도록 클램프 -- 매 반복(iteration)마다 호출된다."""
        return np.clip(q, self.joint_ranges[:, 0], self.joint_ranges[:, 1])

    def _jac(self, scratch):
        """목표 site의 위치 Jacobian(jacp)과 회전 Jacobian(jacr)을 계산하고,
        이 솔버가 담당하는 관절의 열(column)만 잘라서 반환한다."""
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, scratch, jacp, jacr, self.site_id)
        cols = self.dof_ids
        return jacp[:, cols], jacr[:, cols]

    def _pose_error(self, scratch, target_pos, target_quat):
        """Return world-frame position and shortest-path orientation errors."""
        cur_pos = scratch.site_xpos[self.site_id]
        pos_err = target_pos - cur_pos
        cur_quat = np.zeros(4)
        mujoco.mju_mat2Quat(cur_quat, scratch.site_xmat[self.site_id])
        ori_err = kinematics.shortest_orientation_error(target_quat, cur_quat)
        return pos_err, ori_err

    def solve_pose(self, q_init, target_pos, target_quat, max_iter=DEFAULT_MAX_ITER,
                   pos_tol=POS_TOL, ori_tol=ORI_TOL, ori_weight=0.3, context_qpos=None):
        """Hierarchical 6DOF DLS: position (3) task-priority over orientation (3), with a
        backtracking line search that prefers a step which actually reduces error.

        A naive stacked-Jacobian 6D DLS solve (position+orientation errors in one
        least-squares problem) diverges whenever the orientation gap is large relative to
        the damping -- fixing orientation yanks position off target and vice versa. The
        hierarchical fix alone (position DLS + orientation corrected only through the
        position Jacobian's null space) still oscillated on larger gaps: running it *longer*
        made results worse, the signature of overshoot rather than slow convergence. So each
        iteration:
          1. dq_pos  = Jp^T (Jp Jp^T + lam^2 I)^-1 e_pos
          2. g_ori   = Jr^T e_ori_world   (shortest quaternion error, already world-aligned)
          3. dq_ori  = g_ori - Jp^T (Jp Jp^T + lam^2 I)^-1 (Jp g_ori)   (null-space projected)
          4. dq = dq_pos + dq_ori, clamped
          5. backtrack: halve dq (up to 6x) until pos_err + ori_weight*ori_err actually drops;
             if none improves it, take the least-cost tried candidate (not a monotonic guarantee)
        context_qpos seeds every joint this solver doesn't itself control (for example,
        models/full_scene.xml's lift_joint) so the site's world position reflects the real
        simulation state rather than resetting upstream joints to 0.

        Returns (q_solution, pos_err_norm, ori_err_norm_rad).
        """
        q = np.array(q_init, dtype=float)
        scratch = self._initialize_scratch(q, context_qpos)

        pos_err, ori_err = self._pose_error(scratch, target_pos, target_quat)
        pos_err_norm = float(np.linalg.norm(pos_err))
        ori_err_norm = float(np.linalg.norm(ori_err))
        lam2 = self.damping ** 2

        for _ in range(max_iter):
            if pos_err_norm < pos_tol and ori_err_norm < ori_tol:
                break

            jacp, jacr = self._jac(scratch)
            position_system = jacp @ jacp.T + lam2 * np.eye(3)

            # 1) 위치 오차를 DLS로 우선 풀고
            dq_pos = jacp.T @ np.linalg.solve(position_system, pos_err)
            # 2) 방향 보정은 위치 Jacobian의 영공간(null space)에 투영 -- 위치에
            #    영향을 주지 않는 성분만 반영해서, 방향을 맞추려다 위치가 흔들리는
            #    문제를 없앤다(계층형/task-priority IK).
            g_ori = jacr.T @ ori_err
            projected_gradient = np.linalg.solve(position_system, jacp @ g_ori)
            dq_ori = g_ori - jacp.T @ projected_gradient
            dq_full = np.clip(dq_pos + dq_ori, -self.max_joint_delta, self.max_joint_delta)

            # 3) backtracking line search: 전체 스텝을 그대로 쓰지 않고, 실제로
            #    비용(위치+가중 방향 오차)이 줄어드는지 확인하면서 스텝을 절반씩
            #    줄여나간다 -- 오차가 클 때 반복할수록 진동/발산하던 문제를 막는다.
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
                               max_iter=250, success_pos_tol=0.005, success_ori_tol=np.radians(5),
                               context_qpos=None):
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
        # 첫 시도는 q_init(보통 이전 프레임의 해, 값싸고 대개 좋은 시작점) 그대로,
        # 이후는 관절 range 안에서 무작위로 뽑은 후보들 -- 국소해/lockup에 빠졌을 때
        # 다른 자세에서 다시 시도해볼 기회를 준다.
        candidates = [np.array(q_init, dtype=float)]
        candidates.extend(
            rng.uniform(self.joint_ranges[:, 0], self.joint_ranges[:, 1])
            for _ in range(n_restarts)
        )
        for q0 in candidates:
            q, pe, oe = self.solve_pose(q0, target_pos, target_quat, max_iter=max_iter,
                                        context_qpos=context_qpos)
            if pe < success_pos_tol and oe < success_ori_tol:
                return q, pe, oe, True
            # 전부 실패해도 가장 오차가 작았던 시도는 기억해뒀다가 반환한다.
            if best is None or pe + oe < best[1] + best[2]:
                best = (q, pe, oe)
        return best[0], best[1], best[2], False
