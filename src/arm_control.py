"""Phase 3 -- arm torque control: software PD + gravity/Coriolis feedforward.

Diagnosed in Phase 3: MuJoCo's built-in <position> actuator is a bare
proportional term with no integral and no feedforward. Holding any static arm pose against
its own required torque left a residual site error of ~15-20mm. Three candidate causes were
tested and ruled out in order:
  1. Teleport test (write q_grasp directly into a fresh MjData, mj_forward once, no
     stepping): site error was 0.004mm. IK targets and measures the same point correctly.
  2. Per-joint actuator_force vs forcerange at the settled state: no joint was saturated
     (worst case ~11.5 of 31.7 N*m headroom).
  3. ctrl clamping: every sent value matched data.ctrl exactly; no joint was pinned at a
     limit.
A 60s settle test then showed the error slowly decaying (not stuck at a fixed point) with a
multi-second time constant -- the signature of the coupled 7-link chain not actually being
critically damped by <position>'s per-joint `dampratio=1` (which assumes each joint is an
independent SISO system and ignores inertial coupling between links). 5x'ing kp barely
helped because the slow mode isn't primarily kp-limited.

This replaces the arm's <position> actuators with <motor> (pure torque) actuators, driven
every physics step by the standard robot-arm control law (matching the unitree_mujoco
reference the user pointed at):

    tau = qfrc_bias[joint]      (feedforward: exactly cancels gravity/Coriolis/centrifugal
                                  for the CURRENT state -- this is what removes the
                                  steady-state error, not more proportional gain)
        + kp * (q_des - q)      (position feedback)
        - kd * qvel             (velocity feedback / active damping, tuned for the coupled
                                  system instead of assumed per-joint)

The hand keeps its original force-limited <position> actuators -- the compliant, torque
saturates-on-contact grasp behavior there is intentional and already validated in Phase 1/2;
this module only concerns the rigid-body arm positioning problem.
"""

import mujoco
import numpy as np

import mj_util


class ArmTorqueController:
    """팔 관절을 <motor>(순수 토크) 액추에이터로 구동하는 PD + 중력/코리올리
    feedforward 제어기. MuJoCo의 내장 <position> 액추에이터가 정적 하중에서 남기는
    비례오차(모듈 docstring 참고)를 없애기 위해, 매 스텝 직접 토크를 계산해서 쓴다.
    """

    def __init__(self, model, joint_names, kp=600.0, kd=40.0):
        self.model = model
        self.joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in joint_names]
        self.qpos_adrs = [model.jnt_qposadr[j] for j in self.joint_ids]
        self.dof_ids = [model.jnt_dofadr[j] for j in self.joint_ids]
        self.actuator_ids = []
        # 관절 이름 -> 그 관절을 구동하는 motor actuator id를 미리 찾아 캐싱
        # (매 스텝 다시 찾지 않도록 __init__에서 한 번만 수행).
        for n in joint_names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
            aid = mj_util.find_actuator_for_joint(model, jid)
            if aid is None:
                raise ValueError(f"no motor actuator found for joint {n}")
            self.actuator_ids.append(aid)
        self.kp = kp
        self.kd = kd

    def apply(self, data, q_des, kp_scale=1.0):
        """Compute and write torques for this step. Reads data.qpos/qvel/qfrc_bias (state
        feedback), writes only data.ctrl -- never data.qpos."""
        # (한글) 매 물리 스텝마다 호출: 현재 관절각/각속도를 읽고, 목표각(q_des)과의
        # 차이를 PD로 보정하되, qfrc_bias(중력+코리올리+원심력)를 더해 "지금 이
        # 자세를 버티는 데 필요한 힘"을 미리 상쇄해준다 -- 이게 정적 처짐을 없애는
        # 핵심이고, kp를 올리는 것과는 다른 얘기다.
        q = np.array([data.qpos[a] for a in self.qpos_adrs])
        qd = data.qvel[self.dof_ids]
        qfrc_bias = data.qfrc_bias[self.dof_ids]
        tau = qfrc_bias + self.kp * kp_scale * (np.asarray(q_des) - q) - self.kd * qd
        # 액추에이터의 ctrlrange(토크 한계)를 넘지 않도록 클램프 후 기록.
        for aid, t in zip(self.actuator_ids, tau):
            lo, hi = self.model.actuator_ctrlrange[aid]
            data.ctrl[aid] = np.clip(t, lo, hi)
        return tau
