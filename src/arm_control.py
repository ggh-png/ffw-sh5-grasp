"""Phase 3 -- arm torque control: software PD + gravity/Coriolis feedforward.

Diagnosed in NOTES.md "Phase 3": MuJoCo's built-in <position> actuator is a bare
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


class ArmTorqueController:
    def __init__(self, model, joint_names, kp=600.0, kd=40.0):
        self.model = model
        self.joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in joint_names]
        self.qpos_adrs = [model.jnt_qposadr[j] for j in self.joint_ids]
        self.dof_ids = [model.jnt_dofadr[j] for j in self.joint_ids]
        self.actuator_ids = []
        for n in joint_names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
            aid = None
            for a in range(model.nu):
                if model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT and model.actuator_trnid[a, 0] == jid:
                    aid = a
                    break
            if aid is None:
                raise ValueError(f"no motor actuator found for joint {n}")
            self.actuator_ids.append(aid)
        self.kp = kp
        self.kd = kd

    def apply(self, data, q_des):
        """Compute and write torques for this step. Reads data.qpos/qvel/qfrc_bias (state
        feedback), writes only data.ctrl -- never data.qpos."""
        q = np.array([data.qpos[a] for a in self.qpos_adrs])
        qd = data.qvel[self.dof_ids]
        qfrc_bias = data.qfrc_bias[self.dof_ids]
        tau = qfrc_bias + self.kp * (np.asarray(q_des) - q) - self.kd * qd
        for aid, t in zip(self.actuator_ids, tau):
            lo, hi = self.model.actuator_ctrlrange[aid]
            data.ctrl[aid] = np.clip(t, lo, hi)
        return tau
