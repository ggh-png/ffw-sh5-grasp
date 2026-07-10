"""Small MuJoCo model-introspection helpers shared across the teleop modules.

Extracted because the same "which actuator drives this joint" linear scan used to be
copy-pasted independently in grasp.py and arm_control.py -- MuJoCo doesn't expose a
joint-to-actuator reverse lookup directly, only actuator-to-joint (`actuator_trnid`), so
both call sites had to walk every actuator and compare. One canonical implementation here
means there's only one place to fix if the transmission-matching logic ever needs to change.
"""

import mujoco


def find_actuator_for_joint(model, joint_id):
    """Return the id of the actuator that drives `joint_id` via a direct joint
    transmission (`mjTRN_JOINT`), or None if no such actuator exists in this model.

    (한글) 이 관절을 구동하는 actuator id를 찾는다 -- MuJoCo는 actuator->joint 매핑
    (`actuator_trnid`)만 제공하고 그 반대는 없어서 전체 actuator를 선형 탐색해야 한다.
    grasp.py/arm_control.py 둘 다 이 탐색이 필요해서 공용 함수로 뺐다.
    """
    for aid in range(model.nu):
        if (model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT
                and model.actuator_trnid[aid, 0] == joint_id):
            return aid
    return None
