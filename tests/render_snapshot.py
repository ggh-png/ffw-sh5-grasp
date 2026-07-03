"""Dev utility (not a Phase test): render an offscreen snapshot of a model for
visual pose verification. Not part of the automated Phase test suite.

Usage: python3 tests/render_snapshot.py models/hand_only.xml /tmp/out.png [--grasp 0.5]
"""

import sys
import pathlib
import mujoco
import numpy as np
from PIL import Image

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent


def main():
    model_path = sys.argv[1]
    out_path = sys.argv[2]
    grasp = 0.0
    kinematic = False
    for a in sys.argv[3:]:
        if a.startswith("--grasp="):
            grasp = float(a.split("=")[1])
        if a == "--kinematic":
            kinematic = True

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    # Curl joints only: thumb (mcp_pitch, ip) + each finger's (pip, dip, tip).
    # Excludes thumb_cmc/mcp_yaw (pre-shape, left at 0) and finger_mcp (spread, left at 0).
    CURL_JOINTS = {"finger_r_joint3", "finger_r_joint4"}
    for base in (5, 9, 13, 17):
        CURL_JOINTS.update({f"finger_r_joint{base+1}", f"finger_r_joint{base+2}", f"finger_r_joint{base+3}"})

    if kinematic:
        # Pure FK pose check: directly set qpos for curl joints to a `grasp` fraction of
        # their range, single mj_forward, no stepping. One-shot authoring/visualization
        # snapshot, not part of the runtime simulator loop -- distinct from the project's
        # ban on kinematic override during simulation.
        for jid in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if name in CURL_JOINTS:
                lo, hi = model.jnt_range[jid]
                qadr = model.jnt_qposadr[jid]
                data.qpos[qadr] = lo + grasp * (hi - lo)
        mujoco.mj_forward(model, data)
    elif grasp > 0:
        for aid in range(model.nu):
            jid = model.actuator_trnid[aid, 0]
            lo, hi = model.jnt_range[jid]
            data.ctrl[aid] = lo + grasp * (hi - lo)
        for _ in range(2000):
            mujoco.mj_step(model, data)

    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=720, width=960)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam)
    cam.lookat = np.array([0.05, 0.0, 0.15])
    cam.distance = 0.5
    cam.azimuth = 140
    cam.elevation = -20

    show_contacts = "--contacts" in sys.argv
    scene_opt = mujoco.MjvOption()
    if show_contacts:
        scene_opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        scene_opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True

    renderer.update_scene(data, camera=cam, scene_option=scene_opt)
    pixels = renderer.render()
    Image.fromarray(pixels).save(out_path)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
