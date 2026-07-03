"""Phase 0 — official model validation.

Loads the official ROBOTIS FFW-SH5 menagerie scene (unmodified) and:
  1. Reports model structure (dof counts, joints, actuators, geoms, solver options)
    to NOTES.md.
  2. Runs a 5s gravity-only, zero-control simulation and asserts it does not diverge.
  3. Determines whether finger collision geoms are mesh- or primitive-based.

Run headless: `python3 tests/test_phase_0.py`
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
SCENE_PATH = REPO_ROOT / "assets" / "robotis_ffw" / "scene_ffw_sh5.xml"
NOTES_PATH = REPO_ROOT / "NOTES.md"

JOINT_TYPE_NAMES = {
    mujoco.mjtJoint.mjJNT_FREE: "free",
    mujoco.mjtJoint.mjJNT_BALL: "ball",
    mujoco.mjtJoint.mjJNT_SLIDE: "slide",
    mujoco.mjtJoint.mjJNT_HINGE: "hinge",
}

GEOM_TYPE_NAMES = {
    mujoco.mjtGeom.mjGEOM_PLANE: "plane",
    mujoco.mjtGeom.mjGEOM_SPHERE: "sphere",
    mujoco.mjtGeom.mjGEOM_CAPSULE: "capsule",
    mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder",
    mujoco.mjtGeom.mjGEOM_BOX: "box",
    mujoco.mjtGeom.mjGEOM_MESH: "mesh",
}


def joint_name(model, jid):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)


def actuator_for_joint(model, jid):
    for aid in range(model.nu):
        if (
            model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT
            and model.actuator_trnid[aid, 0] == jid
        ):
            return aid
    return None


def build_report(model):
    lines = []
    lines.append(f"nq={model.nq}  nv={model.nv}  nu={model.nu}  ngeom={model.ngeom}")
    lines.append("")

    lines.append("### Option")
    opt = model.opt
    cone_name = {mujoco.mjtCone.mjCONE_PYRAMIDAL: "pyramidal", mujoco.mjtCone.mjCONE_ELLIPTIC: "elliptic"}[opt.cone]
    integrator_name = {
        mujoco.mjtIntegrator.mjINT_EULER: "Euler",
        mujoco.mjtIntegrator.mjINT_RK4: "RK4",
        mujoco.mjtIntegrator.mjINT_IMPLICIT: "implicit",
        mujoco.mjtIntegrator.mjINT_IMPLICITFAST: "implicitfast",
    }[opt.integrator]
    lines.append(f"- timestep: {opt.timestep}")
    lines.append(f"- integrator: {integrator_name}")
    lines.append(f"- cone: {cone_name}")
    lines.append(f"- impratio: {opt.impratio}")
    lines.append(f"- iterations (solver): {opt.iterations}")
    lines.append(f"- ls_iterations: {opt.ls_iterations}")
    lines.append(f"- gravity: {opt.gravity.tolist()}")
    lines.append("")

    lines.append("### Joints (all)")
    lines.append("| name | type | range | damping | armature | actuator | kp | forcerange |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for jid in range(model.njnt):
        name = joint_name(model, jid) or f"<unnamed{jid}>"
        jtype = JOINT_TYPE_NAMES.get(model.jnt_type[jid], str(model.jnt_type[jid]))
        jrange = model.jnt_range[jid].tolist() if model.jnt_limited[jid] else "unlimited"
        dofadr = model.jnt_dofadr[jid]
        damping = model.dof_damping[dofadr]
        armature = model.dof_armature[dofadr]
        aid = actuator_for_joint(model, jid)
        if aid is not None:
            aname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aid)
            kp = model.actuator_gainprm[aid, 0]
            frange = model.actuator_forcerange[aid].tolist() if model.actuator_forcelimited[aid] else "unlimited"
        else:
            aname, kp, frange = "-", "-", "-"
        lines.append(f"| {name} | {jtype} | {jrange} | {damping:.3g} | {armature:.3g} | {aname} | {kp} | {frange} |")
    lines.append("")

    lines.append("### Finger joints detail (finger_l_*, finger_r_*)")
    lines.append("| name | actuator present | kp | forcerange |")
    lines.append("|---|---|---|---|")
    for jid in range(model.njnt):
        name = joint_name(model, jid) or ""
        if not name.startswith("finger_"):
            continue
        aid = actuator_for_joint(model, jid)
        if aid is not None:
            kp = model.actuator_gainprm[aid, 0]
            frange = model.actuator_forcerange[aid].tolist() if model.actuator_forcelimited[aid] else "unlimited"
            lines.append(f"| {name} | yes | {kp} | {frange} |")
        else:
            lines.append(f"| {name} | NO | - | - |")
    lines.append("")

    lines.append("### Geoms")
    lines.append("| name | type | contype | conaffinity | friction | condim |")
    lines.append("|---|---|---|---|---|---|")
    mesh_collision_count = 0
    primitive_collision_count = 0
    finger_collision_types = set()
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"<unnamed{gid}>"
        gtype = GEOM_TYPE_NAMES.get(model.geom_type[gid], str(model.geom_type[gid]))
        contype = model.geom_contype[gid]
        conaffinity = model.geom_conaffinity[gid]
        friction = model.geom_friction[gid].tolist()
        condim = model.geom_condim[gid]
        lines.append(f"| {name} | {gtype} | {contype} | {conaffinity} | {friction} | {condim} |")

        is_collision_geom = contype != 0 or conaffinity != 0
        if is_collision_geom:
            if gtype == "mesh":
                mesh_collision_count += 1
            else:
                primitive_collision_count += 1
            # geom bodies under finger_l_/finger_r_ or hx5_ base
            bid = model.geom_bodyid[gid]
            bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or ""
            if bname.startswith("finger_") or bname.startswith("hx5_"):
                finger_collision_types.add(gtype)
    lines.append("")

    lines.append("### Collision geom summary")
    lines.append(f"- mesh-type collision geoms (contype/conaffinity != 0): {mesh_collision_count}")
    lines.append(f"- primitive-type collision geoms: {primitive_collision_count}")
    lines.append(f"- finger/hand collision geom types found: {sorted(finger_collision_types) or 'none'}")
    lines.append("")

    return "\n".join(lines), finger_collision_types


def run_divergence_test(model, seconds=5.0):
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    n_steps = int(seconds / model.opt.timestep)
    max_qacc = 0.0
    for _ in range(n_steps):
        mujoco.mj_step(model, data)
        step_max = np.max(np.abs(data.qacc))
        if step_max > max_qacc:
            max_qacc = step_max
    return max_qacc, n_steps


def main():
    if not SCENE_PATH.exists():
        print(f"ERROR: scene not found at {SCENE_PATH}", file=sys.stderr)
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    print(f"Loaded {SCENE_PATH.relative_to(REPO_ROOT)} OK: nq={model.nq} nv={model.nv} nu={model.nu}")

    report, finger_collision_types = build_report(model)

    max_qacc, n_steps = run_divergence_test(model, seconds=5.0)
    print(f"5s gravity-only sim: {n_steps} steps, max|qacc|={max_qacc:.3e}")
    diverged = max_qacc >= 1e5

    report += "### Divergence test (5s, gravity only, zero control)\n"
    report += f"- steps: {n_steps}\n"
    report += f"- max |qacc|: {max_qacc:.6e}\n"
    report += f"- diverged (>= 1e5): {diverged}\n\n"

    if finger_collision_types == {"mesh"}:
        conclusion = (
            "**결론: finger/hand collision geom은 전부 mesh 기반이다 (primitive 없음).** "
            "Phase 1에서 capsule/box primitive collision으로 교체하는 작업이 필요하다."
        )
    elif finger_collision_types and "mesh" not in finger_collision_types:
        conclusion = (
            "**결론: finger/hand collision geom은 이미 primitive 기반이다.** "
            "Phase 1의 capsule 교체 작업량이 크게 줄어든다."
        )
    else:
        conclusion = f"**결론: finger/hand collision geom 타입이 혼재되어 있다: {sorted(finger_collision_types)}**"

    report += "### Finger collision mesh vs primitive judgement\n"
    report += conclusion + "\n\n"

    NOTES_PATH.write_text(
        "# NOTES\n\n"
        "## Phase 0 — 공식 모델 검증\n\n"
        f"{report}"
    )
    print(f"Wrote report to {NOTES_PATH.relative_to(REPO_ROOT)}")

    ok = not diverged
    print("PASS" if ok else "FAIL: simulation diverged (max|qacc| >= 1e5)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
