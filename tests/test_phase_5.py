"""Phase 5 -- mobile base locomotion (models/full_scene.xml gets 3 planar joints on
base_link -- base_x/base_y/base_yaw, so it can translate/turn but not tip -- plus real
steer+drive joints on all three wheels, restored from the vendored ffw_sh5.xml's own
wheel_steer/wheel_drive default classes that Phase 4 had removed for simplicity). Scope for
this phase, per user decision: driving only -- the existing can-pick task
(tests/test_phase_4.py) is left untouched and re-passing against this same model file is
itself part of this phase's regression bar.

Session 8 후속 revised this from a direct virtual-joint velocity actuator (the original
Phase 5 design) to genuine wheel-ground friction propulsion per user request: base_x/base_y/
base_yaw are no longer directly actuated, only reacted through the wheels' steer/drive
joints and their real contact with the floor. See src/base_teleop.py's `SwerveDrive` for the
per-wheel steer-angle + drive-speed kinematics (needed because the vendored wheel_steer
joints support the official approximately +/-2pi range; an injected narrow-range solver is
covered separately in ``test_whole_body.py``.

Part 1 (unit, no MuJoCo): BaseTeleop's smoothing math (unchanged) plus ROBOTIS-style
SwerveDrive behavior in isolation (pure forward/turn/strafe cases, checked against the
known wheel mounting geometry, plus the 180deg reversal FSM).

Part 2 (integration): drives the real simulated wheels via SwerveDrive's ctrl outputs
(never qpos) and checks: (a) idle hold with no keys is stable (no creep -- this is the
direct regression check for two real bugs found building this: a keyframe token-count typo,
and a numerically-unstable exact-zero wheel/floor gap that silently dropped two of three
wheels out of contact and produced ~99% wheel slip); (b) driving actually moves the base
with the wheels rolling near-without-slipping (checked directly, not just "it moved") --
this is the "moves via wheel friction, not a virtual actuator" regression check the user
asked for; (c) driving toward the table still gets stopped by the (already-known,
Session 8-documented) arm/table collision rather than tunneling through.

Run headless: `python3 tests/test_phase_5.py`
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
import grasp  # noqa: E402

ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
WHEELS = ("left_wheel", "right_wheel", "rear_wheel")

QACC_LIMIT = 1e5
IDLE_DRIFT_LIMIT = 0.002  # 2mm


def run_unit_tests():
    ok = True

    bt = base_teleop.BaseTeleop()
    vx = vy = vyaw = 0.0
    for _ in range(2000):
        vx, vy, vyaw = bt.update({"w": True}, 0.01, yaw=0.0)
    speed = float(np.hypot(vx, vy))
    ok_a = abs(speed - base_teleop.K_SPEED) < 0.02 and vy == 0.0 and vyaw == 0.0
    print(f"  (a) BaseTeleop forward hold settles at speed={speed:.4f} "
          f"(target {base_teleop.K_SPEED}): {'OK' if ok_a else 'FAIL'}")
    ok &= ok_a

    responsive = base_teleop.BaseTeleop()
    for _ in range(round(0.6 / 0.01)):
        combined = responsive.update_body({"w": True, "left": True}, 0.01)
    combined_ok = combined.vx > 0.90 * base_teleop.K_SPEED and combined.wz > 1.4
    release_time = None
    for step in range(1, round(1.2 / 0.01) + 1):
        released = responsive.update_body({}, 0.01)
        if released.is_zero():
            release_time = step * 0.01
            break
    response_ok = combined_ok and release_time is not None and release_time < 1.0
    print(f"  (a2) combined+response: vx={combined.vx:.3f} wz={combined.wz:.3f} "
          f"release_zero={release_time}s: {'OK' if response_ok else 'FAIL'}")
    ok &= response_ok

    # SwerveDrive: pure forward -> every wheel faces forward (steer=0), same drive speed.
    sd = base_teleop.SwerveDrive()
    for _ in range(300):
        cmds = sd.update({"w": True}, 0.01, yaw=0.0)
    steers = [abs(cmds[w][0]) for w in WHEELS]
    speeds = [cmds[w][1] for w in WHEELS]
    ok_b = all(s < 0.01 for s in steers) and max(speeds) - min(speeds) < 0.01 and speeds[0] > 0
    print(f"  (b) SwerveDrive forward: steer angles={[round(s,4) for s in steers]} "
          f"drive speeds={[round(s,3) for s in speeds]}: {'OK' if ok_b else 'FAIL'}")
    ok &= ok_b

    # SwerveDrive: pure in-place yaw -> rear wheel (directly behind center) points +-90deg,
    # left/right wheels point symmetrically, all consistent with rotating about the origin.
    sd2 = base_teleop.SwerveDrive()
    for _ in range(300):
        cmds2 = sd2.update({"left": True}, 0.01, yaw=0.0)
    rear_steer = cmds2["rear_wheel"][0]
    left_steer, right_steer = cmds2["left_wheel"][0], cmds2["right_wheel"][0]
    ok_c = (abs(abs(rear_steer) - np.pi / 2) < 0.02
            and abs(left_steer + right_steer) < 0.02  # symmetric about 0
            and abs(left_steer) > 0.01)
    print(f"  (c) SwerveDrive in-place yaw: rear={np.degrees(rear_steer):.1f}deg "
          f"left={np.degrees(left_steer):.1f}deg right={np.degrees(right_steer):.1f}deg: "
          f"{'OK' if ok_c else 'FAIL'}")
    ok &= ok_c

    # SwerveDrive: pure strafe -> every wheel perpendicular to forward (+-90deg).
    sd3 = base_teleop.SwerveDrive()
    for _ in range(300):
        cmds3 = sd3.update({"a": True}, 0.01, yaw=0.0)
    strafe_steers = [abs(abs(cmds3[w][0]) - np.pi / 2) for w in WHEELS]
    ok_d = all(s < 0.02 for s in strafe_steers)
    print(f"  (d) SwerveDrive strafe: steer angles={[round(np.degrees(cmds3[w][0]),1) for w in WHEELS]}: "
          f"{'OK' if ok_d else 'FAIL'}")
    ok &= ok_d

    sd4 = base_teleop.SwerveDrive()
    feedback_steer = {wheel: 0.0 for wheel in WHEELS}
    moving_wheels = {wheel: 5.0 for wheel in WHEELS}
    sd4.update_twist(base_teleop.BodyTwist(0.5, 0.0, 0.0), 0.01,
                     feedback_steer, moving_wheels)
    reverse_cmd = sd4.update_twist(base_teleop.BodyTwist(-0.5, 0.0, 0.0), 0.01,
                                   feedback_steer, moving_wheels)
    ok_e = (
        sd4.reversal_phase["left_wheel"] == base_teleop.ReversalPhase.DECELERATING
        and sd4.wheel_speed_scale["left_wheel"] < 1.0
        and reverse_cmd["left_wheel"][1] > 0.0
    )
    print(f"  (e) SwerveDrive 180deg reversal: phase={sd4.reversal_phase['left_wheel'].name} "
          f"scale={sd4.wheel_speed_scale['left_wheel']:.2f} "
          f"wheel_cmd={reverse_cmd['left_wheel'][1]:.3f}: {'OK' if ok_e else 'FAIL'}")
    ok &= ok_e

    stopped = base_teleop.SwerveDrive()
    stopped_reverse = stopped.update_twist(
        base_teleop.BodyTwist(-0.5, 0.0, 0.0), 0.01,
        feedback_steer, {wheel: 0.0 for wheel in WHEELS})
    stopped_ok = (all(stopped.reversal_phase[w] == base_teleop.ReversalPhase.NORMAL for w in WHEELS)
                  and all(stopped_reverse[w][1] < 0.0 for w in WHEELS))

    stalled = base_teleop.SwerveDrive()
    for _ in range(30):
        stalled_cmd = stalled.update_twist(
            base_teleop.BodyTwist(0.0, 0.5, 0.0), 0.01,
            feedback_steer, {wheel: 0.0 for wheel in WHEELS})
    command_progress_ok = all(stalled_cmd[w][0] > 1.45 for w in WHEELS)
    ok_f = stopped_ok and command_progress_ok
    print(f"  (f) stopped reversal + lagging-feedback steering command: "
          f"direct_reverse={stopped_ok} steer_cmd="
          f"{[round(stalled_cmd[w][0], 2) for w in WHEELS]}: {'OK' if ok_f else 'FAIL'}")
    ok &= ok_f

    return ok


def _reset_home(model, data):
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)


def _make_rig(model):
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
    steer_aids = {w: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{w}_steer") for w in WHEELS}
    drive_aids = {w: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{w}_drive") for w in WHEELS}
    steer_qadrs = {
        w: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{w}_steer_joint")]
        for w in WHEELS
    }
    drive_dofs = {w: model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{w}_drive_joint")]
                  for w in WHEELS}
    base_yaw_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_yaw")]
    base_x_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_x")]
    base_y_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_y")]
    base_x_dof = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_x")]
    base_y_dof = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_y")]
    base_yaw_dof = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_yaw")]
    return dict(ctrl_r=ctrl_r, ctrl_l=ctrl_l, steer_aids=steer_aids, drive_aids=drive_aids,
                steer_qadrs=steer_qadrs,
                drive_dofs=drive_dofs, base_yaw_qadr=base_yaw_qadr, base_x_qadr=base_x_qadr,
                base_y_qadr=base_y_qadr, base_x_dof=base_x_dof, base_y_dof=base_y_dof,
                base_yaw_dof=base_yaw_dof)


def _step(model, data, rig, drive, keys, frame_dt):
    yaw = data.qpos[rig["base_yaw_qadr"]]
    steering_positions = {w: float(data.qpos[qadr]) for w, qadr in rig["steer_qadrs"].items()}
    wheel_velocities = {w: float(data.qvel[dof]) for w, dof in rig["drive_dofs"].items()}
    cmds = drive.update(keys, frame_dt, yaw, steering_positions, wheel_velocities)
    dt = model.opt.timestep
    max_qacc = 0.0
    for _ in range(max(1, round(frame_dt / dt))):
        rig["ctrl_r"].apply(data, HOME_Q_R)
        rig["ctrl_l"].apply(data, HOME_Q_L)
        for wheel, (angle, speed) in cmds.items():
            data.ctrl[rig["steer_aids"][wheel]] = angle
            data.ctrl[rig["drive_aids"][wheel]] = speed
        grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="r")
        grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="l")
        mujoco.mj_step(model, data)
        max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))
    return max_qacc


def _step_twist(model, data, rig, drive, twist, frame_dt):
    steering_positions = {w: float(data.qpos[qadr]) for w, qadr in rig["steer_qadrs"].items()}
    wheel_velocities = {w: float(data.qvel[dof]) for w, dof in rig["drive_dofs"].items()}
    cmds = drive.update_twist(twist, frame_dt, steering_positions, wheel_velocities)
    max_qacc = 0.0
    for _ in range(max(1, round(frame_dt / model.opt.timestep))):
        rig["ctrl_r"].apply(data, HOME_Q_R)
        rig["ctrl_l"].apply(data, HOME_Q_L)
        for wheel, (angle, speed) in cmds.items():
            data.ctrl[rig["steer_aids"][wheel]] = angle
            data.ctrl[rig["drive_aids"][wheel]] = speed
        grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="r")
        grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="l")
        mujoco.mj_step(model, data)
        max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))
    return max_qacc


def _run_twist_trial(model, twist, duration):
    data = mujoco.MjData(model)
    _reset_home(model, data)
    rig = _make_rig(model)
    drive = base_teleop.SwerveDrive()
    initial = np.array([
        data.qpos[rig["base_x_qadr"]], data.qpos[rig["base_y_qadr"]],
        data.qpos[rig["base_yaw_qadr"]]])
    max_qacc = 0.0
    for _ in range(round(duration / 0.04)):
        max_qacc = max(max_qacc, _step_twist(model, data, rig, drive, twist, 0.04))
    final = np.array([
        data.qpos[rig["base_x_qadr"]], data.qpos[rig["base_y_qadr"]],
        data.qpos[rig["base_yaw_qadr"]]])
    return final - initial, max_qacc, data, rig, drive


def run_omnidirectional_regression(model):
    """Physical strafe, yaw, combined twist, reversal, and internal-collision gates."""
    audit = mujoco.MjData(model)
    _reset_home(model, audit)
    rig = _make_rig(model)
    for wheel in WHEELS:
        audit.qpos[rig["steer_qadrs"][wheel]] = np.pi / 2
    mujoco.mj_forward(model, audit)
    wheel_geoms = {mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, f"{wheel}_collision") for wheel in WHEELS}
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    internal_contacts = [contact for contact in audit.contact
                         if (contact.geom1 in wheel_geoms or contact.geom2 in wheel_geoms)
                         and floor not in (contact.geom1, contact.geom2)]

    strafe, strafe_acc, *_ = _run_twist_trial(
        model, base_teleop.BodyTwist(0.0, 0.5, 0.0), 2.0)
    strafe_ok = (strafe[1] > 0.55 and abs(strafe[0]) < 0.12
                 and abs(strafe[2]) < 0.15 and strafe_acc < QACC_LIMIT)

    yaw, yaw_acc, *_ = _run_twist_trial(
        model, base_teleop.BodyTwist(0.0, 0.0, 1.0), 2.0)
    yaw_ok = (yaw[2] > 0.8 and np.linalg.norm(yaw[:2]) < 0.10 and yaw_acc < QACC_LIMIT)

    combined, combined_acc, *_ = _run_twist_trial(
        model, base_teleop.BodyTwist(-0.4, 0.0, 0.6), 2.0)
    combined_ok = (np.linalg.norm(combined[:2]) > 0.35 and combined[2] > 0.45
                   and combined_acc < QACC_LIMIT)

    data = mujoco.MjData(model)
    _reset_home(model, data)
    rig = _make_rig(model)
    drive = base_teleop.SwerveDrive()
    max_acc = 0.0
    for _ in range(round(1.2 / 0.04)):
        max_acc = max(max_acc, _step_twist(
            model, data, rig, drive, base_teleop.BodyTwist(0.0, 0.45, 0.0), 0.04))
    positive_y = float(data.qpos[rig["base_y_qadr"]])
    for _ in range(round(1.2 / 0.04)):
        max_acc = max(max_acc, _step_twist(
            model, data, rig, drive, base_teleop.BodyTwist(0.0, -0.45, 0.0), 0.04))
    reversed_y = float(data.qpos[rig["base_y_qadr"]])
    reversal_ok = positive_y > 0.25 and reversed_y < positive_y - 0.18 and max_acc < QACC_LIMIT

    ok = (not internal_contacts and strafe_ok and yaw_ok and combined_ok and reversal_ok)
    print(f"  Omnidirectional: internal_contacts={len(internal_contacts)} "
          f"strafe=({strafe[0]:+.3f},{strafe[1]:+.3f},{np.degrees(strafe[2]):+.1f}deg) "
          f"yaw=({yaw[0]:+.3f},{yaw[1]:+.3f},{np.degrees(yaw[2]):+.1f}deg) "
          f"combined_dist={np.linalg.norm(combined[:2]):.3f}m/"
          f"{np.degrees(combined[2]):.1f}deg reverse_y={positive_y:.3f}->{reversed_y:.3f}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_idle_regression(model):
    """No drive keys held at all -- this is the direct regression check for two bugs found
    while building this: a dropped keyframe token and a numerically-unstable exact-zero
    wheel/floor gap that silently dropped two of three wheels from `data.contact` -- both
    showed up first as exactly this idle-hold test drifting when it shouldn't."""
    data = mujoco.MjData(model)
    _reset_home(model, data)
    rig = _make_rig(model)
    drive = base_teleop.SwerveDrive()

    site_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r")
    site_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l")
    p0_r, p0_l = data.site_xpos[site_r].copy(), data.site_xpos[site_l].copy()

    max_qacc = 0.0
    for _ in range(int(5.0 / 0.04)):
        max_qacc = max(max_qacc, _step(model, data, rig, drive, {}, 0.04))

    drift_r = float(np.linalg.norm(data.site_xpos[site_r] - p0_r))
    drift_l = float(np.linalg.norm(data.site_xpos[site_l] - p0_l))
    base_drift = float(np.linalg.norm(data.qpos[rig["base_x_qadr"]:rig["base_x_qadr"] + 2]))
    print(f"  Idle hold (5s, no drive keys): max|qacc|={max_qacc:.3f} (limit {QACC_LIMIT:.0e}), "
          f"site_r drift={drift_r*1000:.3f}mm site_l drift={drift_l*1000:.3f}mm "
          f"base drift={base_drift*1000:.4f}mm (limit {IDLE_DRIFT_LIMIT*1000:.0f}mm)")
    return (max_qacc < QACC_LIMIT and drift_r < IDLE_DRIFT_LIMIT and drift_l < IDLE_DRIFT_LIMIT
            and base_drift < IDLE_DRIFT_LIMIT)


def run_drive_test(model):
    """Hold 's' (backward -- away from the table, a clean drive with nothing in the way,
    unlike 'w' which rams the arm's already-reaching-for-the-table HOME_Q_R into the table
    after ~0.5m; that case is exercised separately in run_collision_test) for 3s, release
    for 2s. Checks the base actually moves AND that it does so by real wheel rolling (wheel
    rim speed matches base speed within a small margin, i.e. not slipping) -- the direct
    check that propulsion is genuinely friction-driven, not a leftover virtual actuator."""
    data = mujoco.MjData(model)
    _reset_home(model, data)
    rig = _make_rig(model)
    drive = base_teleop.SwerveDrive()

    max_qacc = 0.0
    x0 = data.qpos[rig["base_x_qadr"]]
    for _ in range(int(3.0 / 0.04)):
        max_qacc = max(max_qacc, _step(model, data, rig, drive, {"s": True}, 0.04))
    x_driven = data.qpos[rig["base_x_qadr"]]
    base_vx = data.qvel[rig["base_x_dof"]]
    wheel_qvel = data.qvel[rig["drive_dofs"]["left_wheel"]]
    rolling_speed = abs(wheel_qvel) * base_teleop.WHEEL_RADIUS
    slip = abs(rolling_speed - abs(base_vx)) / max(rolling_speed, 1e-6)

    for _ in range(int(2.0 / 0.04)):
        max_qacc = max(max_qacc, _step(model, data, rig, drive, {}, 0.04))
    speed_released = float(np.linalg.norm(data.qvel[rig["base_x_dof"]:rig["base_x_dof"] + 2]))

    distance = x0 - x_driven  # positive: moved backward, as commanded
    print(f"  Drive test: max|qacc|={max_qacc:.3f} (limit {QACC_LIMIT:.0e}), "
          f"distance after 3s='s'={distance*1000:.1f}mm, base speed while driven={abs(base_vx):.3f}m/s, "
          f"wheel rolling speed={rolling_speed:.3f}m/s (slip={slip*100:.1f}%), "
          f"speed 2s after release={speed_released:.4f}m/s")
    ok = (max_qacc < QACC_LIMIT and distance > 0.15 and abs(base_vx) > 0.1
          and slip < 0.15 and speed_released < 0.01)
    return ok


def run_collision_test(model):
    """Hold 'w' (forward, toward the table the arm is already reaching for) for a generous
    6s -- long enough that an unobstructed drive would travel well over a meter -- and check
    the base does NOT get anywhere near that far: contact between the reaching arm/hand and
    the table should block it well short ("베이스가 테이블 앞에서 정지, 관통 X")."""
    data = mujoco.MjData(model)
    _reset_home(model, data)
    rig = _make_rig(model)
    drive = base_teleop.SwerveDrive()

    max_qacc = 0.0
    for _ in range(int(6.0 / 0.04)):
        max_qacc = max(max_qacc, _step(model, data, rig, drive, {"w": True}, 0.04))

    x_final = data.qpos[rig["base_x_qadr"]]
    print(f"  Collision test: max|qacc|={max_qacc:.3f} (limit {QACC_LIMIT:.0e}), "
          f"base_x after 6s driving toward the table={x_final*1000:.1f}mm "
          f"(unobstructed would be ~1000mm+)")
    return max_qacc < QACC_LIMIT and 0.0 < x_final < 1.0


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    print("Part 1: BaseTeleop + SwerveDrive unit tests")
    unit_ok = run_unit_tests()

    print("Part 2a: idle hold regression (no drive keys)")
    idle_ok = run_idle_regression(model)

    print("Part 2b: drive + release (unobstructed direction, checks real rolling/no-slip)")
    drive_ok = run_drive_test(model)

    print("Part 2c: drive into the table (collision should stop it, not tunnel through)")
    collision_ok = run_collision_test(model)

    print("Part 2d: physical omnidirectional/reversal/self-collision regression")
    omni_ok = run_omnidirectional_regression(model)

    ok = unit_ok and idle_ok and drive_ok and collision_ok and omni_ok
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
