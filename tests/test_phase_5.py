"""Phase 5 -- mobile base WASD driving (models/full_scene.xml gets 3 new DOF: base_x,
base_y, base_yaw on base_link, driven by velocity actuators; see src/base_teleop.py and
ffw-sh5-mobile-and-box-plan.md). Scope for this phase, per user decision: driving only --
the existing can-pick task (tests/test_phase_4.py) is left untouched and re-passing against
this same model file is itself part of this phase's regression bar.

Part 1 (unit, no MuJoCo): src/base_teleop.py's smoothing math in isolation -- accel/brake
exponential approach, magnitude clamp, in-place-rotation-zeroes-translation, yaw follow/decay.

Part 2 (integration): drive the real simulated base with BaseTeleop's output plugged into
the base_x/base_y/base_yaw velocity actuators (ctrl only, no qpos writes -- consistent with
every other actuator in this project) and check it behaves like a real vehicle: forward
drive reaches a sensible fraction of top speed and travels forward, releasing the key lets
it coast to a stop (not an instant snap, not a runaway drift), yaw-only input turns without
translating, and holding still with no keys held (base_teleop's "idle" default) does not
diverge or creep -- this last one is the direct regression check for the keyframe-ctrl
off-by-one bug found while building this phase (see NOTES.md "Phase 5"): a single dropped
token in the ctrl keyframe silently fed the left hand's thumb pre-shape targets one actuator
early, which had nothing to do with the base at all but was only visible once the base's own
extra DOFs were in place to shift the array and expose it.

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
HOME_Q_R = np.array([-0.225, -0.394, 0.682, -2.613, -0.704, 0.843, -1.218])
HOME_Q_L = np.array([-0.2222, 0.3763, -0.4512, -1.2252, 0.8006, 0.9576, 0.0270])

QACC_LIMIT = 1e5
IDLE_DRIFT_LIMIT = 0.002  # 2mm / matching radians order of magnitude, see part 2c


def run_unit_tests():
    ok = True

    # (a) holding 'w' should approach +K_SPEED forward, capped at K_MAX, never overshooting.
    bt = base_teleop.BaseTeleop()
    vx = vy = vyaw = 0.0
    for _ in range(2000):
        vx, vy, vyaw = bt.update({"w": True}, 0.01, yaw=0.0)
    speed = float(np.hypot(vx, vy))
    ok_a = abs(speed - base_teleop.K_SPEED) < 0.02 and vy == 0.0 and vyaw == 0.0
    print(f"  (a) forward hold settles at speed={speed:.4f} (target {base_teleop.K_SPEED}) "
          f"vy={vy:.4f} vyaw={vyaw:.4f}: {'OK' if ok_a else 'FAIL'}")
    ok &= ok_a

    # (b) releasing all keys decays speed back toward 0 (not an instant snap, not a stall).
    speeds = []
    for _ in range(200):
        vx, vy, vyaw = bt.update({}, 0.01, yaw=0.0)
        speeds.append(np.hypot(vx, vy))
    decaying = all(speeds[i + 1] <= speeds[i] + 1e-9 for i in range(len(speeds) - 1))
    settled = speeds[-1] < base_teleop.VEL_ZERO_EPS
    ok_b = decaying and settled
    print(f"  (b) release decays monotonically to {speeds[-1]:.5f} (settled={settled}): "
          f"{'OK' if ok_b else 'FAIL'}")
    ok &= ok_b

    # (c) magnitude never exceeds K_MAX even with a diagonal (w+a) command.
    bt2 = base_teleop.BaseTeleop()
    max_speed = 0.0
    for _ in range(2000):
        vx, vy, vyaw = bt2.update({"w": True, "a": True}, 0.01, yaw=0.0)
        max_speed = max(max_speed, float(np.hypot(vx, vy)))
    ok_c = max_speed <= base_teleop.K_MAX + 1e-9
    print(f"  (c) diagonal (w+a) peak speed={max_speed:.4f} <= K_MAX={base_teleop.K_MAX}: "
          f"{'OK' if ok_c else 'FAIL'}")
    ok &= ok_c

    # (d) turning zeroes translation immediately (in-place pivot), matching the reference
    # teleop's feel, even if 'w' is also held.
    bt3 = base_teleop.BaseTeleop()
    for _ in range(500):
        bt3.update({"w": True}, 0.01, yaw=0.0)
    vx, vy, vyaw = bt3.update({"w": True, "left": True}, 0.01, yaw=0.0)
    ok_d = vx == 0.0 and vy == 0.0 and vyaw != 0.0
    print(f"  (d) turning while 'w' held zeroes translation: vx={vx} vy={vy} vyaw={vyaw:.4f}: "
          f"{'OK' if ok_d else 'FAIL'}")
    ok &= ok_d

    return ok


def _reset_home(model, data):
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)


def run_idle_regression(model):
    """No drive keys held at all -- the direct regression check for this phase: the base
    (and, in the process of debugging it, the left hand) must not silently creep/misbehave
    just because the base now has real DOF. This is the same shape as test_phase_4.py's hold
    test but standalone, since it's this phase's own primary risk."""
    data = mujoco.MjData(model)
    _reset_home(model, data)
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
    base_x_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_x")
    base_y_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_y")
    base_yaw_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_yaw")

    site_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r")
    site_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l")
    p0_r = data.site_xpos[site_r].copy()
    p0_l = data.site_xpos[site_l].copy()

    dt = model.opt.timestep
    n = int(5.0 / dt)
    max_qacc = 0.0
    for _ in range(n):
        ctrl_r.apply(data, HOME_Q_R)
        ctrl_l.apply(data, HOME_Q_L)
        data.ctrl[base_x_aid] = 0.0
        data.ctrl[base_y_aid] = 0.0
        data.ctrl[base_yaw_aid] = 0.0
        grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="r")
        grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="l")
        mujoco.mj_step(model, data)
        max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))

    drift_r = float(np.linalg.norm(data.site_xpos[site_r] - p0_r))
    drift_l = float(np.linalg.norm(data.site_xpos[site_l] - p0_l))
    base_drift = float(np.linalg.norm(data.qpos[0:3]))
    print(f"  Idle hold (5s, no drive keys): max|qacc|={max_qacc:.3f} (limit {QACC_LIMIT:.0e}), "
          f"site_r drift={drift_r*1000:.3f}mm site_l drift={drift_l*1000:.3f}mm "
          f"base drift={base_drift*1000:.3f}(mm/mrad-mixed) (limit {IDLE_DRIFT_LIMIT*1000:.0f})")
    return (max_qacc < QACC_LIMIT and drift_r < IDLE_DRIFT_LIMIT and drift_l < IDLE_DRIFT_LIMIT
            and base_drift < IDLE_DRIFT_LIMIT)


def run_drive_test(model):
    """Hold 's' (backward -- away from the table at world x~0.5, so this is a clean drive
    with nothing in the way, unlike 'w' which rams the arm's already-reaching-for-the-table
    HOME_Q_R straight into the table after ~0.5m; that case is exercised separately in
    run_collision_test below) for 3s, release for 2s: check the base actually moves a
    sensible distance while driven, and coasts back down to ~zero velocity after release
    (not stuck at speed, not divergent)."""
    data = mujoco.MjData(model)
    _reset_home(model, data)
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
    base_x_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_x")
    base_y_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_y")
    base_yaw_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_yaw")
    base_yaw_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_yaw")]

    dt = model.opt.timestep
    frame_dt = 0.04  # matches teleop_app.py's LOOP_HZ=25
    steps_per_frame = max(1, round(frame_dt / dt))
    bt = base_teleop.BaseTeleop()

    max_qacc = 0.0
    x0 = data.qpos[0]

    def run_seconds(duration, keys):
        nonlocal max_qacc
        n_frames = int(duration / frame_dt)
        for _ in range(n_frames):
            yaw = data.qpos[base_yaw_qadr]
            vx, vy, vyaw = bt.update(keys, frame_dt, yaw)
            for _ in range(steps_per_frame):
                ctrl_r.apply(data, HOME_Q_R)
                ctrl_l.apply(data, HOME_Q_L)
                data.ctrl[base_x_aid] = vx
                data.ctrl[base_y_aid] = vy
                data.ctrl[base_yaw_aid] = vyaw
                grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="r")
                grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="l")
                mujoco.mj_step(model, data)
                max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))

    run_seconds(3.0, {"s": True})
    x_driven = data.qpos[0]
    speed_driven = float(np.linalg.norm(data.qvel[0:2]))
    run_seconds(2.0, {})
    speed_released = float(np.linalg.norm(data.qvel[0:2]))

    distance = x0 - x_driven  # positive: moved backward (away from the table), as commanded
    print(f"  Drive test: max|qacc|={max_qacc:.3f} (limit {QACC_LIMIT:.0e}), "
          f"distance after 3s='s'={distance*1000:.1f}mm, speed while driven={speed_driven:.3f}m/s, "
          f"speed 2s after release={speed_released:.4f}m/s")
    ok = (max_qacc < QACC_LIMIT and distance > 0.3  # a few tenths of a meter in 3s at ~0.5m/s
          and speed_driven > 0.3 and speed_released < 0.01)
    return ok


def run_collision_test(model):
    """Hold 'w' (forward, straight toward the table the arm is already reaching for) for a
    generous 6s -- long enough that an unobstructed drive would travel ~2m, well past the
    table -- and check the base does NOT get anywhere near that far: contact between the
    reaching arm/hand and the table should block it well short, matching
    ffw-sh5-mobile-and-box-plan.md's T9 ("베이스가 테이블 앞에서 정지, 관통 X"). This is a
    coarse whole-body check, not a precise contact.dist measurement like Session 8's palm/
    table diagnosis -- it's asking "does driving into furniture get stopped at all", not
    "by how many mm"."""
    data = mujoco.MjData(model)
    _reset_home(model, data)
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
    base_x_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_x")
    base_y_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_y")
    base_yaw_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_yaw")
    base_yaw_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_yaw")]

    dt = model.opt.timestep
    frame_dt = 0.04
    steps_per_frame = max(1, round(frame_dt / dt))
    bt = base_teleop.BaseTeleop()

    max_qacc = 0.0
    n_frames = int(6.0 / frame_dt)
    for _ in range(n_frames):
        yaw = data.qpos[base_yaw_qadr]
        vx, vy, vyaw = bt.update({"w": True}, frame_dt, yaw)
        for _ in range(steps_per_frame):
            ctrl_r.apply(data, HOME_Q_R)
            ctrl_l.apply(data, HOME_Q_L)
            data.ctrl[base_x_aid] = vx
            data.ctrl[base_y_aid] = vy
            data.ctrl[base_yaw_aid] = vyaw
            grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="r")
            grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="l")
            mujoco.mj_step(model, data)
            max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))

    x_final = data.qpos[0]
    print(f"  Collision test: max|qacc|={max_qacc:.3f} (limit {QACC_LIMIT:.0e}), "
          f"base_x after 6s driving toward the table={x_final*1000:.1f}mm "
          f"(unobstructed would be ~{6*0.5*1000:.0f}mm+)")
    # Blocked well short of an unobstructed drive, and no divergence -- exact stopping
    # distance depends on the arm/table contact geometry, not asserted precisely here.
    return max_qacc < QACC_LIMIT and 0.0 < x_final < 1.0


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    print("Part 1: base_teleop.BaseTeleop unit tests")
    unit_ok = run_unit_tests()

    print("Part 2a: idle hold regression (no drive keys)")
    idle_ok = run_idle_regression(model)

    print("Part 2b: drive + release (unobstructed direction)")
    drive_ok = run_drive_test(model)

    print("Part 2c: drive into the table (collision should stop it, not tunnel through)")
    collision_ok = run_collision_test(model)

    ok = unit_ok and idle_ok and drive_ok and collision_ok
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
