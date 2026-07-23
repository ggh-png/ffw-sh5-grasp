"""Single-window teleoperation app for ``models/full_scene.xml``.

Reproduces the reference video's interface as movable ImGui tool windows over the 3D view:
EE pose target sliders (home-relative X/Y/Z/Roll/Pitch/Yaw) drive ROS-free whole-body IK
(base/lift/both arms), arm torque control, grasp/thumb synergy, a joint monitor, and an HUD.

Rendering uses MuJoCo's low-level API with ImGui in the same GLFW window.  GLFW must use
the X11 backend so the ImGui OpenGL renderer receives a compatible GLX context.  The UI,
control, physics, and rendering stages all execute on one thread.

RPY sliders express local offsets from each hand's startup orientation.  Whole-body pose
targets remain world-anchored, while manual driving carries those targets with the chassis.

**Whole-body + mobile base**: EE targets are home-relative UI values anchored to their
startup world poses.  `whole_body_ik.py` solves base x/y/yaw, lift and both 7-DOF arms in one
bounded weighted differential-IK problem.  Its base velocity is converted to a body-frame
`BodyTwist` and sent through `base_teleop.SwerveDrive`; only real steer/drive actuators are
commanded, so wheel-ground friction still produces all base motion.  Keyboard body velocity
has priority while held and uses the exact same swerve path.  No ROS/MoveIt dependency and
no robot qpos override are introduced.

Run: `python3 src/teleop_app.py`.
Mouse: left-drag orbit, right-drag pan, scroll zoom (standard MuJoCo camera controls).
Keyboard: Up/Down = base forward/back (robot-local), Left/Right = base yaw,
[ / ] = base strafe left/right, Q/E = lift down/up, R = reset can (+-2cm random),
G = toggle contact force/point visualization, V = toggle collision geometry/CBF visualization,
C = cycle camera preset (overview / right-hand close-up). R/G/V/C also have
buttons in the on-screen tool windows.

"""

import argparse
import math
import pathlib
import sys
import time

import glfw
# Must precede glfw.init() -- see module docstring.
glfw.init_hint(glfw.PLATFORM, glfw.PLATFORM_X11)

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "full_scene.xml"

import arm_control  # noqa: E402
import base_teleop  # noqa: E402
import grasp  # noqa: E402
import ik  # noqa: E402
import teleop_render  # noqa: E402
import teleop_targets  # noqa: E402
import teleop_ui  # noqa: E402
import whole_body_ik  # noqa: E402

# 양팔 관절 이름 목록 (IK solver / 토크 제어기에 그대로 넘겨진다).
ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
SIDES = ("r", "l")
ARM_JOINTS = {"r": ARM_R, "l": ARM_L}
WHEELS = base_teleop.WHEELS
# Matches models/full_scene.xml's "home" keyframe, which as of Session 8 (Phase 5 follow-up)
# matches the sibling ffw-sh5-mujoco repo's rest pose (only the elbow/joint4 bent -90 deg,
# everything else 0).
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
LIFT_RANGE = (-0.5, 0.0)
VIRTUAL_OBJECT_HOME_POS = np.array([0.4055, 0.0, 0.9716])
# 패널의 "Joint position monitor"에 진행률 막대로 표시할 관절 전체 목록.
MONITOR_JOINTS = (
    [f"arm_r_joint{i}" for i in range(1, 8)] + [f"arm_l_joint{i}" for i in range(1, 8)]
    + [f"finger_r_joint{i}" for i in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)]
    + [f"finger_l_joint{i}" for i in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)]
    + ["lift_joint", "head_joint1", "head_joint2"]
)

WINDOW_W, WINDOW_H = 1440, 900
LOOP_HZ = 25.0
# Target rate limiting caps how far the effective goal can move in one render frame.
# the *effective* IK target can move in one frame, independent of how far the raw slider
# jumped. 0.03m/frame at LOOP_HZ=25 is 0.75 m/s -- brisk but trackable; 8 deg/frame is
# 200 deg/s, generous but bounded.
MAX_POS_STEP_PER_FRAME = 0.03
MAX_RPY_STEP_PER_FRAME_DEG = 8.0


def _named_id(model, object_type, name):
    """Resolve a required MuJoCo object name and fail with useful context."""
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise ValueError(f"MuJoCo object not found: {name!r}")
    return object_id


def _joint_address(model, name, addresses):
    joint_id = _named_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return int(addresses[joint_id])


def rpy_deg_to_quat(rpy_deg):
    return teleop_targets.rpy_deg_to_quat(rpy_deg)


def quat_to_rpy_deg(q):
    """rpy_deg_to_quat의 역변환 -- FK 모드에서 IK 모드로 되돌아갈 때, 현재 실제
    방향(쿼터니언)을 RPY 슬라이더 값(도)으로 되짚어 오기 위해 필요하다."""
    return teleop_targets.quat_to_rpy_deg(q)


def _reset_can_random(model, data, rng):
    """The one qpos write in this file outside of the initial keyframe reset -- resetting a
    freely-placed object's spawn pose is the explicit "no kinematic override" exception,
    not a kinematic override of the robot itself."""
    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    qadr = model.jnt_qposadr[can_jid]
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    home_can_pos = model.key_qpos[key_id][qadr:qadr + 3].copy()
    data.qpos[qadr:qadr + 3] = home_can_pos + rng.uniform(-0.02, 0.02, size=3)
    data.qpos[qadr + 3:qadr + 7] = [1, 0, 0, 0]
    dof = model.jnt_dofadr[can_jid]
    data.qvel[dof:dof + 6] = 0.0


class KeyEdge:
    """Edge-triggered key check (fires once per press, not once per frame held)."""

    def __init__(self):
        self._prev = set()

    def pressed(self, window, key):
        down = glfw.get_key(window, key) == glfw.PRESS
        was_down = key in self._prev
        if down:
            self._prev.add(key)
        else:
            self._prev.discard(key)
        return down and not was_down


class TeleopApp:
    """MuJoCo 주 창과 분리형 도구 창을 쓰는 텔레옵 앱. `run()`의 메인 루프가 매 프레임 아래 단계를
    순서대로 실행한다: 마우스 카메라 -> 엣지 키(R/G/V/C) -> 연속 키(주행/리프트) ->
    UI 패널 -> 물리 스텝 -> 렌더링. 상태는 전부 인스턴스 속성(self.*)에 있다."""

    def __init__(self):
        self._setup_sim()
        self._setup_render()
        self._setup_loop_state()

    # ------------------------------------------------------------------
    # 초기화
    # ------------------------------------------------------------------

    def _setup_sim(self):
        """모델 로드, 홈 자세 리셋, IK 솔버/토크 제어기, actuator/joint id 조회,
        슬라이더 목표값(targets) 초기화까지 -- 렌더링/윈도우와 무관한 부분 전부."""
        model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        data = mujoco.MjData(model)
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        self.model = model
        self.data = data

        # Whole-body solver + 팔 토크 제어기. 기존 ik.py는 Phase 3/4 독립 회귀에 유지한다.
        self.ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
        self.ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
        self.whole_body_enabled = True
        self.whole_body_solver = whole_body_ik.WholeBodyIK(
            model,
            {"r": "grasp_target_r", "l": "grasp_target_l"},
            ARM_JOINTS,
        )
        # FK(관절각 직접 제어) 모드에서 패널 슬라이더의 최소/최대값으로 쓸, 각 팔
        # 관절의 range를 도(degree) 단위로 미리 계산해둔다.
        self.arm_joint_ranges_deg = {
            side: [
                tuple(
                    math.degrees(value)
                    for value in model.jnt_range[
                        _named_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                    ]
                )
                for name in joints
            ]
            for side, joints in ARM_JOINTS.items()
        }
        self.lift_aid = _named_id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lift_joint"
        )
        self.base_x_qadr = _joint_address(model, "base_x", model.jnt_qposadr)
        self.base_y_qadr = _joint_address(model, "base_y", model.jnt_qposadr)
        self.base_yaw_qadr = _joint_address(model, "base_yaw", model.jnt_qposadr)
        self.base_x_dof = _joint_address(model, "base_x", model.jnt_dofadr)
        self.base_y_dof = _joint_address(model, "base_y", model.jnt_dofadr)
        self.base_yaw_dof = _joint_address(model, "base_yaw", model.jnt_dofadr)
        # Real per-wheel steer (position) + drive (velocity) actuators -- see
        # src/base_teleop.py's SwerveDrive and the module docstring's "Mobile base" note. Base
        # motion is now a *consequence* of wheel-ground friction, not a direct actuator on
        # base_x/base_y/base_yaw (those joints still exist so base_link can't tip, but nothing
        # actuates them directly any more).
        self.wheel_steer_aids = {
            wheel: _named_id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{wheel}_steer"
            )
            for wheel in WHEELS
        }
        self.wheel_drive_aids = {
            wheel: _named_id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{wheel}_drive"
            )
            for wheel in WHEELS
        }
        self.wheel_steer_qadrs = {
            wheel: _joint_address(
                model, f"{wheel}_steer_joint", model.jnt_qposadr
            )
            for wheel in WHEELS
        }
        self.wheel_drive_dofs = {
            wheel: _joint_address(
                model, f"{wheel}_drive_joint", model.jnt_dofadr
            )
            for wheel in WHEELS
        }
        # 모바일 베이스: SwerveDrive가 키 입력을 바퀴별 조향각+구동속도로 변환.
        self.base_drive = base_teleop.SwerveDrive()
        monitor_joint_ids = {
            name: _named_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in MONITOR_JOINTS
        }
        self.monitor_qposadr = {
            name: int(model.jnt_qposadr[joint_id])
            for name, joint_id in monitor_joint_ids.items()
        }
        self.monitor_ranges = {
            name: model.jnt_range[joint_id]
            for name, joint_id in monitor_joint_ids.items()
        }
        self.rng = np.random.default_rng()

        self.ik_target_mocap_ids = {
            side: model.body_mocapid[
                _named_id(
                    model, mujoco.mjtObj.mjOBJ_BODY, f"ik_target_{side}"
                )
            ]
            for side in SIDES
        }
        self.virtual_object_mocap_id = model.body_mocapid[
            _named_id(
                model, mujoco.mjtObj.mjOBJ_BODY, "virtual_object_marker"
            )
        ]
        self.virtual_object_marker_geom_id = _named_id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "virtual_object_marker_geom")
        self.virtual_object_marker_site_id = _named_id(
            model, mujoco.mjtObj.mjOBJ_SITE, "virtual_object_marker_site")
        self.virtual_object_marker_rgba = {
            "geom": model.geom_rgba[self.virtual_object_marker_geom_id].copy(),
            "site": model.site_rgba[self.virtual_object_marker_site_id].copy(),
        }
        self.can_jid = _named_id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "can_free"
        )
        self.can_geom_id = _named_id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "can_geom"
        )
        self._disable_legacy_box_asset()
        # Reference pose each hand's XYZ/RPY sliders are relative to (see module docstring).
        teleop_targets.set_home_references(self)

        # 슬라이더가 직접 쓰는 "목표값" 딕셔너리 -- 물리 루프는 이 값만 읽고, 이 값을
        # 쓰는 건 GUI 슬라이더뿐이다(단방향 데이터 흐름, 모듈 docstring 참고).
        self.targets = {
            **{
                f"{field}_{side}": [0.0, 0.0, 0.0]
                for side in SIDES
                for field in ("pos", "rpy")
            },
            **{
                f"{field}_{side}": 0.0
                for side in SIDES
                for field in ("grasp", "thumb")
            },
            "virtual_object_pos": VIRTUAL_OBJECT_HOME_POS.tolist(),
            "virtual_object_rpy": [0.0, 0.0, 0.0],
            "lift": float(
                data.qpos[_joint_address(model, "lift_joint", model.jnt_qposadr)]
            ),
        }
        # Whole-body IK chases smoothed copies so a typed/dragged target cannot jump abruptly.
        self.smoothed_pos = {
            side: np.array(self.targets[f"pos_{side}"]) for side in SIDES
        }
        self.smoothed_rpy = {
            side: np.array(self.targets[f"rpy_{side}"]) for side in SIDES
        }
        self.lift_cmd = self.targets["lift"]
        self.whole_body_base_twist = base_teleop.BodyTwist()
        self.commanded_base_twist = base_teleop.BodyTwist()
        self._manual_override_active = False
        self._manual_reference_base_pose = np.array([
            data.qpos[self.base_x_qadr], data.qpos[self.base_y_qadr],
            data.qpos[self.base_yaw_qadr],
        ], dtype=float)
        self._sync_ik_mocaps_from_targets()
        self.contact_viz = False
        self.collision_viz = False
        self.collision_active_pairs = ()
        self.collision_min_distance = math.inf
        self.collision_constraint_violation = 0.0
        self.camera_preset = 0
        self.grab_state = dict.fromkeys(SIDES)
        self.cyclo_controller = "movel"
        self.cyclo_move_time = 2.0
        self.cyclo_grasp_captured = False
        self.cyclo_capture_offsets = None
        self.cyclo_status = "ready"
        # teleop_ui.draw_panel reads these two off `app` directly (see that module's
        # docstring for why it doesn't import teleop_app's constants instead).
        self.lift_range = LIFT_RANGE
        self.reset_can()

    def _setup_render(self):
        """MuJoCo 주 GLFW 창과 ImGui 플랫폼 창, 저수준 렌더 context를 만든다."""
        teleop_render.setup_render(self, WINDOW_W, WINDOW_H)

    def _setup_loop_state(self):
        """메인 루프에서만 쓰는 상태(IK 웜스타트 값, 타이밍, 입력 헬퍼)."""
        self.q_des_r = HOME_Q_R.copy()
        self.q_des_l = HOME_Q_L.copy()
        # 손별 제어 모드: "ik"(EE 포즈 슬라이더 -> whole-body solver) 또는
        # "fk"(관절각 슬라이더를 그대로 토크 제어기 목표로 사용, IK 자체를 건너뜀).
        # 리프트를 움직이는 동안 IK가 매 프레임에서만 어깨 높이를 다시 읽어들여서
        # 생기는 출렁임(리프트가 프레임 사이에도 계속 움직이는데 IK는 프레임당
        # 한 번만 풀림)을 피하고 싶을 때 FK로 전환해 관절각을 고정해두면, 팔 전체가
        # 리프트에 강체로 붙어 그대로 오르내리기만 해서 흔들림이 아예 없다.
        self.arm_mode = {"r": "ik", "l": "ik"}
        self.fk_q_deg = {"r": [math.degrees(v) for v in self.q_des_r],
                          "l": [math.degrees(v) for v in self.q_des_l]}
        self.frame_dt = 1.0 / LOOP_HZ
        self.steps_per_frame = max(1, round(self.frame_dt / self.model.opt.timestep))
        self.freq_ema = LOOP_HZ
        self.wall_start = time.perf_counter()
        self.last_mouse = list(glfw.get_cursor_pos(self.window))
        self.keys = KeyEdge()
        self.ik_err_mm = {"l": 0.0, "r": 0.0}
        self.gizmo_mouse_active = False

    # ------------------------------------------------------------------
    # R/G/V/C 동작 -- 키보드(_handle_edge_keys)와 패널 버튼
    # (teleop_ui.draw_panel) 양쪽에서 똑같이 호출하는 공용 메서드.
    # ------------------------------------------------------------------

    def reset_can(self):
        _reset_can_random(self.model, self.data, self.rng)

    def reset_active_object(self):
        self.reset_can()
        self.grab_state = {"r": None, "l": None}
        self.cyclo_grasp_captured = False
        self.cyclo_capture_offsets = None
        self.whole_body_solver.set_rigid_grasp(self.data, False)
        self.cyclo_controller = "movel"
        self.cyclo_status = "ready"

    def _disable_legacy_box_asset(self):
        """Keep the old XML body inert while the live app runs the can-only workflow."""
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "box_free")
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
        if jid == -1 or gid == -1:
            return
        qadr = self.model.jnt_qposadr[jid]
        dof = self.model.jnt_dofadr[jid]
        self.data.qpos[qadr:qadr + 3] = [2.0, 2.0, 0.1]
        self.data.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[dof:dof + 6] = 0.0
        self.model.geom_contype[gid] = 0
        self.model.geom_conaffinity[gid] = 0
        self.model.geom_rgba[gid][3] = 0.0

    def cycle_camera(self):
        self.camera_preset = 1 - self.camera_preset
        teleop_render.set_camera_preset(self.cam, self.camera_preset)

    def toggle_collision_visualization(self):
        self.collision_viz = not self.collision_viz

    def set_arm_mode(self, side, mode):
        """이 손을 "ik"(EE 포즈 슬라이더) <-> "fk"(관절각 슬라이더)로 전환한다.
        전환 순간 팔이 튀지 않도록 방향에 따라 다르게 동기화한다:

        - ik -> fk: 지금 토크 제어기가 추종 중이던 관절각(q_des)을 그대로 FK
          슬라이더 값으로 복사 -- 전환 직후 첫 스텝의 목표 관절각이 전환 직전과
          정확히 같아서 포즈 점프가 없다.
        - fk -> ik: 반대로 EE 포즈 슬라이더 쪽이 전환 전 값(어쩌면 한참 전에 마지막
          으로 IK 모드였을 때의 낡은 목표)을 그대로 들고 있으므로, 그걸 쓰는 대신
          "지금 실제 site가 있는 월드 포즈"를 홈 기준 XYZ offset/RPY로 역산해 targets/
          smoothed_pos/smoothed_rpy에 다시 채워 넣는다 -- 그래야 IK가 방금 있던
          자리에서부터 이어서 풀리지, 옛 목표로 갑자기 끌려가지 않는다.
        """
        if mode == self.arm_mode[side]:
            return
        q_des = self.q_des_r if side == "r" else self.q_des_l

        if mode == "fk":
            self.fk_q_deg[side] = [math.degrees(v) for v in q_des]
        else:
            state = self.whole_body_solver.site_state(self.data, side)
            target_pos = self._world_to_target_pos(side, state.position)
            rpy_deg = self._world_quat_to_target_rpy(side, state.quaternion)

            self.targets[f"pos_{side}"] = target_pos
            self.targets[f"rpy_{side}"] = rpy_deg
            self.smoothed_pos[side] = np.array(target_pos)
            self.smoothed_rpy[side] = np.array(rpy_deg)

        self.arm_mode[side] = mode

    def _local_to_world_pos(self, p_local):
        return teleop_targets.local_to_world_pos(self, p_local)

    def _world_to_base_pos(self, p_world):
        return teleop_targets.world_to_base_pos(self, p_world)

    def _world_to_target_pos(self, side, world_pos):
        return teleop_targets.world_to_target_pos(self, side, world_pos)

    def _target_world_quat(self, side):
        return teleop_targets.target_world_quat(self, side)

    def _world_quat_to_target_rpy(self, side, world_quat):
        return teleop_targets.world_quat_to_target_rpy(self, side, world_quat)

    def _world_quat_to_virtual_rpy(self, world_quat):
        return teleop_targets.world_quat_to_virtual_rpy(self, world_quat)

    def _quat_to_mat(self, quat):
        return teleop_targets.quat_to_mat(quat)

    def _mat_to_quat(self, mat):
        return teleop_targets.mat_to_quat(mat)

    def _target_world_pose(self, side):
        return teleop_targets.target_world_pose(self, side)

    def _virtual_object_world_pose(self):
        return teleop_targets.virtual_object_world_pose(self)

    def set_whole_body_enabled(self, enabled):
        """Switch between whole-body and arm-only IK without moving a world target.

        The two modes deliberately use different UI frames: whole-body targets stay at the
        startup world anchor, while arm-only targets ride with the current chassis.  The
        numerical target values therefore have to be re-expressed at the transition.  We
        also clear every cached base command so OFF is an immediate stop request rather
        than a delayed weight change.
        """
        enabled = bool(enabled)
        if enabled == self.whole_body_enabled:
            return

        hand_world_poses = {
            side: tuple(value.copy() for value in self._target_world_pose(side))
            for side in ("r", "l")
        }
        virtual_world_pose = tuple(value.copy() for value in self._virtual_object_world_pose())
        self.whole_body_enabled = enabled

        virtual_pos, virtual_quat = virtual_world_pose
        if enabled:
            self.targets["virtual_object_pos"] = teleop_targets.world_to_anchor_local_pos(
                self, virtual_pos).tolist()
        else:
            self.targets["virtual_object_pos"] = self._world_to_base_pos(virtual_pos).tolist()
        self.targets["virtual_object_rpy"] = list(
            self._world_quat_to_virtual_rpy(virtual_quat))

        if self.cyclo_grasp_captured:
            # The captured virtual object is authoritative in bimanual MoveL mode.
            self.apply_virtual_object_target()
        else:
            for side, (world_pos, world_quat) in hand_world_poses.items():
                self.targets[f"pos_{side}"] = self._world_to_target_pos(
                    side, world_pos)
                self.targets[f"rpy_{side}"] = list(
                    self._world_quat_to_target_rpy(side, world_quat))

        for side in ("r", "l"):
            self.smoothed_pos[side] = np.asarray(
                self.targets[f"pos_{side}"], dtype=float).copy()
            self.smoothed_rpy[side] = np.asarray(
                self.targets[f"rpy_{side}"], dtype=float).copy()

        rebased_targets = {side: self._target_world_pose(side) for side in ("r", "l")}
        self.whole_body_solver.rebase(self.data, rebased_targets)
        self.whole_body_base_twist = base_teleop.BodyTwist()
        self.commanded_base_twist = base_teleop.BodyTwist()
        self._sync_ik_mocaps_from_targets()

    def toggle_whole_body_control(self):
        self.set_whole_body_enabled(not self.whole_body_enabled)

    def sync_virtual_object_to_hand_targets(self):
        teleop_targets.sync_virtual_object_to_hand_targets(self)

    def capture_grasp(self):
        """Cyclo-style `/capture_grasp true`: record both hand target poses relative to
        the virtual object marker. After this, `virtual_object_pos/rpy` becomes the command
        source and the two hand MoveL targets are derived from the captured offsets."""
        teleop_targets.capture_grasp(self)
        self.whole_body_solver.set_rigid_grasp(self.data, True)

    def release_grasp(self):
        """Cyclo-style `/capture_grasp false`: return to independent hand MoveL targets."""
        teleop_targets.release_grasp(self)
        self.whole_body_solver.set_rigid_grasp(self.data, False)

    def apply_virtual_object_target(self):
        teleop_targets.apply_virtual_object_target(self)

    def _active_gizmo_target(self):
        return teleop_targets.active_gizmo_target(self)

    def _gizmo_target_world_pose(self, target):
        return teleop_targets.gizmo_target_world_pose(self, target)

    def _set_gizmo_target_world_pose(self, target, world_pos, world_quat):
        teleop_targets.set_gizmo_target_world_pose(self, target, world_pos, world_quat)

    def _pose_to_imguizmo_matrix(self, world_pos, world_quat):
        return teleop_render.pose_to_imguizmo_matrix(self, world_pos, world_quat)

    def _imguizmo_matrix_to_pose(self, matrix):
        return teleop_render.imguizmo_matrix_to_pose(self, matrix)

    def _sync_ik_mocaps_from_targets(self):
        teleop_targets.sync_ik_mocaps_from_targets(self)

    # ------------------------------------------------------------------
    # 메인 루프
    # ------------------------------------------------------------------

    def run(self):
        # 매 프레임 (1) 입력 처리 (2) IK 풀기 (3) 물리 스텝 (4) 렌더링을 전부 한
        # 스레드/한 루프 안에서 순서대로 실행한다.
        while not glfw.window_should_close(self.window):
            t0 = time.perf_counter()
            io = teleop_render.begin_frame(self)

            teleop_render.handle_camera_mouse(self, io)
            self._handle_edge_keys(io)
            drive_keys = self._read_drive_and_lift_keys(io)
            self._draw_ui_panel()
            self._step_physics(drive_keys)
            teleop_render.render_scene(self)
            teleop_render.end_frame(self, t0)

        teleop_render.shutdown(self)

    def _handle_edge_keys(self, io):
        """눌렀다 뗄 때 한 번만 반응하는 R/G/V/C 유틸리티 키."""
        if io.want_capture_keyboard:
            return
        if self.keys.pressed(self.window, glfw.KEY_R):
            self.reset_active_object()
        if self.keys.pressed(self.window, glfw.KEY_G):
            self.contact_viz = not self.contact_viz
        if self.keys.pressed(self.window, glfw.KEY_V):
            self.toggle_collision_visualization()
        if self.keys.pressed(self.window, glfw.KEY_C):
            self.cycle_camera()

    def _read_drive_and_lift_keys(self, io):
        """continuous drive/lift keys (level-triggered, not edge-triggered like R/G/V/C
        above -- driving should keep responding for as long as the key is held).
        (한글) 주행/리프트 키는 누르고 있는 동안 계속 반응해야 하므로(edge-triggered
        아님) 매 프레임 눌림 상태를 그대로 읽는다.

        Arrow keys + brackets (no WASD): Up/Down = forward/back, Left/Right = yaw,
        [ / ] = strafe left/right -- WASD was dropped since it collides with the
        keybindings a MuJoCo user already expects from other tools. Strafe moved off
        Shift+Left/Right (Session 11, per user request) onto its own dedicated keys so it
        doesn't share the Left/Right yaw keys at all -- no modifier to hold down.
        """
        drive_keys = {"w": False, "a": False, "s": False, "d": False, "left": False, "right": False}
        lift_dir = 0.0
        if not io.want_capture_keyboard:
            drive_keys["w"] = glfw.get_key(self.window, glfw.KEY_UP) == glfw.PRESS
            drive_keys["s"] = glfw.get_key(self.window, glfw.KEY_DOWN) == glfw.PRESS
            drive_keys["left"] = glfw.get_key(self.window, glfw.KEY_LEFT) == glfw.PRESS
            drive_keys["right"] = glfw.get_key(self.window, glfw.KEY_RIGHT) == glfw.PRESS
            drive_keys["a"] = glfw.get_key(self.window, glfw.KEY_LEFT_BRACKET) == glfw.PRESS
            drive_keys["d"] = glfw.get_key(self.window, glfw.KEY_RIGHT_BRACKET) == glfw.PRESS
            if glfw.get_key(self.window, glfw.KEY_E) == glfw.PRESS:
                lift_dir += 1.0
            if glfw.get_key(self.window, glfw.KEY_Q) == glfw.PRESS:
                lift_dir -= 1.0
        if lift_dir != 0.0:
            self.targets["lift"] = float(np.clip(
                self.targets["lift"] + lift_dir * 0.3 * self.frame_dt, LIFT_RANGE[0], LIFT_RANGE[1]))
        return drive_keys

    def _draw_ui_panel(self):
        """상태 창과 탭형 Control/Diagnostics 워크스페이스를 그린다.
        실제 창 배치는 teleop_ui.draw_panel로 옮겼다(그 모듈 docstring 참고) --
        여기서 바뀌는 값은 전부 self.targets로만 들어가고, 물리 반영은
        _step_physics에서 이뤄진다."""
        teleop_ui.draw_panel(self)

    def _read_base_feedback(self):
        """Read wheel and chassis feedback used by manual/base handover control."""
        data = self.data
        steering_positions = {
            wheel: float(data.qpos[address])
            for wheel, address in self.wheel_steer_qadrs.items()
        }
        wheel_velocities = {
            wheel: float(data.qvel[address])
            for wheel, address in self.wheel_drive_dofs.items()
        }
        yaw = float(data.qpos[self.base_yaw_qadr])
        cosine, sine = math.cos(yaw), math.sin(yaw)
        vx_world = float(data.qvel[self.base_x_dof])
        vy_world = float(data.qvel[self.base_y_dof])
        body_twist = base_teleop.BodyTwist(
            cosine * vx_world + sine * vy_world,
            -sine * vx_world + cosine * vy_world,
            float(data.qvel[self.base_yaw_dof]),
        )
        base_pose = np.array(
            [
                data.qpos[self.base_x_qadr],
                data.qpos[self.base_y_qadr],
                data.qpos[self.base_yaw_qadr],
            ],
            dtype=float,
        )
        return steering_positions, wheel_velocities, body_twist, base_pose

    def _update_grasp_targets(self):
        """Rate-limit one-touch open/close commands into the hand sliders."""
        for side in SIDES:
            state = self.grab_state[side]
            if state is None:
                continue
            desired = 1.0 if state else 0.0
            for name in (f"grasp_{side}", f"thumb_{side}"):
                delta = np.clip(
                    desired - self.targets[name], -self.frame_dt, self.frame_dt
                )
                self.targets[name] += float(delta)

    def _smooth_hand_targets(self):
        """Limit slider target motion to rates the IK/controller can track."""
        for side in SIDES:
            raw_position = np.asarray(self.targets[f"pos_{side}"], dtype=float)
            position_delta = np.clip(
                raw_position - self.smoothed_pos[side],
                -MAX_POS_STEP_PER_FRAME,
                MAX_POS_STEP_PER_FRAME,
            )
            self.smoothed_pos[side] += position_delta

            raw_rpy = np.asarray(self.targets[f"rpy_{side}"], dtype=float)
            rpy_delta = np.clip(
                raw_rpy - self.smoothed_rpy[side],
                -MAX_RPY_STEP_PER_FRAME_DEG,
                MAX_RPY_STEP_PER_FRAME_DEG,
            )
            self.smoothed_rpy[side] += rpy_delta

    def _smoothed_target_poses(self):
        """Build world-space poses from the rate-limited UI targets."""
        return {
            side: (
                teleop_targets.target_pos_to_world_pos(
                    self, side, self.smoothed_pos[side]
                ),
                teleop_targets.target_rpy_to_world_quat(
                    self, side, self.smoothed_rpy[side]
                ),
            )
            for side in SIDES
        }

    def _step_actuators(self, wheel_commands):
        """Apply the current command set throughout one rendered frame."""
        data = self.data
        for _ in range(self.steps_per_frame):
            self.ctrl_r.apply(data, self.q_des_r)
            self.ctrl_l.apply(data, self.q_des_l)
            data.ctrl[self.lift_aid] = self.lift_cmd
            for wheel, (steer_angle, drive_speed) in wheel_commands.items():
                data.ctrl[self.wheel_steer_aids[wheel]] = steer_angle
                data.ctrl[self.wheel_drive_aids[wheel]] = drive_speed
            for side in SIDES:
                grasp.apply_grasp(
                    self.model,
                    data,
                    grasp=self.targets[f"grasp_{side}"],
                    thumb=self.targets[f"thumb_{side}"],
                    side=side,
                )
            mujoco.mj_step(self.model, data)

    def _step_physics(self, drive_keys):
        """실제 물리 반영: target rate-limit -> world-fixed pose -> whole-body solve ->
        팔 torque/lift position/swerve/grasp actuator ctrl -> ``mj_step``. Solver는 live
        qpos를 읽기만 하며, robot qpos를 직접 쓰는 kinematic override는 없다."""
        data = self.data
        (
            steering_positions,
            wheel_velocities,
            measured_body_twist,
            current_base_pose,
        ) = self._read_base_feedback()
        # Use chassis feedback for the manual-to-WBIK stop handover. During active braking,
        # temporary tire slip makes wheel-only odometry a worse stop detector than the
        # model's directly observable planar base velocity.
        manual_twist = self.base_drive.base.update_body(
            drive_keys, self.frame_dt, measured_body_twist)
        manual_keys_active = any(drive_keys.values())
        measured_motion_active = (
            math.hypot(measured_body_twist.vx, measured_body_twist.vy) > 0.01
            or abs(measured_body_twist.wz) > 0.02)
        if manual_keys_active and not self._manual_override_active:
            # Rising edge: do not mistake any preceding WBIK motion for manual motion.
            self._manual_reference_base_pose = current_base_pose.copy()
        carry_manual_targets = manual_keys_active or self._manual_override_active
        if carry_manual_targets:
            # Includes the falling edge so the last frame of physical braking is captured.
            teleop_targets.carry_world_targets_with_base(
                self, self._manual_reference_base_pose, current_base_pose)
        self._manual_reference_base_pose = current_base_pose.copy()

        if self.cyclo_grasp_captured:
            self.apply_virtual_object_target()
        self._update_grasp_targets()
        self._smooth_hand_targets()

        # World-anchored targets are essential for whole-body IK: if a target were rebuilt
        # from the *current* base pose each frame, moving the base would move the goal by the
        # same amount and could never reduce task error.
        target_poses = self._smoothed_target_poses()

        if carry_manual_targets:
            # Handover must redefine zero common motion at the new chassis pose; otherwise
            # startup references make WBIK issue a reverse command that feels like inertia.
            self.whole_body_solver.rebase(data, target_poses)

        active_sides = tuple(side for side in SIDES if self.arm_mode[side] == "ik")
        whole_body_cmd = self.whole_body_solver.solve(
            data, target_poses, self.frame_dt,
            active_sides=active_sides,
            arm_nominal={"r": HOME_Q_R, "l": HOME_Q_L},
            lift_nominal=self.targets["lift"],
            rigid_grasp=(self.cyclo_controller == "bimanual_movel"
                         and self.cyclo_grasp_captured),
            whole_body_enabled=self.whole_body_enabled,
        )
        self.whole_body_base_twist = whole_body_cmd.base_twist
        self.collision_active_pairs = whole_body_cmd.active_collision_pairs
        self.collision_min_distance = whole_body_cmd.minimum_collision_distance
        self.collision_constraint_violation = whole_body_cmd.collision_constraint_violation
        self.lift_cmd = (whole_body_cmd.lift_position if self.whole_body_enabled
                         else self.targets["lift"])
        for side in SIDES:
            if side in whole_body_cmd.arm_positions:
                if side == "r":
                    self.q_des_r = whole_body_cmd.arm_positions[side]
                else:
                    self.q_des_l = whole_body_cmd.arm_positions[side]
                self.ik_err_mm[side] = whole_body_cmd.position_errors[side] * 1000.0
            else:
                if side == "r":
                    self.q_des_r = np.radians(self.fk_q_deg[side])
                else:
                    self.q_des_l = np.radians(self.fk_q_deg[side])

        # Explicit keyboard motion has priority. Release commands zero until chassis feedback
        # confirms a stop; only then may whole-body IK resume through the same swerve path.
        if manual_keys_active:
            self.commanded_base_twist = manual_twist
        elif self._manual_override_active:
            self.commanded_base_twist = base_teleop.BodyTwist()
        elif self.whole_body_enabled:
            self.commanded_base_twist = self.whole_body_base_twist
        else:
            self.commanded_base_twist = base_teleop.BodyTwist()
        previous_manual_override = self._manual_override_active
        wheel_cmds = self.base_drive.update_twist(
            self.commanded_base_twist, self.frame_dt,
            steering_positions, wheel_velocities)
        self._manual_override_active = bool(
            manual_keys_active
            or (previous_manual_override
                and measured_motion_active))
        if previous_manual_override and not self._manual_override_active:
            self.base_drive.base.reset_motion()

        self._step_actuators(wheel_cmds)

def _parse_args(argv):
    parser = argparse.ArgumentParser(description="FFW-SH5 teleop app")
    parser.parse_args(argv)


def main(argv=None):
    _parse_args(sys.argv[1:] if argv is None else argv)
    TeleopApp().run()


if __name__ == "__main__":
    main()
