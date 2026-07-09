# `src/teleop_app.py` — 텔레옵 앱, 모든 모듈이 실제로 합쳐지는 지점

## 이 파일이 하는 일

`TeleopApp` 클래스 하나가 이 프로젝트 전체를 하나의 실행 가능한 앱으로 묶는다.
GLFW 창 하나를 띄우고, 그 위에 MuJoCo의 저수준 렌더 API로 3D 장면을, Dear ImGui로
슬라이더 패널을 함께 그리면서, 매 프레임 (1) 입력 처리 (2) IK/FK (3) 물리 스텝
(4) 렌더링을 순서대로 실행한다. 다른 다섯 파일(`ik`, `arm_control`, `grasp`,
`base_teleop`, `teleop_ui`)은 전부 이 파일에서 import되어 조립된다 — **이 파일을
읽으면 나머지 파일들이 정확히 언제, 어떤 순서로, 어떤 데이터를 주고받으며
호출되는지 전부 드러난다.**

## 구현: 클래스 하나, 메서드 여러 개

`__init__`이 세 그룹으로 나눠 초기화한다:

```python title="src/teleop_app.py — TeleopApp.__init__"
def __init__(self):
    self._setup_sim()      # 모델/솔버/제어기/슬라이더 목표값 -- 렌더링과 무관한 전부
    self._setup_render()   # GLFW 창 + MuJoCo 렌더 컨텍스트
    self._setup_loop_state()  # IK 웜스타트 값, 타이밍, 입력 헬퍼
```

`run()`은 그 자체로 이 프로젝트의 데이터 흐름을 요약하는 6줄이다:

```python title="src/teleop_app.py — 메인 루프"
while not glfw.window_should_close(self.window):
    t0 = time.perf_counter()
    glfw.poll_events()
    self.impl.process_inputs()
    imgui.new_frame()
    io = imgui.get_io()

    self._handle_camera_mouse(io)
    self._handle_edge_keys(io)             # R(현재 물체 리셋)/G(접촉 시각화)/C(카메라)
    drive_keys = self._read_drive_and_lift_keys(io)
    self._draw_ui_panel()                  # teleop_ui.draw_panel(self) 호출
    self._step_physics(drive_keys)         # ik/arm_control/grasp/base_teleop 전부 호출
    self._render_scene()
    self._end_frame(t0)
```

각 메서드는 원래 하나의 거대한 `main()` 함수 안에 지역 변수로 얽혀 있던 코드를
그대로 단계별 메서드로 나눈 것뿐이라, 동작 자체는 나누기 전과 완전히 같다 — 상태는
전부 `self.*` 속성으로 공유된다.

### `_step_physics` — 실제로 모든 모듈이 호출되는 곳

```python title="src/teleop_app.py — _step_physics 핵심 (오른손 IK/FK 분기)"
if self.arm_mode["r"] == "ik":
    quat_r = ...  # home_quat_r ⊗ rpy_delta, base_quat 적용
    pos_r_world = local_to_world_pos(self.smoothed_pos["r"])
    iter_r = STUCK_MAX_ITER if self.stuck_counter["r"] >= STUCK_FRAMES_THRESHOLD else IK_MAX_ITER
    self.q_des_r, perr_r, _ = self.solver_r.solve_pose(self.q_des_r, pos_r_world, quat_r,
                                                        max_iter=iter_r, context_qpos=ctx_qpos)
    ...
else:
    self.q_des_r = np.radians(self.fk_q_deg["r"])   # FK: IK를 아예 건너뜀

wheel_cmds = self.base_drive.update(drive_keys, self.frame_dt, base_yaw)

for _ in range(self.steps_per_frame):
    self.ctrl_r.apply(data, self.q_des_r, kp_scale=arm_kp_scale)     # arm_control.py
    self.ctrl_l.apply(data, self.q_des_l, kp_scale=arm_kp_scale)
    data.ctrl[self.lift_aid] = self.targets["lift"]
    for wheel, (steer_angle, drive_angvel) in wheel_cmds.items():
        data.ctrl[self.wheel_steer_aids[wheel]] = steer_angle
        data.ctrl[self.wheel_drive_aids[wheel]] = drive_angvel
    if self.scenario == "box":
        grasp.apply_open_hand(model, data, side="r")
        grasp.apply_open_hand(model, data, side="l")
    else:
        grasp.apply_grasp(model, data, grasp=self.targets["grasp_r"], thumb=self.targets["thumb_r"], side="r")
        grasp.apply_grasp(model, data, grasp=self.targets["grasp_l"], thumb=self.targets["thumb_l"], side="l")
    mujoco.mj_step(model, data)
```

렌더 프레임 하나(`frame_dt`, 25Hz)당 물리 스텝을 여러 번(`steps_per_frame`) 돌리는
이유는 렌더링은 25Hz면 충분해도 물리는 훨씬 촘촘한 timestep이 필요하기 때문이다.
IK는 프레임당 한 번만 풀지만, `arm_control.apply`/`grasp.apply_grasp`/바퀴 ctrl은
**매 물리 서브스텝마다 다시 적용**된다 — 목표는 프레임당 한 번만 바뀌어도, 그
목표를 향한 토크 계산 자체는 물리 스텝만큼 촘촘하게 다시 해야 하기 때문이다.

`can`과 `box`는 실행 인자로 시작 시나리오를 고른다(`python3 src/teleop_app.py can|box`).
두 물체는 같은 모델 안에 함께 있지만, 비활성 물체는 화면 밖으로 파킹되고 충돌/렌더가
꺼진다. `box` 모드에서는 손가락 synergy를 쓰지 않고 양손을 open hand로 유지한 채,
`squeeze_gap`으로 상자 양옆 target 간격을 조이고 `is_box_held()`가 성립하면
`bimanual_constraint.project_desired_delta()`가 두 팔의 목표 관절 델타를 상대 pose 유지
manifold에 투영한다.

화면의 IK target 마커는 숫자 X/Y/Z + Roll/Pitch/Yaw target을 보여주는 표시용이다.
마우스 드래그로 pose를 바꾸지 않고, `_sync_ik_mocaps_from_targets()`가 매 렌더 프레임
마커 mocap pose를 슬라이더 target에 맞춰 따라가게 한다.

### IK ↔ FK 모드 전환 — `set_arm_mode`

리프트를 움직이는 동안 IK가 매 프레임에만 어깨 높이를 다시 읽어 들이는데 리프트는
프레임 사이에도 계속 움직여서 생기는 출렁임을, 손별로 관절각을 직접 고정하는
FK 모드로 회피할 수 있다(팔이 리프트에 강체로 붙어 그대로 오르내리기만 함). 전환
순간 팔이 튀지 않도록 방향에 따라 다르게 동기화한다:

```python title="src/teleop_app.py — set_arm_mode"
if mode == "fk":
    # ik -> fk: 지금 추종 중이던 관절각(q_des)을 그대로 FK 슬라이더 값으로 복사
    # -- 전환 직후 첫 스텝의 목표 관절각이 전환 직전과 정확히 같아 점프가 없다.
    self.fk_q_deg[side] = [math.degrees(v) for v in q_des]
else:
    # fk -> ik: "지금 실제 site가 있는 월드 포즈"를 베이스-로컬 좌표/RPY로 역산해
    # targets/smoothed_pos/smoothed_rpy에 채워 넣는다 -- 낡은 목표로 갑자기
    # 끌려가지 않고, 방금 있던 자리에서부터 IK가 이어서 풀린다.
    ...
    rpy_deg = quat_to_rpy_deg(rpy_delta_quat)
    self.targets[f"pos_{side}"] = local_pos
    self.targets[f"rpy_{side}"] = rpy_deg
    self.smoothed_pos[side] = np.array(local_pos)
    self.smoothed_rpy[side] = np.array(rpy_deg)
```

fk→ik 방향은 순전히 기하 변환이다: 실제 site의 월드 위치/자세를,
`_step_physics`가 쓰는 것과 정확히 같은 base-local + home-relative-RPY 변환의
**역변환**으로 되짚는다. 이 역변환이 정확한지는 render 비교와 `solve_pose`가
그 결과에서 즉시(0 iteration에 가깝게) 수렴하는지로 직접 검증했다.

### 렌더링 — 왜 `mujoco.viewer.launch_passive`가 아닌가

`mujoco.viewer.launch_passive`는 편리한 내장 뷰어지만 자체 창을 소유해서 커스텀
위젯을 넣을 훅이 없다. 대신 MuJoCo의 C++ "simulate" 앱과 같은 방식으로, 하나의
GLFW 창 안에서 저수준 API(`MjvScene`/`MjrContext`/`mjv_updateScene`/`mjr_render`)를
직접 호출해 3D 장면을 그리고, 그 위에 같은 프레임버퍼로 Dear ImGui를 그린다
(`_render_scene`).

```python title="src/teleop_app.py — _render_scene"
mujoco.mjv_updateScene(self.model, self.data, self.opt, self.pert, self.cam,
                        mujoco.mjtCatBit.mjCAT_ALL, self.scene)
mujoco.mjr_render(viewport, self.scene, self.context)
imgui.render()
self.impl.render(imgui.get_draw_data())
glfw.swap_buffers(self.window)
```

### `src/imgui.ini` — 코드가 아니라 ImGui가 쓰는 파일

같은 디렉터리의 `imgui.ini`는 사람이 작성한 코드가 아니라, `imgui.create_context()`
+ `GlfwRenderer`가 앱을 실행할 때마다 자동으로 읽고 쓰는 **창 레이아웃 상태 파일**
(패널 위치/크기)이다. 10줄짜리 텍스트(`[Window][FFW-SH5 Teleop] Pos=... Size=...`)
뿐이라 구현 로직은 없다 — 다음에 앱을 켰을 때 패널이 마지막으로 둔 자리에 그대로
뜨게 해주는 역할만 한다.

## 이 파일이 다른 파일과 합쳐지는 방식 (전체 그림)

```text
teleop_app.py
├── import ik            → InverseKinematics 인스턴스 2개 (손당 하나)
├── import arm_control   → ArmTorqueController 인스턴스 2개 (손당 하나)
├── import grasp         → apply_grasp()를 물리 서브스텝마다 양손 호출
├── import base_teleop   → SwerveDrive 인스턴스 1개, 프레임당 한 번 호출
└── import teleop_ui     → draw_panel(self)를 프레임당 한 번 호출 (app 자신을 넘김)
```

- **ik.py / arm_control.py**는 손마다 짝을 이뤄 협업한다: `ik`가 "어디로"를,
  `arm_control`이 "어떻게(토크)"를 담당하고, 그 사이를 잇는 게 `q_des_r`/`q_des_l`
  뿐이다. FK 모드에서는 이 짝의 앞쪽(`ik`)이 빠지고 `teleop_ui`의 슬라이더 값이
  그 자리를 대신한다 — `arm_control` 쪽 코드는 이 사실을 전혀 모른다.
- **grasp.py**는 팔과 완전히 독립된 액추에이터(손가락)를 담당해서, 같은 물리
  서브스텝 루프 안에서 그냥 나란히 호출된다. `box` 모드에서는 `apply_open_hand()`와
  `get_box_hand_contacts()`/`is_box_held()`가 쓰이고, `can` 모드에서는 기존
  `apply_grasp()`/`is_grasped()` 경로가 쓰인다.
- **base_teleop.py**는 프레임당 한 번만 호출되고(물리 서브스텝마다는 아님), 그
  결과(`wheel_cmds`)만 서브스텝 루프 안에서 반복 사용된다 — IK/토크제어보다 훨씬
  가벼운 계산이라 프레임당 재계산으로 충분하다.
- **teleop_ui.py**는 유일하게 "값을 읽는 쪽"이 아니라 "`self`(=app) 객체 자체를
  받아 그 상태를 직접 읽고 쓰는" 모듈이다 — 자세한 이유는 `teleop_ui.md` 참고.
- 물리 상태(`data`)에 대한 예외적 직접 쓰기는 `reset_active_object()`와 시나리오 전환의
  물체 초기 배치/파킹뿐이다 — 자유물체의 리셋/초기 배치로, 로봇 관절에 대한 kinematic
  치팅이 아니다.
