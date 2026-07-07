# `src/ik.py` — 6DOF 역기구학(IK) 솔버

## 이 파일이 하는 일

`InverseKinematics` 클래스 하나가 이 파일의 전부다. 생성할 때 site 이름과 담당할
관절 이름 목록을 받아두고(`InverseKinematics(model, "grasp_target_r", ARM_R)`),
`solve_pose(q_init, target_pos, target_quat, ...)`를 호출하면 그 site가 목표
위치/자세에 도달하는 관절각을 damped least squares(DLS)로 반복 계산해 돌려준다.
**실시간 시뮬레이션의 `data`에는 절대 손대지 않는다** — 생성자에서 만들어 두는
자기 소유의 `self._scratch` (`mujoco.MjData`) 안에서만 `mj_forward`를 반복 호출해
기구학만 계산하고, 결과 관절각을 배열로 반환할 뿐이다. 그 배열을 실제로 로봇에
적용하는 건(액추에이터 ctrl에 쓰는 것) 전적으로 호출자의 몫이다.

## 구현: 왜 이렇게 복잡한가

### site를 잡는 이유

목표는 body 원점이 아니라 **site** (`grasp_target_r`/`grasp_target_l`, 손바닥
기준 오프셋에 미리 심어둔 참조점)다. `mj_jacSite`가 이 site의 위치/회전 Jacobian을
바로 계산해주므로, body 원점 기준 offset을 손으로 보정할 필요가 없다.

### 위치만 먼저, 그다음 자세

```python title="src/ik.py — solve_position (3DOF)"
def solve_position(self, q_init, target_pos, max_iter=DEFAULT_MAX_ITER, tol=POS_TOL,
                   context_qpos=None):
    scratch = self._scratch
    mujoco.mj_resetData(self.model, scratch)
    if context_qpos is not None:
        scratch.qpos[:] = context_qpos
    q = np.array(q_init, dtype=float)
    ...
    for _ in range(max_iter):
        cur_pos = scratch.site_xpos[self.site_id]
        err = target_pos - cur_pos
        if float(np.linalg.norm(err)) < tol:
            break
        jacp, _ = self._jac(scratch)
        dq = np.clip(self._dls_step(jacp, err), -self.max_joint_delta, self.max_joint_delta)
        q = self._clamp_to_limits(q + dq)
        ...
```

`_dls_step`이 핵심 수식이다: `dq = Jᵀ(JJᵀ + λ²I)⁻¹ e`. 일반 역행렬(`J⁻¹e`) 대신
감쇠항 `λ²I`를 더하는 이유는, 팔이 특이 자세(singularity) 근처에 오면 `J`가 거의
특이행렬이 되어 역행렬이 폭발하기 때문 — DLS는 그 근처에서 정확도를 살짝 희생하는
대신 발산을 막는다.

이 3DOF 버전이 수렴을 검증하기 위한 1단계였고, 실제로 쓰는 건 위치+자세를 함께
푸는 `solve_pose()`다. 이 둘을 하나의 6×n 스택 Jacobian으로 한 번에 풀면
**발산한다** — 자세 오차가 크면 그걸 줄이려는 보정이 위치를 흔들고, 그 반대도
마찬가지였기 때문이다. 그래서 `solve_pose`는 계층형(hierarchical) 방식을 쓴다:

```python title="src/ik.py — solve_pose 반복 한 스텝"
site_R = scratch.site_xmat[self.site_id].reshape(3, 3)
ori_err_world = site_R @ ori_err   # site-local 오차를 world로 회전

jacp, jacr = self._jac(scratch)
JJt_inv = np.linalg.inv(jacp @ jacp.T + lam2 * np.eye(3))

dq_pos = jacp.T @ (JJt_inv @ pos_err)                       # 1) 위치 먼저
g_ori = jacr.T @ ori_err_world
dq_ori = g_ori - jacp.T @ (JJt_inv @ (jacp @ g_ori))        # 2) 방향은 위치의 영공간에 투영
dq_full = np.clip(dq_pos + dq_ori, -self.max_joint_delta, self.max_joint_delta)
```

방향 보정(`dq_ori`)을 위치 Jacobian의 **영공간(null space)**에 투영하는 게 핵심 —
위치에 영향을 주지 않는 성분만 반영해서, 자세를 맞추려다 위치가 흔들리는 문제를
없앤다. `mju_subQuat`로 구한 방향 오차(`ori_err`)는 site 로컬 프레임인데
`jacr`(회전 Jacobian)은 월드 프레임이라, `site_R`로 먼저 회전시켜 프레임을 맞춰야
한다 — 이 변환을 빼먹으면 오차 gradient가 엉뚱한 관절로 새어나간다.

그래도 오차가 클 때는 반복할수록 진동하는 경우가 있어서, 매 반복마다
**backtracking line search**를 건다: 전체 스텝을 그대로 쓰지 않고, 실제로 비용
(위치오차 + 가중치×방향오차)이 줄어드는지 확인하면서 스텝을 최대 6번까지 절반씩
줄여나간다. 그래도 안 줄면 가장 작았던 시도를 채택해 최소한 발산은 하지 않게 한다.

### `context_qpos` — 내가 안 푸는 관절은 어떻게 되는가

```python
context_qpos: full-model qpos to seed every *other* joint from (e.g. models/
full_scene.xml's lift_joint, which sits upstream of the arm chain and is not part
of this solver's own joint_names) -- without it those joints reset to 0, silently
moving the whole chain's base to the wrong place
```

`mj_resetData`는 scratch를 완전히 초기화하므로, `full_scene.xml`처럼 팔 위에
`lift_joint`가 있는 모델에서는 그 값도 매번 시드해줘야 한다. 안 그러면 리프트가
0으로 리셋되며 어깨 자체가 엉뚱한 높이로 튀어버린다 — `teleop_app.py`는 매 프레임
`ctx_qpos = data.qpos.copy()`를 만들어 넘겨준다.

### `solve_pose_multistart` — 국소해 탈출

`solve_pose` 한 번으로는 큰 자세 변화에서 국소해(local minimum)나 관절 한계
lockup에 빠질 수 있다. `solve_pose_multistart`는 `q_init`(보통 이전 프레임의
해)을 먼저 시도하고, 실패하면 관절 range 안에서 무작위로 뽑은 후보를 여러 번(기본
8회) 재시도해 가장 나은 결과를 채택한다 — 테스트(`tests/test_phase_3.py`의 IK
단위테스트)에서만 쓰이고, 실시간 텔레옵 루프는 매 프레임 예산이 빠듯해 `solve_pose`
단발만 쓴다(아래 참고).

## 이 파일이 다른 파일과 합쳐지는 방식

- **`teleop_app.py`**가 유일한 실시간 사용자다. `TeleopApp._setup_sim()`에서 손당
  하나씩 생성(`self.solver_r = ik.InverseKinematics(model, "grasp_target_r",
  ARM_R)`)해두고, `_step_physics()`에서 그 손이 `arm_mode == "ik"`일 때만 매 프레임
  `solve_pose()`를 한 번 호출한다 — `arm_mode == "fk"`인 손은 이 클래스를 아예
  건드리지 않는다(관절각 슬라이더 값을 직접 쓴다, `teleop_ui.md` 참고).
- `solve_pose`가 반환하는 관절각(`q_des_r`/`q_des_l`)은 IK 결과 그 자체로 로봇을
  움직이는 게 아니라, **`arm_control.py`의 `ArmTorqueController.apply()`에 넘겨지는
  "목표"**일 뿐이다. 실제로 토크를 계산해서 관절을 움직이는 건 `arm_control.py`의
  몫 — `ik.py`는 "어디로 가야 하는지"만 계산하고 "어떻게 그 힘을 낼지"는 전혀
  모른다. 이 경계가 이 프로젝트에서 반복해서 강조하는 지점이다: IK도, 실제 물리
  시뮬레이션도 둘 다 `data.qpos`를 직접 쓰지 않는다.
- `tests/test_phase_3.py`/`test_phase_4.py`는 `teleop_app.py` 없이 `ik.py`만
  단독으로 임포트해 IK 단위테스트(랜덤 타겟 100개)와 통합 pick 시퀀스를 검증한다.
