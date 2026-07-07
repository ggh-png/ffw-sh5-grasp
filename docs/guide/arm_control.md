# `src/arm_control.py` — 팔 토크 제어 (PD + 중력/코리올리 feedforward)

## 이 파일이 하는 일

`ArmTorqueController` 클래스 하나. `ik.py`가 계산해준 목표 관절각(`q_des`)을 실제
토크로 바꿔 로봇 팔을 움직인다. MuJoCo의 내장 `<position>` 액추에이터(목표각을
주면 알아서 PD 제어)를 쓰지 않고, 팔 관절은 `<motor>`(순수 토크) 액추에이터로
바꾼 뒤 이 클래스가 매 물리 스텝 직접 토크를 계산해서 쓴다.

## 구현: 왜 `<position>`이 아니라 직접 토크 제어인가

파일 docstring에 그 진단 과정이 그대로 남아 있다:

> MuJoCo의 내장 `<position>` 액추에이터는 적분항도 feedforward도 없는 순수
> 비례항이다. 정적인 자세를 유지하는 데만도 그 자세를 버티는 토크만큼 정상상태
> 오차(약 15~20mm)가 남았다. 원인 후보 3가지를 순서대로 검사해 기각했다:
> (1) 텔레포트 테스트(오차 0.004mm — IK 자체는 정확함), (2) 관절별 토크 포화
> 여부(포화 안 됨), (3) ctrl 클램핑(정상). 60초 정착 테스트에서 오차가 서서히
> 줄어드는 걸 보고, 7-링크 결합계가 `<position>`의 관절별 `dampratio=1` 가정
> (각 관절이 독립 SISO계라는 가정)만큼 실제로는 임계감쇠되지 않는다는 결론을 냈다.

해결책은 표준 로봇팔 제어 법칙이다:

```python title="src/arm_control.py — apply() 본문"
def apply(self, data, q_des):
    q = np.array([data.qpos[a] for a in self.qpos_adrs])
    qd = data.qvel[self.dof_ids]
    qfrc_bias = data.qfrc_bias[self.dof_ids]
    tau = qfrc_bias + self.kp * (np.asarray(q_des) - q) - self.kd * qd
    for aid, t in zip(self.actuator_ids, tau):
        lo, hi = self.model.actuator_ctrlrange[aid]
        data.ctrl[aid] = np.clip(t, lo, hi)
    return tau
```

`τ = qfrc_bias + kp·(q_des − q) − kd·q̇`. 세 항의 역할이 다르다:

- **`qfrc_bias`** — MuJoCo가 매 스텝 계산해주는 "중력 + 코리올리 + 원심력" 항을
  그대로 더한다. 이게 정상상태 오차를 없애는 핵심이다: 로봇 동역학 방정식을
  직접 유도할 필요 없이, "지금 이 자세를 버티는 데 필요한 힘"을 MuJoCo가 대신
  계산해준 값을 그대로 상쇄해버리는 것 — **kp를 올리는 것과는 완전히 다른 얘기**다
  (5배로 올려도 정상상태 오차는 거의 안 줄었다).
- **`kp·(q_des − q)`** — 목표 관절각과의 오차에 비례하는 위치 피드백.
- **`−kd·q̇`** — 각속도에 비례하는 감쇠(능동 댐핑), 결합계 전체를 대상으로 튜닝된
  값(`kd=40`)이라 `<position>`의 관절별 독립 가정과 달리 실제로 동작한다.

생성자는 관절 이름마다 그 관절을 구동하는 `<motor>` actuator id를 미리 찾아
캐싱해둔다(매 스텝 다시 찾지 않도록):

```python title="src/arm_control.py — __init__"
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
```

손가락 액추에이터는 이 대상이 아니다 — 손은 여전히 force-limited `<position>`
액추에이터를 그대로 쓴다(`grasp.py` 참고). 접촉 시 토크가 알아서 포화되며 캔
형상에 순응하는 그립 동작이 바로 그 "정확하지 않은" `<position>`의 특성 덕분이라,
여기서는 오히려 의도된 동작이다 — 이 파일이 고치는 건 **강체 팔 위치 제어
문제**뿐이다.

## 이 파일이 다른 파일과 합쳐지는 방식

- **`teleop_app.py`**가 유일한 사용자다. `TeleopApp._setup_sim()`에서 손당 하나씩
  생성(`self.ctrl_r = arm_control.ArmTorqueController(model, ARM_R)`)하고,
  `_step_physics()`의 물리 서브스텝 루프 안에서 **매 스텝** `ctrl_r.apply(data,
  self.q_des_r)`를 호출한다.
- `q_des_r`/`q_des_l`이 어디서 오는지는 그 손의 `arm_mode`에 따라 달라지지만
  (`ik.py`의 `solve_pose` 결과이거나, `teleop_ui.py`의 관절각 슬라이더 값이거나),
  **`arm_control.py`는 그 출처를 전혀 모른다** — 그냥 "지금 주어진 목표 관절각을
  향해 토크를 낸다"만 한다. 이 무관심 덕분에 IK 모드/FK 모드 전환이 `arm_control.py`
  쪽 코드는 한 줄도 안 건드리고 성립한다(`teleop_app.md`의 `set_arm_mode` 참고).
- `ik.py`/`grasp.py`/`base_teleop.py`와 직접적인 호출 관계는 없다 — 전부
  `teleop_app.py`를 통해서만 간접적으로 연결된다.
