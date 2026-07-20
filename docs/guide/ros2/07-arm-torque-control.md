[← 전체 안내](../ros2-guide.md)

# Part 7 — 팔 토크 제어: `src/arm_control.py` {: #part-7 }

!!! info "함께 볼 개발자 가이드"
    클래스 생성 인자와 `apply()`의 실제 데이터 흐름은
    [`arm_control.py` 개발자 가이드](../arm_control.md)에서 확인할 수 있다.

## 기능 구현 요약

| 구분 | 내용 |
|---|---|
| 해결할 문제 | 무거운 팔이 중력을 버티면서 IK 목표각에 수렴해야 하며, 순수 P 제어의 정상상태 위치 오차를 줄여야 한다. |
| 해결 방법 | MuJoCo가 계산한 bias force를 feed-forward하고, 위치 오차 P항과 속도 감쇠 D항을 더한 뒤 actuator control range로 제한한다. |
| 사용 수식 | \(\tau=h(q,\dot q)+K_p(q_d-q)-K_d\dot q\). 포화가 없고 bias model이 정확한 이상 조건에서 정상상태 오차가 0이 되는 과정을 7.2에서 모든 중간 등식으로 전개한다. |
| 코드 구현 과정 | `ArmTorqueController.apply()`가 joint별 `qpos`, `qvel`, `data.qfrc_bias`를 읽고 PD+bias torque를 계산한 뒤 `np.clip()`하고 `data.ctrl[aid]`에 기록한다. |
| 수식 없이 사용하는 함수 | `mj_util.find_actuator_for_joint()`는 각 joint의 torque actuator를 찾는다. 생성자는 joint/actuator id와 qpos/dof 주소를 저장하고, `apply()`는 현재 actuator `ctrlrange`를 읽어 토크를 제한한다. |

## 7.1 왜 `<position>` 액추에이터로는 부족했나 {: #part-7-1 }

MuJoCo 내장 `<position>` 액추에이터는 순수 비례(P) 제어다. 무거운 7링크 팔이
정지 자세를 유지할 때, 중력을 버티려면 어느 정도 오차가 "남아있는 상태"에서
힘이 나와야 하는데(비례 제어의 근본 특성), 이 잔류 오차가 사이트 기준
15~20mm나 됐다. 진단은 순서대로 네 가지 확인을 진행했다(이 프로젝트의
"파라미터 무작위 조정 대신 순서대로 가설 검증" 원칙 그대로):

1. 텔레포트 테스트(목표각을 새 `MjData`에 직접 넣고 `mj_forward` 한 번) → 오차
   0.004mm. IK 자체는 문제없음.
2. 액추에이터 힘이 `forcerange`에 포화됐는지 확인 → 포화 안 됨(최대 부하의
   1/3 수준).
3. `ctrl` 클램핑이 실제로 걸렸는지 확인 → 안 걸림.
4. 60초 정지 테스트로 오차가 서서히(수 초 시간상수로) 줄어드는 걸 확인 →
   `<position>`의 `dampratio=1` 설정은 관절 하나하나가 서로 무관하게 따로
   움직인다고 가정하고 "출렁임 없이 딱 멈추는" 감쇠값을 계산해주는데, 실제로는
   7개 관절이 서로 물리적으로 연결된 하나의 사슬이라 한 관절의 움직임이 다른
   관절에도 영향을 준다 — 그 가정이 이 결합된 시스템에는 안 맞아서 감쇠가
   제대로 걸리지 않는 게 근본 원인.

## 7.2 PD + 중력/코리올리 feedforward {: #part-7-2 }

### 코리올리(Coriolis) 힘이란

본격적인 식으로 들어가기 전에, 계속 나오는 "코리올리"부터 짚고 넘어가자. 이
프로젝트 코드는 중력·코리올리·원심력을 굳이 셋으로 나누지 않고 `qfrc_bias`
하나로 뭉뚱그려 쓰지만, "이게 대체 어디서 나온 힘인지" 정도는 알고 넘어가는
게 낫다.

**놀이터 회전무대 비유**: 빠르게 도는 회전무대 위에 가만히 서 있으면 아무
일도 없다. 그런데 무대가 도는 동안 중심을 향해 똑바로 "걸어가려고" 하면,
걷는 방향과 다른 옆으로 자꾸 밀리는 느낌을 받는다 — 실제로 누가 옆에서
민 게 아니라, **회전하고 있는 좌표계 "안에서" 움직이려 했다는 사실 자체가**
추가로 힘이 작용하는 것처럼 보이게 만든다. 이 겉보기 힘이 코리올리 힘이다.

로봇 팔에서도 똑같은 일이 벌어진다. 팔꿈치가 돌면 팔꿈치보다 먼 손목/손
입장에서는 자기 기준 좌표계 자체가 회전하고 있는 셈이다 — 그래서 여러
관절이 동시에 움직이는 동안(속도가 0이 아닌 동안)에는 한 관절의 움직임이
다른 관절에 **중력과 무관한** 겉보기 토크를 만들어낸다. 이 힘은 관절
속도들의 곱(\(\dot q_i \dot q_j\))에 비례하기 때문에 **팔이 완전히 멈추면
(\(\dot q = 0\)) 정확히 0이 된다** — "원심력"도 속도의 제곱에 비례해 똑같이
사라지므로, 이 프로젝트에서는 이 둘을 "속도가 있을 때만 나타나는 힘"으로
묶어 중력과 함께 다룬다.

### 제어식

관절 벡터 \(q\), 목표 관절각 \(q_{des}\), 관절 속도 \(\dot q\)에 대해 매 스텝
계산하는 토크는:

\[
\tau = \underbrace{h(q, \dot q)}_{\text{중력+코리올리+원심력 feedforward}}
     \;+\; \underbrace{K_p\,(q_{des} - q)}_{\text{위치 피드백}}
     \;-\; \underbrace{K_d\,\dot q}_{\text{속도 피드백(능동 댐핑)}}
\]

여기서 \(h(q,\dot q)\)는 방금 설명한 중력+코리올리+원심력의 합을 MuJoCo가
매 스텝 계산해주는 `qfrc_bias`를 그대로 가져다 쓴 것이고, \(K_p\), \(K_d\)는
대각 게인 행렬(코드에서는 스칼라 `kp`, `kd`를 모든 관절에 동일하게 적용).
표준 로봇 팔 제어의 bias compensation을 넣은 PD 형태다. 전체 관성행렬과 원하는
가속도를 곱하는 완전한 computed-torque 제어는 아니며, 코드는 아래 식을 그대로
옮긴 것이다:

```python
tau = qfrc_bias[joint]         # h(q, qdot): 지금 상태에서 중력+코리올리+원심력을 정확히 상쇄
    + kp * (q_des - q)          # Kp (q_des - q)
    - kd * qvel                 # -Kd qdot
```

### 왜 이 항이 있고 없고에 따라 결과가 이렇게 달라지는가

로봇 팔의 운동방정식은 일반적으로 \(M(q)\ddot q + h(q,\dot q) = \tau\) 형태다
(\(M\)은 관성행렬). 가만히 멈춰서 자세를 유지하는 정적 평형 상태에서는
\(\ddot q = \dot q = 0\)이므로, 그 자세를 버티는 데 **필요한 토크는 정확히
\(h(q,0)\)** 다(중력을 상쇄하는 힘 — 정지 상태라 코리올리/원심력은 이미 0).
이제 feedforward가 없는 경우와 있는 경우를 중간 단계를 생략하지 않고 각각
운동방정식에 대입한다.

#### P 제어만 사용한 경우

제어 토크가

\[
\tau=K_p(q_{des}-q)
\]

이면 이를 운동방정식의 우변에 대입해서

\[
M(q)\ddot q+h(q,\dot q)=K_p(q_{des}-q)
\]

를 얻는다. 정상상태에서는 \(\dot q=0\), \(\ddot q=0\)이므로

\[
M(q)\,0+h(q,0)=K_p(q_{des}-q)
\]

이고, \(M(q)0=0\)을 지우면

\[
h(q,0)=K_p(q_{des}-q)
\]

이다. 정상상태 오차를 \(e=q_{des}-q\)로 정의하면

\[
h(q,0)=K_pe
\]

이고, \(K_p\)가 가역인 양의 대각행렬이면 양변에 \(K_p^{-1}\)를 왼쪽에서
곱해서

\[
\boxed{e=K_p^{-1}h(q,0)}
\]

를 얻는다. 스칼라 게인을 모든 관절에 동일하게 쓰는 코드에서는 관절별로
\(e_i=h_i(q,0)/K_p\)다. 따라서 유한한 \(K_p\)에서 중력 토크가 0이 아니면
오차도 0이 아니다.

#### PD와 bias feedforward를 사용한 경우

코드의 제어 토크

\[
\tau=h(q,\dot q)+K_p(q_{des}-q)-K_d\dot q
\]

를 같은 운동방정식에 대입하면

\[
M(q)\ddot q+h(q,\dot q)
=h(q,\dot q)+K_p(q_{des}-q)-K_d\dot q
\]

이다. 양변에서 동일한 \(h(q,\dot q)\)를 빼면

\[
M(q)\ddot q=K_p(q_{des}-q)-K_d\dot q
\]

가 남는다. 정상상태 조건 \(\dot q=0\), \(\ddot q=0\)을 대입하면

\[
M(q)0=K_p(q_{des}-q)-K_d0
\]

이고, 영벡터 항을 지우면

\[
0=K_p(q_{des}-q)
\]

이다. \(e=q_{des}-q\)로 다시 쓰고 양변에 \(K_p^{-1}\)를 곱하면

\[
0=K_pe
\]

\[
K_p^{-1}0=K_p^{-1}K_pe
\]

\[
0=Ie=e
\]

따라서 actuator가 포화되지 않고 MuJoCo의 bias가 실제 plant bias와 일치하며
외란과 정적 마찰을 무시할 수 있는 이상 조건에서는

\[
\boxed{e=0\quad\Longleftrightarrow\quad q=q_{des}}
\]

이다. 위 전개를 한눈에 비교하면 다음과 같다.

| | 제어식 | 평형 조건(\(\tau=h(q,0)\)) | 정상상태 오차 |
|---|---|---|---|
| **P 제어만** | \(\tau = K_p(q_{des}-q)\) | \(K_p(q_{des}-q) = h(q,0)\) | \(e = K_p^{-1}h(q,0)\) — **0이 아니고 \(K_p\)에 반비례할 뿐** |
| **PD+feedforward** | \(\tau = h(q,\dot q)+K_p(q_{des}-q)-K_d\dot q\) | 양변에서 \(h(q,0)\) 상쇄 → \(K_p(q_{des}-q)=0\) | 이상 조건에서 \(e=0\); 포화·모델 오차·외란이 있으면 잔차 가능 |

P 제어만 쓰면 \(K_p\)를 5배로 올려도 오차가 1/5로 줄 뿐 절대 0이 되지
않는다(무한대 \(K_p\)가 아닌 한) — 실제로 이 프로젝트에서 \(K_p\)를 5배
올려봐도 처짐이 거의 안 줄었던 게 바로 이 관계식대로다. feedforward를
더하면 오차가 "줄어드는" 게 아니라 **애초에 0이 되는 평형점 자체가 바뀐다.**
\(K_p\)를 올리는 문제가 아니라 애초에 안 하고 있던 feedforward를 넣는
문제였다는 게 이 진단의 핵심 결론이다.

```python
def apply(self, data, q_des, kp_scale=1.0):
    q = np.array([data.qpos[a] for a in self.qpos_adrs])
    qd = data.qvel[self.dof_ids]
    qfrc_bias = data.qfrc_bias[self.dof_ids]
    tau = qfrc_bias + self.kp * kp_scale * (np.asarray(q_des) - q) - self.kd * qd
    for aid, t in zip(self.actuator_ids, tau):
        lo, hi = self.model.actuator_ctrlrange[aid]
        data.ctrl[aid] = np.clip(t, lo, hi)
```

기본 게인은 `kp=600.0`, `kd=40.0`. `kp_scale`은 외부 호출자가 비례 게인을
조절할 수 있게 남긴 확장 hook이며, 현재 실행 경로에서는 항상 1.0이다.

## 7.3 `ros2_control`과 비교 {: #part-7-3 }

이건 정확히 `ros2_control`의 `effort_controllers/JointGroupEffortController`가
하는 일을 직접 구현한 것이다 — 차이라면 `ros2_control`은 하드웨어 추상화 계층
(`hardware_interface`)을 거쳐 실제 모터 드라이버까지 이어지지만, 여기서는
`data.ctrl[aid] = tau`가 그대로 MuJoCo의 다음 `mj_step`에 반영된다는 점.

---

[← Part 6](./06-inverse-kinematics.md) · [전체 안내](../ros2-guide.md) · [Part 8 →](./08-mobile-base.md)
