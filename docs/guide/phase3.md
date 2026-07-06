# Phase 3 — 팔에 IK를 붙이기: site, Jacobian, 그리고 "생각보다 안 맞는" 액추에이터

`src/ik.py` · `src/arm_control.py` · `models/arm_hand.xml`

Phase 2까지는 손이 허공에 고정돼 있었다. Phase 3부터는 진짜 팔(7자유도)이 붙고,
"슬라이더로 원하는 손끝 위치를 주면 관절 각도를 역산"하는 **역기구학(Inverse
Kinematics, IK)**이 필요해진다.

## 목표점은 body가 아니라 site로 정의한다

손끝 기준점을 `<site name="grasp_target" pos="0.065 0.01 0.105"/>`처럼 손바닥 body
아래에 심어둔다. site는 body 원점이 아니라 그 body 기준 임의의 오프셋/회전을 가진
순수 참조점이라, "손바닥에서 정확히 이 지점"을 IK 목표로 삼을 때 오프셋을 따로 계산할
필요가 없다. `mj_jacSite`는 이 site의 위치·회전 Jacobian을 바로 계산해준다.

## damped least squares — 역행렬이 아니라 "감쇠 최소자승"을 쓰는 이유

!!! info "핵심 개념 · Jacobian과 DLS IK"
    Jacobian `J`는 "관절 속도가 조금 바뀌면 손끝 위치가 얼마나 바뀌는가"를 나타내는
    행렬이다. 목표와의 오차 `e`를 줄이는 관절 변화량을 구하려면 `J`를 역행렬로
    풀면 되지만, 팔이 특이 자세(singularity) 근처에 오면 `J`가 거의 특이행렬이 되어
    역행렬이 폭발한다. **damped least squares(DLS)**는 여기에 작은 감쇠항 `λ²I`를
    더해 안정성을 확보한다:

    $$ \Delta q = J^\top (J J^\top + \lambda^2 I)^{-1} e $$

    이 프로젝트는 `λ = 0.05`를 쓴다. 계획서가 요구한 대로 먼저 위치 3자유도만으로
    수렴을 검증한 뒤(`solve_position`), 방향까지 더한 6자유도(`solve_pose`)로
    확장하는 2단계로 개발했다.

```python title="src/ik.py — DLS 한 스텝"
def _dls_step(self, J, err):
    lam2 = self.damping ** 2
    JJt = J @ J.T + lam2 * np.eye(J.shape[0])
    return J.T @ np.linalg.solve(JJt, err)
```

위치와 방향을 하나의 스택된(6×n) Jacobian으로 한 번에 풀면 **발산**했다 — 방향
오차가 크면 그걸 줄이려는 보정이 위치를 도로 흔들고, 그 반대도 마찬가지였다. 그래서
**계층형(hierarchical) IK**로 바꿨다: 위치를 먼저 DLS로 풀고, 방향 보정은 위치
Jacobian의 **영공간(null space)**에 투영해서 위치에 영향을 주지 않는 성분만
반영한다. 그래도 오차가 클 때 반복할수록 나빠지는 진동이 남아서, **backtracking line
search**(스텝이 실제로 비용을 줄일 때만 채택, 아니면 절반으로 줄여 재시도)까지
추가해야 안정됐다.

!!! bug "실제로 만난 버그 · site-local 오차와 world-frame Jacobian을 그대로 섞어버림"
    **증상**: 관절을 하나씩 흔들어보면(perturbation test) "계산된 gradient가 큰
    관절"과 "실제로 움직여보면 영향이 큰 관절"이 서로 다름 — 보정이 엉뚱한 관절로
    새고 있었다.

    **원인**: `mju_subQuat`으로 구한 방향 오차는 **site의 로컬 좌표계** 기준인데,
    `mj_jacSite`의 회전 Jacobian(`jacr`)은 **월드 좌표계** 기준이었다. 서로 다른
    기준계의 벡터를 그대로 곱하면 안 된다.

    **해결**: `site_xmat @ ori_err`로 로컬 오차를 월드로 회전시켜 맞춰줌.

이 프로젝트에서 IK 자체는 랜덤 타겟 100개에 대해 **100/100**(위치오차 5mm, 방향오차
5° 이내) 수렴하는 걸로 검증됐다. 그런데도 실제 팔에 그 관절각을 넣고 서보한 뒤
정착시키면 손끝이 목표에서 **15~20mm**나 벗어났다 — IK가 틀린 게 아니라, 다음 절의
액추에이터 문제였다.

## &lt;position&gt;의 한계와 motor + PD + 중력 feedforward

!!! info "핵심 개념 · qfrc_bias와 중력 보상"
    MuJoCo의 `<position>` 액추에이터는 순수 비례(P) 제어에 가깝다 — 적분항이 없어서,
    중력처럼 계속 걸리는 하중을 받는 관절은 **게인만큼의 정상상태 오차**가 항상
    남는다(오차 ≈ 하중/kp). 7개 링크가 서로 관성 결합된 팔 전체를 `dampratio=1`로
    임계감쇠 처리한다는 가정도 실제로는 깨진다(개별 관절은 독립이 아니므로). 그래서
    팔은 `<position>` 대신 `<motor>`(순수 토크)로 바꾸고, 매 스텝 직접 토크를
    계산했다:

    $$ \tau = q_{frc\_bias} + k_p (q_{des} - q) - k_d \dot q $$

    `data.qfrc_bias`는 MuJoCo가 매 스텝 계산해주는 "중력 + 코리올리 + 원심력"
    항이다 — 이걸 그대로 더해주면 **현재 자세를 유지하는 데 필요한 중력 보상을
    공짜로 얻는다**(직접 로봇의 동역학 방정식을 유도할 필요가 없다). 이게
    정상상태 오차를 실제로 없애는 핵심이지, kp를 올리는 게 핵심이 아니다.

```python title="src/arm_control.py"
def apply(self, data, q_des):
    q = np.array([data.qpos[a] for a in self.qpos_adrs])
    qd = data.qvel[self.dof_ids]
    qfrc_bias = data.qfrc_bias[self.dof_ids]
    tau = qfrc_bias + self.kp * (np.asarray(q_des) - q) - self.kd * qd
    for aid, t in zip(self.actuator_ids, tau):
        lo, hi = self.model.actuator_ctrlrange[aid]
        data.ctrl[aid] = np.clip(t, lo, hi)
```

!!! bug "실제로 만난 버그 (가장 값진 교훈) · 진단 순서를 지켜서 찾은 진짜 원인"
    **가설1**: 텔레포트 테스트(목표 관절각을 별도 데이터에 넣고 `mj_forward` 한 번)
    → 오차 0.004mm. 좌표계/기준점 문제는 기각.

    **가설2**: 토크 포화 검사 → 어떤 관절도 최대 토크에 안 걸림. 힘 부족은 기각.

    **가설3**: ctrl 클램핑 검사 → 보낸 값과 실제 ctrl 완전 일치. 기각.

    **진짜 원인**: motor+PD로 바꾼 뒤에도 grasp 위치에서 여전히 18.5mm 오차가
    남았다 — 알고 보니 **세 손가락 전부가 정확히 같은 벡터만큼** 어긋나 있었다.
    손가락마다 다르게 어긋나야 정상인데(각 관절 설정이 다르므로) 전부 똑같이
    어긋난다는 건 **손바닥 자체가 통째로 잘못된 지점을 겨냥한다는 신호**였다.
    역산해보니 `grasp_target` site의 `pos` 값이, 다른 씬에서 측정한 **캔의 월드
    좌표**를 손바닥 기준 **로컬 오프셋**인 것처럼 그대로 재사용한 것이었다.

    **배울 점 3가지**: (1) "게인을 올려도 안 바뀐다"는 신호는 원인이 게인과
    무관하다는 뜻이니 가설을 버릴 신호로 받아들일 것. (2) 여러 지점에 **똑같은**
    오프셋이 나타나면 각각의 설정이 아니라 공통 기준점(좌표계)을 의심할 것. (3) 한
    씬에서 world 좌표로 검증한 숫자를 다른 body의 로컬 오프셋으로 재사용할 땐 반드시
    좌표 변환을 명시적으로 계산해야 한다 — 숫자가 그럴듯해 보여도 검증 없이
    재사용하면 안 된다.

결과: IK 단위테스트 100/100 유지, 통합 pick(접근→파지→들어올리기) **10/10** 성공
(목표 7/10).

---

다음: [Phase 4 — 로봇 전체 조립 + 렌더링 파이프라인](phase4.md)
