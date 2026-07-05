# ffw-sh5-grasp — 모바일 베이스 제어 이식 + 박스 관통 개선 계획

> 대상: https://github.com/ggh-png/ffw-sh5-grasp (⚠ 작성 시점 404 — §0 참조)
> 조작감 레퍼런스: https://github.com/ggh-png/ffw-sh5-teleoperation (C++/Bullet3, 분석 완료)
> 선행 문서: `ffw-sh5-agent-spec.md` (물리 파지 전환 지시서) — 본 문서는 그 연장선이며 §0의 절대 규칙(qpos 직접 쓰기 금지 등)을 그대로 상속한다.

---

## 0. 전제 확인 — 리포 접근 불가 (블로커)

`ffw-sh5-grasp`는 현재 404다 (비공개 또는 이름 상이). 따라서 **박스 관통의 확정 진단은 아직 수행되지 않았다.** §2는 이전 리포(ffw-sh5-mujoco)의 아키텍처와 작업 경로에서 추론한 확률 순 점검표다. 에이전트는 작업 시작 전 반드시:

1. 리포 접근 확보 (public 전환 또는 코드 제공).
2. §2.1의 진단 명령을 실행해 실제 원인을 확정한 뒤 해당 항목만 수정한다. 점검표 전체를 맹목적으로 적용하지 않는다.

---

## 1. teleoperation 리포에서 이식할 것 — 분석 결과

C++/Bullet3 구현에서 **이식 대상은 코드가 아니라 조작감 스펙**이다. 확인된 값:

### 1.1 키맵 (README + InputManager.cpp)

| 키 | 동작 |
|---|---|
| `W/S` | 베이스 전/후 (로봇 로컬 +X/−X) |
| `A/D` | 좌/우 스트레이프 (로봇 로컬) |
| `←/→` | yaw 회전 |
| `Q/E` | 리프트 상승/하강 |
| `TAB`, `↑/↓` | 관절 선택/조절 (grasp 리포의 기존 키와 충돌 시 재배치) |

### 1.2 조작감 파라미터 (main.cpp / InputManager.cpp 원본 값)

```
kBaseSpeed = 0.5 m/s      (명령 속도)
kMaxSpd    = 0.55 m/s     (상한)
kAccel     = 3.0 m/s²     (입력 시: v ← v + (v_target−v)·(1−e^(−kAccel·dt)))
kBrake     = 6.0 m/s²     (해제 시: v ← v·e^(−kBrake·dt), |v|<0.001이면 0)
kYawSpeed  = 1.2 rad/s    (yaw 목표. 관성: (1−e^(−4dt)) 추종, 해제 시 e^(−10dt) 감쇠)
kLiftSpeed = 0.3 m/s
제자리 회전 시 병진 속도 즉시 0 (회전 조작 명료화)
```

이 스무딩 로직은 순수 수학이므로 Python으로 그대로 옮긴다 (`robot/base_teleop.py`).

### 1.3 이식하면 안 되는 것

teleop 리포의 베이스는 **키네마틱 transform 직접 설정 + convex sweep test로 벽 슬라이딩**이다 (PhysicsWorld::setBaseVelocity). 이것을 MuJoCo에서 `qpos` 쓰기로 재현하면 이전 리포의 floor constraint/yaw 적분/steer 핵으로 정확히 회귀한다. **금지.** MuJoCo에서는 §3의 방식이 물리적 등가물이며, 충돌 슬라이딩은 솔버가 공짜로 해준다.

---

## 2. 박스 관통 — 진단 우선, 수정은 그다음

### 2.1 진단 명령 (grasp 리포에서 실행, 원인 확정용)

```bash
# (a) 부착/qpos 치팅 잔존 여부 — 이전 리포의 1번 원인
grep -rn "qpos\[" robot/ main.py | grep -v "reset\|init"
grep -rn "attach\|_can_ee_offset\|box.*offset" robot/ main.py

# (b) 박스가 body+freejoint인지, worldbody 정적 geom인지
grep -n "box" -A5 <씬 생성 코드> | grep -n "freejoint\|body\|mass"

# (c) 손 collision이 여전히 mesh hull인지 (Phase 3 완료 여부)
python3 -c "import mujoco; m=mujoco.MjModel.from_xml_path('...');
print([(m.geom(i).name, m.geom_type[i], m.geom_contype[i], m.geom_conaffinity[i])
       for i in range(m.ngeom) if 'finger' in (m.geom(i).name or '')])"
# geom_type 7=mesh, 3=capsule, 6=box

# (d) 손가락 kp / forcerange
python3 -c "... print(m.actuator_gainprm[:, 0], m.actuator_forcerange)"

# (e) 런타임 관통 깊이 실측 (진실은 여기 있다)
# 매 스텝: min(data.contact[i].dist for 손↔박스 쌍) 로깅
```

### 2.2 원인 후보 — 확률 순 (진단으로 확정 후 해당 항목만 수정)

**H1. 부착/qpos 오버라이드 잔존 (최우선 의심).** 이전 리포의 관통 원인 1순위였고, 대상이 캔→박스로 바뀌었어도 부착 코드가 이식됐다면 증상은 동일하다. 수정 = agent-spec §3 (전면 삭제 + shadow IK). 진단 (a)에서 한 줄이라도 나오면 나머지 후보를 볼 필요도 없이 이것부터다.

**H2. 박스가 freejoint 없는 정적 geom.** worldbody에 `<geom type="box">`로만 정의하면 무한 질량 고정물이다. kp 높은 손가락 서보가 밀면 solimp 연성만큼 파고든 상태로 유지 → 시각적 관통. 수정 = `<body><freejoint/><geom .../></body>` + 현실적 질량(골판지 상자 0.1~0.5kg 수준이면 `inertiafromgeom` 확인).

**H3. 손 collision이 여전히 mesh convex hull + kp=100 (Phase 2·3 미완).** 박스 평면 vs 손 hull 접촉에서 고게인 서보가 밀어붙이면 수 mm~cm 관통이 정상 동작이다. 수정 = agent-spec §4.2(kp 1~5) + §5(primitive 피팅). **박스 상대라면 특히**: 말단 지골 capsule에 `condim=4`가 없으면 박스 면 위에서 회전 미끄러짐이 커서 더 세게 쥐게 되고 관통이 깊어진다.

**H4. 박스 관성/치수 문제.** 박스가 너무 가볍거나(<50g) 얇으면(<15mm) timestep 0.002에서 손가락 사이로 터널링한다. 수정 = 질량 ≥0.1kg, 최소 두께 확보, 그래도 불안정하면 timestep 0.001 (iterations 올리는 것보다 우선).

**H5. 접촉 파라미터.** solimp를 과하게 무르게 커스텀했거나 margin이 크면 정적 관통이 보인다. 수정 = 특례 삭제, Adroit 기준(`friction 1 0.5 0.01`, margin ≤0.0005).

### 2.3 완료 판정

agent-spec §9의 T1(관통 깊이 ≥ −3mm 일시 / ≥ −1mm 정상상태)을 박스 대상으로 실행. "눈으로 안 뚫려 보임"은 판정 기준이 아니다 — `contact.dist` 수치로 판정한다.

---

## 3. 모바일 베이스 제어 — MuJoCo 설계

### 3.1 구조: freejoint → 평면 가상 관절 3개 (권장)

베이스의 freejoint를 제거하고 다음으로 교체한다:

```xml
<body name="base_root" pos="0 0 0">
  <joint name="base_x"   type="slide" axis="1 0 0" damping="30"/>
  <joint name="base_y"   type="slide" axis="0 1 0" damping="30"/>
  <joint name="base_yaw" type="hinge" axis="0 0 1" damping="10"/>
  <!-- 기존 베이스 body 트리 전체를 이 아래로 -->
</body>
<actuator>
  <velocity name="a_base_x"   joint="base_x"   kv="800" forcerange="-500 500"/>
  <velocity name="a_base_y"   joint="base_y"   kv="800" forcerange="-500 500"/>
  <velocity name="a_base_yaw" joint="base_yaw" kv="200" forcerange="-200 200"/>
</actuator>
```

이 선택의 근거와 효과:

- **qpos 핵 4개가 한 번에 삭제된다**: floor constraint(z/roll/pitch 락), yaw 키네마틱 적분, 스티어 qpos, 베이스 X 락. z/roll/pitch 자유도가 애초에 존재하지 않으므로 락이 불필요하다. agent-spec §7.1에서 "허용 부채"로 남겼던 항목이 전부 청산된다.
- **물리적으로 올바르다**: 베이스는 velocity actuator의 힘으로 움직이므로 장애물에 막히면 멈추고, 벽에 비스듬히 밀면 솔버가 자연스럽게 슬라이딩시킨다 (teleop 리포가 convex sweep으로 수동 구현한 것을 공짜로 얻는다). 파지 중인 박스에 베이스가 부딪혀도 접촉이 정상 처리된다.
- **바퀴 접촉 시뮬레이션을 하지 않는다**: 실제 휠-지면 마찰 모델링은 이 프로젝트 목적(파지)에 비용 대비 무가치. 기존 휠은 시각 전용(contype=0)으로 강등.
- kv/damping/forcerange는 베이스+상체 총질량(~100kg)이 kAccel≈3 m/s²로 가속되도록 산정: F≈m·a≈300N → forcerange 500N에서 시작해 튜닝.

### 3.2 로봇 로컬 → 관절 속도 변환 + 조작감 이식 (`robot/base_teleop.py`)

```python
class BaseTeleop:
    K_SPEED, K_MAX  = 0.5, 0.55     # teleop 리포 원값
    K_ACCEL, K_BRAKE = 3.0, 6.0
    K_YAW = 1.2

    def update(self, keys, dt, yaw):
        # 1) 키 → 로컬 목표속도 (W/S=+x/−x, A/D=∓y), ←/→ = yaw
        # 2) teleop 리포의 스무딩 그대로:
        #    입력 시  v += (v_tgt - v) * (1 - exp(-K_ACCEL*dt)); |v|≤K_MAX 클램프
        #    해제 시  v *= exp(-K_BRAKE*dt); 0.001 미만 절사
        #    yaw:     추종 (1-exp(-4dt)) / 감쇠 exp(-10dt)
        #    회전 입력 시 병진 즉시 0 (제자리 회전)
        # 3) 로컬 → 월드: [vx_w, vy_w] = Rz(yaw) @ [vx_l, vy_l]
        # 4) 반환값은 ctrl 명령: d.ctrl[a_base_x]=vx_w, [a_base_y]=vy_w, [a_base_yaw]=w
```

**qpos/qvel에는 아무것도 쓰지 않는다.** yaw 각도는 `data.qpos[base_yaw]`를 읽기만 해서 변환에 사용.

- Q/E 리프트: 리프트 position actuator의 **ctrl 목표를** `±0.3·dt`씩 적분 (관절 범위 클램프). qpos hold 삭제 — agent-spec §4.2와 동일 원칙.
- 키맵 충돌 처리: grasp 리포 기존 바인딩과 대조표를 만들고 겹치는 키는 태스크 계열을 이동. WASD/화살표/QE는 teleop 표준이므로 우선권을 준다.

### 3.3 주행 × 파지 상호작용 — 미리 정할 규칙

- **파지 중 주행은 막지 않는다.** 물리 파지가 제대로 됐다면 저속(0.55 m/s) 주행의 관성은 마찰 파지가 버텨야 정상이고, 못 버티면 그게 파지 품질의 진짜 척도다 (§4 T8).
- 단 IK 목표는 **베이스 로컬 프레임**으로 유지돼야 한다. IK 목표가 월드 고정이면 베이스가 움직일 때 팔이 월드의 옛 지점을 쫓아가며 몸과 꼬인다. 현재 코드가 월드 프레임 목표라면 `target_local = R_base⁻¹(target_world − p_base)`로 저장하고 매 프레임 월드로 환산하도록 수정.
- 급제동(kBrake=6)은 파지물에 ~0.6g 관성을 건다. 필요 시 "파지 중 kBrake 3.0" 같은 완화는 허용하되, 마찰을 올려서 해결하는 것은 금지 (invariant 3).

---

## 4. 검증 추가 (agent-spec §9 확장)

| 테스트 | 방법 | 합격 기준 |
|---|---|---|
| T8 주행 중 파지 유지 | 박스 파지 → 전진 0.5 m/s 2m → 급제동 → yaw 90° | 박스 이탈 없음, EE-박스 상대 미끄러짐 ≤ 15mm |
| T9 베이스 충돌 | 테이블을 향해 전진 명령 유지 | 베이스가 테이블 앞에서 정지(관통 X), 팔·손 무손상 |
| T10 조작감 회귀 | 스텝 입력에 대한 속도 응답 로깅 | 0→0.5 m/s 도달 ~1s(τ=1/3s), 정지 ~0.5s — teleop 리포 상수와 일치 |
| T1(박스) | §2.3 | 손↔박스 contact.dist ≥ −3mm 일시 / −1mm 정상 |

---

## 5. 실행 순서

| 단계 | 내용 | 선행 조건 | 규모 |
|---|---|---|---|
| 0 | grasp 리포 접근 확보 + §2.1 진단 실행, 원인 확정 | 리포 공개 | 1시간 |
| 1 | 박스 관통 수정 (확정된 H-항목만) | 0 | H1이면 agent-spec Phase1 잔여분, H2~H5면 반나절 |
| 2 | 베이스 freejoint → 평면 관절 3개 + velocity actuator, 기존 베이스 qpos 핵 4개 삭제 | — | 1일 |
| 3 | `base_teleop.py` 이식 (조작감 상수 §1.2) + 키맵 정리 + Q/E 리프트 ctrl화 | 2 | 반나절 |
| 4 | IK 목표 베이스-로컬 프레임화 | 2 | 반나절 |
| 5 | T8~T10 + T1(박스) 통과 | 1~4 | 반나절 |

주의: 단계 2는 조인트 인덱스 지형을 바꾼다 (freejoint 7-dof → 3-dof). 컨트롤러의 하드코딩된 qpos/qvel 주소, MJMODEL 기반 인덱스 캐시가 전부 깨지므로, 인덱스는 반드시 `mj_name2id` 기반 조회로 통일한 뒤 진행할 것.

---

## 부록 — 이 계획에서 하지 않기로 한 것과 이유

- **Bullet식 sweep + transform 직접 설정 이식**: MuJoCo에서 qpos 오버라이드 회귀. 평면 관절 + velocity actuator가 등가 이상의 결과를 솔버에게 위임.
- **실물 휠-지면 접촉 시뮬레이션**: 파지 프로젝트 목적 대비 비용 과다. 향후 주행 동역학이 목적이 되면 그때 Menagerie 휠 모델 참조.
- **관통을 solimp/마찰로 덮기**: agent-spec invariant 3 상속. 진단(§2.1) 없이는 어떤 수정도 착수 금지.
