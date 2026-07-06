# FFW-SH5 MuJoCo Physics-Based Grasp Simulator — 개발 플랜

## 프로젝트 목표

kinematic 치팅 없이, **contact force만으로** ROBOTIS FFW-SH5(HX5-D20 5지 핸드)가 테이블 위 캔을 집어 들어올리는 MuJoCo **텔레오퍼레이션** 시뮬레이션을 구축한다.

레퍼런스는 "AI Worker FFW-SH5 Teleoperation" 영상이다. 재현할 핵심 요소:
- EE pose target 6D 슬라이더(X/Y/Z/Roll/Pitch/Yaw) 기반 텔레옵 + 6DOF IK (ik_err 실시간 표시)
- **Grasp synergy**: 스칼라 슬라이더 하나(0~1)로 손 전체가 닫히고, force-limited position actuator의 토크 포화로 손가락이 캔을 자연스럽게 감싸쥠
- Joint position monitor 패널, sim/wall time, loop freq HUD
- 자율 실행(FSM) 없음. 사람이 조작해서 집는다

최종 산출물:
1. 깨끗한 구조의 GitHub 레포
2. 슬라이더 텔레옵으로 캔을 접근 → 감싸쥐기 → 들어올리는 데모 (물리 기반)
3. contact force 시각화가 켜진 검증 영상

## 기반 모델

- 공식 모델: https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie 의 `robotis_ffw` (FFW-SH5)
- 참고 구현: https://github.com/jeongeun980906/lerobot-mujoco-tutorial (동일 생태계의 manipulation 파이프라인. 씬 구성, 카메라, 물체 배치 패턴 참고)
- **contact/actuator 레시피의 기준점: https://github.com/google-deepmind/mujoco_menagerie/tree/main/shadow_hand** — DeepMind가 관리하는 dexterous hand 표준 모델. 이 프로젝트의 물리 설정은 shadow_hand의 검증된 값에서 출발하고, 임의의 값을 발명하지 않는다. 핵심 레시피:
  - `option cone="elliptic" impratio="10"` (timestep은 별도 명시)
  - 손 geom: `solimp="0.5 0.99 0.0001"` `solref="0.005 1"`
  - **물체(캔)에 `priority="1"` + `condim="6"`** — 손-캔 접촉에서 캔의 파라미터가 우선 적용되므로, 손 geom 수십 개 대신 캔 하나만 튜닝하면 된다. rolling friction 포함으로 물체가 손안에서 구르며 빠지는 것 방지
  - 물체 friction은 `0.5 0.01 0.003` 수준에서 시작 — **마찰을 키워서 잡는 게 아니라 contact을 안정시켜서 잡는다.** friction > 1.5는 튜닝 실패의 신호로 간주
  - finger actuator: kp 0.5~2, forcerange ±1~2 N·m 스케일 (과한 힘은 관통과 발산의 원인)
  - collision: 마디는 capsule, 손바닥은 box 분해, fingertip은 필요 시 전용 저폴리 collision geom
  - DIP+PIP 언더액추에이션은 fixed tendon(coef)으로 XML에서 구현
- 기존 자체 레포(ggh-png/ffw-sh5-mujoco)는 **참고만** 하고 코드를 복사하지 않는다. 특히 kinematic override 패턴은 절대 가져오지 않는다.

## 절대 규칙 (모든 Phase에 적용)

1. **kinematic override 금지.** `data.qpos[...] = value` 로 물리 상태를 직접 덮어쓰는 코드를 작성하지 않는다. 유일한 예외: reset 함수, 그리고 물체 초기 배치. 물리가 원하는 대로 안 되면 qpos를 덮지 말고 XML(질량, 마찰, solver, actuator)로 돌아가 원인을 고친다.
2. **물리 파라미터는 전부 XML에.** 파이썬에서 `model.geom_friction[...] = ...` 같은 post-compile 수정 금지. 모델의 진실은 XML 한 곳에만 존재해야 한다.
3. **Phase 순서 준수.** 각 Phase의 성공 기준(Success Criteria)을 통과하기 전에 다음 Phase 코드를 작성하지 않는다. 성공 기준은 스크립트로 자동 검증 가능해야 한다.
4. **Phase 완료마다 git commit + tag** (`phase-0`, `phase-1`, ...). 되는 상태를 잃어버리지 않는다.
5. 각 Phase마다 `tests/test_phase_N.py` 형태의 검증 스크립트를 남긴다. headless(mujoco offscreen)로 실행 가능해야 한다.

## 디렉토리 구조

```
ffw-sh5-grasp/
├── assets/
│   └── robotis_ffw/          # 공식 menagerie에서 복사 (수정 금지, 원본 보존)
├── models/
│   ├── hand_only.xml          # Phase 1-2: 오른손 단독 + 캔
│   ├── arm_hand.xml           # Phase 3: 오른팔 + 오른손 + 테이블 + 캔
│   └── full_scene.xml         # Phase 4: 전체 로봇 (베이스 고정)
├── src/
│   ├── ik.py                  # 6DOF DLS IK (Phase 3)
│   ├── grasp.py               # grasp synergy 매핑 + contact 기반 파지 검증 (Phase 2)
│   └── teleop_app.py          # 뷰어 + 슬라이더 GUI 텔레옵 (Phase 4)
├── tests/
│   ├── test_phase_0.py ... test_phase_4.py
└── PLAN.md                    # 이 문서
```

---

## Phase 0 — 공식 모델 검증

### 작업
1. `robotis_mujoco_menagerie/robotis_ffw`를 `assets/`로 복사한다. 원본을 수정하지 않는다.
2. 공식 scene XML을 로드해서 다음을 리포트로 출력하는 스크립트를 작성한다 (`tests/test_phase_0.py`):
   - nq, nv, nu, 전체 joint 이름/type/range/damping/armature
   - 각 finger joint의 actuator 존재 여부, kp, forcerange
   - 각 geom의 type (mesh인지 primitive인지), contype/conaffinity, friction, condim
   - option: timestep, integrator, cone, impratio, solver iterations
3. 중력만 켜고 5초 시뮬레이션: 아무 제어 없이 로봇이 폭발/발산하지 않는지 확인 (qacc 상한 체크).
4. 공식 모델의 finger collision이 mesh 기반인지 primitive 기반인지 판별하고 결과를 `NOTES.md`에 기록한다. **공식 모델이 이미 primitive collision을 제공하면 Phase 1의 작업량이 크게 줄어든다. 반드시 먼저 확인할 것.**

### 성공 기준
- [ ] 공식 scene이 에러 없이 로드
- [ ] 5초 무제어 시뮬레이션에서 발산 없음 (max |qacc| < 1e5)
- [ ] 모델 구조 리포트가 NOTES.md에 기록됨

---

## Phase 1 — hand_only 씬 + collision 정비

### 작업
1. `models/hand_only.xml` 작성: 오른손(HX5-D20)만 world에 **고정 부착**(mocap body에 weld). 팔, 베이스, 왼손, 헤드 전부 제외. 손바닥이 위를 향하거나 옆을 향하게 배치하고, 그 앞에 freejoint 캔(cylinder, r=0.033, h=0.11, mass=0.35) 배치.
2. Phase 0에서 finger collision이 mesh로 판명된 경우에만: 각 finger link STL의 AABB를 trimesh로 측정하는 스크립트를 만들고, 측정값 기반으로 capsule collision geom을 정의한다. visual mesh는 유지, collision만 primitive로 교체. fingertip은 측정 반지름보다 1mm 크게(패드 효과).
3. solver 설정을 XML에 명시 (shadow_hand 검증값 기준): `timestep=0.001`, `integrator="implicitfast"`, `cone="elliptic"`, `impratio=10`. finger geom에 `solimp="0.5 0.99 0.0001"` `solref="0.005 1"`. **캔에 `priority="1"` `condim="6"` `friction="0.5 0.01 0.003"`** — priority 덕분에 손-캔 접촉 파라미터는 캔 쪽만 튜닝하면 된다. fingertip capsule은 실측 반지름 +1mm.
4. HX5-D20의 마디 커플링 검토: 실물 스펙에서 PIP/DIP가 독립 구동인지 확인하고, 언더액추에이션이면 shadow_hand처럼 fixed tendon(coef=1)으로 XML에서 묶는다. 이러면 grasp synergy 매핑이 단순해지고 순응 파지가 tendon 수준에서 나온다.
5. 관통 테스트 스크립트 (`tests/test_phase_1.py`): 손가락을 최대 속도로 캔을 향해 닫는 시나리오를 20회 반복. 매 스텝 finger geom과 can geom의 penetration depth(`contact.dist`)를 기록. 최대 penetration이 2mm를 넘으면 실패.
6. 뷰어에서 `mjVIS_CONTACTPOINT`, `mjVIS_CONTACTFORCE` 활성화 옵션 제공.

### 성공 기준
- [ ] 20회 관통 테스트에서 max penetration < 2mm
- [ ] contact point가 finger 표면 위치에 정확히 생성됨 (시각 확인)
- [ ] 실시간 팩터 ≥ 0.5 (RTX 없는 CPU 기준)

---

## Phase 2 — 고정 손 grasp (프로젝트의 핵심)

### 작업
1. grasp 모듈 (`src/grasp.py`) — 두 가지 역할:
   - **Grasp synergy 매핑**: `apply_grasp(model, data, grasp: float, thumb: float)` — 스칼라(0~1)를 각 finger joint의 position target으로 매핑. 레퍼런스 영상의 "Right grasp / Right thumb" 슬라이더와 동일한 인터페이스.
   - **파지 검증**: `get_finger_can_contacts(model, data)` → 캔과 접촉 중인 finger link 목록 + normal force. `is_grasped(...)` → 서로 다른 손가락 2개 이상(엄지 포함 필수)이 접촉 중이고 normal force 합 > 임계값. 이 함수는 자율 실행용이 아니라 **테스트 스크립트의 성공 판정용**이다.
2. 감싸쥐기 원리 이해하고 구현할 것: position actuator + 제한된 forcerange 조합에서, 접촉으로 토크가 포화된 마디는 멈추고 아직 접촉 안 한 마디가 계속 감기며 캔 형상에 순응(conform)한다. 별도의 토크 제어기를 만들지 않는다. forcerange가 너무 크면 관통하고, 너무 작으면 못 든다 — 이 균형이 튜닝의 본질이다.
3. 파지 시퀀스: 손가락 open 상태에서 grasp 스칼라를 램프(rate limit)로 올린다. 스텝 함수로 급격히 닫지 않는다.
3. mocap body를 z축으로 10cm 천천히(2cm/s) 올리는 lift 테스트.
4. 좋은 pre-grasp/파지 자세를 찾으면 shadow_hand의 keyframes.xml처럼 **keyframe으로 XML에 저장**한다. 테스트 재현성의 기본이다.
5. 실패 시 튜닝 순서 (이 순서를 지킬 것, 한 번에 하나만 변경):
   a. 파지 자세(캔 대비 손 위치/방향) 조정 — 엄지와 3지가 캔을 마주보게. keyframe 갱신
   b. finger actuator kp / forcerange (XML) — shadow_hand 스케일(kp 0.5~2, forcerange ±1~2 N·m) 안에서만. 힘을 키우는 방향의 튜닝은 마지막 수단
   c. 캔 friction (priority=1이므로 캔만) — 최대 1.5까지. 그 이상 필요하면 contact이 불안정한 것이니 d로
   d. solref/solimp (shadow_hand 기본값에서 벗어날 땐 사유를 NOTES.md에 기록)
   e. 위 전부 실패 시에만: 캔 질량을 0.2kg로 낮추고 그 사실을 NOTES.md에 명시
5. 반복 검증 스크립트 (`tests/test_phase_2.py`): 캔 초기 위치에 ±5mm 랜덤 노이즈를 주고 grasp+lift 10회 실행, 성공률 기록. 성공 = 10cm lift 후 5초간 슬립 < 1cm.

### 성공 기준
- [ ] grasp+lift 10회 중 8회 이상 성공
- [ ] 성공 판정이 contact force 기반 (위치 기반 치팅 없음)
- [ ] 캔 attach/weld 코드가 레포 어디에도 없음

### 이 Phase에서 막힐 때
5지 전부 쓰는 grasp이 안 되면 엄지+검지+중지 3점 파지로 단순화한다. pinky/ring은 XML에서 range를 0으로 고정한다(파이썬 lock 금지). 목표는 인상적인 5지 조작이 아니라 정직한 물리 파지다.

---

## Phase 3 — arm_hand 씬 + IK

### 작업
1. `models/arm_hand.xml`: 오른팔 7관절 + 오른손. 팔 base를 world에 고정(FFW-SH5에서 어깨가 위치하는 실제 높이 ~1.0m에 맞춤). 테이블(box, 상판 z=0.65 수준) + 캔 배치. Phase 1-2에서 확정한 물리 파라미터 그대로 사용.
2. **6DOF** DLS IK 구현 (`src/ik.py`): `mj_jacBody` 기반 position+orientation, damping λ=0.05 시작, joint limit clamp, 스텝당 최대 관절 변화량 제한. IK 오차(position/orientation)를 리턴해서 HUD에 표시 가능하게 한다(레퍼런스 영상의 ik_err). 기존 레포의 qp_ik를 가져오지 않고 새로 작성한다. 먼저 position 3DOF로 수렴 검증 후 orientation을 추가하는 2단계로 개발한다.
3. IK 단위 검증 (`tests/test_phase_3.py` 전반부): 도달 가능 workspace 내 랜덤 타겟 100개(위치+자세)에 대해 수렴 후 error 통계. 95% 이상이 position error < 5mm, orientation error < 5°.
4. 스크립트 접근 궤적으로 통합 검증 (`tests/test_phase_3.py` 후반부): home → pre-grasp(캔 측면 8cm, 손바닥이 캔을 향함) → 직선 접근(3cm/s) → Phase 2 파지 시퀀스 → 10cm lift를 스크립트로 실행, 10회 성공률 기록. 이 스크립트는 자율 기능이 아니라 텔레옵 없이 파이프라인을 회귀 테스트하기 위한 것이다.

### 성공 기준
- [ ] IK 단위 테스트 통과 (95% < 5mm)
- [ ] 자동 pick 10회 중 7회 이상 성공

---

## Phase 4 — full scene + 텔레옵 GUI + 데모

### 작업
1. `models/full_scene.xml`: 전체 FFW-SH5. **베이스의 freejoint를 제거하고 world에 고정한다.** 모바일 주행은 이 프로젝트 scope 밖이다. 바퀴는 visual만 유지. lift joint는 실제 position actuator로 유지하되 qpos 덮어쓰기 없이 서보가 중력을 이기는지 확인하고, 처지면 XML에서 kp/forcerange를 올린다.
2. 텔레옵 앱 (`src/teleop_app.py`) — 레퍼런스 영상의 인터페이스를 재현한다:
   - **EE pose target 패널** (오른손/왼손 각각): X, Y, Z, Roll, Pitch, Yaw 슬라이더 → 6DOF IK 타겟
   - **Hand grasp targets 패널**: Right/Left grasp(0~1), Right/Left thumb(0~1) 슬라이더 → Phase 2의 grasp synergy로 매핑
   - **Joint position monitor 패널**: 전체 joint의 현재 qpos 실시간 표시
   - **HUD**: ik_err, sim time, wall time, loop freq
   - 키보드 백업: R=캔 리셋(±2cm 랜덤 위치), G=contact force 시각화 토글
   - GUI 프레임워크는 기존 레포에서 쓰던 것 재사용 가능 (dearpygui/tkinter 등, 단 GLFW/EGL 컨텍스트 충돌 주의)
3. 루프 주기 목표 ≥ 20Hz (레퍼런스 영상이 23~26Hz). GUI 콜백과 physics 스레드 간 race condition 방지: GUI는 target 값만 쓰고, physics 루프가 읽는 단방향 구조.
4. 데모 영상 촬영용 카메라 프리셋 2개(전체 뷰, 손 클로즈업).
5. README 작성: 설치, 실행, 결과 GIF, 물리 설정 요약(kinematic 치팅 없음을 명시).

### 성공 기준
- [x] 슬라이더 텔레옵만으로 접근 → 감싸쥐기 → 10cm 들어올리기 성공 (조작 5회 시도 중 3회 이상)
      -- `src/teleop_app.py`로 직접 조작해 확인할 것(에이전트가 사람 대신 슬라이더를 못 끎).
      대신 `tests/test_phase_4.py`가 동일 파이프라인(IK+토크제어+grasp synergy)을
      스크립트로 10/10 재현해 물리적으로 가능함을 검증함. NOTES.md "Phase 4" 참고.
- [x] 파지 시 손가락이 캔 형상에 순응하며 감기는 것이 시각적으로 확인됨 (관통 없음) -- `docs/assets/demo.gif`
- [x] 전체 코드베이스에 kinematic override 없음 (grep으로 `qpos[` 쓰기 검사, reset 제외)
- [x] contact force 시각화 켠 상태의 데모 영상 촬영 완료 -- `docs/assets/demo.gif`
      (`tests/record_demo.py`로 생성; 실시간 텔레옵 GUI 자체의 화면 녹화는 아님)

---

## 에이전트 작업 방식

- 한 세션에서 하나의 Phase만 진행한다.
- 코드 작성 전에 반드시 해당 Phase의 성공 기준을 다시 읽는다.
- 테스트 실패 시 파라미터를 무작위로 바꾸지 말고, Phase 2의 튜닝 순서처럼 한 번에 하나의 변수만 바꾸고 결과를 NOTES.md에 기록한다.
- "일단 qpos로 고정하고 나중에 물리로 바꾸자"는 제안을 하지 않는다. 그 "나중"은 오지 않는다.
