# ffw-sh5-grasp

ROBOTIS FFW-SH5(양팔 7DOF x2 + HX5-D20 5지 핸드 x2 + 모바일 베이스)가 **contact force만으로**
(kinematic 치팅 없이) 테이블 위 캔을 집어 드는 MuJoCo 시뮬레이터. 모바일 베이스는 실제
바퀴-지면 마찰로 구동되고, 텔레옵은 사람이 조작하는 슬라이더+키보드 기반 단일 네이티브 창
애플리케이션이다.

개발 계획은 [PLAN.md](./PLAN.md) 참고. 진행 상황과 튜닝 기록(버그 원인, 시도했다가 되돌린
접근, 실측 수치)은 [NOTES.md](./NOTES.md)에 세션별로 상세히 남아 있다.

**문서 사이트**(mkdocs): 프로젝트 개요와, MuJoCo를 처음 쓰는 사람을 위해 이 시뮬레이터가
만들어진 순서(Phase 0-5) 그대로 MuJoCo 기능을 하나씩 설명하는 튜토리얼은
`mkdocs serve`로 로컬에서 보거나(`pip install --break-system-packages mkdocs
mkdocs-material` 먼저), `docs/index.md`부터 읽으면 된다.

![demo](docs/assets/demo.gif)

`tests/record_demo.py`로 생성한 스크립트 기반 pick-and-lift 시퀀스(contact force 시각화 켜짐,
`models/full_scene.xml`, 오른손). 사람이 직접 조작하는 실시간 텔레옵은 `src/teleop_app.py`로
직접 실행해서 확인한다 (아래 Quick start).

## Status

**Phase 0-5 완료** -- 전체 로봇(양팔+양손+헤드+리프트) + 모바일 베이스(평면 관절 3개 +
실제 바퀴 3개의 조향/구동 관절과 지면 마찰) + 단일 네이티브 창 텔레옵(GLFW+ImGui) + 데모 GIF.

```bash
for p in 0 1 2 3 4 5; do python3 tests/test_phase_$p.py; done
```

각 Phase가 정확히 무엇을 검증하는지는 아래 [Directory layout](#directory-layout)의 `tests/`
표와 각 테스트 파일 상단 docstring 참고.

## Quick start

```bash
pip install --break-system-packages mujoco numpy trimesh pillow glfw imgui-bundle   # 최초 1회
python3 src/teleop_app.py
```

하나의 네이티브 창에 3D 뷰 + 슬라이더 패널이 같이 뜬다(브라우저 불필요).

**마우스** (3D 뷰 위, 패널 위에서는 비활성): 좌클릭 드래그 = 궤도 회전, 우클릭 드래그 = 팬,
휠 = 줌.

**키보드**:

| 키 | 동작 |
|---|---|
| `Up` / `Down` | 베이스 전진 / 후진 (실제 바퀴-지면 마찰로 구동) |
| `Left` / `Right` | 베이스 제자리 yaw 회전 |
| `Shift` + `Left` / `Right` | 베이스 좌/우 스트레이프 |
| `Q` / `E` | 리프트 하강 / 상승 |
| `R` | 캔 리스폰 (+-2cm 랜덤) |
| `G` | contact force/point 시각화 토글 |
| `C` | 카메라 프리셋 전환 (전체 뷰 / 오른손 클로즈업) |

`R`/`G`/`C`는 패널 버튼으로도 가능. 양손 EE 포즈(X/Y/Z + Roll/Pitch/Yaw)와 grasp/thumb
시너지, 리프트는 패널의 슬라이더로 직접 조작한다 -- RPY는 각 손의 홈 포즈 기준 로컬 회전이라
슬라이더 0,0,0이 자연스러운 기본 자세다.

## Directory layout

```
ffw-sh5-grasp/
├── assets/
│   ├── robotis_ffw/       # 공식 robotis_mujoco_menagerie 원본 (수정 금지)
│   └── soda_can/          # 캔 시각 전용 mesh (ffw-sh5-teleoperation에서 이식, 물리는 캔
│                           # 콜리전 실린더 그대로 -- Phase 1/2 검증 형상 불변)
├── models/
│   ├── hand_only.xml       # Phase 1-2: 오른손 단독(mocap+weld) + 캔
│   ├── arm_hand.xml        # Phase 3: 오른팔 + 오른손 + 테이블 + 캔
│   └── full_scene.xml      # Phase 4-5: 전체 FFW-SH5 -- 베이스 평면 관절 3개 + 바퀴 3개
│                           # (조향+구동 관절, 지면 마찰) + 양팔/양손/헤드/리프트
├── src/
│   ├── ik.py               # hierarchical 6DOF DLS IK (mj_jacSite 기반)
│   ├── grasp.py             # grasp synergy 매핑 + contact 기반 파지 검증 (양손, side=l/r)
│   ├── arm_control.py       # 팔 토크 제어 (중력/코리올리 feedforward + PD)
│   ├── base_teleop.py       # 베이스 조작감(가속/제동) + SwerveDrive(조향각+구동속도 변환)
│   └── teleop_app.py        # 단일 네이티브 창(GLFW+ImGui) 물리 루프 + 슬라이더/키보드 텔레옵
├── tests/
│   ├── test_phase_0.py      # 모델 구조 리포트 + 5s 발산 테스트
│   ├── test_phase_1.py      # 손가락-캔 관통 깊이 (20회 반복)
│   ├── test_phase_2.py      # 고정 손 grasp + lift (±5mm 노이즈, 10회)
│   ├── test_phase_3.py      # IK 단위테스트(100개) + 통합 pick(오른팔, 10회)
│   ├── test_phase_4.py      # 전체 로봇 hold 회귀 + IK 100개 + 통합 pick(10회)
│   ├── test_phase_5.py      # BaseTeleop/SwerveDrive 단위테스트 + 유휴/주행/충돌 회귀
│   ├── record_demo.py       # 데모 GIF 생성 dev 툴 (docs/assets/demo.gif)
│   ├── measure_hand_meshes.py  # STL AABB 측정 -> capsule 파라미터 도출용 dev 툴
│   └── render_snapshot.py   # 오프스크린 렌더 dev 툴
├── docs/                    # mkdocs 문서 사이트 (mkdocs.yml의 docs_dir)
│   ├── index.md             # 홈 -- 프로젝트 개요/튜토리얼 링크
│   ├── overview.md          # 프로젝트 개요 (왜 이렇게 만들었는가)
│   ├── run.md                # 실행 방법
│   ├── guide/                # MuJoCo 튜토리얼 (Phase 0-5 순서로 MuJoCo 기능 설명)
│   └── assets/               # demo.gif, 렌더 스냅샷
├── mkdocs.yml
├── PLAN.md
└── NOTES.md
```

## Rules

- kinematic override 금지 (`data.qpos[...] = value` 직접 대입 없음, reset/초기 배치/베이스
  캔 리스폰/IK 자체 scratch 버퍼 제외 -- 전부 라이브 시뮬레이션의 로봇 관절과 무관)
- 물리 파라미터는 전부 XML에 정의 (post-compile 파이썬 수정 금지)
- Phase 순서를 지키고, 각 Phase 완료 시 git tag(`phase-N`)를 남긴다
- keyframe의 `qpos`/`ctrl` 문자열을 손으로 이어붙이지 말 것 -- 컴파일된 모델에서
  `mj_name2id`/`jnt_qposadr`로 조인트별 슬롯을 찾아 배열을 만들고 정규식으로 파일에
  써넣는 스크립트를 쓴다 (NOTES.md "Phase 5" 참고 -- 손으로 만들다가 토큰 개수를 세 번
  틀렸다)
