# ffw-sh5-grasp

ROBOTIS FFW-SH5 (HX5-D20 5지 핸드)가 **contact force만으로** (kinematic 치팅 없이) 테이블 위 캔을 텔레오퍼레이션으로 집어 드는 MuJoCo 시뮬레이터.

개발 계획은 [PLAN.md](./PLAN.md) 참고. 진행 상황과 튜닝 기록은 [NOTES.md](./NOTES.md) 참고.

![demo](docs/demo.gif)

`tests/record_demo.py`로 생성한 스크립트 기반 pick-and-lift 시퀀스(contact force 시각화 켜짐,
`models/full_scene.xml`, 오른손). 사람이 슬라이더를 조작하는 실시간 텔레옵은
`src/teleop_app.py`로 직접 실행해서 확인한다 (아래 Quick start).

## Status

**Phase 4 완료** -- 전체 로봇(양팔+양손, 베이스 고정) + 브라우저 슬라이더 텔레옵 + 데모 GIF.
Phase 0-4 전체 테스트: `for p in 0 1 2 3 4; do python3 tests/test_phase_$p.py; done`

## Quick start

```bash
pip install --break-system-packages mujoco numpy trimesh pillow   # 최초 1회
python3 src/teleop_app.py
# -> http://localhost:8000 를 브라우저로 열어 슬라이더 조작
#    (물리 3D 뷰는 별도 MuJoCo 창으로 뜬다)
```

MuJoCo 창에 포커스가 있을 때: `R` 캔 리스폰(+-2cm 랜덤), `G` contact force/point 시각화 토글,
`C` 카메라 프리셋 전환(전체 뷰 / 오른손 클로즈업). 같은 세 동작은 브라우저 페이지의 버튼으로도
가능하다.

## Directory layout

```
ffw-sh5-grasp/
├── assets/           # 공식 robotis_mujoco_menagerie 원본 (수정 금지)
├── models/
│   ├── hand_only.xml     # Phase 1-2: 오른손 단독 + 캔
│   ├── arm_hand.xml      # Phase 3: 오른팔 + 오른손 + 테이블 + 캔
│   └── full_scene.xml    # Phase 4: 전체 FFW-SH5 (베이스 고정, 양팔+양손+헤드+리프트)
├── src/
│   ├── ik.py             # 6DOF DLS IK
│   ├── grasp.py          # grasp synergy 매핑 + contact 기반 파지 검증 (양손)
│   ├── arm_control.py    # 팔 토크 제어 (feedforward + PD)
│   └── teleop_app.py     # 물리 루프 + 브라우저 슬라이더 텔레옵
├── tests/
│   ├── test_phase_0.py ... test_phase_4.py
│   └── record_demo.py    # 데모 GIF 생성 dev 툴
├── docs/demo.gif
├── PLAN.md
└── NOTES.md
```

## Rules

- kinematic override 금지 (`data.qpos[...] = value` 직접 대입 없음, reset/초기 배치 제외)
- 물리 파라미터는 전부 XML에 정의 (post-compile 파이썬 수정 금지)
- Phase 순서를 지키고, 각 Phase 완료 시 git tag(`phase-N`)를 남긴다
