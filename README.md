# ffw-sh5-grasp

ROBOTIS FFW-SH5 (HX5-D20 5지 핸드)가 **contact force만으로** (kinematic 치팅 없이) 테이블 위 캔을 텔레오퍼레이션으로 집어 드는 MuJoCo 시뮬레이터.

개발 계획은 [PLAN.md](./PLAN.md) 참고. 진행 상황과 튜닝 기록은 [NOTES.md](./NOTES.md) 참고.

## Status

Phase 0 진행 중.

## Directory layout

```
ffw-sh5-grasp/
├── assets/           # 공식 robotis_mujoco_menagerie 원본 (수정 금지)
├── models/           # 프로젝트 전용 MJCF 씬
├── src/              # ik.py, grasp.py, teleop_app.py
├── tests/            # phase별 headless 검증 스크립트
├── PLAN.md
└── NOTES.md
```

## Rules

- kinematic override 금지 (`data.qpos[...] = value` 직접 대입 없음, reset/초기 배치 제외)
- 물리 파라미터는 전부 XML에 정의 (post-compile 파이썬 수정 금지)
- Phase 순서를 지키고, 각 Phase 완료 시 git tag(`phase-N`)를 남긴다
