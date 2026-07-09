# 체크리스트

MuJoCo 코드 수정 시 확인할 항목.

| 항목 | 확인 |
|---|---|
| `mj_name2id` 결과 | `-1` 여부 확인 후 사용 |
| actuator lookup | `aid is None` 여부 확인 후 `data.ctrl[aid]` 사용 |
| numpy index | `arr[None] = value`가 전체 배열 broadcast가 되지 않는지 확인 |
| qpos 직접 쓰기 | reset/초기 배치 외 live robot qpos 수정 금지 |
| IK scratch | live `data`가 아니라 solver scratch `MjData` 사용 |
| site/body 기준 | IK target은 body origin이 아니라 `site` 기준인지 확인 |
| quaternion frame | local/world frame 변환 순서 확인 |
| contact force | 위치 조건이 아니라 `mj_contactForce()` 기반 판정인지 확인 |
| actuator range | `ctrlrange`, `forcerange`, joint `range` 확인 |
| wheel command | 조향 정렬 전 wheel velocity가 0으로 gated 되는지 확인 |
| marker state | UI target, mocap marker, IK world target 동기화 확인 |
| 문서 | 함수 역할이나 target 의미가 바뀌면 `docs/guide/` 업데이트 |
