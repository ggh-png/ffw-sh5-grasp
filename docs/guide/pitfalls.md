# 체크리스트

MuJoCo 코드 수정 시 확인할 항목.

각 항목이 실제로 어떤 장애를 만들었는지는
[Part 13 — 버그 사례집](ros2/13-bug-cases.md)에 원인과 수정 과정으로 정리돼 있다.

| 항목 | 확인 |
|---|---|
| `mj_name2id` 결과 | `-1` 여부 확인 후 사용 |
| actuator lookup | `aid is None` 여부 확인 후 `data.ctrl[aid]` 사용 |
| numpy index | `arr[None] = value`가 전체 배열 broadcast가 되지 않는지 확인 |
| qpos 직접 쓰기 | reset/초기 배치 외 live robot qpos 수정 금지 |
| IK 상태 분리 | live `data.qpos`를 넘겨도 solver가 복사한 배열과 자체 트리만 수정하는지 확인 |
| site/body 기준 | IK target은 body origin이 아니라 `site` 기준인지 확인 |
| quaternion frame | local/world frame 변환 순서 확인 |
| contact force | 위치 조건이 아니라 `mj_contactForce()` 기반 판정인지 확인 |
| actuator range | `ctrlrange`, `forcerange`, joint `range` 확인 |
| wheel command | 조향 정렬 전 wheel velocity가 0으로 gated 되는지 확인 |
| marker state | UI target, mocap marker, IK world target 동기화 확인 |
| whole-body toggle | ON/OFF 전후 hand/virtual world pose와 cached base command 확인 |
| arm-only gate | base/lift weight가 아니라 lower/upper velocity bound를 0으로 고정 |
| manual handover | key release 동안 zero 유지, 정지 뒤 target/reference rebase 확인 |
| collision 범위 | finger-object와 wheel-floor 의도 접촉을 CBF pair에서 제외 |
| 문서 | 함수 역할이나 target 의미가 바뀌면 `docs/guide/` 업데이트 |
