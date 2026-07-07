# API 치트시트 — 이 프로젝트가 실제로 쓴 MuJoCo API 전부

아래는 이 프로젝트의 소스 코드에 실제로 등장하는 `mujoco` 모듈 함수/클래스 전부다.
MuJoCo Python API 전체(수백 개)에 비하면 극히 일부지만, "물리 기반 조작 시뮬레이터
하나를 처음부터 끝까지 만드는 데" 실제로 필요했던 최소 집합이라는 의미가 있다.

## Python API

| API | 역할 |
|---|---|
| `MjModel.from_xml_path` | MJCF(XML) 파일을 읽어 컴파일된 모델(설계도)을 만든다. |
| `MjData(model)` | 그 모델에 대한 상태(qpos/qvel/ctrl/contact 등)를 담을 그릇을 만든다. 같은 model로 여러 개 생성 가능. |
| `mj_forward` | 현재 qpos 기준으로 운동학/힘을 다시 계산한다(시간은 안 흐름). IK처럼 "이 자세라면 어떻게 되는지"만 알고 싶을 때. |
| `mj_step` | 한 timestep만큼 실제로 적분해 시간을 흘린다. 진짜 물리 시뮬레이션을 진행시키는 함수. |
| `mj_resetData` / `mj_resetDataKeyframe` | data를 초기 상태로, 또는 저장해둔 keyframe 상태로 되돌린다. |
| `mj_name2id` / `mj_id2name` | "finger_r_joint6" 같은 이름과 내부 정수 인덱스를 서로 변환한다. |
| `mj_jacSite` | 지정한 site의 위치 Jacobian(jacp)과 회전 Jacobian(jacr)을 계산한다 — IK의 핵심 재료. |
| `mj_contactForce` | i번째 접촉점의 힘(접촉 로컬 프레임, 법선+마찰+비틀림 6차원)을 읽는다. |
| `mju_subQuat` | 두 쿼터니언 사이의 회전 오차를 3차원 벡터(로컬 프레임)로 구한다. |
| `mju_mat2Quat` | 3×3 회전 행렬을 쿼터니언으로 변환한다. |
| `mju_mulQuat` | 두 쿼터니언을 합성(Hamilton product)한다 — `teleop_app.py`가 `base_quat ⊗ home_quat ⊗ rpy_delta`로 손 목표 자세를 조립하는 데 쓴다. |
| `mju_negQuat` | 쿼터니언의 켤레(단위 쿼터니언이면 역회전)를 구한다 — FK→IK 전환 시 조립 순서를 거꾸로 풀 때 필요. |
| `MjvScene` / `MjrContext` | 렌더링용 장면 버퍼와 GPU 컨텍스트 — 커스텀 GUI를 만들 때 `launch_passive` 대신 직접 다룬다. |
| `mjv_updateScene` / `mjr_render` | 현재 data 상태로 장면을 갱신하고, 실제로 화면에 그린다. |
| `mjv_moveCamera` | 궤도 회전/팬/줌 등 마우스 카메라 조작을 직접 구현할 때 쓴다. |

## MJCF 요소/속성

| 요소/속성 | 의미 |
|---|---|
| `mocap="true"` | 물리 연산에서 제외되고 파이썬에서 자유롭게 위치를 지정할 수 있는 참조 body. |
| `<equality><weld>` | 두 body를 제약력으로 "용접" — qpos를 직접 쓰지 않고 물리적으로 붙잡아 둔다. |
| `<freejoint>` | body에 6자유도(위치+회전)를 통째로 부여한다. |
| `solref` / `solimp` | 접촉을 가상 스프링-댐퍼로 볼 때의 시간상수/감쇠비(solref)와 임피던스 곡선(solimp). |
| `condim` | 접촉이 전달하는 힘의 차원(1=법선만 ~ 6=법선+마찰+비틀림+굴림). |
| `priority` | 두 geom의 접촉 파라미터가 다를 때, 우선순위가 높은 쪽 값을 통째로 채택한다(섞이지 않음). |
| `<exclude>` | 특정 body 쌍의 접촉 계산을 아예 생략한다. |
| `<pair>` | 특정 geom 쌍에만 개별 접촉 파라미터를 지정한다(기본 geom 속성보다 우선). |
| `<keyframe>` | qpos/ctrl 스냅샷을 이름 붙여 저장 — 재현 가능한 초기 자세로 즉시 복원. |
| `<site>` | 질량·충돌이 없는 순수 참조 좌표계 — IK 목표점 정의에 사용. |
| `<default class="...">` | 여러 geom/joint/actuator에 공통 속성을 상속시키는 템플릿(반복 방지). |

## 더 읽어볼 곳

- [MuJoCo 공식 문서](https://mujoco.readthedocs.io/) — 모든 XML 요소/Python API의
  1차 출처.
- [mujoco_menagerie/shadow_hand](https://github.com/google-deepmind/mujoco_menagerie/tree/main/shadow_hand)
  — 이 프로젝트의 손 콜리전/actuator 레시피의 기준점.
- [robotis_mujoco_menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)
  — FFW-SH5 공식 모델 출처.
- 이 저장소의 `PLAN.md`(설계 원칙), `NOTES.md`(Phase별 상세 튜닝 기록·실측 수치)도
  함께 보면 이 문서에서 요약한 내용의 원문을 볼 수 있다.

---

다음: [직접 실행하기](../run.md)
