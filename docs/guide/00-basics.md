# MuJoCo 기본 용어

## Core

| 용어 | 의미 |
|---|---|
| MJCF | MuJoCo XML 모델 파일 |
| `MjModel` | 컴파일된 모델 구조. body, joint, geom, actuator 정보 |
| `MjData` | 현재 시뮬레이션 상태. qpos, qvel, ctrl, contact 등 |
| `qpos` | 위치 상태 배열 |
| `qvel` | 속도 상태 배열 |
| `ctrl` | actuator 입력 배열 |
| `nq` | qpos 길이 |
| `nv` | qvel 길이 |
| `nu` | actuator 개수 |

## Model Elements

| 요소 | 의미 |
|---|---|
| `body` | 질량과 관성을 가진 강체 |
| `joint` | body 사이 자유도 |
| `geom` | 충돌/시각 형상 |
| `site` | 질량/충돌 없는 참조 좌표계 |
| `actuator` | joint에 힘, 토크, 속도, 위치 목표를 적용하는 요소 |
| `keyframe` | qpos/ctrl 초기 상태 스냅샷 |

## Simulation Calls

| API | 역할 |
|---|---|
| `mj_forward(model, data)` | 현재 qpos 기준으로 운동학/힘/contact 정보를 다시 계산. 시간은 흐르지 않음 |
| `mj_step(model, data)` | 한 timestep 물리 적분. qpos/qvel이 실제로 변함 |
| `mj_resetData(model, data)` | data 초기화 |
| `mj_resetDataKeyframe(model, data, key_id)` | keyframe 상태로 data 초기화 |

## Actuator Types

| 타입 | 역할 | 사용 위치 |
|---|---|---|
| `<position>` | 목표 위치를 향해 힘/토크 적용 | 손가락, 리프트, 헤드, 바퀴 조향 |
| `<motor>` | 직접 토크 입력 | 팔 관절 |
| `<velocity>` | 목표 속도 추종 | 바퀴 구동 |

## Contact

| 속성/API | 역할 |
|---|---|
| `solref` | 접촉 spring-damper 시간상수/감쇠 |
| `solimp` | 접촉 impedance 곡선 |
| `friction` | 접촉 마찰 |
| `condim` | 접촉 force 차원 |
| `priority` | 접촉 파라미터 우선순위 |
| `mj_contactForce()` | 특정 contact의 force 읽기 |

## 이 프로젝트의 규칙

- 로봇 관절 live `data.qpos`를 직접 덮어쓰지 않는다.
- IK는 scratch `MjData`에서만 계산한다.
- 실제 움직임은 actuator command와 `mj_step()`으로 만든다.
