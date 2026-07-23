[← 전체 안내](../ros2-guide.md)

# Part 2 — MuJoCo 속성 강의 (ROS2/Gazebo 경험자를 위해) {: #part-2 }

!!! info "함께 볼 개발자 가이드"
    용어를 빠르게 찾으려면 [MuJoCo 기본 용어](../00-basics.md), 배열·객체 대응을
    나란히 보려면 [코드 읽기 치트시트](../cheatsheet.md)를 함께 참고한다.

## 2.1 URDF vs MJCF {: #part-2-1 }

ROS2 생태계에서 로봇 모델은 URDF(또는 xacro로 조립되는 URDF)로 쓴다. MuJoCo는
자기만의 XML 방언인 **MJCF**(`models/*.xml`)를 쓴다. 개념은 비슷하지만
(둘 다 "링크 트리 + 관절 + 충돌/시각 형상"을 XML로 서술) 세부가 다르다:

| URDF | MJCF | 차이 |
|---|---|---|
| `<link>` | `<body>` | body는 관성(mass/inertia)뿐 아니라 그 안에 자식 body/joint/geom을 중첩해서 담을 수 있다(트리 구조가 XML 중첩 그 자체) |
| `<joint type="revolute">` | `<joint type="hinge">` | 개념 동일. MJCF는 `range`, `damping`, `armature`, `frictionloss` 등을 훨씬 세밀하게 노출 |
| `<visual>`/`<collision>` mesh | `<geom>` (visual/collision 겸용, `contype`/`conaffinity`로 구분) | URDF는 시각/충돌을 별도 태그로 분리, MJCF는 태그 하나에 role을 플래그로 표시 |
| 없음(tf가 대신함) | `<site>` | **이 프로젝트에서 가장 중요한 개념 중 하나** — 질량도 충돌도 없는 순수 "참조 좌표계" 마커. IK 목표점(`grasp_target_r/l`)이 바로 이 site다. tf frame과 비슷하지만 자동 발행되지 않으며, `KinematicTree`가 body–joint 경로를 순회해 pose와 Jacobian을 직접 계산한다 |
| `<transmission>` + ros2_control `<ros2_control>` 블록 | `<actuator>` | 관절을 "무엇으로 어떻게 구동하는지" 지정. 아래 2.5 |
| 없음(Gazebo SDF에서 따로) | `<keyframe>` | qpos/ctrl의 스냅샷을 이름 붙여 저장(`home`, `pregrasp` 등) — 리셋할 때 그 상태로 즉시 복원 |

## 2.2 body / joint / site / geom / actuator 요약표 {: #part-2-2 }

| MJCF 요소 | 의미 | ROS2/URDF 유사 개념 |
|---|---|---|
| `<body>` | 질량+관성을 가진 강체, 트리의 노드 | `<link>` |
| `<joint>` | body 사이의 자유도(각도, 위치 등) | `<joint>` |
| `<geom>` | 충돌/시각 형상(캡슐, 박스, mesh 등) | `<collision>`+`<visual>` |
| `<site>` | 질량 없는 참조 좌표계(센서/타겟 부착점) | tf frame(단, 자동 트리 발행은 없음) |
| `<actuator>` | joint에 힘/토크/속도/위치 목표를 넣는 구동기 | `ros2_control` command interface |
| `<keyframe>` | qpos/ctrl 스냅샷 | 없음(가장 가까운 게 MoveIt named target) |
| `mocap="true"` body | 물리(질량/충돌) 없이 외부 코드가 pose를 직접 지정하는 body | RViz Interactive Marker의 내부 표현과 비슷 |

## 2.3 `MjModel`과 `MjData` — "정적 서술"과 "동적 상태"의 분리 {: #part-2-3 }

ROS2에서 URDF는 한 번 파싱되면 불변이고, 실제로 시시각각 변하는 상태(관절각 등)는
`/joint_states` 토픽으로 흐른다. MuJoCo도 정확히 이 둘을 나눈다:

- **`MjModel`**: XML을 컴파일한 결과. body/joint/geom/actuator 개수, 이름,
  질량, 관절 range, 액추에이터 게인 등 **바뀌지 않는 구조**. `mujoco.MjModel.from_xml_path(...)`로 한 번만 만든다.
- **`MjData`**: 그 모델의 **현재 상태**. `qpos`(관절 위치), `qvel`(관절 속도),
  `ctrl`(액추에이터 입력), `contact`(현재 접촉 목록) 등. `mujoco.MjData(model)`로
  만들고, 시뮬레이션이 진행되며 계속 바뀐다.

이 프로젝트가 "kinematic override 금지"라고 부르는 규칙(Part 3.2)은 결국
"`data.qpos`를 파이썬에서 직접 대입하지 마라, `data.ctrl`만 써서 액추에이터를
통해 간접적으로 움직여라"는 뜻이다. Gazebo에서 `SetModelState` 서비스로 모델을
순간이동시키는 것과 비슷한 치팅을 막는 것이다.

## 2.4 시뮬레이션을 진행시키는 두 함수 {: #part-2-4 }

| 함수 | 하는 일 | 시간이 흐르는가 |
|---|---|---|
| `mj_forward(model, data)` | 지금 `qpos` 기준으로 운동학(forward kinematics)/힘/접촉을 다시 계산 | 아니오 |
| `mj_step(model, data)` | 한 타임스텝만큼 물리를 적분 — `qpos`/`qvel`이 실제로 바뀜 | 예 |

기구학 솔버는 `MjModel`에서 body–joint–site 트리를 한 번 복사한 뒤 NumPy로 FK와
Jacobian을 직접 계산하므로 `mj_forward`를 반복 호출하지 않는다. 앱 초기화/reset과
물리 검증 테스트에서는 여전히 `mj_forward`가 필요하다. 실제 로봇이 움직이는 것은
메인 루프가 actuator command를 쓰고 `mj_step`을 부를 때뿐이라는 원칙은 같다.

## 2.5 액추에이터 3종 — `ros2_control` command interface와 비교 {: #part-2-5 }

| MJCF 태그 | 동작 방식 | 이 프로젝트에서 쓰는 곳 | `ros2_control` 유사 인터페이스 |
|---|---|---|---|
| `<position>` | 목표 각도를 향해 내장 P(비례) 제어로 힘을 낸다 | 손가락, 리프트, 바퀴 조향 | `PositionJointInterface` |
| `<velocity>` | 목표 각속도를 추종 | 바퀴 구동(`wheel_drive`) | `VelocityJointInterface` |
| `<motor>` | 목표값을 그대로 토크로 출력(내장 제어 없음) | 팔 7관절(`arm_r_joint1..7`) | `EffortJointInterface` |

**왜 팔만 `<motor>`(순수 토크)를 쓰는가**가 이 프로젝트의 중요한 설계 결정이다.
MuJoCo의 `<position>` 액추에이터는 순수 비례 제어라 적분(I)도 피드포워드도
없어서, 무거운 7링크 팔이 정적 자세를 버틸 때 중력 때문에 15~20mm 정도의 잔류
오차가 생겼다(`arm_control.py` 참고). 그래서 팔은 `<motor>`로 바꾸고
파이썬에서 직접 `tau = qfrc_bias + kp*(q_des-q) - kd*qvel`(Part 7)을 계산해
써준다 — `ros2_control`의 `effort_controllers/JointGroupEffortController`를
직접 구현한 셈이다. 반면 손가락은 일부러 `<position>`(P 제어, 힘 제한 있음)을
그대로 쓰는데, 접촉하면 토크가 **포화(saturate, 액추에이터가 낼 수 있는 최대
힘까지 이미 다 써버려서 더 이상 늘어나지 않는 상태)**돼서 스스로 멈추는
"순응(compliant) 그립"이 바로 이 힘 제한 위치 제어에서 나오는 의도된
효과이기 때문이다(Part 5).

## 2.6 접촉(contact) — Gazebo에는 없던 새 개념들 {: #part-2-6 }

Gazebo(ODE/Bullet/DART)를 써봤다면 마찰(friction), restitution 정도는 익숙할
텐데, MuJoCo는 접촉을 훨씬 세밀하게 조절하는 파라미터를 노출한다. 이 프로젝트의
grasp가 성립하는 이유가 전부 이 파라미터들에 있으므로 정리한다.

| 파라미터 | 의미 |
|---|---|
| `solref` | 접촉을 스프링-댐퍼로 모델링할 때의 시간상수/감쇠비. 작을수록(더 뻣뻣, stiff) 관통이 줄지만 발산하기 쉽다 |
| `solimp` | 접촉 impedance(얼마나 "단단하게" 위반을 막을지) 곡선 |
| `friction` | 접촉 마찰 계수(접선 방향 2개 + 비틀림/구름 방향까지, 최대 5차원) |
| `condim` | 접촉력의 차원(1=법선만, 3=마찰까지, 6=비틀림/구름까지) — `condim=6`은 캔이 손안에서 겉돌지 않게 회전 마찰까지 모델링 |
| `priority` | 두 geom이 접촉할 때 어느 쪽의 solref/solimp/friction/condim을 쓸지 우선순위. **이 프로젝트는 캔에 `priority="1"`을 줘서, 손가락 수십 개 geom을 하나하나 튜닝하는 대신 캔 쪽 파라미터 하나만 맞추면 되게 만들었다** |

Gazebo에서 물체를 "쥔다"는 걸 구현할 때 흔히 유혹받는 지름길이 fixed joint를
런타임에 붙였다 뗐다 하는 것(kinematic attach)인데, 이 프로젝트는 그 지름길을
전면 금지한다(Part 3.2) — 순전히 위 표의 파라미터들로 만든 접촉력만으로 캔이
붙어 있어야 한다.

## 2.7 "물리 치팅 금지" 철학이 왜 이렇게 강한가 {: #part-2-7 }

이 프로젝트는 사실 **세 번째 시도**다. 이전 두 저장소(`ffw-sh5-mujoco`,
`ffw-sh5-teleoperation`)가 각각 "kinematic 부착 방식이라 사실 물리가 아님",
"C++ Bullet3인데 관통(penetration)을 못 잡음"으로 실패했다. 그래서 이번 저장소는
맨 처음부터 절대 규칙 1번으로 "kinematic override 금지"를 못 박고 시작했다 —
ROS2로 치면 "이 패키지는 `set_entity_state` 서비스를 그 어떤 이유로도 호출하지
않는다"는 프로젝트 헌장 같은 것이다.

---

[← Part 1](./01-concepts.md) · [전체 안내](../ros2-guide.md) · [Part 3 →](./03-project-identity.md)
