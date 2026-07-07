# MuJoCo 최소 개념 사전

아래 다섯 가지 개념은 이어지는 파일별 설명 전체에서 계속 전제로 깔린다 — 이후
페이지를 읽다가 낯선 용어가 나오면 여기로 돌아와 확인하면 된다.

## MJCF, model, data — "설계도"와 "지금 이 순간"

MuJoCo는 로봇/물체의 구조를 **MJCF**라는 XML 포맷(이 저장소의 `models/*.xml`)으로
기술한다. 이 XML을 읽어서 만들어지는 `MjModel`은 **바뀌지 않는 설계도**다 — 몇 개의
관절이 있는지, 각 링크의 질량이 얼마인지, 관절의 가동 범위가 몇 도인지 같은 것들.
반면 `MjData`는 **매 순간 바뀌는 상태** — 지금 관절 각도가 몇 도인지(`qpos`),
얼마나 빠르게 움직이고 있는지(`qvel`), 지금 어떤 접촉이 발생했는지(`contact`) 등을
담는다. 같은 `model`을 두고 여러 개의 `data`를 동시에 굴릴 수도 있다
([`src/ik.py`](ik.md)가 정확히 이렇게 한다 — 실시간 시뮬레이션의 `data`와
IK 전용 `self._scratch`를 분리해서 쓴다).

```python title="가장 기본적인 MuJoCo Python 패턴"
import mujoco

model = mujoco.MjModel.from_xml_path("scene.xml")   # 설계도 (한 번만 로드)
data = mujoco.MjData(model)                          # 상태 (여러 개 만들 수 있음)

mujoco.mj_forward(model, data)   # 현재 qpos로부터 운동학/힘을 "다시 계산"만 함 (시간 안 흐름)
mujoco.mj_step(model, data)      # 한 스텝(timestep)만큼 실제로 물리를 적분해 시간을 흘림
```

!!! info "핵심 개념 · mj_forward vs mj_step"
    `mj_forward`는 "지금 qpos가 이 값이라면 접촉력·중력·관성이 어떻게 계산되는지"만
    다시 채워 넣을 뿐 시간을 흘리지 않는다. 반대로 `mj_step`은 그 힘들을 적분해서
    **다음 timestep의 qpos/qvel**을 실제로 만든다. [`src/ik.py`](ik.md)의 솔버는
    `mj_step`을 절대 호출하지 않고 `mj_forward`만 반복 호출한다 — "물리를 실행"하는
    게 아니라 "이 관절각이면 손끝이 어디 있을지"를 기구학적으로만 계산하기 위해서다.

- **nq / nv / nu**: 각각 위치 자유도 개수(`qpos` 길이), 속도 자유도 개수(`qvel` 길이,
  구속 등으로 nq와 다를 수 있음 — 예: 쿼터니언은 4개 수로 표현되지만 자유도는 3),
  actuator 개수(`ctrl` 길이)다.
- **body / joint / geom / site**: `body`는 관성을 가진 강체, `joint`는 두 body 사이의
  자유도, `geom`은 충돌·시각용 형상(질량은 없고 body에 딸림), `site`는 질량도 충돌도
  없는 순수 "이름 붙은 위치/좌표계" — [`src/ik.py`](ik.md)가 손끝 IK 목표점
  (`grasp_target_r`/`_l`)을 정의하는 데 쓴다.

## actuator 세 가지: position / motor / velocity

MuJoCo는 관절을 직접 원하는 각도로 "순간이동"시키는 API가 없다(그렇게 하면 그건 물리
시뮬레이션이 아니다). 대신 **actuator**가 매 스텝 관절에 힘/토크를 가하고, 그 결과로
관절이 물리적으로 움직인다. 이 프로젝트는 세 종류를 상황에 맞게 골라 썼다.

| 타입 | 동작 | 이 프로젝트에서 쓴 곳 |
|---|---|---|
| `<position>` | 목표 각도로 가는 내장 PD 제어기. `kp`(비례 게인), `dampratio`(감쇠비), `forcerange`(낼 수 있는 최대 토크)를 XML에 선언만 하면 끝. 적분항이 없어 정적 하중에서 "게인만큼" 오차가 남는다. | 손가락 관절([grasp.py](grasp.md)), 리프트/헤드/바퀴 조향([teleop_app.py](teleop_app.md)) |
| `<motor>` | 순수 토크 액추에이터. 목표 각도 개념이 없고, 파이썬에서 매 스텝 직접 토크값을 계산해 `data.ctrl`에 써야 한다 — 대신 중력 보상 같은 것도 직접 넣을 수 있어 자유도가 크다. | 팔 7개 관절([arm_control.py](arm_control.md)) |
| `<velocity>` | 목표 **각속도**를 향해 토크를 낸다(`kv` × (목표 속도 − 현재 속도)). 위치 개념이 없어 "제자리에 서 있으라"는 명령이 불가능하다 — 정확히 바퀴처럼 계속 굴러가는 것에 맞는 액추에이터. | 바퀴 구동 관절([base_teleop.py](base_teleop.md)) |

## contact 파라미터: solref / solimp / friction / condim / priority

두 geom이 맞닿으면 MuJoCo는 그 접촉을 하나의 **가상 스프링-댐퍼**로 처리한다. 이
스프링이 얼마나 부드러운지/뻣뻣한지를 정하는 게 `solref`/`solimp`이고, 마찰이 얼마나
큰지가 `friction`, 접촉이 몇 차원(수직/마찰/비틀림)까지 힘을 전달하는지가 `condim`이다.
이 값들은 `models/*.xml`에서 설정되고(이 가이드가 다루는 `src/` 파이썬 코드의 범위
밖이다), [grasp.py](grasp.md)의 `get_finger_can_contacts()`/`is_grasped()`가 바로
그 결과로 생기는 접촉력을 `mj_contactForce`로 읽어서 판정에 쓴다 — 지금은 "접촉도
완전히 딱딱한 벽이 아니라 조율 가능한 스프링"이라는 것만 기억해두자.

!!! tip "가장 자주 반복되는 진단 원칙"
    이 문서 전체에서 가장 자주 반복되는 진단 원칙 하나: **파라미터를 몇 배씩 바꿔도
    결과가 거의 그대로면, 그 파라미터는 원인이 아니다.** "게인을 더 올리면 되겠지"
    식으로 큰 수를 넣어보는 습관은 시간을 크게 낭비시킨다.
