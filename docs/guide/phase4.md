# Phase 4 — 로봇 전체를 조립하고, 직접 조작할 창을 띄우기

`models/full_scene.xml` · `src/teleop_app.py`

Phase 4는 세 번째이자 마지막 모델(`full_scene.xml`)로, 로봇 전체(양팔+양손+헤드+
리프트+베이스)를 조립하고 사람이 직접 슬라이더로 조작하는 텔레옵 앱을 만드는
단계다. 여기서는 **세 모델을 잇는 트릭**과 **MuJoCo의 저수준 렌더링 API**가 핵심이다.

## 세 모델(hand_only → arm_hand → full_scene)이 순수 평행이동으로만 다른 이유

로봇의 어깨-체인(`base_link → lift_link → arm_base_link`)은 공식 모델에서 **회전이
하나도 없다**(전부 `pos`만 있고 `quat`는 항등원). 즉 `lift_joint`를 홈 값으로
고정하면 어깨는 world의 특정 좌표에 **무회전으로** 고정된다 — `arm_hand.xml`에서
썼던 좌표와는 순수 평행이동 관계뿐이다. 덕분에 [Phase 3](phase3.md)에서 찾은 홈
자세, 엄지 pre-shape, site 오프셋, 캡슐 파라미터를 전부 그대로 재사용하고, 테이블/
캔의 world 좌표만 같은 평행이동만큼 옮기면 끝났다 — 순운동학(FK)으로 직접 검증까지
마쳤다.

!!! info "핵심 개념 · IK용 \"scratch\" MjData와 context_qpos"
    `full_scene.xml`은 팔 위에 `lift_joint`가 있는데, IK 솔버의 scratch 버퍼는
    호출마다 전체 리셋된다. 이 리셋이 `lift_joint`를 조용히 0으로 되돌려버려서,
    실제로는 팔이 아니라 **어깨 자체가 0.5m 위로 튀는** 것과 같은 효과가 났다(IK
    오차 233mm로 pick 전멸). 해결책은 `context_qpos` 파라미터를 추가해서, IK가
    직접 풀지 않는 다른 관절들(리프트 등)은 실시간 시뮬레이션의 현재 값으로 시드하는
    것 — "이 솔버가 책임지지 않는 자유도는 살아있는 상태를 그대로 반영해야 한다"는
    일반적인 패턴이다.

## 커스텀 GUI를 위한 저수준 렌더링 API

MuJoCo는 `mujoco.viewer.launch_passive`라는 편리한 내장 뷰어를 제공하지만, 이건
자체 창을 띄우기 때문에 그 안에 커스텀 슬라이더 패널을 넣을 수 없다. 이 프로젝트는
MuJoCo의 C++ "simulate" 앱과 같은 방식으로, **저수준 렌더링 API를 직접 호출**해서
하나의 GLFW 창 안에 3D 뷰와 ImGui 패널을 함께 그린다.

```python title="src/teleop_app.py — 렌더 루프 핵심"
scene = mujoco.MjvScene(model, maxgeom=10000)
context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
...
mujoco.mjv_updateScene(model, data, opt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
mujoco.mjr_render(viewport, scene, context)
# 그 위에 같은 프레임버퍼로 Dear ImGui 패널을 그림
```

마우스 궤도/팬/줌도 `mujoco.mjv_moveCamera`로 직접 구현하고,
`imgui.get_io().want_capture_mouse`로 "지금 마우스가 패널 위에 있는지"를 확인해
패널 조작 중엔 카메라가 안 돌아가게 막는다. GUI와 물리 루프가 완전히 같은 스레드/
같은 루프 안에서 순차 실행되므로, "GUI는 목표값만 쓰고 물리는 상태만 읽는" 단방향
데이터 흐름이 저절로 성립한다(레이스 컨디션 걱정이 없다).

결과: 팔 5초 유지 시 site 드리프트 0.3mm 이하, IK 단위테스트 100/100, 통합 pick
(오른손) **10/10**, 텔레옵 루프 실측 약 24.6Hz.

---

다음: [Phase 5 — 바퀴로 굴러가기](phase5.md)
