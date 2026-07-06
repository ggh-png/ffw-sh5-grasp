# Phase 1 — 손 하나만 떼어내, 콜리전을 처음부터 다시 만든다

`models/hand_only.xml` · `tests/measure_hand_meshes.py`

Phase 1의 목표는 오른손(HX5-D20) 하나만 world에 고정해두고, mesh 콜리전을 캡슐로
바꾸는 것이다. 여기서 이 프로젝트 전체를 관통하는 첫 번째 핵심 트릭 — **물리 상태를
절대 직접 쓰지 않고 손을 "고정"하는 법** — 이 등장한다.

## mocap body + weld 제약 — "붙잡아 두기"를 물리적으로 하는 법

손을 허공에 고정하고 싶다고 해서 `data.qpos[...] = 고정값`을 매 스텝 덮어쓰면, 그건
더 이상 물리 시뮬레이션이 아니다(캔이 손을 밀어내려 해도 손이 전혀 반응하지 않는,
무한한 힘을 가진 유령이 되어버린다). 대신 MuJoCo는 **mocap body**라는 특수한 body를
제공한다 — 물리 연산에서 제외되고 파이썬에서 자유롭게 위치를 지정할 수 있는
"참조점"이다. 이 mocap body와 손을 **equality weld constraint**로 용접하면, 물리
엔진이 **제약력(constraint force)**을 계산해서 손을 그 위치로 "당긴다" — 캔이 손을
밀면 그 반작용력도 정상적으로 반영된다.

```xml title="models/hand_only.xml"
<body name="hand_mocap" mocap="true" pos="0 0 0.15" quat="0.5 0.5 0.5 0.5"></body>

<body name="hx5_r_base" ...>
  <freejoint name="hand_free"/>
  ...
</body>

<equality>
  <weld body1="hx5_r_base" body2="hand_mocap" solref="0.005 1" solimp="0.9 0.95 0.001"/>
</equality>
```

!!! info "핵심 개념 · freejoint + equality vs qpos 직접 대입"
    `freejoint`는 body에 6자유도(위치 3 + 회전 3)를 통째로 주는 조인트다. 손에
    `freejoint`를 주고 `weld`로 mocap에 묶으면, "손을 어디에 두고 싶다"는
    의도(mocap 위치 갱신)와 "그 결과 손이 실제로 거기 있다"는 물리적 사실(제약력으로
    실현됨) 사이에 물리 엔진이 끼어든다. 이 차이는 사소해 보이지만, **파지가 진짜
    힘으로 이뤄지는지 위치를 억지로 맞춘 것인지**를 가르는 이 프로젝트의 핵심 판단
    기준이다 — PLAN.md의 "kinematic override 금지" 규칙이 정확히 이 구분을 말한다.

## Mesh를 실측해서 캡슐로 근사하기

Phase 0에서 finger collision이 mesh라는 걸 확인했으니, 각 손가락 마디의 실제 치수를
재서 캡슐(원기둥 양 끝에 반구를 붙인 도형)로 바꿔야 한다. `trimesh` 라이브러리로 각
STL의 로컬 AABB(축 정렬 경계 상자)를 구하고, 가장 긴 축을 캡슐의 길이축으로, 나머지
두 축의 평균을 반지름으로 근사했다. fingertip은 실측값보다 1mm 더 크게 잡아 "패드"
여유를 줬다.

```xml title="models/hand_only.xml — capsule/box geom 정의"
<default class="collision">
  <geom group="3" contype="1" conaffinity="1" condim="3"
        solimp="0.5 0.99 0.0001" solref="0.005 1"/>
</default>

<geom type="capsule" fromto="0 0.00525 0  0 0.02275 0" size="0.0115" class="collision"/>
```

`fromto`는 캡슐 중심축의 시작점과 끝점(로컬 좌표), `size`는 반지름이다. 시각용 mesh
geom은 `contype="0" conaffinity="0"`로 아예 충돌 계산에서 빠지도록 별도
`class="visual"`로 분리했다 — 화면에는 정교한 mesh가 보이지만, 물리 엔진은 그 밑에
숨은 캡슐만 계산한다.

## solimp/solref/priority — shadow_hand 레시피를 그대로 가져온 이유

접촉 파라미터를 임의로 발명하지 않고, DeepMind의 검증된
[shadow_hand](https://github.com/google-deepmind/mujoco_menagerie/tree/main/shadow_hand)
모델 값을 그대로 가져왔다: 손 geom엔 `solimp="0.5 0.99 0.0001"` `solref="0.005 1"`,
캔(잡히는 물체)엔 `priority="1"` `condim="6"` `friction="0.5 0.01 0.003"`.
`condim="6"`은 접촉이 수직 힘 + 마찰 2방향 + 회전 마찰(비틀림 + 굴림) 6차원 전부를
전달한다는 뜻 — 캔이 손 안에서 미끄러지거나 굴러 빠지는 걸 막는 데 필요하다.

!!! bug "실제로 만난 버그 · priority가 solref/solimp까지 통째로 가져가는 줄 몰랐던 사건"
    **증상**: 손가락 geom에 shadow_hand 레시피(`solimp="0.5 0.99 0.0001"`)를 정확히
    그대로 넣었는데도 관통 테스트에서 최대 28mm 관통(허용치 2mm의 14배).

    **원인**: 캔 geom에 `priority="1"`을 준 순간, MuJoCo는 마찰뿐 아니라
    **solref/solimp까지 통째로 우선순위가 높은 쪽(캔)의 값을 쓴다**. 그런데 캔 geom
    자체엔 solref/solimp를 안 넣어서 엔진 기본값(훨씬 무른 스프링)이 쓰이고 있었다.

    **해결**: 캔 geom에도 손과 동일한 `solimp`/`solref`를 명시적으로 넣음 — 28mm →
    3.16mm로 즉시 개선.

    **배울 점**: `priority`는 "이 geom의 마찰만 이긴다"가 아니라 **접촉 파라미터 세트
    전체(솔버 파라미터 포함)를 통째로** 가져간다. 두 geom의 속성이 "섞이는" 게 아니라
    "한쪽이 이긴다"는 이 비직관적인 동작은 문서를 꼼꼼히 읽지 않으면 놓치기 쉽다.

추가로, 캡슐 근사는 실제 mesh보다 약간 더 뚱뚱해서 인접한 손가락 마디끼리 자체 충돌이
생겼다. 부모-자식 관계가 아닌 이런 쌍은 `<exclude>`로 명시적으로 접촉 계산에서 뺀다.

결과: 관통 테스트 20회 반복, 최대 관통 **1.174mm**(기준 2mm) 통과, 실시간 배율(RTF)
약 32배.

---

다음: [Phase 2 — 잡는다는 것의 물리](phase2.md)
