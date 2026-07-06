# MuJoCo 튜토리얼 — 무조코에게 로봇 손으로 캔을 쥐는 법을 가르치기까지

이 문서는 **MuJoCo를 한 번도 써본 적 없는 대학생**을 대상으로 쓴 실전 가이드다. ROBOTIS
FFW-SH5 로봇이 **kinematic 치팅 없이, 오직 접촉력(contact force)만으로** 테이블 위
캔을 집어 드는 시뮬레이터를 실제로 만든 과정을, 그 프로젝트가 진행된 순서(Phase 0 →
5) 그대로 따라가면서 **그 단계에서 새로 필요해진 MuJoCo 기능이 정확히 무엇이고 왜 그
기능을 써야 했는지**를 하나씩 짚는다.

이론을 먼저 늘어놓지 않는다 — 실제로 막혔던 문제, 그걸 풀기 위해 찾아 쓴 MuJoCo API,
그리고 숫자로 확인한 결과 순서로 읽는다. 코드/XML 스니펫은 전부 실제 레포
(`ffw-sh5-grasp`)에서 그대로 가져온 것이다.

이 프로젝트를 **왜** 이런 구조로 설계했는지(실패했던 이전 두 번의 시도, 설계 판단의
근거)가 궁금하다면 [프로젝트 개요](../overview.md)를 먼저 보는 것도 좋다. 이 튜토리얼은
그 반대편 — **어떻게(How)**, 그중에서도 **MuJoCo의 어떤 기능을 어떤 순서로 썼는가**에
집중한다.

<span class="phase-track">
<span>PHASE 0 · 공식 모델 검증</span>
<span>PHASE 1 · 손 콜리전 정비</span>
<span>PHASE 2 · 고정 손 grasp</span>
<span>PHASE 3 · 팔 + IK</span>
<span>PHASE 4 · 전체 조립 + 텔레옵</span>
<span>PHASE 5 · 바퀴 주행</span>
</span>

## 읽는 순서

1. [MuJoCo 최소 개념 사전](00-basics.md) — model/data, actuator 세 가지, contact
   파라미터. 처음이라면 여기부터.
2. [Phase 0 — 공식 모델 검증](phase0.md)
3. [Phase 1 — 손 콜리전 정비](phase1.md) — mocap+weld, capsule 콜리전, priority
4. [Phase 2 — 고정 손 grasp](phase2.md) — position actuator + forcerange, contact
   force로 성공 판정
5. [Phase 3 — 팔 + IK](phase3.md) — site, Jacobian, DLS IK, motor + PD + 중력
   feedforward
6. [Phase 4 — 전체 조립 + 텔레옵](phase4.md) — 렌더링 파이프라인, context_qpos
7. [Phase 5 — 바퀴 주행](phase5.md) — velocity actuator, `<pair>`, 접촉 강성
8. [흔한 함정 총정리](pitfalls.md) — 이 프로젝트가 반복해서 배운 것들
9. [API 치트시트](cheatsheet.md) — 실제로 쓴 MuJoCo API/MJCF 요소 전부 + 더 읽어볼 곳

!!! tip "이 문서 전체에서 가장 자주 반복되는 진단 원칙"
    파라미터를 몇 배씩 바꿔도 결과가 거의 그대로면, 그 파라미터는 원인이 아니다.
    "게인을 더 올리면 되겠지" 식으로 큰 수를 넣어보는 습관은 시간을 크게 낭비시킨다 —
    이 프로젝트에서 최소 세 번, 그 신호를 무시하고 계속 파라미터를 밀어붙였다가
    나중에야 진짜 원인(좌표계 버그, keyframe 오타, 구조적 접촉 조건)을 찾은 사례가
    나온다.

!!! quote "절대 규칙 하나만 먼저 기억하자"
    이 프로젝트 전체에서 `data.qpos[...] = 값`으로 로봇의 물리 상태를 직접 덮어쓰는
    코드는 (리셋과 물체 초기 배치를 제외하고) 단 한 줄도 없다. "물리 엔진이 원하는
    결과를 안 주면, 상태를 억지로 만들지 말고 XML의 물리 파라미터(질량, 마찰,
    solver, actuator)를 고친다"는 태도로 이해하면 된다. 이게 바로 **kinematic
    시뮬레이션**(좌표를 직접 지정)과 **dynamic 시뮬레이션**(힘과 접촉으로 좌표가
    결과로 나옴)의 차이이고, 이 프로젝트가 정확히 후자를 지키려고 만들어졌다.
