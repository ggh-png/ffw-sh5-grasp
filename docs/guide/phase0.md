# Phase 0 — 공식 모델을 믿기 전에, 먼저 뜯어본다

`assets/robotis_ffw/` · `tests/test_phase_0.py`

이 프로젝트는 ROBOTIS의 공식 `robotis_mujoco_menagerie` 모델을 그대로 가져와
시작한다. 그런데 코드를 한 줄 쓰기 전에, 그 모델이 실제로 어떻게 생겼는지부터
프로그램으로 확인했다 — "공식 모델이니 당연히 이럴 것이다"라는 가정을 검증 없이 쓰지
않는다는 이 프로젝트의 태도가 Phase 0부터 나타난다.

!!! info "핵심 개념 · 모델 구조를 코드로 리포트하기"
    MuJoCo는 컴파일된 `model` 객체의 모든 필드를 파이썬에서 그대로 읽을 수 있다.
    관절 이름은 `mj_id2name`, 개수는 `model.nq/nv/nu`, geom이 mesh인지
    primitive(캡슐/박스/구)인지는 `model.geom_type`으로 바로 확인 가능하다 — 문서를
    뒤지는 대신 모델 자체에 물어보면 된다.

```python title="Phase 0에서 실제로 확인한 것 (tests/test_phase_0.py 핵심 아이디어)"
model = mujoco.MjModel.from_xml_path("assets/robotis_ffw/scene.xml")
print(model.nq, model.nv, model.nu)   # 70 69 63

for j in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
    print(name, model.jnt_range[j], model.jnt_limited[j])

for g in range(model.ngeom):
    # geom_type == mujoco.mjtGeom.mjGEOM_MESH 이면 mesh, 아니면 capsule/box/sphere 등 primitive
    print(model.geom_type[g], model.geom_contype[g], model.geom_condim[g])
```

확인 결과 두 가지가 이후 모든 단계를 결정했다. 첫째, 5초짜리 무제어(중력만 켠)
시뮬레이션에서 관절 가속도(`qacc`)가 발산하지 않았다(최대 172, 기준 100,000 이하 통과)
— 모델 자체의 질량/관성 값이 건전하다는 뜻. 둘째, **손가락 충돌 geom이 전부 mesh
기반**이라는 걸 발견했다 — 이게 [Phase 1](phase1.md) 전체를 결정한다.

!!! tip "배울 점 · mesh 콜리전은 시각적으로 정확하지만 물리적으로는 비싸고 불안정하다"
    삼각형 수천 개짜리 mesh 두 개가 맞닿는 접촉을 매 스텝 계산하는 건 느릴 뿐 아니라,
    뾰족한 모서리끼리 만나면 접촉점이 불안정해지기 쉽다. MuJoCo 커뮤니티(그리고
    DeepMind의 `shadow_hand` 참조 모델)의 표준 관행은 **시각용 mesh와 충돌용 geom을
    분리**하는 것 — 화면에는 정교한 mesh를 그리되, 물리 계산에는 그 mesh를 감싸는
    캡슐/박스 같은 단순 도형을 쓴다. Phase 1은 이걸 실제로 구현하는 단계다.

---

다음: [Phase 1 — 손 하나, 콜리전부터](phase1.md)
