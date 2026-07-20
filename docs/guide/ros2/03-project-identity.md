[← 전체 안내](../ros2-guide.md)

# Part 3 — 프로젝트 정체성 {: #part-3 }

## 3.1 무엇을 만들었나 {: #part-3-1 }

ROBOTIS FFW-SH5(양팔 7DOF×2 + HX5-D20 5지 핸드×2 + 모바일 베이스)가 테이블
위의 캔을 **오직 접촉력만으로** 집어 드는 MuJoCo 텔레오퍼레이션 시뮬레이터.
기준 모델은 공식 `robotis_mujoco_menagerie`, contact/actuator 레시피의
기준점은 DeepMind `mujoco_menagerie/shadow_hand`(검증된 값에서 시작하고
임의로 발명하지 않는다는 원칙).

## 3.2 절대 규칙 (모든 Phase에 적용) {: #part-3-2 }

1. **kinematic override 금지** — `data.qpos[...] = value`로 물리 상태를 직접
   덮어쓰지 않는다. 유일한 예외는 reset과 물체 초기 배치(`teleop_app.py`의
   `_reset_can_random`이 이 예외에 해당하는 유일한 곳).
2. **물리 파라미터는 전부 XML에** — 파이썬에서 `model.geom_friction[...] = ...`
   같은 컴파일 후 수정 금지.
3. **Phase 순서 준수** — 성공 기준을 통과하기 전 다음 Phase 코드를 쓰지 않는다.
4. **Phase 완료마다 git commit + tag**.
5. **Phase마다 headless 테스트 스크립트를 남긴다.**

이 다섯 규칙은 지금까지 한 번도 깨진 적이 없다 — Part 14의 버그
사례들도 전부 "qpos를 덮어써서 대충 해결"하는 대신 XML/알고리즘에서 진짜 원인을
찾은 기록이다. (이 규칙들과 Phase별 상세 튜닝 기록은 저장소 초기에 `PLAN.md`/
`NOTES.md`라는 별도 파일에 있었으나, 이 문서를 포함한 mkdocs 사이트가 실제
문서 역할을 대체하면서 정리됐다 — 원본 기록은 git 히스토리에 남아 있다.)

## 3.3 Phase 0~6 여정 요약 {: #part-3-3 }

| Phase | 무엇을 추가했나 | 핵심 성공 기준 | 테스트 파일 |
|---|---|---|---|
| 0 | 공식 모델 그대로 로드해서 구조 검증 | 5초 무제어 시뮬레이션 발산 없음 | `test_phase_0.py` |
| 1 | 오른손 단독 씬, mesh collision → capsule 교체 | 관통 20회 테스트 < 2mm | `test_phase_1.py` |
| 2 | grasp synergy(`grasp.py`) + 접촉력 기반 판정 | grasp+lift 10회 중 8회 이상 성공 | `test_phase_2.py` |
| 3 | 오른팔 + IK(`ik.py`) | IK 100개 샘플 95% 수렴, pick 7/10 | `test_phase_3.py` |
| 4 | 전신 로봇 + 텔레옵 GUI | pick 10회 중 3회 이상(수동), 스크립트 재현 10/10 | `test_phase_4.py` |
| 5 | 모바일 베이스, 실제 바퀴 마찰 주행(`base_teleop.py`) | 유휴/직진/충돌정지/제자리회전 회귀 | `test_phase_5.py` |
| 6 | Cyclo Control 스타일 3D 마커 텔레옵(`teleop_targets.py`, gizmo) | marker jog, capture/release, XYZ/RPY IK 게이트 | `test_phase_6.py` |

Phase 4까지는 "손이 캔을 쥔다"가 목표였고, Phase 5는 "로봇 전체가 바퀴로
움직인다", Phase 6은 "조작 인터페이스를 어떻게 손으로 직관적으로 다루는가"에
집중한 단계다. 특히 Phase 6은 요구사항이 여러 번 정정되며 진화했다
(슬라이더 → +/- jog 버튼 → Cyclo 패널 → 화면 안 3D gizmo, Part 9.5 참고) —
"내가 생각한 UI"와 "사용자가 실제로 원한 UI"가 다를 수 있다는 걸 보여주는 좋은
사례다.

## 3.4 현재 상태 스냅샷 {: #part-3-4 }

- 최신 버전 태그: `1.1.1` (`1.1.0`에는 ROS-free WBIK와 모바일 안정화, `1.1.1`에는
  IK/FK·collision 개선과 whole-body/arm-only 전환이 포함된다.)
- 캔 시나리오만 라이브 — 한때 있었던 "상자 양손 들기(box scenario)" teleop 경로와
  사용되지 않던 양손 제약 파이썬 코드는 제거됐다. XML에 남은 `box_body` 자산은
  `teleop_app.py`가 시작 시 `_disable_legacy_box_asset()`로 비활성화한다.
- 조작 철학은 "Cyclo Control"(ROBOTIS 자체 텔레옵 UI) 스타일 3D 마커
  기반으로 정착 — 자세한 내용은 Part 10.
- 문서 사이트: https://ggh-png.github.io/ffw-sh5-grasp/ (mkdocs-material,
  GitHub Pages를 쓰려고 저장소가 의도적으로 public이다)

---

[← Part 2](./02-mujoco-model-data.md) · [전체 안내](../ros2-guide.md) · [Part 4 →](./04-runtime-architecture.md)
