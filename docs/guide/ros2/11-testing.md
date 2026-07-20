[← 전체 안내](../ros2-guide.md)

# Part 11 — 테스트와 검증 철학 {: #part-11 }

## 11.1 Phase별 테스트 게이트 {: #part-11-1 }

| 파일 | 무엇을 검증하나 | 실행 방식 |
|---|---|---|
| `test_phase_0.py` | 공식 모델 로드, 5초 무제어 안정성 | headless |
| `test_phase_1.py` | 손가락-캔 관통 깊이(20회) < 2mm | headless |
| `test_phase_2.py` | 고정 손 grasp+lift(±5mm 노이즈 10회) ≥ 8/10 | headless |
| `test_phase_3.py` | IK 100개 샘플 95% 수렴 + 통합 pick ≥ 7/10 | headless |
| `test_phase_4.py` | 전신 hold 회귀 + IK + pick 10회 | headless |
| `test_phase_5.py` | 입력 응답/FSM/조향 limiter 단위 + 유휴/전후/strafe/yaw/복합/반전/충돌 회귀 | headless |
| `test_phase_6.py` | model/keyframe gate, marker jog, Bimanual MoveL capture/release, 3D gizmo pose round-trip, whole-body ON/OFF pose 불변, collision overlay, XYZ/RPY IK | headless |
| `test_whole_body.py` | ROS-free import gate, arm-only hard gate, 스워브 왕복, 관절·self/table collision CBF와 수치 gradient, 무작위 18DOF solver 40회, 실제 바퀴의 XYZ/yaw whole-body 추종 | headless |

모두 `mujoco`의 **오프스크린(headless)** 모드로 돈다 — 창을 띄우지 않고
물리만 돌려서 자동 판정한다. ROS2의 `launch_testing`/`colcon test`가 실제
노드를 띄우고 토픽을 관찰해 assert하는 것과 목적은 같지만, 여기는 노드도
토픽도 없으니 그냥 함수를 직접 호출해서 결과를 확인한다.

## 11.2 실행 방법 {: #part-11-2 }

```bash
for p in 0 1 2 3 4 5 6; do python3 tests/test_phase_$p.py; done
```

각 파일은 독립적으로 실행 가능하다(공유 상태 없음, 매번 새 `MjModel`/`MjData`
생성) — `pytest` 같은 프레임워크 없이 `if __name__ == "__main__"` + 리턴
코드로 성공/실패를 판단하는 아주 단순한 구조다.

## 11.3 git tag = release 전략 {: #part-11-3 }

Phase가 하나 끝날 때마다 `git tag phase-N`을 찍는다(3.2절 규칙 4).
지금은 `0.0.1`/`0.0.2`/`0.1.0`/`1.0.0`/`1.1.0`/`1.1.1` 같은 semver 태그도 병행한다. 이건
ROS2 패키지의 릴리즈 태깅과 같은 개념이지만, CI가 자동으로 태그를 찍어주는
게 아니라 사람이 "이 Phase의 성공 기준을 통과했다"고 판단한 시점에 수동으로
찍는다.

---

[← Part 10](./10-coordinate-frames.md) · [전체 안내](../ros2-guide.md) · [Part 12 →](./12-running.md)
