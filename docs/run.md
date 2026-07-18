# 실행 방법

## 설치

```bash
pip install --break-system-packages mujoco numpy trimesh pillow glfw imgui-bundle
```

문서 미리보기까지 필요하면:

```bash
pip install --break-system-packages mkdocs mkdocs-material
```

## 텔레옵 실행

```bash
python3 src/teleop_app.py
```

하나의 네이티브 창에 3D 뷰와 ImGui 패널이 함께 열린다.

## 조작

| 입력 | 기능 |
|---|---|
| Mouse left drag | 카메라 orbit |
| Mouse right drag | 카메라 pan |
| Mouse wheel | 카메라 zoom |
| `Up` / `Down` | 베이스 전진 / 후진 |
| `Left` / `Right` | 베이스 yaw 회전 |
| `[` / `]` | 베이스 좌 / 우 strafe |
| `Q` / `E` | 리프트 하강 / 상승 |
| `R` | 캔 리셋 |
| `G` | contact point/force 표시 토글 |
| `V` | collision geometry와 활성 CBF 최근접점/연결선 표시 토글 |
| `C` | 카메라 프리셋 전환 |

## UI 패널

| 패널 | 역할 |
|---|---|
| Status | controller, marker, IK 오차, base pose 표시 |
| Cyclo / Marker Control | `MoveL`, `Bimanual MoveL`, marker jog, capture/release |
| Right Arm / Left Arm | 손별 IK pose 또는 FK joint target |
| Can Grasp | 손별 grasp/thumb synergy |
| Lift / Utilities | 전신 제어 ON/OFF, lift, reset, contact/collision 표시, camera |
| Joint Monitor | 주요 관절 위치 표시 |

### 전신 제어 버튼

`Lift / Utilities`의 **Whole-body Control** 버튼으로 모드를 바꾼다.

| 표시 | IK에 참여하는 축 | 별도 수동 조작 |
|---|---|---|
| `ON` | base x/y/yaw + lift + IK 모드 팔 | 키보드 base 명령이 우선 |
| `OFF (arm-only)` | IK 모드 팔만 | 화살표/`[/]` base와 `Q/E`·lift slider 사용 가능 |

OFF는 base/lift task weight만 낮추는 옵션이 아니라 해당 네 속도를 solver bound에서
0으로 고정한다. 전환 직전 손과 virtual-object world pose를 새 좌표계로 다시 표현해
marker가 튀지 않으며, 남아 있던 whole-body base twist도 0으로 지운다. Collision CBF는
계속 활성화되어 OFF에서는 팔 자유도만으로 안전한 명령을 찾는다.

## 테스트

전체 테스트:

```bash
for p in 0 1 2 3 4 5 6; do python3 tests/test_phase_$p.py; done
python3 tests/test_whole_body.py
```

자주 쓰는 테스트:

```bash
python3 tests/test_phase_5.py  # swerve drive/base
python3 tests/test_phase_6.py  # Cyclo marker/XYZ/RPY target
python3 tests/test_whole_body.py  # ROS-free whole-body IK + 물리 이동
```

## 문서 빌드

```bash
mkdocs build --strict
mkdocs serve
```
