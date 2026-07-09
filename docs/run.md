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
| `C` | 카메라 프리셋 전환 |

## UI 패널

| 패널 | 역할 |
|---|---|
| Status | controller, marker, IK 오차, base pose 표시 |
| Cyclo / Marker Control | `MoveL`, `Bimanual MoveL`, marker jog, capture/release |
| Right Arm / Left Arm | 손별 IK pose 또는 FK joint target |
| Can Grasp | 손별 grasp/thumb synergy |
| Lift / Utilities | lift, reset, contact 표시, camera |
| Joint Monitor | 주요 관절 위치 표시 |

## 테스트

전체 테스트:

```bash
for p in 0 1 2 3 4 5 6; do python3 tests/test_phase_$p.py; done
```

자주 쓰는 테스트:

```bash
python3 tests/test_phase_5.py  # swerve drive/base
python3 tests/test_phase_6.py  # Cyclo marker/XYZ/RPY target
```

## 문서 빌드

```bash
mkdocs build --strict
mkdocs serve
```
