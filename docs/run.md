# 직접 실행하기

## 설치 (최초 1회)

```bash
pip install --break-system-packages mujoco numpy trimesh pillow glfw imgui-bundle
```

## 각 Phase 자동 검증 테스트 (headless, 전부 독립 실행 가능)

```bash
for p in 0 1 2 3 4 5 6; do python3 tests/test_phase_$p.py; done
```

## 직접 조작 (하나의 네이티브 창, 3D 뷰 + 슬라이더 패널)

```bash
python3 src/teleop_app.py
```

- **마우스**(3D 뷰 위): 좌클릭 드래그 = 궤도 회전, 우클릭 드래그 = 팬, 휠 = 줌.
- 텔레옵 앱은 can workflow 하나만 사용하며, 별도의 시나리오 인자는 받지 않는다.
- **Up/Down** = 베이스 전진/후진, **Left/Right** = 제자리 yaw 회전, **[/]**
  = 좌우 스트레이프 (전부 실제 바퀴-지면 마찰로 구동), **Q/E** = 리프트 하강/상승.
- **R** = 캔 리셋, **G** = contact force/point 시각화 토글,
  **C** = 카메라 프리셋 전환.
- 양손 EE 포즈(X/Y/Z + Roll/Pitch/Yaw)는 `Cyclo Control` 패널에서 조작한다.
  `MoveL`에서는 `right_goal_marker`/`left_goal_marker`가 각 손 목표이고,
  `Bimanual MoveL`에서 `Capture Grasp`를 누른 뒤에는 `virtual_object_marker`가
  양손을 함께 움직이는 목표가 된다.
- 3D 화면에 표시되는 X/Y/Z 화살표를 드래그하면 위치가 바뀌고, Roll/Pitch/Yaw 회전
  링을 드래그하면 자세가 바뀐다. 패널의 +/- 버튼과 숫자 슬라이더는 같은 target을
  보조적으로 미세 조정하는 용도다.
- RPY는 각 손의 홈 포즈 기준 로컬 회전이라 0,0,0이 자연스러운 기본 자세다.
- Can grasp는 손별 grasp/thumb 시너지로 조작한다. 양팔 동시 이동은
  `Bimanual MoveL` capture 후 `virtual_object_marker`를 통해 제어한다.

## 이 문서 사이트를 로컬에서 미리보기

```bash
pip install --break-system-packages mkdocs mkdocs-material
mkdocs serve   # http://127.0.0.1:8000
```

---

[프로젝트 개요](overview.md) · [MuJoCo 튜토리얼로 돌아가기](guide/index.md)
