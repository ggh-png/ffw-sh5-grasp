[← 전체 안내](../ros2-guide.md)

# Part 12 — 직접 실행해보기 {: #part-12 }

여기서는 감을 잡을 정도로 최소한만 보여준다. 키 조작표/UI 패널 투어/테스트
명령 전체는 [직접 실행하기](../../run.md)에 최신 상태로 정리돼 있으니(이 문서와
두 곳에서 따로 관리하면 한쪽이 바뀔 때 다른 쪽이 조용히 낡은 채로 남을 수
있어서, 운영 관련 세부사항은 그쪽 한 곳만 갱신하면 되게 해뒀다), 실제로
손으로 조작해볼 때는 그 문서를 펼쳐두는 걸 권장한다.

```bash
pip install --break-system-packages mujoco numpy trimesh pillow glfw imgui-bundle
python3 src/teleop_app.py
```

하나의 네이티브 창에 3D 뷰 + ImGui 패널이 함께 뜬다(`mujoco.viewer.launch_passive`를
안 쓰고 GLFW+MuJoCo 저수준 렌더 API+ImGui를 직접 합성하는 이유는
`teleop_app.py` 모듈 docstring의 "Rendering note" 참고 — 요약하면 커스텀
위젯을 그 창 안에 같이 그리기 위해서다). 마우스로 카메라를 돌리고, 방향키로
베이스를 몰고, 왼쪽 패널의 "Cyclo / Marker Control"에서 조작할 손(Right/Left
goal)을 고른 뒤 화면 안 3D gizmo를 드래그하면 그 손의 목표 pose가 움직인다.

---

[← Part 11](./11-testing.md) · [전체 안내](../ros2-guide.md) · [Part 13 →](./13-bug-cases.md)
