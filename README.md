# Motion Sculpt — 세상에서 가장 가벼운 FK 모션패스 스컬프팅

컨트롤러 선택 없는 **Rigless UX**: 보이는 메시를 누르면 정점 웨이트로 FK 뼈가 잡히고,
뷰포트 **3D 모션패스**를 직접 잡아당기면 가상 IK(CCD)가 FK 회전값으로 역산되어 키로 박힌다.
내부 데이터는 항상 **순수 FK + F-Curve**라서 연산이 가볍고 Mixamo / ARP(FK모드) 모두 그대로 동작한다.

## 왜 FK인가 (핵심 통찰 반영)

- FK 체인 행렬 합성은 `부모 @ rest_rel @ offset` 한 줄이라 프레임당 O(뼈수), IK 솔버·컨스트레인트 평가가 없음.
- 그래서 `frame_set()`+depsgraph 없이 F-Curve만 직접 읽어 **미래 궤적 전체를 미리 계산**해도 부담이 없다 → 모션패스가 흔들림 없이 60fps.
- 드래그 순간에만 CCD 한 번 돌려 회전값으로 환원 → 평소엔 FK라서 극강으로 가벼움.

## 설치

1. 이 폴더를 zip (`motion_sculpt/` + `cpp/` + `README.md`) → Blender 설정 > 애드온 > 파일에서 설치.
2. 또는 `motion_sculpt/` 폴더를 `%APPDATA%\Blender Foundation\Blender\5.x\scripts\addons\` 에 복사 후 활성화.
3. View3D 사이드바(N) > **Motion Sculpt** 패널.

요구: Blender 4.0+ (5.2 포함), numpy (내장).

## 사용법 (30초)

1. armature + mesh 선택 → `Path + Onion Overlay` 켜기 → 활성 FK 뼈의 노란 궤적 + 전후 잔상 표시.
2. `Alt+W` = **Weight Pick**: 메시 위 호버 시 헤더에 `mesh -> Bone (가중치)` 미리보기, 클릭=뼈 선택, 드래그=가상 IK 스컬프팅.
3. `Alt+G` = **Grab Path**: 궤적선 가장 가까운 점으로 점프 후 드래그 → 놓으면 FK 키 확정.
4. `Brush: Smooth Path` = 현재 프레임 주변 R프레임을 S강도로 스무딩 (속도감 tension 조절의 출발점).
5. `Wormhole` = 제자리 동작을 시간축(X)으로 쫙 펼쳐 겹침 없이 수정.

## 모듈 대응 (요청 구조 그대로)

| 요청 | 구현 |
|---|---|
| C++ Core (.pyd) | `cpp/src/motion_core.cpp` + `CMakeLists.txt` (pybind11, 멀티스레드). `compose_globals` / `skin_deform` 제공 |
| fk_numpy.py | `motion_sculpt/fk_core.py` (F-Curve 직접 파싱, C++ 있으면 자동 위임) |
| overlay.py | `motion_sculpt/overlay.py` (gpu 셰이더 패스+어니언+웜홀) |
| picker.py & ui.py | `motion_sculpt/picker.py`, `ui.py` (+ `virtual_ik.py`, `path_edit.py`) |

## C++ 빌드 (GitHub용)

```bat
git submodule add https://github.com/pybind/pybind11 cpp/third_party/pybind11
cmake -S cpp -B cpp/build -Dpybind11_DIR="%PYTHON%\Lib\site-packages\pybind11\share\cmake\pybind11"
cmake --build cpp/build --config Release
copy cpp\build\Release\motion_core_cpp*.pyd motion_sculpt\
```

빌드 없이도 **numpy 폴백으로 전부 동작**한다. `HAS_CPP` 표시가 패널에 뜬다.

## 성능 메모 (정직 버전)

- 현 MVP 어니언은 **아마추어 스틱 고스트**(선분)라서 진짜 60fps가 나온다.
  풀 메시 고스트는 `skin_deform` C++가 준비됐고, 다음 단계에서 `overlay.py`가
  `gpu` 버텍스 버퍼로 직행하면 메시 잔상도 60fps가 된다 (셰이더 스키닝 TODO 주석 참고).
- `frame_set()`을 패스 계산에 쓰지 않는다. 키 확정 후 1회만 `frame_set(current)`로 리프레시.

## 로드맵 (같이 발전시키자)

- [ ] overlay에 GPU LBS 셰이더 연결 (vettex+weight 버퍼 → `skin_deform` 결과를 GPU로)
- [ ] 베지어 핸들 tension을 궤적 곡률로 역산 표시 (In/Out 속도감 브러시)
- [ ] 백그라운드 스레드 프리페치 (frame range job queue, C++ 스레드풀)
- [ ] Mixamo/ARP 자동 체인 프리셋 (`fk_chain_for_bone` 고도화)

## 라이선스

MIT — C++ 코어 포함 자유롭게 포크/PR 환영.
