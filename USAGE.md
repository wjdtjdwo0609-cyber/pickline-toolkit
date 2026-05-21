# pickline-toolkit 사용 가이드

비전 가이드 픽앤플레이스 라인 툴킷의 **구조**와 **사용법**.
설계 배경은 [ARCHITECTURE.md](ARCHITECTURE.md), 개요는 [README.md](README.md) 참고.

---

## 1. 한눈에 보는 모듈 구조

```
pickline/
│
├── shared/         ◀── 공유 커널 (모든 계층이 사용)
│   ├── value_objects.py   Pose6D · PixelPoint · Detection · PickTarget · ...
│   ├── events.py          DomainEvent · EventBus · PickCompleted · ...
│   └── errors.py          PicklineError 예외 계층
│
├── infra/          ◀── 인프라 (외부 세계 어댑터)
│   ├── file_ipc.py        atomic_write_json · read_json · max_known_seq
│   └── device.py          select_device (cuda/xpu/mps/cpu 자동)
│
├── vision/         ◀── 도메인: 비전
│   ├── camera.py          CameraSource 포트 · Frame
│   ├── sources.py         RealSenseSource · WebcamSource · DryRunSource
│   ├── detector.py        YoloDetector · parse_yolo_result
│   └── stable.py          StableDecision (시간축 투표 안정화)
│
├── calibration/    ◀── 도메인: 캘리브레이션
│   ├── transforms.py      pose_to_matrix · invert_transform (순수 수학)
│   ├── charuco.py         BoardSpec · CharucoBoardModel
│   ├── hand_eye.py        HandEyeCalibrator (천장 카메라)
│   ├── plane_homography.py PlaneHomographyCalibrator (평면 4점)
│   └── store.py           CalibrationStore (.npz 저장/로드)
│
├── robot/          ◀── 도메인: 로봇 제어
│   ├── controller.py      RobotController 포트
│   ├── indy_controller.py IndyController (Indy7/IndyDCP3)
│   ├── gripper.py         Gripper 포트 · VacuumGripper
│   └── dio_handshake.py   RobotDioHandshake (PLC 인터록)
│
├── plc/            ◀── 도메인: PLC 연동
│   └── client.py          PlcClient (MC Protocol)
│
├── picking/        ◀── 애플리케이션 (도메인을 조합)
│   ├── pick_cycle.py      PickConfig · PickCycle (9단계 픽 시퀀스)
│   └── file_ipc_worker.py FileIpcWorker (워커 추상 베이스)
│
└── measurement/    ◀── (추출 예정 — 로드셀 / 무게 OCR)
```

---

## 2. 계층 구조 (Clean Architecture)

의존성은 **항상 바깥 → 안쪽** 으로만 흐른다. 안쪽은 바깥을 모른다.

```
        ┌─────────────────────────────────────────────┐
        │  Presentation   (앱이 소유 — 툴킷 밖)            │   GUI · CLI · 웹
        └───────────────────────┬─────────────────────┘
                                │ 호출
        ┌───────────────────────▼─────────────────────┐
        │  Application    picking/                     │   PickCycle
        │                 (PickCycle · FileIpcWorker)   │   FileIpcWorker
        └───────────────────────┬─────────────────────┘
                                │ 조합 (포트로 주입)
        ┌───────────────────────▼─────────────────────┐
        │  Domain         vision · robot · calibration  │   YoloDetector
        │                 · plc                         │   IndyController
        │                 (하드웨어 SDK는 어댑터 뒤에 격리)   │   HandEyeCalibrator
        └───────────────────────┬─────────────────────┘
                                │ 사용
        ┌───────────────────────▼─────────────────────┐
        │  Infrastructure infra/  (file_ipc · device)   │
        └───────────────────────────────────────────────┘

        shared/  ── 값 객체·이벤트·예외. 모든 계층이 공유 (최하위 커널).
```

핵심 원칙:
- **도메인 계층은 외부 라이브러리에 직접 의존하지 않는다.** `cv2`, `torch`,
  `pyrealsense2`, `neuromeka`, `pymcprotocol`은 전부 메서드 안에서 lazy import.
  → 하드 의존성은 `numpy` 하나. 나머지는 `pip install pickline[vision]` 식 옵션.
- **포트(Protocol)로 결합한다.** `PickCycle`은 `RobotController`/`Gripper`
  인터페이스만 알지, `IndyController`/`VacuumGripper` 구현을 모른다.
  → 로봇/그리퍼를 바꿔도 `PickCycle`은 그대로.
- **모든 하드웨어 어댑터는 `dry_run=True` 지원.** 장비 없이 전 계층 테스트 가능.

---

## 3. 모듈 의존 관계

```
   picking ──────┬────────────┬─────────────┐
     │           │            │             │
     ▼           ▼            ▼             ▼
   robot      vision     calibration       plc
     │           │            │             │
     └───────────┴─────┬──────┴─────────────┘
                       ▼
                  infra · shared
```

- `picking`(애플리케이션)이 `robot`/`vision`/`calibration`/`plc`(도메인)를 조합한다.
- 도메인끼리는 서로 의존하지 않는다 (독립적인 bounded context).
- 모두 `infra`/`shared`(최하위)만 공유한다.

---

## 4. 설치

```bash
git clone <repo-url> && cd pickline-toolkit

pip install -e .                 # 핵심만 (numpy)
pip install -e ".[vision]"       # + YOLO·OpenCV·torch
pip install -e ".[realsense]"    # + RealSense D435
pip install -e ".[robot]"        # + Indy7 SDK (neuromeka)
pip install -e ".[plc]"          # + pymcprotocol
pip install -e ".[all]"          # 전부
```

필요한 하드웨어 라이브러리만 골라 설치하면 된다. (Python 3.9+, 크로스플랫폼)

---

## 5. 컨텍스트별 사용법

### 5.1 shared — 값 객체 · 이벤트

```python
from pickline.shared import Pose6D, Detection, EventBus, PickCompleted

home = Pose6D(x=400, y=0, z=500, rx=0, ry=180, rz=0)
approach = home.offset_z(50)              # 50mm 위 (회전값 보존)

# 도메인 이벤트로 컨텍스트 간 느슨한 결합
bus = EventBus()
bus.subscribe(PickCompleted, lambda e: print(f"픽 완료: {e.success}"))
```

### 5.2 infra — 파일 IPC

```python
from pickline.infra import atomic_write_json, read_json, select_device

# 프로세스 간 크래시 안전 통신 (절반 쓰인 파일을 절대 안 봄)
atomic_write_json("ipc/r1/command.json", {"seq": 1, "command": "pick"})
cmd = read_json("ipc/r1/command.json", default={})

device = select_device()                  # "cuda" | "xpu" | "mps" | "cpu"
```

### 5.3 vision — 카메라 + YOLO

```python
from pickline.vision import open_camera, YoloDetector, StableDecision

cam = open_camera("auto")                  # RealSense 우선, 없으면 웹캠
det = YoloDetector("runs/detect/best.pt", class_names={0: "8pin", 1: "12pin"})

cam.start()
frame = cam.read()                         # Frame(color_bgr, depth_mm, ...)
detections = det.infer(frame.color_bgr, conf=0.25)   # list[Detection]
cam.close()

# 깜빡임 제거 — 여러 프레임 투표로 안정 판정
stable = StableDecision(window=7, good_frames=5, reject_frames=2,
                        reject_threshold=0.8)
best, code, votes, reason = stable.update(detections)   # code: GOOD/REJECT/NONE
```

### 5.4 calibration — 좌표 정합

```python
from pickline.calibration import PlaneHomographyCalibrator, CalibrationStore

# 평면 호모그래피 — 픽셀 4점 ↔ 로봇 base XY 4점
cal = PlaneHomographyCalibrator()
cal.add_point((u1, v1), [rx1, ry1, z])     # 카메라 클릭 → 로봇 교시
# ... 4점 이상
result = cal.solve()
robot_x, robot_y = result.pixel_to_base(320, 240)   # 픽셀 → 로봇 좌표

CalibrationStore.save_plane(result, "plane_calib_r1.npz", channel_id="r1")
```

### 5.5 robot — 로봇 + 그리퍼

```python
from pickline.robot import IndyController, VacuumGripper

robot = IndyController("192.168.3.2")               # dry_run=True 면 장비 불요
robot.set_safety_limits(z_safe_max_mm=130)
robot.move_l(Pose6D(400, 0, 300, 0, 180, 0), vel_mmps=50)

gripper = VacuumGripper(backend="robot_do", robot=robot, robot_do_index=2)
gripper.engage()
ok = gripper.verify_grip()                          # 흡착 센서 확인
gripper.release()
```

### 5.6 plc — PLC 통신

```python
from pickline.plc import PlcClient

with PlcClient("192.168.3.10", port=1025,
               device_map={"start": "B200", "good": "B251"}) as plc:
    if plc.read_bit("start"):                       # 심볼릭 이름 OK
        plc.write_bit("good", True)
```

### 5.7 picking — 픽 사이클 (애플리케이션)

```python
from pickline.picking import PickCycle, PickConfig
from pickline.shared import PickTarget

cycle = PickCycle(robot, gripper, PickConfig(z_safe_max_mm=130))
target = PickTarget(class_id=0, confidence=0.9,
                    pick_xyz_mm=(350.0, -20.0, 80.0), pick_yaw_deg=15.0)
result = cycle.run(target)                          # 9단계 자동 실행 + 재시도
print(result.success, result.attempts)
cycle.place(Pose6D(500, 200, 150, 0, 180, 0))       # 배치
```

---

## 6. 엔드투엔드 예제 — 검출에서 픽까지

```python
from pickline.vision import open_camera, YoloDetector
from pickline.calibration import CalibrationStore
from pickline.robot import IndyController, VacuumGripper
from pickline.picking import PickCycle, PickConfig
from pickline.shared import PickTarget

# 1) 구성요소 준비
cam    = open_camera("realsense", serial="036222071696")
det    = YoloDetector("runs/detect/best.pt")
plane  = CalibrationStore.load_plane("plane_calib_r1.npz")
robot  = IndyController("192.168.3.2")
grip   = VacuumGripper(backend="robot_do", robot=robot)
cycle  = PickCycle(robot, grip, PickConfig())

# 2) 검출 → 좌표 변환 → 픽
cam.start()
frame = cam.read()
detections = det.infer(frame.color_bgr, conf=0.5)
for d in detections:
    # 픽셀 중심 → 로봇 base 좌표 (호모그래피)
    import numpy as np
    H = plane["H"]
    p = H @ np.array([d.center.u, d.center.v, 1.0]); p /= p[2]
    target = PickTarget(class_id=d.class_id, confidence=d.confidence,
                        pick_xyz_mm=(float(p[0]), float(p[1]), 80.0))
    result = cycle.run(target)
    if result.success:
        cycle.place(Pose6D(500, 200, 150, 0, 180, 0))
cam.close()
```

> **장비 없이 시험**: `IndyController(..., dry_run=True)`,
> `VacuumGripper(..., dry_run=True)`, `open_camera("dryrun")` 으로
> 위 흐름 전체를 하드웨어 없이 그대로 돌려볼 수 있다.

헤드리스 워커로 돌리려면 `FileIpcWorker`를 상속해
`setup()` + `execute_cycle(command)` 둘만 구현하면 된다 (JSON 파일 IPC).

---

## 7. 빠른 참조

| 하고 싶은 것 | import | 핵심 호출 |
|---|---|---|
| 카메라 프레임 받기 | `pickline.vision.open_camera` | `cam.read()` |
| YOLO 검출 | `pickline.vision.YoloDetector` | `det.infer(img, conf)` |
| 검출 안정화 | `pickline.vision.StableDecision` | `sd.update(dets)` |
| 픽셀→로봇 좌표 | `pickline.calibration.PlaneHomographyCalibrator` | `result.pixel_to_base(u,v)` |
| hand-eye 캘리브 | `pickline.calibration.HandEyeCalibrator` | `cal.solve()` |
| 로봇 이동 | `pickline.robot.IndyController` | `robot.move_l(pose)` |
| 흡착 | `pickline.robot.VacuumGripper` | `grip.engage()` / `verify_grip()` |
| PLC 인터록 | `pickline.robot.RobotDioHandshake` | `poll_start_event()` |
| PLC 비트 I/O | `pickline.plc.PlcClient` | `read_bit()` / `write_bit()` |
| 픽앤플레이스 | `pickline.picking.PickCycle` | `cycle.run(target)` |
| 헤드리스 워커 | `pickline.picking.FileIpcWorker` | 상속 후 `execute_cycle()` |
| 프로세스 간 통신 | `pickline.infra.atomic_write_json` | `read_json()` |

---

## 8. 검증 상태

- 테스트 77개 통과 (`pytest`)
- 7개 bounded context 중 6개 추출 완료, `measurement`만 placeholder
- 모든 하드웨어 어댑터 `dry_run` 지원 — 장비 없이 CI 가능

```bash
pip install -e ".[dev]"
pytest                       # 전체 테스트
python examples/basic_usage.py
```
