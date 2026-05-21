# pickline-toolkit — 아키텍처 & 추출 로드맵

이 문서는 원본 **Intel5** 프로젝트(51개 `step*.py`, ~30,000줄)를 도메인별로 정밀
분석하고, 재사용 가능한 모듈을 어떻게 추출하는지를 정의한다.

설계 방법론은 DDD(Domain-Driven Design): god object를 bounded context로 분해하고,
클린 아키텍처 4계층으로 의존성을 정리하며, 도메인 이벤트로 컨텍스트 간 결합을 낮춘다.

---

## 1. 클린 아키텍처 계층

```
┌─────────────────────────────────────────────┐
│  Presentation   GUI(Tkinter), CLI, 웹 대시보드   │  ← pickline 외부 (앱이 소유)
├─────────────────────────────────────────────┤
│  Application    picking (PickCycle, 오케스트레이션)│
├─────────────────────────────────────────────┤
│  Domain         vision · calibration · robot ·  │
│                 plc · measurement              │
├─────────────────────────────────────────────┤
│  Infrastructure file_ipc · device · SDK 어댑터   │
└─────────────────────────────────────────────┘
        의존 방향: 바깥 → 안. 도메인은 외부 의존 없음.
```

`shared`(값 객체·이벤트·예외)는 모든 계층이 공유하는 커널이다.

---

## 2. Bounded Context 분해

원본 51개 파일은 8개 컨텍스트로 나뉜다.

| 컨텍스트 | 책임 | 원본 Intel5 파일 |
|---|---|---|
| **shared** | 값 객체, 도메인 이벤트, 예외 | (신규 작성) |
| **infra** | 파일 IPC, 디바이스 선택 | `step27_file_ipc`, `common/device` |
| **vision** | 카메라, YOLO 검출, 안정화 | `step8_vision`, `step33_yolo_rgb_worker`, `step26`, `step33_camera_roles`, `step28_camera_guard`, `realsense_rgb_frame_helper` |
| **calibration** | 좌표 변환, hand-eye, 호모그래피 | `step6`, `step7_solve_hand_eye`, `step21`, `step22` |
| **robot** | Indy7 제어, 그리퍼, DIO | `step9_robot`, `step10_vacuum`, `robot_dio_handshake`, `step27/37_*_worker` |
| **picking** | 픽 사이클, 라인 오케스트레이션 | `step11_pick_pipeline`, `step17_orchestrator`, `step24` |
| **plc** | MC Protocol 브리지 | `step29_plc_bridge` |
| **measurement** | 로드셀, 무게 OCR | `loadcell/` 패키지, `step16`, `step32`, `step33_weight*` |

GUI 파일(`step18/20/23/24/30/31/32/33_*_gui`)은 presentation 계층 — 툴킷이 아닌
앱에 남는다. 단 그 안의 도메인 로직은 위 컨텍스트로 추출한다.

---

## 3. 컨텍스트별 상세

### 3.1 shared ✅ 추출 완료

`pickline/shared/` — 모든 컨텍스트가 공유하는 불변 타입.

- `value_objects.py` — `Pose6D`, `PixelPoint`, `CameraIntrinsics`, `Detection`,
  `PickTarget`, `WeightRange`, `QualityDecision`
- `events.py` — `DomainEvent` + `EventBus` + 구체 이벤트(`PickCompleted` 등)
- `errors.py` — 도메인 예외 계층 (`PicklineError` 베이스)

### 3.2 infra ✅ 추출 완료

`pickline/infra/`

- `file_ipc.py` — `atomic_write_json` / `read_json` / `max_known_seq`.
  `step27_file_ipc.py`에서 **무수정 추출**. 임시파일 + `os.replace`로 원자적 쓰기.
- `device.py` — `select_device()` (cuda>xpu>mps>cpu 자동), `device_info()`.

### 3.3 vision 🔜 포트 확정, 어댑터 추출 예정

**핵심 문제: 중복.** 카메라 초기화가 5곳, YOLO 파싱이 4곳, `find_latest_model`이
3곳에 복붙돼 있다.

| 추출 대상 | 출처 | 상태 |
|---|---|---|
| `CameraSource` 포트 + `Frame` | (통합 설계) | ✅ `camera.py` |
| `ObjectDetector` 포트, `find_latest_model` | 3중복 통합 | ✅ `detector.py` |
| `RealSenseSource` 어댑터 | `step8.CameraStream`, `realsense_rgb_frame_helper` | 🔜 |
| `AVFoundationSource` 어댑터 | `step26.FfmpegCamera`, `step24._init_ffmpeg_*` | 🔜 |
| `WebcamSource` 어댑터 | `common/device.Camera` | 🔜 |
| `YoloDetector` 구현 (OBB+detect) | `step33_yolo_rgb_worker.parse_yolo_result` | 🔜 |
| `StableDecision` (시간축 투표 안정화) | `step33_yolo_rgb_worker` | 🔜 (거의 무수정 가능) |
| `draw_detections` 오버레이 | `step26._draw_dets` | 🔜 |

**주의:** `step33_camera_roles.py`에 절대경로(`/Users/tjdwo0609/...`)가 박혀 있다 —
추출 시 `project_root`를 인자/환경변수로 주입.

### 3.4 calibration 🔜 순수 수학 추출 완료, solver 예정

| 추출 대상 | 출처 | 상태 |
|---|---|---|
| `transforms.py` (rpy/rotvec/pose_to_matrix/invert) | `step7` 순수 함수 | ✅ |
| `charuco.py` — `BoardSpec`, `CharucoBoardModel` | `load_charuco_board` **4중복** 통합 | 🔜 |
| `hand_eye.py` — `HandEyeCalibrator`, `HandEyeResult` | `step7.main` + `step21._solve_thread` **2중복** | 🔜 |
| `plane_homography.py` — `PlaneHomographyCalibrator` | `step22.solve` | 🔜 |
| `store.py` — `CalibrationStore` (.npz 리포지토리) | `step7/21/22`의 npz 저장 로직 | 🔜 |

**핵심 수학(보존 필수):** hand-eye는 eye-to-base swap 트릭(EE→base를 역행렬로
넣어 `cv2.calibrateHandEye` 호출)을 쓴다. TSAI 방식은 swap에서 부호가 꼬여 제외.
검증 RMS는 board 원점을 두 경로로 재계산해 비교(목표 1~3mm, 5mm 초과 시 재캡처).

### 3.5 robot 🔜 포트 확정, 구현 추출 예정

원본의 SDK 격리 계층(`IndyController`/`VacuumController`/`RobotDioHandshake`)은
이미 재사용 등급 — 거의 그대로 옮기면 된다.

| 추출 대상 | 출처 | 상태 |
|---|---|---|
| `RobotController` 포트 | (통합 설계) | ✅ `controller.py` |
| `IndyController` 구현 | `step9_robot.py` (806줄) | 🔜 |
| `Gripper`/`Vacuum` (plc/robot_do/endtool 백엔드) | `step10_vacuum.py` | 🔜 |
| `DioHandshake` | `robot_dio_handshake.py` | 🔜 |
| `FileIpcWorker` 베이스 클래스 | `step27`+`step37` **60~70% 중복** | 🔜 ★최대 절감 |

**최대 중복:** `step27_robot_file_worker`와 `step37_robot1_file_worker`는 ~2,000줄이
복붙. config 로드 / IPC / 상태 루프 / 명령 디스패치는 동일하고, 다른 건 픽 사이클
본문(고정좌표 reject vs 비전 타겟 full)뿐. → `FileIpcWorker` 베이스 + `execute_cycle()`
추상 메서드 하나로 통합.

### 3.6 picking 🔜 PickConfig 완료, 유스케이스 예정

`step24_auto_pick_gui.py` (4,493줄) god object의 17개 책임:

| 책임 | 대략 줄 범위 | → 목적지 |
|---|---|---|
| 설정 수집 (~150 키) | 191–430 | `ChannelConfig` 값 객체 |
| UI 구성 (Tk) | 430–1100, 1945–2360 | presentation에 잔류 |
| 캘리브레이션 | 366–410, 1366–1388 | `calibration` |
| 고정좌표 티칭 | 1101–1365 | `FixedPoseStore` 리포지토리 |
| 카메라 관리 | 1388–2185 | `vision.CameraSource` |
| 로봇/진공 초기화 | 2186–2220 | `robot` |
| 파일 IPC | 2221–2280, 2605–2941 | `infra.file_ipc` + IPC 값 객체 |
| 품질 판정 | 2280–2604 | `QualityJudge` 도메인 서비스 |
| 검출/추론 루프 | 2976–3275 | `vision.YoloDetector` |
| 픽 모션 / 풀 사이클 (3변형) | 3392–4396 | `picking.PickCycle` |
| PLC DIO 사이클 | 3686–3913 | `plc` + `robot.DioHandshake` |

→ `AutoPickGUI`는 ~600줄 얇은 뷰로 축소: 프레임 렌더 + 버튼→유스케이스 위임 +
도메인 이벤트 구독.

| 추출 대상 | 출처 | 상태 |
|---|---|---|
| `PickConfig`, `PickResult` | `step11.PickConfig` + 워커 상수 | ✅ |
| `PickCycle` 유스케이스 (mode: full/pickup/fixed/reject) | `step11` + `step24` 3중복 | 🔜 ★최대 절감 |
| `LineOrchestrator` | `step17_orchestrator.py` | 🔜 |
| `QualityJudge` 도메인 서비스 | `step24` 2280–2604 | 🔜 |

### 3.7 plc 🔜 추출 예정

| 추출 대상 | 출처 | 상태 |
|---|---|---|
| `PlcClient` (read/write bit·word, device offset) | `step29_plc_bridge` 하위 계층 | 🔜 |
| `LadderPlcBridge` (ladder 정책) | `step29` 상위 계층 | 🔜 |

MC Protocol(`pymcprotocol`). ladder 디바이스 맵(`B200/B300/Y160/X145` 등)은
프로젝트별 설정으로 분리.

### 3.8 measurement — `loadcell/`는 이미 완성형

원본 `loadcell/` 패키지는 **이미 교과서적 4계층 DDD**
(`interface→infrastructure→application→domain`). 이 툴킷의 아키텍처 목표 그 자체다.
→ 그대로 `pickline/measurement/loadcell/`로 편입, 레거시 `step15/16`은 폐기.

`weight_ocr`(저울 LCD 7세그먼트 OCR)은 3,691줄 GUI(`step32`) 안에 핵심 로직이
갇혀 있다 — `SevenSegmentWeightReader`, `WeightOcrModel`, 카메라 클래스들을
`pickline/measurement/weight_ocr/`로 추출. (tesseract/easyocr 아님 — 직접 만든
OpenCV 세그먼트 매처.)

---

## 4. 중복 요약 (추출 시 통합 대상)

| 중복 로직 | 복사본 수 | 통합 후 |
|---|---|---|
| 카메라 초기화 (RealSense/AVF) | 5+ | `vision.CameraSource` 1개 |
| YOLO 결과 파싱 | 4 | `vision.YoloDetector.infer` |
| `find_latest_model` | 3 | `vision.find_latest_model` ✅ |
| ChArUco 보드 생성 | 4 | `calibration.CharucoBoardModel` |
| `load_board_spec` | 3 | `calibration.BoardSpec.from_file` |
| hand-eye solve + RMS | 2 | `calibration.HandEyeCalibrator` |
| 픽 사이클 오케스트레이션 | 4 | `picking.PickCycle` |
| 파일 워커 스캐폴딩 | 2 (~2000줄) | `robot.FileIpcWorker` 베이스 |
| `atomic_write_json` | 3 | `infra.file_ipc` ✅ |

---

## 5. 추출 로드맵 (단계별)

원본 시스템 동작을 깨지 않도록 **저위험 → 고위험** 순서로.

- **Phase 0 ✅** — repo 골격, 공유 커널, 인프라, 순수 수학, 포트 정의.
- **Phase 1 ✅** — 비전 어댑터: `RealSenseSource`/`WebcamSource`/`DryRunSource`/
  `open_camera`, `YoloDetector`/`parse_yolo_result`, `StableDecision`. *(현재)*
  남음: `AVFoundationSource`(macOS ffmpeg 우회), `PlcClient`.
- **Phase 2 (진행 중)** — 도메인 서비스. ✅ `IndyController` 완료.
  남음: `Gripper`(Vacuum), `DioHandshake`, `HandEyeCalibrator`,
  `PlaneHomographyCalibrator`, `CharucoBoardModel`, `QualityJudge`.
- **Phase 3** — 애플리케이션: `PickCycle`, `FileIpcWorker` 베이스, `LineOrchestrator`.
  `step24`/`step27`/`step37`의 중복 4,000+줄 통합.
- **Phase 4** — GUI를 presentation-only로 다이어트 (도메인 호출만).
- **Phase 5** — 도메인 이벤트 연결, 필요 시 microkernel/plugin.

각 Phase는 별도 PR. 추출한 모듈은 `tests/`에 단위 테스트 동반.

---

## 6. 추출 시 공통 정리 항목

- **하드코딩 제거** — 로봇 IP(`192.168.3.2/3.3`), PLC IP/레지스터, Z 높이,
  속도/타이밍 상수, 절대경로 → `line_config.yaml` / 생성자 인자로.
- **IP 규약 불일치** — `step17`은 `192.168.0.x`, `step24/29`는 `192.168.3.x`. 통일.
- **`print()` 로깅** → 주입 가능한 logger.
- **절대경로** — `step33_camera_roles`, `step31`에 박힌 `/Users/tjdwo0609/...` 제거.
