# pickline-toolkit

**비전 가이드 픽앤플레이스 라인 재사용 툴킷**

산업용 비전(YOLO) + 로봇(Indy7) 픽앤플레이스 프로젝트를 위한 모듈형 Python 라이브러리.
[Intel5 릴레이 소켓 검사 라인 프로젝트](#출처)에서 검증된 코드를 추출해, 향후 비슷한
프로젝트에서 바로 가져다 쓸 수 있도록 DDD(Domain-Driven Design) 구조로 모듈화했다.

> 카메라로 부품을 검출하고 → 좌표를 로봇 좌표계로 변환하고 → 로봇이 집어서
> 옮기고 → PLC와 신호를 주고받는, 그 일련의 빌딩 블록 모음.

---

## 왜 만들었나

원본 프로젝트(Intel5)는 51개 `step*.py` 파일, 약 30,000줄로 커지면서
한 파일(`step24_auto_pick_gui.py`)이 4,493줄 god object가 되는 등 재사용이 어려웠다.
이 툴킷은 그 안의 **재사용 가능한 핵심**만 골라 bounded context로 분리한 것이다.

자세한 분석과 추출 로드맵은 **[ARCHITECTURE.md](ARCHITECTURE.md)** 참고.

## 구조

```
pickline/
├── shared/        공유 커널 — 값 객체, 도메인 이벤트, 예외
├── infra/         인프라 — 파일 IPC, 디바이스 자동 선택
├── vision/        비전 — 카메라 소스, YOLO 검출
├── calibration/   캘리브레이션 — 좌표 변환, hand-eye, 평면 호모그래피
├── robot/         로봇 제어 — Indy7/IndyDCP3 추상화, 그리퍼, DIO
├── picking/       픽킹 — 픽 사이클 오케스트레이션
├── plc/           PLC 연동 — MC Protocol
└── measurement/   측정 — 로드셀, 무게 OCR
```

클린 아키텍처 4계층 의존 방향 (바깥 → 안):
`presentation → application(picking) → domain(vision/calibration/robot...) → infra`
도메인 계층은 외부 의존성이 없다.

## 설치

```bash
pip install -e .                  # 핵심만 (numpy)
pip install -e ".[vision]"        # + YOLO/OpenCV/torch
pip install -e ".[realsense]"     # + RealSense
pip install -e ".[robot]"         # + Indy7 SDK
pip install -e ".[all]"           # 전부
```

Python 3.9+ (개발/검증은 3.11, macOS·Windows·Linux 크로스플랫폼).

## 빠른 시작

```python
# 1. 파일 IPC — 여러 프로세스 간 크래시 안전 JSON 통신
from pickline.infra import atomic_write_json, read_json
atomic_write_json("ipc/cmd.json", {"seq": 1, "action": "pick"})
cmd = read_json("ipc/cmd.json", default={})

# 2. 값 객체 — 자세/검출
from pickline.shared import Pose6D, Detection
home = Pose6D(x=400, y=0, z=500, rx=0, ry=180, rz=0)
approach = home.offset_z(50)          # 50mm 위

# 3. 좌표 변환 — 로봇 pose ↔ 4x4 행렬
from pickline.calibration import pose_to_matrix, invert_transform
T = pose_to_matrix([400, 0, 500, 0, 180, 0])   # mm+deg → 4x4 (m)

# 4. 디바이스 자동 선택 (cuda/xpu/mps/cpu)
from pickline.infra import select_device
device = select_device()             # 환경에 맞게 자동
```

## 현재 상태 (v0.1.0)

이 저장소는 **단계적 추출** 중이다. (테스트 24개 통과)

| 모듈 | 상태 |
|---|---|
| `shared` (값 객체·이벤트·예외) | ✅ Phase 0 |
| `infra.file_ipc` / `infra.device` | ✅ Phase 0 |
| `calibration.transforms` (순수 수학) | ✅ Phase 0 |
| 포트: `RobotController`/`CameraSource`/`ObjectDetector` | ✅ Phase 0 |
| `picking.PickConfig` | ✅ Phase 0 |
| `vision` 카메라 어댑터 (RealSense/Webcam/DryRun) | ✅ Phase 1 |
| `vision.YoloDetector` / `parse_yolo_result` / `StableDecision` | ✅ Phase 1 |
| `robot` 도메인 전체 (IndyController/VacuumGripper/RobotDioHandshake) | ✅ Phase 2 |
| hand-eye/호모그래피 solver, PickCycle, AVFoundation 카메라 | 🔜 Phase 2~3 (ARCHITECTURE.md) |

## 출처

[Neuromeka Indy7](https://www.neuromeka.com/) 듀얼 라인 릴레이 소켓(8핀/12핀) 검사·픽킹
프로젝트에서 추출. 원본 학습 모델 mAP@0.5 = 0.95.

## 라이선스

MIT — [LICENSE](LICENSE) 참고.
