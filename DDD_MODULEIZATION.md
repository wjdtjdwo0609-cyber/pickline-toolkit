# Pickline Toolkit DDD Moduleization

이 패키지는 Intel5의 step script들에서 반복되던 비전, 로봇, 캘리브레이션, PLC, 픽 사이클 로직을 재사용 가능한 모듈로 추출한 툴킷이다. DDD 경계는 원본 파일 번호가 아니라 픽앤플레이스 라인의 업무 능력 기준이다.

## Bounded Contexts

### shared

모든 컨텍스트가 공유하는 값 객체, 이벤트, 예외를 제공한다.

- `pickline/shared/value_objects.py`
- `pickline/shared/events.py`
- `pickline/shared/errors.py`

### infra

공통 파일 IPC와 장치 선택을 담당한다.

- `pickline/infra/file_ipc.py`
- `pickline/infra/device.py`

### vision

카메라, YOLO 검출, 안정화 판정을 소유한다.

- `pickline/vision/camera.py`
- `pickline/vision/detector.py`
- `pickline/vision/sources.py`
- `pickline/vision/stable.py`

### calibration

좌표 변환, ChArUco, hand-eye, plane homography, calibration store를 소유한다.

- `pickline/calibration/transforms.py`
- `pickline/calibration/charuco.py`
- `pickline/calibration/hand_eye.py`
- `pickline/calibration/plane_homography.py`
- `pickline/calibration/store.py`

### robot

Indy7 제어, gripper, DIO handshake, file IPC worker를 소유한다.

- `pickline/robot/controller.py`
- `pickline/robot/indy_controller.py`
- `pickline/robot/gripper.py`
- `pickline/robot/dio_handshake.py`

### picking

픽 사이클과 파일 IPC worker orchestration을 소유한다.

- `pickline/picking/pick_cycle.py`
- `pickline/picking/file_ipc_worker.py`

### plc

PLC MC Protocol client를 소유한다.

- `pickline/plc/client.py`

### measurement

로드셀/무게 OCR 계열 확장 문맥의 자리다.

## Adapter Boundary

이 툴킷은 presentation을 소유하지 않는다. GUI, CLI, 웹 대시보드는 사용하는 앱 쪽에 남기고, pickline은 domain/application/infrastructure 재사용 모듈만 제공한다.

## Dependency Rule

도메인 값 객체와 정책은 외부 SDK에 의존하지 않는다. optional dependency가 필요한 실제 RealSense, YOLO, Indy7, PLC 호출은 adapter 모듈로 격리한다.

## Verification

- `python -m pytest -q`
- `python -m py_compile pickline/**/*.py`
