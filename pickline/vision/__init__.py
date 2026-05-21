"""비전 — 카메라 + YOLO 객체 검출.

추출 완료:
- 포트: CameraSource, ObjectDetector
- 카메라 어댑터: RealSenseSource, WebcamSource, DryRunSource, open_camera()
- 검출: YoloDetector, parse_yolo_result, find_latest_model
- 안정화: StableDecision (시간축 투표)

추출 예정: AVFoundationSource (macOS ffmpeg 우회), draw_detections.
"""

from pickline.vision.camera import CameraSource, Frame
from pickline.vision.sources import (
    RealSenseSource,
    WebcamSource,
    DryRunSource,
    open_camera,
)
from pickline.vision.detector import (
    ObjectDetector,
    YoloDetector,
    parse_yolo_result,
    find_latest_model,
)
from pickline.vision.stable import StableDecision

__all__ = [
    "CameraSource", "Frame",
    "RealSenseSource", "WebcamSource", "DryRunSource", "open_camera",
    "ObjectDetector", "YoloDetector", "parse_yolo_result", "find_latest_model",
    "StableDecision",
]
