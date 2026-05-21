"""비전 — YOLO 객체 검출기.

Intel5의 4중복 YOLO 파싱(step8/step26/step33/test_detect_obb)을 통합.
일반 detect 모델과 OBB(회전 박스) 모델을 모두 처리해 `shared.Detection`으로 통일.

`parse_yolo_result`는 step33_yolo_rgb_worker.py 에서 추출.
"""

from __future__ import annotations

import glob
import os
from typing import Any, List, Optional, Protocol, Sequence, runtime_checkable

from pickline.shared.errors import ModelNotFoundError
from pickline.shared.value_objects import Detection


@runtime_checkable
class ObjectDetector(Protocol):
    """객체 검출 포트. 프레임 → Detection 리스트."""

    def infer(self, frame_bgr, conf: float = 0.25) -> List[Detection]: ...

    @property
    def class_names(self) -> dict[int, str]: ...


def find_latest_model(search_roots: Sequence[str],
                      pattern: str = "**/weights/best.pt") -> Optional[str]:
    """학습 결과 디렉터리들에서 가장 최근 best.pt 경로를 찾는다.

    Intel5의 find_latest_model 3중복을 대체하는 단일 구현.

    Args:
        search_roots: 탐색할 루트 디렉터리들 (예: ["runs/obb", "runs/detect"]).
        pattern: glob 패턴.
    Returns:
        최근 수정된 .pt 절대경로, 없으면 None.
    """
    candidates: list[str] = []
    for root in search_roots:
        candidates += glob.glob(os.path.join(root, pattern), recursive=True)
    if not candidates:
        return None
    candidates = [os.path.abspath(p) for p in candidates]
    return max(candidates, key=os.path.getmtime)


def parse_yolo_result(result: Any) -> List[Detection]:
    """ultralytics YOLO 결과 → Detection 리스트.

    OBB 모델이면 회전 사각형 4꼭짓점(corners_px)을 채우고,
    일반 detect 모델이면 축정렬 박스만 채운다 (corners_px=None).
    """
    import numpy as np

    out: List[Detection] = []
    r0 = result[0]

    # --- OBB 모델 ---
    obb = getattr(r0, "obb", None)
    if obb is not None and len(obb) > 0:
        polys = obb.xyxyxyxy.cpu().numpy()
        for i in range(len(obb)):
            cls = int(obb.cls[i].item())
            conf = float(obb.conf[i].item())
            corners = polys[i].astype(np.float32).reshape(4, 2)
            x1, y1 = corners.min(axis=0)
            x2, y2 = corners.max(axis=0)
            out.append(Detection(
                class_id=cls, confidence=conf,
                x1=float(x1), y1=float(y1), x2=float(x2), y2=float(y2),
                corners_px=tuple((float(px), float(py)) for px, py in corners)))
        return out

    # --- 일반 detect 모델 ---
    boxes = getattr(r0, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return out
    xyxy = boxes.xyxy.cpu().numpy()
    cls = boxes.cls.cpu().numpy().astype(int)
    conf = boxes.conf.cpu().numpy()
    for i in range(len(boxes)):
        out.append(Detection(
            class_id=int(cls[i]), confidence=float(conf[i]),
            x1=float(xyxy[i][0]), y1=float(xyxy[i][1]),
            x2=float(xyxy[i][2]), y2=float(xyxy[i][3]), corners_px=None))
    return out


class YoloDetector:
    """ultralytics YOLO 래퍼. OBB/detect 모델 모두 지원, 디바이스 자동 선택.

    사용 예:
        det = YoloDetector("runs/detect/best.pt", class_names={0: "8pin", 1: "12pin"})
        detections = det.infer(frame_bgr, conf=0.25)
    """

    def __init__(self, model_path: str, device: Optional[str] = None,
                 class_names: Optional[dict[int, str]] = None):
        if not os.path.exists(model_path):
            raise ModelNotFoundError(f"모델 파일 없음: {model_path}")
        self.model_path = model_path
        from ultralytics import YOLO
        from pickline.infra.device import select_device
        self.device = device or select_device()
        self._model = YOLO(model_path)
        try:
            self._model.to(self.device)
        except Exception:
            self.device = "cpu"
        self._class_names = class_names or dict(getattr(self._model, "names", {}) or {})

    @classmethod
    def from_latest(cls, search_roots: Sequence[str], **kwargs) -> "YoloDetector":
        """search_roots에서 가장 최근 학습 모델을 찾아 로드."""
        path = find_latest_model(search_roots)
        if path is None:
            raise ModelNotFoundError(f"학습 모델을 찾지 못함: {list(search_roots)}")
        return cls(path, **kwargs)

    def infer(self, frame_bgr, conf: float = 0.25) -> List[Detection]:
        """BGR 이미지 1장 추론."""
        result = self._model(frame_bgr, conf=conf, device=self.device, verbose=False)
        return parse_yolo_result(result)

    @property
    def class_names(self) -> dict[int, str]:
        return self._class_names
