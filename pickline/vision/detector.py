"""비전 — YOLO 검출기 포트.

Intel5에는 YOLO 결과 파싱이 4곳에 중복돼 있다. 하나로 통합.
일반 detect 모델과 OBB(회전 박스) 모델을 모두 처리한다.

구현체 `YoloDetector`는 ultralytics를 감싼다. 추출 ROADMAP은 ARCHITECTURE.md.
"""

from __future__ import annotations

from typing import List, Optional, Protocol, Sequence, runtime_checkable

from pickline.shared.value_objects import Detection


@runtime_checkable
class ObjectDetector(Protocol):
    """객체 검출 포트. 프레임 → Detection 리스트."""

    def infer(self, frame_bgr, conf: float = 0.25) -> List[Detection]:
        """BGR 이미지 1장 추론. OBB/detect 모델 모두 Detection으로 통일 반환."""
        ...

    @property
    def class_names(self) -> dict[int, str]:
        """class_id → 사람이 읽는 이름."""
        ...


def find_latest_model(search_roots: Sequence[str],
                      pattern: str = "**/weights/best.pt") -> Optional[str]:
    """학습 결과 디렉터리들에서 가장 최근 best.pt 경로를 찾는다.

    Intel5의 find_latest_model 3중복을 대체하는 단일 구현.

    Args:
        search_roots: 탐색할 루트 디렉터리들 (예: ["runs/obb", "runs/detect"]).
        pattern: glob 패턴.
    Returns:
        최근 수정된 .pt 경로, 없으면 None.
    """
    import glob
    import os

    candidates: list[str] = []
    for root in search_roots:
        candidates += glob.glob(os.path.join(root, pattern), recursive=True)
    if not candidates:
        return None
    return max(candidates, key=os.path.getmtime)
