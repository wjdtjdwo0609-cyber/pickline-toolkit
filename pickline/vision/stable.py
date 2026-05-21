"""비전 — 시간축 검출 안정화 (StableDecision).

단일 프레임 검출은 흔들린다(깜빡임, 오검출). 슬라이딩 윈도우에 프레임별
판정을 쌓고 투표(vote)해서 안정된 GOOD/REJECT/NONE 판정을 낸다.

검사 라인에서 "양품/불량" 신호를 PLC로 보내기 전 노이즈 제거에 사용.
step33_yolo_rgb_worker.StableDecision 에서 추출 — 순수 로직, 하드웨어 의존 없음.
"""

from __future__ import annotations

from collections import deque
from typing import List, Optional, Tuple

from pickline.shared.value_objects import Detection


class StableDecision:
    """슬라이딩 윈도우 투표 기반 검출 안정화기.

    Args:
        window: 윈도우 프레임 수.
        good_frames: GOOD 확정에 필요한 윈도우 내 GOOD 표 수.
        reject_frames: REJECT 확정에 필요한 표 수.
        reject_threshold: confidence가 이 값 미만이면 불량 후보 (0이면 비활성).
        reject_priority: True면 불량 후보를 양품보다 우선 판정.
        roi_px: (x0,y0,x1,y1) ROI. 중심이 ROI 밖인 검출은 무시. None이면 전체.
        valid_class_ids: 유효 클래스 id 집합. None이면 모든 클래스 허용.
    """

    def __init__(self, *, window: int, good_frames: int, reject_frames: int,
                 reject_threshold: float = 0.0, reject_priority: bool = False,
                 roi_px: Optional[Tuple[int, int, int, int]] = None,
                 valid_class_ids: Optional[set[int]] = None):
        self.history: deque = deque(maxlen=max(1, int(window)))
        self.good_frames = max(1, int(good_frames))
        self.reject_frames = max(1, int(reject_frames))
        self.reject_threshold = float(reject_threshold)
        self.reject_priority = bool(reject_priority)
        self.roi_px = roi_px
        self.valid_class_ids = valid_class_ids
        self.stable_det: Optional[Detection] = None
        self.stable_code = "NONE"
        self.stable_votes = 0

    def _in_roi(self, det: Detection) -> bool:
        if not self.roi_px:
            return True
        x0, y0, x1, y1 = self.roi_px
        c = det.center
        return x0 <= c.u <= x1 and y0 <= c.v <= y1

    def _quality_dets(self, dets: List[Detection]) -> List[Detection]:
        return [d for d in dets
                if (self.valid_class_ids is None or d.class_id in self.valid_class_ids)
                and self._in_roi(d)]

    def _raw_choice(self, dets: List[Detection]) -> Tuple[Optional[Detection], str, str]:
        quality = self._quality_dets(dets)
        if not quality:
            return None, "NONE", "ROI 내 검출 없음"
        rejects = [d for d in quality
                   if self.reject_threshold > 0 and d.confidence < self.reject_threshold]
        if self.reject_priority and rejects:
            target = min(rejects, key=lambda d: d.confidence)
            return target, "REJECT", f"reject conf<{self.reject_threshold:.2f}"
        target = max(quality, key=lambda d: d.confidence)
        if self.reject_threshold > 0 and target.confidence < self.reject_threshold:
            return target, "REJECT", f"best conf<{self.reject_threshold:.2f}"
        return target, "GOOD", "ROI 내 최고 confidence"

    def update(self, dets: List[Detection]) -> Tuple[Optional[Detection], str, int, str]:
        """프레임 검출 결과를 추가하고 안정 판정을 반환.

        Returns:
            (stable_det, stable_code, votes, reason)
            stable_code: 'GOOD' | 'REJECT' | 'NONE'
        """
        raw_det, raw_code, raw_reason = self._raw_choice(dets)
        self.history.append((raw_code, raw_det, raw_reason))

        counts = {"REJECT": 0, "GOOD": 0, "NONE": 0}
        for code, _det, _reason in self.history:
            counts[code] = counts.get(code, 0) + 1
        reject_count = counts["REJECT"]
        good_count = counts["GOOD"]
        none_count = counts["NONE"]

        if reject_count >= self.reject_frames:
            stable_code, stable_votes = "REJECT", reject_count
        elif good_count >= self.good_frames:
            stable_code, stable_votes = "GOOD", good_count
        elif none_count >= self.good_frames:
            stable_code, stable_votes = "NONE", none_count
        else:
            self.stable_votes = max(reject_count, good_count, none_count)
            return self.stable_det, self.stable_code, self.stable_votes, "안정화 대기"

        candidates = [d for code, d, _r in self.history
                      if code == stable_code and d is not None]
        self.stable_code = stable_code
        self.stable_votes = stable_votes
        if stable_code == "REJECT":
            self.stable_det = (min(candidates, key=lambda d: d.confidence)
                               if candidates else raw_det)
        elif stable_code == "GOOD":
            self.stable_det = (max(candidates, key=lambda d: d.confidence)
                               if candidates else raw_det)
        else:
            self.stable_det = None
        reasons = [r for code, _d, r in self.history if code == stable_code and r]
        return self.stable_det, self.stable_code, self.stable_votes, (
            reasons[-1] if reasons else raw_reason)

    def reset(self) -> None:
        self.history.clear()
        self.stable_det = None
        self.stable_code = "NONE"
        self.stable_votes = 0
