"""Phase 1 비전 모듈 단위 테스트 — 카메라 어댑터, 검출 파싱, 안정화."""

import numpy as np

from pickline.shared.value_objects import Detection
from pickline.vision import (
    DryRunSource,
    StableDecision,
    find_latest_model,
    parse_yolo_result,
)


# --- DryRunSource ---

def test_dryrun_source_frames():
    with DryRunSource(width=320, height=240) as cam:
        f1 = cam.read()
        f2 = cam.read()
    assert f1.color_bgr.shape == (240, 320, 3)
    assert f1.depth_mm.shape == (240, 320)
    assert f2.count == 2 and f1.count == 1
    assert cam.kind == "dryrun"

def test_dryrun_source_intrinsics():
    cam = DryRunSource(width=640, height=480)
    intr = cam.intrinsics
    assert intr is not None and intr.cx == 320 and intr.cy == 240


# --- find_latest_model ---

def test_find_latest_model_picks_newest(tmp_path):
    import time
    for name in ("a", "b"):
        d = tmp_path / "runs" / name / "weights"
        d.mkdir(parents=True)
        (d / "best.pt").write_bytes(b"x")
        time.sleep(0.01)
    newest = find_latest_model([str(tmp_path / "runs")])
    assert newest is not None and newest.endswith("best.pt")

def test_find_latest_model_none_when_empty(tmp_path):
    assert find_latest_model([str(tmp_path)]) is None


# --- parse_yolo_result (mock ultralytics 결과) ---

class _Arr:
    def __init__(self, a): self._a = np.asarray(a)
    def cpu(self): return self
    def numpy(self): return self._a
    def item(self): return self._a.item()
    def __getitem__(self, i): return _Arr(self._a[i])
    def __len__(self): return len(self._a)

class _Boxes:
    def __init__(self, xyxy, cls, conf):
        self.xyxy = _Arr(xyxy); self.cls = _Arr(cls); self.conf = _Arr(conf)
        self._n = len(xyxy)
    def __len__(self): return self._n

class _Result:
    def __init__(self, boxes=None, obb=None):
        self.boxes = boxes; self.obb = obb
    def __getitem__(self, i): return self

def test_parse_yolo_detect_boxes():
    r = _Result(boxes=_Boxes(
        xyxy=[[10, 20, 30, 40], [50, 60, 70, 80]],
        cls=[0, 1], conf=[0.9, 0.7]))
    dets = parse_yolo_result(r)
    assert len(dets) == 2
    assert dets[0].class_id == 0 and abs(dets[0].confidence - 0.9) < 1e-6
    assert dets[0].center.as_tuple() == (20.0, 30.0)
    assert dets[0].is_obb is False

def test_parse_yolo_empty():
    assert parse_yolo_result(_Result(boxes=_Boxes([], [], []))) == []


# --- StableDecision ---

def _det(cls=0, conf=0.9, cx=100, cy=100):
    return Detection(class_id=cls, confidence=conf,
                     x1=cx - 10, y1=cy - 10, x2=cx + 10, y2=cy + 10)

def test_stable_decision_good_after_enough_votes():
    sd = StableDecision(window=5, good_frames=3, reject_frames=3)
    for _ in range(2):
        _, code, _, _ = sd.update([_det(conf=0.95)])
        assert code in ("NONE", "GOOD")  # 아직 안정화 전
    _, code, votes, _ = sd.update([_det(conf=0.95)])
    assert code == "GOOD" and votes >= 3

def test_stable_decision_reject_priority():
    sd = StableDecision(window=5, good_frames=3, reject_frames=2,
                        reject_threshold=0.8, reject_priority=True)
    sd.update([_det(conf=0.5)])
    det, code, _, _ = sd.update([_det(conf=0.5)])
    assert code == "REJECT" and det is not None

def test_stable_decision_roi_filters_out():
    sd = StableDecision(window=3, good_frames=1, reject_frames=1,
                        roi_px=(0, 0, 50, 50))
    _, code, _, _ = sd.update([_det(cx=500, cy=500)])  # ROI 밖
    assert code == "NONE"

def test_stable_decision_reset():
    sd = StableDecision(window=3, good_frames=1, reject_frames=1)
    sd.update([_det()])
    sd.reset()
    assert sd.stable_code == "NONE" and len(sd.history) == 0
