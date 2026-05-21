"""추출된 핵심 모듈(Phase 0)에 대한 단위 테스트.

실행: pytest    또는    python -m pytest tests/
"""

import math
import numpy as np

from pickline.shared import Pose6D, Detection, EventBus, PickCompleted, WeightRange
from pickline.infra import atomic_write_json, read_json, max_known_seq, select_device
from pickline.calibration import pose_to_matrix, invert_transform, rpy_to_rotmat
from pickline.picking import PickConfig


# --- shared.value_objects ---

def test_pose6d_roundtrip():
    p = Pose6D(400, 0, 500, 0, 180, 0)
    assert p.as_list() == [400, 0, 500, 0, 180, 0]
    assert Pose6D.from_list([400, 0, 500]) == Pose6D(400, 0, 500, 0, 0, 0)

def test_pose6d_z_helpers():
    p = Pose6D(10, 20, 30, 1, 2, 3)
    assert p.offset_z(50).z == 80
    assert p.with_z(99).z == 99
    assert p.with_z(99).rx == 1   # 회전은 보존

def test_detection_center():
    d = Detection(class_id=0, confidence=0.9, x1=10, y1=20, x2=30, y2=40)
    assert d.center.as_tuple() == (20.0, 30.0)
    assert d.is_obb is False

def test_weight_range():
    r = WeightRange(3.0, 6.5)
    assert r.contains(5.0) and not r.contains(7.0)


# --- shared.events ---

def test_event_bus_publish_subscribe():
    bus = EventBus()
    received = []
    bus.subscribe(PickCompleted, lambda e: received.append(e.success))
    bus.publish(PickCompleted(channel_id="r1", success=True))
    assert received == [True]

def test_event_bus_handler_error_isolated():
    bus = EventBus()
    bus.subscribe(PickCompleted, lambda e: 1 / 0)   # 폭탄 핸들러
    bus.publish(PickCompleted(success=False))         # publish는 죽지 않아야 함


# --- infra.file_ipc ---

def test_atomic_write_read(tmp_path):
    p = tmp_path / "ipc" / "cmd.json"
    atomic_write_json(p, {"seq": 7, "action": "pick"})
    assert read_json(p) == {"seq": 7, "action": "pick"}

def test_read_json_missing_returns_default(tmp_path):
    assert read_json(tmp_path / "nope.json", default={"x": 1}) == {"x": 1}

def test_max_known_seq():
    assert max_known_seq({"seq": 3}, {"last_processed_seq": 9}, {"current_seq": 5}) == 9
    assert max_known_seq(None, "bad", {}) == 0


# --- infra.device ---

def test_select_device_returns_valid():
    assert select_device() in ("cuda", "xpu", "mps", "cpu")


# --- calibration.transforms ---

def test_pose_to_matrix_units():
    T = pose_to_matrix([1000, 2000, 3000, 0, 0, 0])  # mm
    assert np.allclose(T[:3, 3], [1.0, 2.0, 3.0])     # → m
    assert np.allclose(T[:3, :3], np.eye(3))

def test_invert_transform_roundtrip():
    T = pose_to_matrix([400, -100, 500, 30, 180, 45])
    assert np.allclose(T @ invert_transform(T), np.eye(4), atol=1e-9)

def test_rpy_90deg_z():
    R = rpy_to_rotmat(0, 0, 90, order="zyx", unit="deg")
    # Z축 90도 회전: x축 → y축
    assert np.allclose(R @ np.array([1, 0, 0]), [0, 1, 0], atol=1e-9)


# --- picking.PickConfig ---

def test_pick_config_defaults():
    cfg = PickConfig()
    assert cfg.fast_approach_above_mm > cfg.slow_approach_above_mm
    assert cfg.max_retries >= 0
    assert len(cfg.compliance_stiffness) == 6
