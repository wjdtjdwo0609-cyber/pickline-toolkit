"""Phase 2b 테스트 — calibration solver (charuco, hand_eye, plane_homography, store)."""

import numpy as np
import pytest

from pickline.calibration import (
    BoardSpec, HandEyeCalibrator, HandEyeResult, PlaneHomographyCalibrator,
    PlaneCalibResult, CalibrationStore, get_d435_intrinsics_default,
)
from pickline.shared.errors import CalibrationError, CalibrationNotSolvedError


# --- BoardSpec (cv2 불필요) ---

def test_board_spec_defaults():
    s = BoardSpec()
    assert s.squares_x == 7 and s.aruco_dict == "DICT_5X5_100"

def test_board_spec_file_roundtrip(tmp_path):
    p = tmp_path / "board.txt"
    BoardSpec(squares_x=5, squares_y=8, square_length_mm=40.0,
              marker_length_mm=30.0, aruco_dict="DICT_4X4_50").to_file(p)
    loaded = BoardSpec.from_file(p)
    assert loaded.squares_x == 5 and loaded.squares_y == 8
    assert loaded.square_length_mm == 40.0 and loaded.aruco_dict == "DICT_4X4_50"

def test_board_spec_missing_file_returns_default(tmp_path):
    assert BoardSpec.from_file(tmp_path / "none.txt").squares_x == 7


# --- intrinsics ---

def test_d435_intrinsics():
    K, dist = get_d435_intrinsics_default()
    assert K.shape == (3, 3) and K[0, 0] == 615.0 and len(dist) == 5


# --- HandEyeCalibrator (cv2 불필요한 부분) ---

def test_hand_eye_rejects_tsai():
    K, dist = get_d435_intrinsics_default()
    with pytest.raises(ValueError):
        HandEyeCalibrator(board=None, K=K, dist=dist, method="TSAI")

def test_hand_eye_solve_too_few_pairs():
    K, dist = get_d435_intrinsics_default()
    cal = HandEyeCalibrator(board=None, K=K, dist=dist, method="PARK")
    with pytest.raises(CalibrationError):
        cal.solve()                      # 0 페어 → 부족

def test_hand_eye_result_quality():
    r = HandEyeResult(None, None, None, None, rms_mm=2.0, max_mm=4.0,
                      n_pairs=10, method="PARK", rotation_format="euler-zyx-deg")
    assert r.quality() == "우수"
    r2 = HandEyeResult(None, None, None, None, rms_mm=12.0, max_mm=20.0,
                       n_pairs=10, method="PARK", rotation_format="euler-zyx-deg")
    assert r2.quality() == "정밀도 부족"


# --- PlaneHomographyCalibrator (cv2 필요) ---

def test_plane_homography_recovers_known_map():
    pytest.importorskip("cv2")
    cal = PlaneHomographyCalibrator()
    # 알려진 변환: base_xy = pixel * 0.5 + (100, 200)
    pixels = [(0, 0), (640, 0), (640, 480), (0, 480)]
    for u, v in pixels:
        cal.add_point((u, v), [u * 0.5 + 100, v * 0.5 + 200, 95.0])
    result = cal.solve()
    assert isinstance(result, PlaneCalibResult)
    assert result.rms_mm < 0.1                       # 정확히 복원
    rx, ry = result.pixel_to_base(320, 240)          # 중심점
    assert abs(rx - (320 * 0.5 + 100)) < 0.5
    assert abs(ry - (240 * 0.5 + 200)) < 0.5

def test_plane_homography_too_few_points():
    cal = PlaneHomographyCalibrator()
    cal.add_point((0, 0), [0, 0, 0])
    with pytest.raises(CalibrationError):
        cal.solve()


# --- CalibrationStore ---

def test_store_plane_roundtrip(tmp_path):
    pytest.importorskip("cv2")
    cal = PlaneHomographyCalibrator()
    for u, v in [(0, 0), (100, 0), (100, 100), (0, 100)]:
        cal.add_point((u, v), [u + 10, v + 20, 95.0])
    result = cal.solve()
    npz = tmp_path / "plane.npz"
    CalibrationStore.save_plane(result, npz, channel_id="r1", z_pickup_mm=95.0)
    loaded = CalibrationStore.load_plane(npz)
    assert loaded["n_points"] == 4
    assert np.allclose(loaded["H"], result.H)

def test_store_load_missing_raises(tmp_path):
    with pytest.raises(CalibrationNotSolvedError):
        CalibrationStore.load_plane(tmp_path / "nope.npz")
    with pytest.raises(CalibrationNotSolvedError):
        CalibrationStore.load_hand_eye(tmp_path / "nope.npz")
