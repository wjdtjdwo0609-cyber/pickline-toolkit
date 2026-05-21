"""캘리브레이션 — ChArUco 보드 모델.

Intel5의 ChArUco 보드 생성 4중복(step6/7/21/22)과 load_board_spec 3중복을
하나로 통합. OpenCV 4.7+ 신 API와 4.6 이하 레거시 API를 모두 지원.

`BoardSpec`     : 보드 사양 값 객체 (파일 입출력)
`CharucoBoardModel` : 보드 생성 + 검출 + PnP 자세 추정
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class BoardSpec:
    """ChArUco 보드 사양. charuco_board_info.txt 와 호환."""
    squares_x: int = 7
    squares_y: int = 10
    square_length_mm: float = 50.0
    marker_length_mm: float = 35.0
    aruco_dict: str = "DICT_5X5_100"

    @classmethod
    def from_file(cls, path: str) -> "BoardSpec":
        """`key=value` 줄 형식 파일에서 로드. 없는 키는 기본값."""
        import os
        spec = cls()
        if not os.path.exists(path):
            return spec
        int_keys = {"squares_x", "squares_y"}
        float_keys = {"square_length_mm", "marker_length_mm"}
        for line in open(path, encoding="utf-8"):
            if "=" not in line:
                continue
            k, v = (s.strip() for s in line.split("=", 1))
            if k in int_keys:
                setattr(spec, k, int(float(v)))
            elif k in float_keys:
                setattr(spec, k, float(v))
            elif k == "aruco_dict":
                spec.aruco_dict = v
        return spec

    def to_file(self, path: str) -> None:
        lines = [f"{k}={getattr(self, k)}" for k in
                 ("squares_x", "squares_y", "square_length_mm",
                  "marker_length_mm", "aruco_dict")]
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")


@dataclass
class CharucoDetection:
    """한 이미지의 ChArUco 검출 결과."""
    n_markers: int
    n_corners: int
    charuco_corners: Optional[object] = None    # np.ndarray
    charuco_ids: Optional[object] = None        # np.ndarray


class CharucoBoardModel:
    """ChArUco 보드 — 생성 + 검출 + 자세 추정.

    square/marker 길이는 내부적으로 m 단위로 변환해 OpenCV에 넘긴다.
    """

    def __init__(self, spec: BoardSpec):
        import cv2
        self.spec = spec
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, spec.aruco_dict))
        sx, sy = spec.squares_x, spec.squares_y
        sl = spec.square_length_mm / 1000.0
        ml = spec.marker_length_mm / 1000.0
        try:
            self._board = cv2.aruco.CharucoBoard((sx, sy), sl, ml, self._aruco_dict)
        except AttributeError:        # OpenCV <= 4.6
            self._board = cv2.aruco.CharucoBoard_create(sx, sy, sl, ml, self._aruco_dict)
        self._detector = (cv2.aruco.CharucoDetector(self._board)
                          if hasattr(cv2.aruco, "CharucoDetector") else None)

    @property
    def board(self):
        return self._board

    def detect(self, image) -> CharucoDetection:
        """이미지에서 ChArUco 코너/마커 검출 (자세 추정 없음)."""
        import cv2
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self._detector is not None:
            ch_corners, ch_ids, m_corners, m_ids = self._detector.detectBoard(gray)
            n_markers = 0 if m_ids is None else len(m_ids)
            n_corners = 0 if ch_corners is None else len(ch_corners)
            return CharucoDetection(n_markers, n_corners, ch_corners, ch_ids)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict)
        if ids is None:
            return CharucoDetection(0, 0)
        ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self._board)
        n_corners = 0 if not ret or ch_corners is None else len(ch_corners)
        return CharucoDetection(len(ids), n_corners, ch_corners, ch_ids)

    def estimate_pose(self, image, K, dist, min_corners: int = 6):
        """이미지 → T_target_to_cam (4x4). 검출 실패 시 None.

        K: 카메라 행렬 (3x3), dist: 왜곡 계수.
        """
        import cv2
        import numpy as np
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        rvec = tvec = None
        if self._detector is not None:
            ch_corners, ch_ids, _, _ = self._detector.detectBoard(gray)
            if ch_corners is None or len(ch_corners) < min_corners:
                return None
            obj_pts, img_pts = self._board.matchImagePoints(ch_corners, ch_ids)
            if obj_pts is None or len(obj_pts) < min_corners:
                return None
            ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist)
            if not ok:
                return None
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict)
            if ids is None or len(ids) < 4:
                return None
            ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, self._board)
            if not ret or ch_corners is None or len(ch_corners) < min_corners:
                return None
            ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                ch_corners, ch_ids, self._board, K, dist, np.zeros(3), np.zeros(3))
            if not ok:
                return None
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        return T
