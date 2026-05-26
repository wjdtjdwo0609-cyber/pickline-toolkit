"""캘리브레이션 — 평면 호모그래피 솔버.

작업대가 하나의 평면이라 가정하고, 카메라 픽셀 (u,v)를 로봇 base XY로
바꾸는 3x3 호모그래피 H를 푼다. Z는 별도 설정/티칭값.

hand-eye 대비 단순·안정적 — 작업면이 평평하고 픽 높이가 일정하면 충분.
4점이면 정확해(자유도 8 = 4점×2식). RANSAC 불필요.

Intel5 step22_plane_calib_gui.py 의 solve() 로직 추출.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

from pickline.shared.errors import CalibrationError


@dataclass
class PlaneCalibResult:
    """평면 호모그래피 캘리브레이션 결과."""
    H: object                  # np.ndarray (3,3) — 픽셀 → base XY
    pixels: object             # np.ndarray (N,2)
    base_xy: object            # np.ndarray (N,2)
    base_z: object             # np.ndarray (N,)
    rms_mm: float
    max_mm: float
    z_std_mm: float
    z_median_mm: float
    n_points: int

    def pixel_to_base(self, u: float, v: float) -> Tuple[float, float]:
        """픽셀 (u,v) → 로봇 base (x,y) mm."""
        import numpy as np
        p = np.array([u, v, 1.0])
        est = self.H @ p
        return float(est[0] / est[2]), float(est[1] / est[2])


class PlaneHomographyCalibrator:
    """픽셀 4점 + 로봇 base 4점을 모아 호모그래피를 푼다.

    사용:
        cal = PlaneHomographyCalibrator()
        cal.add_point((u, v), [x, y, z, ...])   # 4점 이상
        result = cal.solve()
        rx, ry = result.pixel_to_base(320, 240)
    """

    def __init__(self, min_points: int = 4):
        self.min_points = min_points
        self._pixels: List[Tuple[float, float]] = []
        self._base_poses: List[Sequence[float]] = []

    def add_point(self, pixel: Tuple[float, float], base_pose: Sequence[float]) -> None:
        """픽셀 좌표 (u,v)와 그 지점의 로봇 base pose [x,y,z,...] 한 쌍 추가."""
        if len(base_pose) < 3:
            raise ValueError("base_pose는 최소 [x,y,z] 필요")
        self._pixels.append((float(pixel[0]), float(pixel[1])))
        self._base_poses.append(list(base_pose))

    @property
    def n_points(self) -> int:
        return len(self._pixels)

    def reset(self) -> None:
        self._pixels.clear()
        self._base_poses.clear()

    def solve(self) -> PlaneCalibResult:
        """누적된 점들로 호모그래피를 푼다."""
        n = self.n_points
        if n < self.min_points:
            raise CalibrationError(f"점 부족 ({n}, 최소 {self.min_points})")

        import cv2
        import numpy as np

        pixels = np.array(self._pixels, dtype=np.float32)
        base_xy = np.array([p[:2] for p in self._base_poses], dtype=np.float32)
        base_z = np.array([p[2] for p in self._base_poses], dtype=np.float64)

        H, _ = cv2.findHomography(pixels, base_xy, method=0)
        if H is None:
            raise CalibrationError("호모그래피 풀이 실패 — 점들이 일직선?")

        # 재투영 RMS
        pts_h = np.hstack([pixels, np.ones((n, 1), dtype=np.float32)])
        est = (H @ pts_h.T).T
        est = est[:, :2] / est[:, 2:3]
        errors = np.linalg.norm(est - base_xy, axis=1)
        rms_mm = float(np.sqrt(np.mean(errors ** 2)))
        max_mm = float(errors.max())

        z_std = float(np.std(base_z))
        z_median = float(np.median(base_z))

        return PlaneCalibResult(
            H=H, pixels=pixels, base_xy=base_xy, base_z=base_z,
            rms_mm=rms_mm, max_mm=max_mm,
            z_std_mm=z_std, z_median_mm=z_median, n_points=n)
