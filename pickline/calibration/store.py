"""캘리브레이션 — 결과 저장소 (.npz 리포지토리).

캘리브레이션 결과를 .npz로 저장/로드. 솔버 클래스를 npz 스키마에서 분리한다.

Intel5에서 step7/21/22가 각자 npz 저장 로직을 중복 구현 → 여기로 통합.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

from pickline.calibration.hand_eye import HandEyeResult
from pickline.calibration.plane_homography import PlaneCalibResult
from pickline.shared.errors import CalibrationNotSolvedError


class CalibrationStore:
    """캘리브레이션 .npz 파일 저장/로드."""

    @staticmethod
    def save_hand_eye(result: HandEyeResult, path: str | Path) -> None:
        import numpy as np
        np.savez(
            str(path),
            T_cam_to_base=result.T_cam_to_base,
            T_target_to_ee=result.T_target_to_ee,
            camera_matrix=result.camera_matrix,
            dist_coeffs=result.dist_coeffs,
            rms_mm=result.rms_mm,
            max_mm=result.max_mm,
            n_pairs=result.n_pairs,
            rotation_format=result.rotation_format,
        )

    @staticmethod
    def load_hand_eye(path: str | Path) -> Dict[str, Any]:
        """hand_eye.npz 로드 → dict. 파일 없으면 CalibrationNotSolvedError."""
        import numpy as np
        p = Path(path)
        if not p.exists():
            raise CalibrationNotSolvedError(f"hand-eye 캘리브 파일 없음: {p}")
        data = np.load(str(p), allow_pickle=True)
        return {k: data[k] for k in data.files}

    @staticmethod
    def save_plane(result: PlaneCalibResult, path: str | Path,
                   *, channel_id: str = "", z_pickup_mm: float = 0.0,
                   z_safe_max_mm: float = 0.0,
                   approach_uvw_deg=(0.0, -180.0, 0.0)) -> None:
        import numpy as np
        np.savez(
            str(path),
            H=result.H,
            pixels=result.pixels,
            base_xy=result.base_xy,
            base_z=result.base_z,
            rms_mm=result.rms_mm,
            max_mm=result.max_mm,
            z_std_mm=result.z_std_mm,
            z_median_mm=result.z_median_mm,
            n_points=result.n_points,
            channel_id=channel_id,
            z_pickup_mm=z_pickup_mm,
            z_safe_max_mm=z_safe_max_mm,
            approach_uvw_deg=np.array(approach_uvw_deg, dtype=np.float64),
        )

    @staticmethod
    def load_plane(path: str | Path) -> Dict[str, Any]:
        """plane_calib.npz 로드 → dict. 파일 없으면 CalibrationNotSolvedError."""
        import numpy as np
        p = Path(path)
        if not p.exists():
            raise CalibrationNotSolvedError(f"평면 캘리브 파일 없음: {p}")
        data = np.load(str(p), allow_pickle=True)
        return {k: data[k] for k in data.files}
