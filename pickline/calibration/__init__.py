"""캘리브레이션 — 좌표 변환과 카메라↔로봇 정합.

추출 완료:
- transforms: 순수 좌표 변환 수학
- charuco: BoardSpec, CharucoBoardModel (보드 생성·검출·PnP)
- hand_eye: HandEyeCalibrator (eye-to-base, 천장 카메라)
- plane_homography: PlaneHomographyCalibrator (평면 4점)
- store: CalibrationStore (.npz 저장/로드)
"""

from pickline.calibration.transforms import (
    rpy_to_rotmat,
    rotvec_to_rotmat,
    pose_to_matrix,
    invert_transform,
    apply_transform,
)
from pickline.calibration.charuco import BoardSpec, CharucoBoardModel, CharucoDetection
from pickline.calibration.hand_eye import (
    HandEyeCalibrator,
    HandEyeResult,
    get_d435_intrinsics_default,
)
from pickline.calibration.plane_homography import (
    PlaneHomographyCalibrator,
    PlaneCalibResult,
)
from pickline.calibration.store import CalibrationStore

__all__ = [
    "rpy_to_rotmat", "rotvec_to_rotmat", "pose_to_matrix",
    "invert_transform", "apply_transform",
    "BoardSpec", "CharucoBoardModel", "CharucoDetection",
    "HandEyeCalibrator", "HandEyeResult", "get_d435_intrinsics_default",
    "PlaneHomographyCalibrator", "PlaneCalibResult",
    "CalibrationStore",
]
