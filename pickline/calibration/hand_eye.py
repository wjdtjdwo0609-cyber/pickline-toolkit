"""캘리브레이션 — Hand-Eye (eye-to-base) 솔버.

천장 고정 카메라 + EE에 부착된 ChArUco 보드 구성에서
T_cam_to_base (카메라 → 로봇 base 변환)를 푼다.

Intel5 step7_solve_hand_eye.py 의 솔버 + step21 GUI의 중복 구현을 통합.

핵심 — eye-to-base swap 트릭:
    cv2.calibrateHandEye는 eye-on-hand(카메라가 EE에 부착)용 시그니처다.
    eye-to-base(카메라 고정)를 풀려면 EE→base를 역행렬(base→EE)로 넣는다.
    그러면 결과 (R_x, t_x)가 T_cam_to_base가 된다.
    TSAI 방식은 이 swap에서 회전 부호가 꼬이므로 제외 (PARK 권장).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from pickline.calibration.charuco import CharucoBoardModel
from pickline.calibration.transforms import pose_to_matrix
from pickline.shared.errors import CalibrationError


def get_d435_intrinsics_default():
    """D435 640x480 factory 내부 파라미터 (K 3x3, dist 5,). numpy 필요."""
    import numpy as np
    K = np.array([[615.0, 0.0, 320.0],
                  [0.0, 615.0, 240.0],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    return K, np.zeros(5, dtype=np.float64)


_METHODS = {"PARK", "DANIILIDIS", "HORAUD", "ANDREFF"}  # TSAI 제외


@dataclass
class HandEyeResult:
    """Hand-eye 캘리브레이션 결과."""
    T_cam_to_base: object             # np.ndarray (4,4)
    T_target_to_ee: object            # np.ndarray (4,4)
    camera_matrix: object             # np.ndarray (3,3)
    dist_coeffs: object               # np.ndarray
    rms_mm: float
    max_mm: float
    n_pairs: int
    method: str
    rotation_format: str

    def quality(self) -> str:
        if self.rms_mm < 3:
            return "우수"
        if self.rms_mm < 5:
            return "양호 (픽킹 가능)"
        if self.rms_mm < 10:
            return "한계치 (재캡처 권장)"
        return "정밀도 부족"


class HandEyeCalibrator:
    """ChArUco 이미지 + EE pose 페어를 모아 hand-eye를 푼다.

    사용:
        cal = HandEyeCalibrator(board, K, dist, method="PARK")
        for image, ee_pose in pairs:
            cal.add_pair(image, ee_pose)      # bool 반환 (보드 검출 성공 여부)
        result = cal.solve()
    """

    def __init__(self, board: CharucoBoardModel, K, dist,
                 method: str = "PARK", rotation: str = "euler-zyx-deg"):
        method = method.upper()
        if method not in _METHODS:
            raise ValueError(f"지원 method: {sorted(_METHODS)} (TSAI 제외)")
        self.board = board
        self.K = K
        self.dist = dist
        self.method = method
        self.rotation = rotation
        self._T_target_to_cam: List[object] = []
        self._T_ee_to_base: List[object] = []

    def add_pair(self, image, ee_pose: List[float]) -> bool:
        """(보드 이미지, EE pose) 페어 추가. 보드 검출 성공 시 True."""
        T_tc = self.board.estimate_pose(image, self.K, self.dist)
        if T_tc is None:
            return False
        self._T_target_to_cam.append(T_tc)
        self._T_ee_to_base.append(pose_to_matrix(ee_pose, rotation=self.rotation))
        return True

    @property
    def n_pairs(self) -> int:
        return len(self._T_target_to_cam)

    def solve(self) -> HandEyeResult:
        """누적된 페어로 T_cam_to_base를 푼다. 최소 6쌍 필요."""
        n = self.n_pairs
        if n < 6:
            raise CalibrationError(f"유효 페어 부족 ({n}, 최소 6)")

        import cv2
        import numpy as np

        R_tc = [T[:3, :3] for T in self._T_target_to_cam]
        t_tc = [T[:3, 3] for T in self._T_target_to_cam]
        R_eb = [T[:3, :3] for T in self._T_ee_to_base]
        t_eb = [T[:3, 3] for T in self._T_ee_to_base]

        # eye-to-base swap: EE→base를 역행렬로 넣는다.
        R_g2b_inv = [R.T for R in R_eb]
        t_g2b_inv = [-R.T @ t for R, t in zip(R_eb, t_eb)]

        method_const = getattr(cv2, f"CALIB_HAND_EYE_{self.method}")
        R_x, t_x = cv2.calibrateHandEye(
            R_gripper2base=R_g2b_inv, t_gripper2base=t_g2b_inv,
            R_target2cam=R_tc, t_target2cam=t_tc, method=method_const)

        T_cam_to_base = np.eye(4)
        T_cam_to_base[:3, :3] = R_x
        T_cam_to_base[:3, 3] = t_x.flatten()

        # T_target_to_ee = inv(T_ee_to_base) @ T_cam_to_base @ T_target_to_cam, 평균
        estimates = []
        for T_eb_i, T_tc_i in zip(self._T_ee_to_base, self._T_target_to_cam):
            estimates.append(np.linalg.inv(T_eb_i) @ T_cam_to_base @ T_tc_i)
        T_target_to_ee = np.mean(estimates, axis=0)

        # 검증 RMS — 보드 원점을 두 경로로 계산해 비교 (m→mm)
        origin = np.array([0, 0, 0, 1.0])
        diffs = []
        for T_eb_i, T_tc_i in zip(self._T_ee_to_base, self._T_target_to_cam):
            a = T_cam_to_base @ T_tc_i @ origin
            b = T_eb_i @ T_target_to_ee @ origin
            diffs.append(np.linalg.norm(a[:3] - b[:3]) * 1000.0)
        diffs = np.array(diffs)
        rms_mm = float(np.sqrt(np.mean(diffs ** 2)))
        max_mm = float(diffs.max())

        return HandEyeResult(
            T_cam_to_base=T_cam_to_base, T_target_to_ee=T_target_to_ee,
            camera_matrix=np.asarray(self.K), dist_coeffs=np.asarray(self.dist),
            rms_mm=rms_mm, max_mm=max_mm, n_pairs=n,
            method=self.method, rotation_format=self.rotation)
