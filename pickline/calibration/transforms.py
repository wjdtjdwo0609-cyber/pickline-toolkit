"""캘리브레이션 — 순수 좌표 변환 수학.

외부 상태/하드웨어 의존 없음 (numpy 필수, cv2는 rotvec에만 선택적).
도메인 계층의 가장 안쪽 — 단위 테스트가 쉬운 순수 함수들.

Intel5 step7_solve_hand_eye.py 에서 추출.

단위 규약:
- 로봇 pose 입력: [x, y, z, rx, ry, rz], 위치 mm, 회전 deg
- 4x4 변환행렬 출력: 위치 m (mm/1000), 회전 무차원
"""

from __future__ import annotations

import numpy as np


def rpy_to_rotmat(rx: float, ry: float, rz: float,
                  order: str = "zyx", unit: str = "deg") -> np.ndarray:
    """오일러 각 → 3x3 회전 행렬.

    order: 'zyx' (Indy7 기본), 'xyz', 'zyz'.
    unit:  'deg' 또는 'rad'.
    """
    if unit == "deg":
        rx, ry, rz = np.deg2rad([rx, ry, rz])

    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

    if order == "zyx":
        return Rz @ Ry @ Rx
    if order == "xyz":
        return Rx @ Ry @ Rz
    if order == "zyz":
        Rz2 = np.array([[cx, -sx, 0], [sx, cx, 0], [0, 0, 1]])
        return Rz @ Ry @ Rz2
    raise ValueError(f"unsupported euler order: {order}")


def rotvec_to_rotmat(rx: float, ry: float, rz: float) -> np.ndarray:
    """축각(rotation vector, rad) → 3x3 회전 행렬 (Rodrigues)."""
    import cv2
    rvec = np.array([rx, ry, rz], dtype=np.float64)
    R, _ = cv2.Rodrigues(rvec)
    return R


def pose_to_matrix(pose, rotation: str = "euler-zyx-deg") -> np.ndarray:
    """로봇 pose [x,y,z,rx,ry,rz] (mm+deg) → 4x4 변환행렬 (위치 m).

    rotation:
        'euler-zyx-deg' / 'euler-xyz-deg' / 'euler-zyx-rad' / ...
        'rotvec-rad'  (Rodrigues 축각)
        'rotvec-deg'

    주의: Indy7 SDK가 회전을 어떤 표기로 주는지 반드시 한 번 확인할 것.
    틀리면 hand-eye 캘리브 RMS가 수십~수백 mm로 폭발한다.
    """
    x, y, z, rx, ry, rz = pose

    if rotation.startswith("euler-"):
        _, order, unit = rotation.split("-")
        R = rpy_to_rotmat(rx, ry, rz, order=order, unit=unit)
    elif rotation == "rotvec-rad":
        R = rotvec_to_rotmat(rx, ry, rz)
    elif rotation == "rotvec-deg":
        R = rotvec_to_rotmat(*np.deg2rad([rx, ry, rz]))
    else:
        raise ValueError(f"unsupported rotation: {rotation}")

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array([x, y, z]) / 1000.0   # mm → m
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """4x4 강체 변환행렬의 역행렬 (회전 전치 + 평행이동 보정)."""
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def apply_transform(T: np.ndarray, point_xyz: np.ndarray) -> np.ndarray:
    """4x4 변환을 3D 점에 적용. point_xyz: (3,) → (3,)."""
    p = np.asarray(point_xyz, dtype=np.float64)
    homog = np.array([p[0], p[1], p[2], 1.0])
    return (T @ homog)[:3]
