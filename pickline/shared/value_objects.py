"""공유 커널 — 값 객체 (value objects).

모든 bounded context가 공유하는 불변(immutable) 데이터 타입.
도메인 로직 없이 "값"만 담는다. 외부 의존성 없음 (numpy만 선택적).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple


# ---------------------------------------------------------------------------
# 좌표 / 자세
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Pose6D:
    """로봇 6자유도 자세. 위치 단위 mm, 회전 단위 deg.

    Indy7 task pose 규약: [x, y, z, rx, ry, rz] (ZYX 오일러, deg).
    """
    x: float
    y: float
    z: float
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]

    @classmethod
    def from_list(cls, v: Sequence[float]) -> "Pose6D":
        if len(v) < 3:
            raise ValueError(f"Pose6D needs >=3 values, got {len(v)}")
        vals = list(v) + [0.0] * (6 - len(v))
        return cls(*vals[:6])

    def with_z(self, z: float) -> "Pose6D":
        """Z만 바꾼 새 자세 (접근/리프트 높이 계산용)."""
        return Pose6D(self.x, self.y, z, self.rx, self.ry, self.rz)

    def offset_z(self, dz: float) -> "Pose6D":
        return Pose6D(self.x, self.y, self.z + dz, self.rx, self.ry, self.rz)


@dataclass(frozen=True)
class PixelPoint:
    """이미지 픽셀 좌표."""
    u: float
    v: float

    def as_tuple(self) -> Tuple[float, float]:
        return (self.u, self.v)


@dataclass(frozen=True)
class CameraIntrinsics:
    """카메라 내부 파라미터."""
    fx: float
    fy: float
    cx: float
    cy: float
    dist: Tuple[float, ...] = field(default_factory=tuple)
    width: int = 0
    height: int = 0


# ---------------------------------------------------------------------------
# 비전 검출
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Detection:
    """YOLO 검출 1건. 일반 detect 박스와 OBB(회전 박스)를 모두 표현.

    corners_px: OBB 모델일 때 회전 사각형 4꼭짓점 [(x,y) x4]. detect 모델이면 None.
    """
    class_id: int
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float
    corners_px: Optional[Tuple[Tuple[float, float], ...]] = None

    @property
    def center(self) -> PixelPoint:
        return PixelPoint((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    @property
    def is_obb(self) -> bool:
        return self.corners_px is not None


@dataclass(frozen=True)
class PickTarget:
    """비전이 계산해 로봇에 넘기는 픽 목표 (로봇 base 좌표계).

    pick_xyz_mm: 집을 위치 [x, y, z] mm.
    pick_yaw_deg: 그리퍼 회전각 deg (OBB 장축 기준, 없으면 0).
    """
    class_id: int
    confidence: float
    pick_xyz_mm: Tuple[float, float, float]
    pick_yaw_deg: float = 0.0
    depth_m: float = 0.0
    pixel_center: Optional[PixelPoint] = None


# ---------------------------------------------------------------------------
# 검사 / 품질
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class WeightRange:
    """양품 무게 허용 범위 (g)."""
    min_g: float
    max_g: float

    def contains(self, weight_g: float) -> bool:
        return self.min_g <= weight_g <= self.max_g


@dataclass(frozen=True)
class QualityDecision:
    """검사 판정 결과."""
    verdict: str           # "GOOD" | "REJECT" | "UNKNOWN"
    class_id: Optional[int] = None
    confidence: float = 0.0
    reason: str = ""
