"""픽킹 — 픽 사이클 유스케이스 (application 계층).

`PickConfig`는 픽 동작의 모든 튜닝값을 담는 값 객체 — 지금 바로 사용 가능.
`PickCycle`은 vision/robot/gripper 포트를 주입받아 픽앤플레이스 1회를
오케스트레이션하는 유스케이스 — 인터페이스 확정, 구현은 ROADMAP.

Intel5에는 픽 사이클이 4곳에 중복 구현돼 있다
(step11.PickPipeline, step24의 _full_cycle_worker / _fixed_pose_cycle_worker /
_pickup_only_worker). 이 클래스 하나로 mode 파라미터로 통일한다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class PickConfig:
    """픽앤플레이스 1회의 모든 튜닝 파라미터.

    Intel5 step11_pick_pipeline.PickConfig 기반 + 워커들의 상수 통합.
    프로젝트별로 이 값만 바꿔 재사용한다.
    """
    # 접근 / 이동 속도
    approach_vel_mmps: float = 30.0
    descend_vel_mmps: float = 5.0
    lift_vel_mmps: float = 30.0

    # 높이 (mm)
    z_approach_above_mm: float = 50.0      # 물체 위 접근 높이
    z_pickup_mm: float = 75.0              # 집는 높이
    z_lift_mm: float = 130.0               # 들어올린 높이
    z_safe_max_mm: float = 130.0           # 안전 Z 상한

    # 힘 제어
    force_threshold_n: float = 5.0         # 접촉 감지 임계
    use_force_descent: bool = False        # force_guided_descent 사용 여부
    compliance_stiffness: List[float] = field(
        default_factory=lambda: [2000, 2000, 300, 10, 10, 80])

    # 그리퍼/진공
    vacuum_settle_ms: int = 200
    vacuum_timeout_ms: int = 1500

    # 접근 자세 (deg)
    approach_uvw_deg: List[float] = field(default_factory=lambda: [0.0, -180.0, 0.0])

    # 재시도
    max_retries: int = 2
    retry_offset_mm: float = 3.0

    # 검출 게이트
    detection_min_confidence: float = 0.7


@dataclass
class PickResult:
    """픽 사이클 1회 결과."""
    success: bool
    class_id: int = -1
    attempts: int = 0
    duration_s: float = 0.0
    error: str = ""


# PickCycle 유스케이스의 인터페이스 스케치 (구현은 ROADMAP Phase 3):
#
# class PickCycle:
#     def __init__(self, robot: RobotController, gripper: Gripper,
#                  vision: Optional[VisionPipeline], calibration: Optional[Calibration],
#                  config: PickConfig, event_bus: Optional[EventBus] = None,
#                  mode: str = "full"):   # full | pickup_only | fixed_pose | reject
#         ...
#     def run(self, prefer_class: Optional[int] = None) -> PickResult:
#         """비전 검출 → 좌표 변환 → 접근 → 하강 → 흡착 → 검증 → 리프트 → 배치."""
#         ...
