"""로봇 제어 — 포트(인터페이스) 정의.

`RobotController`는 이 bounded context의 핵심 포트다.
어떤 로봇 SDK(neuromeka IndyDCP3, 레거시 indy_utils 등)를 쓰든
이 인터페이스 뒤에 숨긴다. 상위 계층(picking 등)은 이 Protocol에만 의존한다.

구현체: `IndyController` (Intel5 step9_robot.py 에서 추출 예정 — ROADMAP 참고).
"""

from __future__ import annotations

from typing import List, Optional, Protocol, runtime_checkable

from pickline.shared.value_objects import Pose6D


@runtime_checkable
class RobotController(Protocol):
    """6축 로봇 팔 제어 포트. 단위: 위치 mm, 회전 deg, 힘 N.

    모든 구현체는 `dry_run` 모드(하드웨어 없이 동작)를 지원해야 한다.
    """

    # --- 연결 ---
    def connect(self) -> bool: ...
    def reconnect(self, reason: str = "") -> bool: ...
    def disconnect(self) -> None: ...

    # --- 상태 조회 ---
    def get_pose(self) -> Pose6D:
        """현재 TCP 자세 [x,y,z,rx,ry,rz]."""
        ...

    def get_joint_pos(self) -> List[float]:
        """6개 관절 각도 (rad)."""
        ...

    def get_force(self) -> List[float]:
        """F/T 센서 [Fx,Fy,Fz,Tx,Ty,Tz]."""
        ...

    # --- 모션 ---
    def move_l(self, target: Pose6D, vel_mmps: float = 30.0,
               wait: bool = True, timeout_s: float = 30.0) -> bool:
        """직선 이동(Cartesian). 안전범위 클리핑 후 실행."""
        ...

    def go_home(self, wait: bool = True, timeout_s: float = 60.0) -> bool: ...
    def go_zero(self, wait: bool = True, timeout_s: float = 60.0) -> bool: ...
    def stop_motion(self) -> None: ...
    def emergency_stop(self) -> None: ...

    # --- 설정 ---
    def set_tcp(self, tcp_mm_deg: List[float]) -> bool: ...
    def set_safety_limits(self, z_safe_max_mm: Optional[float] = None,
                          approach_uvw_deg: Optional[List[float]] = None) -> None: ...

    # --- 디지털 I/O (그리퍼/PLC 핸드셰이크용) ---
    def set_do(self, address: int, state: bool) -> bool: ...
    def get_di(self) -> List[int]: ...
    def get_do(self) -> List[int]: ...
    def read_di(self, address: int) -> bool: ...
    def pulse_do(self, address: int, pulse_s: float = 1.0) -> None: ...
    def set_endtool_do(self, index: int, state: bool) -> bool: ...

    # --- 컴플라이언스 / 힘 제어 (정밀 삽입용) ---
    def set_compliance(self, stiffness: Optional[List[float]] = None,
                       damping: Optional[List[float]] = None) -> None: ...
    def disable_compliance(self) -> None: ...
    def force_guided_descent(self, target: Pose6D, axis: str = "z",
                             threshold_n: float = 5.0, vel_mmps: float = 5.0,
                             timeout_s: float = 8.0) -> bool:
        """힘 임계값에 닿을 때까지 하강 (접촉 기반 픽업)."""
        ...
