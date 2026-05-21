"""로봇 제어 — Indy7/IndyDCP3 추상화, 그리퍼, DIO 핸드셰이크.

추출 완료:
- RobotController 포트
- IndyController (neuromeka/indy_utils SDK 자동 감지, dry_run 지원)
- Gripper 포트 + VacuumGripper (plc/robot_do/robot_endtool_do 백엔드)
- RobotDioHandshake (SmartDI/DO PLC 인터록)

추출 예정: FileIpcWorker (Phase 3).
"""

from pickline.robot.controller import RobotController
from pickline.robot.indy_controller import IndyController
from pickline.robot.gripper import Gripper, VacuumGripper
from pickline.robot.dio_handshake import (
    RobotDioHandshake,
    DioStartEvent,
    DioOrderError,
)

__all__ = [
    "RobotController", "IndyController",
    "Gripper", "VacuumGripper",
    "RobotDioHandshake", "DioStartEvent", "DioOrderError",
]
