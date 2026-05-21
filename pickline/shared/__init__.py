"""공유 커널 — 모든 bounded context가 공유하는 타입."""

from pickline.shared.value_objects import (
    Pose6D,
    PixelPoint,
    CameraIntrinsics,
    Detection,
    PickTarget,
    WeightRange,
    QualityDecision,
)
from pickline.shared.errors import (
    PicklineError,
    RobotError,
    RobotConnectionError,
    RobotMotionError,
    SafetyLimitError,
    VisionError,
    CameraError,
    ModelNotFoundError,
    CalibrationError,
    CalibrationNotSolvedError,
    MeasurementError,
    PlcError,
    IpcError,
)
from pickline.shared.events import (
    DomainEvent,
    EventBus,
    SocketDetected,
    PickStarted,
    PickCompleted,
    RejectTriggered,
    WeightMeasured,
)

__all__ = [
    "Pose6D", "PixelPoint", "CameraIntrinsics", "Detection", "PickTarget",
    "WeightRange", "QualityDecision",
    "PicklineError", "RobotError", "RobotConnectionError", "RobotMotionError",
    "SafetyLimitError", "VisionError", "CameraError", "ModelNotFoundError",
    "CalibrationError", "CalibrationNotSolvedError", "MeasurementError",
    "PlcError", "IpcError",
    "DomainEvent", "EventBus", "SocketDetected", "PickStarted",
    "PickCompleted", "RejectTriggered", "WeightMeasured",
]
