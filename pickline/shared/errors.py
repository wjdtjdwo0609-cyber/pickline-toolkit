"""공유 커널 — 도메인 예외.

계층 간 의미 있는 실패를 표현. 인프라 예외(SDK 에러 등)는
각 어댑터에서 잡아 아래 타입으로 변환해 올린다.
"""

from __future__ import annotations


class PicklineError(Exception):
    """모든 pickline 도메인 예외의 베이스."""


# --- 로봇 ---
class RobotError(PicklineError):
    """로봇 제어 실패 일반."""


class RobotConnectionError(RobotError):
    """로봇 연결/재연결 실패."""


class RobotMotionError(RobotError):
    """이동 명령 실패 (목표 미도달, 안전범위 초과 등)."""


class SafetyLimitError(RobotMotionError):
    """안전 한계(Z 상한, 워크스페이스 등) 위반."""


# --- 비전 ---
class VisionError(PicklineError):
    """비전 파이프라인 실패 일반."""


class CameraError(VisionError):
    """카메라 초기화/읽기 실패."""


class ModelNotFoundError(VisionError):
    """학습된 YOLO 모델을 찾지 못함."""


# --- 캘리브레이션 ---
class CalibrationError(PicklineError):
    """캘리브레이션 실패 일반."""


class CalibrationNotSolvedError(CalibrationError):
    """캘리브레이션 결과가 아직 없음 (파일 없음/미해결)."""


# --- 측정 ---
class MeasurementError(PicklineError):
    """무게/측정 실패 일반."""


# --- PLC / IPC ---
class PlcError(PicklineError):
    """PLC 통신 실패."""


class IpcError(PicklineError):
    """파일 IPC 실패."""
