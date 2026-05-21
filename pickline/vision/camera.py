"""비전 — 카메라 소스 포트(인터페이스).

Intel5 코드베이스의 가장 큰 중복: 카메라 초기화가 5곳 이상에 복붙돼 있다
(common.Camera, CameraStream, RealSenseRgbCamera, FfmpegCamera, RealSenseColorCamera).
이 포트 하나로 통합한다.

구현체(어댑터)는 backend별로:
- RealSenseSource     : pyrealsense2, depth 지원, 시리얼 핀 고정
- AVFoundationSource  : macOS ffmpeg subprocess (RealSense SDK 불안정 시 우회)
- WebcamSource        : cv2.VideoCapture 일반 웹캠
- DryRunSource        : 테스트용 더미 프레임

추출 ROADMAP: ARCHITECTURE.md 의 vision 섹션 참고.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator, Optional, Protocol, runtime_checkable

from pickline.shared.value_objects import CameraIntrinsics


@dataclass
class Frame:
    """카메라 1프레임. depth는 RealSense일 때만 not-None."""
    color_bgr: object               # np.ndarray (H,W,3) BGR
    depth_mm: Optional[object] = None   # np.ndarray (H,W) uint16, mm
    count: int = 0
    ts: float = 0.0


@runtime_checkable
class CameraSource(Protocol):
    """카메라 소스 통합 인터페이스. 백엔드 무관.

    컨텍스트 매니저로 쓰거나 start()/close()를 직접 호출.
    """

    def start(self) -> None: ...
    def read(self) -> Frame:
        """한 프레임 읽기. 실패 시 CameraError."""
        ...
    def close(self) -> None: ...

    @property
    def intrinsics(self) -> Optional[CameraIntrinsics]:
        """내부 파라미터 (depth→3D 변환용). 없으면 None."""
        ...

    @property
    def kind(self) -> str:
        """'realsense' | 'avfoundation' | 'webcam' | 'dryrun'."""
        ...

    def frames(self) -> Iterator[Frame]:
        """프레임 제너레이터 (편의)."""
        ...
