"""비전 — 카메라 소스 어댑터 (CameraSource 포트 구현체).

Intel5의 5중복 카메라 코드를 통합한 결과:
- RealSenseSource : pyrealsense2, RGB+Depth, 시리얼 핀 고정, depth 노이즈 필터
                    (step8_vision.CameraStream 기반)
- WebcamSource    : cv2.VideoCapture 일반 웹캠 (common/device.Camera 기반)
- DryRunSource    : 하드웨어 없이 더미 프레임 (테스트/CI용)

`open_camera()` 팩토리로 backend 문자열에 따라 적절한 어댑터를 만든다.

AVFoundation(ffmpeg) 어댑터는 macOS RealSense-SDK 우회용 — 별도 추출 예정.
"""

from __future__ import annotations

import time
from typing import Iterator, Optional

from pickline.shared.errors import CameraError
from pickline.shared.value_objects import CameraIntrinsics
from pickline.vision.camera import Frame


class _BaseSource:
    """공통 편의 메서드 (frames 제너레이터, 컨텍스트 매니저)."""

    _kind = "base"

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.close()

    @property
    def kind(self) -> str:
        return self._kind

    def frames(self) -> Iterator[Frame]:
        while True:
            try:
                yield self.read()
            except CameraError:
                break


class DryRunSource(_BaseSource):
    """하드웨어 없이 더미 프레임을 내는 소스. 테스트/CI 전용."""

    _kind = "dryrun"

    def __init__(self, width: int = 640, height: int = 480,
                 depth_mm: int = 700, gray: int = 128):
        self.width = width
        self.height = height
        self._depth_mm = depth_mm
        self._gray = gray
        self._count = 0
        self._started = False

    def start(self) -> None:
        self._started = True

    def read(self) -> Frame:
        if not self._started:
            raise CameraError("DryRunSource.start() 먼저 호출")
        import numpy as np
        self._count += 1
        color = np.full((self.height, self.width, 3), self._gray, dtype=np.uint8)
        depth = np.full((self.height, self.width), self._depth_mm, dtype=np.uint16)
        return Frame(color_bgr=color, depth_mm=depth, count=self._count, ts=time.time())

    def close(self) -> None:
        self._started = False

    @property
    def intrinsics(self) -> Optional[CameraIntrinsics]:
        # 더미 카메라 — 대략적인 D435 640x480 기본값
        return CameraIntrinsics(fx=615.0, fy=615.0, cx=self.width / 2,
                                cy=self.height / 2, width=self.width, height=self.height)


class WebcamSource(_BaseSource):
    """cv2.VideoCapture 일반 웹캠. depth 없음. 크로스플랫폼."""

    _kind = "webcam"

    def __init__(self, cam_id: int = 0, width: int = 640, height: int = 480):
        self.cam_id = cam_id
        self.width = width
        self.height = height
        self._cap = None
        self._count = 0

    def start(self) -> None:
        import cv2
        cap = cv2.VideoCapture(self.cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if not cap.isOpened():
            cap.release()
            raise CameraError(f"웹캠 {self.cam_id}을(를) 열 수 없습니다.")
        self._cap = cap

    def read(self) -> Frame:
        if self._cap is None:
            raise CameraError("WebcamSource.start() 먼저 호출")
        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise CameraError("웹캠 프레임 읽기 실패")
        self._count += 1
        return Frame(color_bgr=frame, depth_mm=None, count=self._count, ts=time.time())

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    @property
    def intrinsics(self) -> Optional[CameraIntrinsics]:
        return None


class RealSenseSource(_BaseSource):
    """RealSense D435 RGB+Depth 소스. 시리얼 핀 고정 + depth 노이즈 필터.

    step8_vision.CameraStream 기반. 다중 카메라 라인에서 serial로 특정 장치 지정.
    """

    _kind = "realsense"

    def __init__(self, serial: Optional[str] = None, width: int = 640,
                 height: int = 480, fps: int = 30, use_depth: bool = True,
                 depth_filters: bool = True):
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.use_depth = use_depth
        self.depth_filters = depth_filters
        self._rs = None
        self._pipeline = None
        self._align = None
        self._spatial = None
        self._temporal = None
        self._hole = None
        self._intrinsics: Optional[CameraIntrinsics] = None
        self._count = 0

    def start(self) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise CameraError("pyrealsense2 미설치 — pip install pickline[realsense]") from exc
        try:
            self._rs = rs
            self._pipeline = rs.pipeline()
            cfg = rs.config()
            if self.serial:
                cfg.enable_device(self.serial)
            cfg.enable_stream(rs.stream.color, self.width, self.height,
                              rs.format.bgr8, self.fps)
            if self.use_depth:
                cfg.enable_stream(rs.stream.depth, self.width, self.height,
                                  rs.format.z16, self.fps)
            profile = self._pipeline.start(cfg)
            self._align = rs.align(rs.stream.color)
            if self.use_depth and self.depth_filters:
                self._spatial = rs.spatial_filter()
                self._temporal = rs.temporal_filter()
                self._hole = rs.hole_filling_filter()
            color_stream = profile.get_stream(rs.stream.color)
            intr = color_stream.as_video_stream_profile().get_intrinsics()
            self._intrinsics = CameraIntrinsics(
                fx=intr.fx, fy=intr.fy, cx=intr.ppx, cy=intr.ppy,
                dist=tuple(intr.coeffs), width=intr.width, height=intr.height)
        except Exception as exc:
            raise CameraError(f"RealSense 초기화 실패 (serial={self.serial}): {exc}") from exc

    def read(self, n_avg: int = 1) -> Frame:
        """프레임 1장 읽기. n_avg>1이면 depth를 N프레임 median으로 노이즈 저감."""
        if self._pipeline is None:
            raise CameraError("RealSenseSource.start() 먼저 호출")
        import numpy as np
        depths = []
        last_color = None
        for _ in range(max(1, n_avg)):
            frames = self._pipeline.wait_for_frames()
            aligned = self._align.process(frames)
            color_frame = aligned.get_color_frame()
            if not color_frame:
                continue
            last_color = np.asanyarray(color_frame.get_data())
            if self.use_depth:
                depth_frame = aligned.get_depth_frame()
                if depth_frame:
                    if self._spatial is not None:
                        depth_frame = self._spatial.process(depth_frame)
                        depth_frame = self._temporal.process(depth_frame)
                        depth_frame = self._hole.process(depth_frame)
                    depths.append(np.asanyarray(depth_frame.get_data()))
        if last_color is None:
            raise CameraError("RealSense 컬러 프레임 읽기 실패")
        depth_mm = None
        if depths:
            depth_mm = (np.median(depths, axis=0).astype(np.uint16)
                        if len(depths) > 1 else depths[0])
        self._count += 1
        return Frame(color_bgr=last_color, depth_mm=depth_mm,
                     count=self._count, ts=time.time())

    def close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None

    @property
    def intrinsics(self) -> Optional[CameraIntrinsics]:
        return self._intrinsics


def open_camera(backend: str = "auto", *, serial: Optional[str] = None,
                cam_id: int = 0, width: int = 640, height: int = 480,
                fps: int = 30, use_depth: bool = True, dry_run: bool = False):
    """backend 문자열에 따라 적절한 CameraSource 어댑터를 만든다.

    backend: 'auto' | 'realsense' | 'webcam' | 'dryrun'
    'auto'는 RealSense 시도 후 실패하면 웹캠으로 fallback.
    """
    if dry_run or backend == "dryrun":
        return DryRunSource(width=width, height=height)
    if backend == "webcam":
        return WebcamSource(cam_id=cam_id, width=width, height=height)
    if backend == "realsense":
        return RealSenseSource(serial=serial, width=width, height=height,
                               fps=fps, use_depth=use_depth)
    if backend == "auto":
        # RealSense 가용 여부만 probe (start→close)하고, 항상 미시작 소스를 반환.
        probe = RealSenseSource(serial=serial, width=width, height=height,
                                fps=fps, use_depth=use_depth)
        try:
            probe.start()
            probe.close()
            return RealSenseSource(serial=serial, width=width, height=height,
                                   fps=fps, use_depth=use_depth)
        except CameraError:
            return WebcamSource(cam_id=cam_id, width=width, height=height)
    raise CameraError(f"알 수 없는 backend: {backend}")
