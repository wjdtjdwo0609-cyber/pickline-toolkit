"""인프라 — 크로스플랫폼 가속 디바이스 자동 선택.

OS/하드웨어에 맞는 torch 디바이스를 고른다.
- Windows/Linux + NVIDIA GPU: 'cuda'
- Windows/Linux + Intel iGPU:  'xpu'  (intel_extension_for_pytorch 필요)
- macOS Apple Silicon:         'mps'
- 그 외:                        'cpu'

torch가 설치돼 있지 않으면 'cpu'를 반환 (선택적 의존성).
"""

from __future__ import annotations

import platform
from typing import Optional


def select_device(preferred: Optional[str] = None) -> str:
    """사용 가능한 가속 디바이스를 자동 선택.

    preferred를 명시하면 우선 시도하고, 불가능하면 fallback.
    """
    try:
        import torch
    except ImportError:
        return "cpu"

    candidates = []
    if preferred:
        candidates.append(preferred)
    candidates += ["cuda", "xpu", "mps", "cpu"]

    for dev in candidates:
        if dev == "cuda" and torch.cuda.is_available():
            return "cuda"
        if dev == "xpu":
            try:
                import intel_extension_for_pytorch  # noqa: F401
                if hasattr(torch, "xpu") and torch.xpu.is_available():
                    return "xpu"
            except Exception:
                continue
        if dev == "mps" and torch.backends.mps.is_available():
            return "mps"
        if dev == "cpu":
            return "cpu"
    return "cpu"


def device_info() -> str:
    """디바이스/OS/torch 정보 한 줄 요약."""
    dev = select_device()
    try:
        import torch
        torch_ver = torch.__version__
    except ImportError:
        torch_ver = "not installed"
    return f"OS={platform.system()} | device={dev} | torch={torch_ver}"
