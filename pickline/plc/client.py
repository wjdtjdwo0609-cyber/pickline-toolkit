"""PLC 연동 — MC Protocol 클라이언트.

Mitsubishi PLC와 MC Protocol(TCP)로 비트/워드 디바이스를 읽고 쓴다.
pymcprotocol 래퍼. ladder 정책(어떤 비트가 무슨 의미인지)은 이 계층이 모른다 —
그건 프로젝트별 상위 계층(브리지)의 몫.

Intel5 step29_plc_bridge.py 의 디바이스 접근 계층(PlcBridge.read_bit/write_bit/
read_word/index bit + _device_offset)을 추출.

`device_map`을 주면 심볼릭 이름("plc3_start_bit")으로도, 생(raw) 디바이스
주소("B200")로도 읽고 쓸 수 있다.
"""

from __future__ import annotations

import logging
from typing import Dict, Optional

from pickline.shared.errors import PlcError

_log = logging.getLogger("pickline.plc")


def device_offset(device: str, delta: int) -> str:
    """디바이스 주소에 인덱스 오프셋을 더한다.

    B/W 계열은 16진수, 나머지는 10진수로 증가.
    예: device_offset("B260", 3) -> "B263",  device_offset("D100", 5) -> "D105"
    """
    i = 0
    while i < len(device) and device[i].isalpha():
        i += 1
    prefix, number = device[:i], device[i:]
    if prefix.upper() in {"B", "W"}:
        return f"{prefix}{int(number, 16) + delta:X}"
    return f"{prefix}{int(number) + delta}"


class PlcClient:
    """MC Protocol PLC 클라이언트.

    Args:
        ip/port: PLC 주소 (기본 포트 1025).
        comm_type: "binary" | "ascii".
        network/pc: MC Protocol 네트워크/PC 번호.
        plc_type: pymcprotocol PLC 타입 ("Q", "L", "iQ-R" 등).
        device_map: {심볼릭 이름: 디바이스 주소} (선택).
        dry_run: 하드웨어 없이 동작 (읽기는 0/False 반환).
    """

    def __init__(self, ip: str, port: int = 1025, *, comm_type: str = "binary",
                 network: int = 0, pc: int = 255, plc_type: str = "Q",
                 device_map: Optional[Dict[str, str]] = None,
                 dry_run: bool = False, logger: Optional[logging.Logger] = None):
        self.ip = ip
        self.port = port
        self.comm_type = comm_type
        self.network = network
        self.pc = pc
        self.plc_type = plc_type
        self.device_map = dict(device_map or {})
        self.dry_run = dry_run
        self.log = logger or _log
        self.plc = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.close()

    def connect(self) -> None:
        if self.dry_run:
            self.log.info("DRY-RUN: PLC 연결 생략 (%s:%s)", self.ip, self.port)
            return
        try:
            import pymcprotocol
        except ImportError as exc:
            raise PlcError("pymcprotocol 미설치 — pip install pickline[plc]") from exc
        try:
            self.plc = pymcprotocol.Type3E(plctype=self.plc_type)
            self.plc.setaccessopt(commtype=self.comm_type,
                                  network=self.network, pc=self.pc)
            self.plc.connect(self.ip, self.port)
            self.log.info("PLC 연결: %s:%s comm=%s", self.ip, self.port, self.comm_type)
        except Exception as exc:
            raise PlcError(f"PLC 연결 실패 ({self.ip}:{self.port}): {exc}") from exc

    def close(self) -> None:
        if self.plc is not None:
            try:
                self.plc.close()
            except Exception:
                pass
            self.plc = None

    def resolve(self, device_or_key: str) -> str:
        """심볼릭 이름이면 device_map으로 변환, 아니면 그대로."""
        return self.device_map.get(device_or_key, device_or_key)

    # --- 비트 ---
    def read_bit(self, device: str) -> bool:
        if self.dry_run:
            return False
        dev = self.resolve(device)
        try:
            return bool(self.plc.batchread_bitunits(headdevice=dev, readsize=1)[0])
        except Exception as exc:
            raise PlcError(f"비트 읽기 실패 ({dev}): {exc}") from exc

    def write_bit(self, device: str, value: bool) -> None:
        if self.dry_run:
            return
        dev = self.resolve(device)
        try:
            self.plc.batchwrite_bitunits(headdevice=dev, values=[1 if value else 0])
        except Exception as exc:
            raise PlcError(f"비트 쓰기 실패 ({dev}<-{value}): {exc}") from exc

    def read_index_bit(self, base: str, index: int) -> bool:
        """base 디바이스에서 index만큼 오프셋된 비트를 읽는다."""
        return self.read_bit(device_offset(self.resolve(base), index))

    def write_index_bit(self, base: str, index: int, value: bool) -> None:
        self.write_bit(device_offset(self.resolve(base), index), value)

    def clear_index_bits(self, base: str, count: int) -> None:
        """base부터 count개 비트를 모두 0으로."""
        for i in range(count):
            self.write_index_bit(base, i, False)

    # --- 워드 ---
    def read_word(self, device: str) -> int:
        if self.dry_run:
            return 0
        dev = self.resolve(device)
        try:
            return int(self.plc.batchread_wordunits(headdevice=dev, readsize=1)[0])
        except Exception as exc:
            raise PlcError(f"워드 읽기 실패 ({dev}): {exc}") from exc

    def write_word(self, device: str, value: int) -> None:
        if self.dry_run:
            return
        dev = self.resolve(device)
        try:
            self.plc.batchwrite_wordunits(headdevice=dev, values=[int(value)])
        except Exception as exc:
            raise PlcError(f"워드 쓰기 실패 ({dev}<-{value}): {exc}") from exc
