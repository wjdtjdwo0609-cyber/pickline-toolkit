"""인프라 — 크래시 안전 JSON 파일 IPC.

여러 프로세스(비전 워커 ↔ 로봇 워커 ↔ PLC 브리지 ↔ 슈퍼바이저)가
JSON 파일로 명령/상태를 주고받을 때 사용.

쓰기는 항상 임시 파일에 먼저 쓴 뒤 os.replace()로 원자적 교체한다.
따라서 읽는 쪽은 절반만 쓰인 파일을 절대 보지 않는다.

Intel5 step27_file_ipc.py 에서 추출 — 무수정 재사용 가능한 모듈.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any, Optional


def utc_ts() -> float:
    """현재 시각 (epoch seconds)."""
    return time.time()


def atomic_write_json(path: str | Path, data: dict[str, Any]) -> None:
    """JSON을 원자적으로 기록. 임시 파일 → os.replace.

    상위 디렉터리는 자동 생성된다.
    """
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_name(f".{target.name}.{os.getpid()}.{time.time_ns()}.tmp")
    payload = json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True)
    tmp.write_text(payload + "\n", encoding="utf-8")
    os.replace(tmp, target)


def read_json(path: str | Path, default: Optional[Any] = None) -> Any:
    """JSON 읽기. 파일이 없거나 깨졌으면 예외 없이 default 반환."""
    try:
        p = Path(path)
        if not p.exists():
            return default
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        return default


def max_known_seq(*items: Any) -> int:
    """여러 IPC dict에서 가장 큰 시퀀스 번호를 찾는다.

    seq / last_processed_seq / current_seq 키를 모두 살핀다.
    명령 중복 처리를 막는 단조 증가 카운터 관리에 사용.
    """
    seq = 0
    for item in items:
        if not isinstance(item, dict):
            continue
        for key in ("seq", "last_processed_seq", "current_seq"):
            try:
                seq = max(seq, int(item.get(key) or 0))
            except (TypeError, ValueError):
                pass
    return seq
