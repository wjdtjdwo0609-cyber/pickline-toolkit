"""공유 커널 — 도메인 이벤트 + 경량 이벤트 버스.

DDD의 도메인 이벤트를 Python답게 단순 구현한다.
bounded context 간 느슨한 결합을 위해 사용 — 한 컨텍스트가 이벤트를
publish 하면, 구독한 다른 컨텍스트의 핸들러가 호출된다.

claude-flow v3 DDD 스킬의 DomainEvent/EventBus 개념을 Python으로 옮긴 것.
TypeScript의 데코레이터 기반 DI 대신 명시적 subscribe()를 쓴다.
"""

from __future__ import annotations

import time
import uuid
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Callable, DefaultDict, List, Type


@dataclass(frozen=True)
class DomainEvent:
    """모든 도메인 이벤트의 베이스. 하위 클래스가 페이로드 필드를 추가한다."""
    event_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    occurred_at: float = field(default_factory=time.time)


# --- 비전 ---
@dataclass(frozen=True)
class SocketDetected(DomainEvent):
    class_id: int = -1
    confidence: float = 0.0


# --- 픽킹 ---
@dataclass(frozen=True)
class PickStarted(DomainEvent):
    channel_id: str = ""
    class_id: int = -1


@dataclass(frozen=True)
class PickCompleted(DomainEvent):
    channel_id: str = ""
    class_id: int = -1
    success: bool = False
    duration_s: float = 0.0


@dataclass(frozen=True)
class RejectTriggered(DomainEvent):
    channel_id: str = ""
    reason: str = ""


# --- 측정 ---
@dataclass(frozen=True)
class WeightMeasured(DomainEvent):
    weight_g: float = 0.0
    stable: bool = False


class EventBus:
    """동기(synchronous) 인메모리 이벤트 버스.

    사용 예:
        bus = EventBus()
        bus.subscribe(PickCompleted, lambda e: print(e.success))
        bus.publish(PickCompleted(channel_id="r1", success=True))
    """

    def __init__(self) -> None:
        self._handlers: DefaultDict[Type[DomainEvent], List[Callable]] = defaultdict(list)

    def subscribe(self, event_type: Type[DomainEvent], handler: Callable[[DomainEvent], None]) -> None:
        self._handlers[event_type].append(handler)

    def publish(self, event: DomainEvent) -> None:
        for handler in self._handlers.get(type(event), []):
            try:
                handler(event)
            except Exception as exc:  # 핸들러 오류가 publish를 막지 않도록
                print(f"[EventBus] handler error for {type(event).__name__}: {exc}")

    def clear(self) -> None:
        self._handlers.clear()
