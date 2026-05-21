"""픽킹 — 파일 IPC 워커 베이스 클래스.

비전 GUI/PLC 브리지가 JSON 명령 파일을 쓰면, 로봇 워커 프로세스가 그걸
폴링해 사이클을 실행하고 상태를 JSON으로 되쓴다. 카메라/YOLO 코드와 로봇
소켓 제어를 프로세스 단위로 분리하는 구조.

Intel5의 step27_robot_file_worker.py / step37_robot1_file_worker.py는
이 스캐폴딩(설정 로드·IPC 경로·상태 루프·명령 디스패치·staleness 체크·
생명주기)을 ~2,000줄 복붙으로 공유했다. 여기로 통합한다.

서브클래스는 `setup()`(하드웨어 연결)과 `execute_cycle(command)`(픽 사이클
본문)만 구현하면 된다. 그 외 루프/상태/명령 라우팅은 베이스가 처리한다.
"""

from __future__ import annotations

import logging
import threading
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Optional

from pickline.infra.file_ipc import atomic_write_json, max_known_seq, read_json, utc_ts

_log = logging.getLogger("pickline.picking.worker")


class FileIpcWorker(ABC):
    """JSON 파일 IPC 기반 로봇 워커의 추상 베이스.

    서브클래스 필수 구현:
        setup()                : 하드웨어(로봇/그리퍼) 연결
        execute_cycle(command) : 명령 1건의 픽 사이클 본문
    선택 오버라이드:
        teardown()             : 정리 (기본 no-op)
        on_emergency_stop(cmd) : 비상정지 처리
        poll_extra()           : 매 루프 추가 폴링 (예: PLC-DIO)
        status_extra()         : 상태 JSON에 넣을 추가 필드 dict

    클래스 속성:
        WORKER_NAME    : 상태 JSON의 worker 식별자
        STATUS_SCHEMA  : 상태 JSON schema 문자열
        CYCLE_COMMANDS : execute_cycle를 트리거하는 명령 이름 집합
    """

    WORKER_NAME = "file_ipc_worker"
    STATUS_SCHEMA = "intel5.robot_status.v1"
    CYCLE_COMMANDS: set[str] = set()

    def __init__(self, channel_id: str, ipc_dir: str | Path, *,
                 command_file: str = "robot_command.json",
                 status_file: str = "robot_status.json",
                 poll_s: float = 0.1, command_max_age_s: float = 5.0,
                 accept_file_commands: bool = True,
                 logger: Optional[logging.Logger] = None):
        self.channel_id = channel_id
        self.ipc_dir = Path(ipc_dir)
        self.command_path = self.ipc_dir / command_file
        self.status_path = self.ipc_dir / status_file
        self.poll_s = float(poll_s)
        self.command_max_age_s = float(command_max_age_s)
        self.accept_file_commands = bool(accept_file_commands)
        self.log = logger or _log

        self.started_at = utc_ts()
        prev_status = read_json(self.status_path, {}) or {}
        prev_command = (read_json(self.command_path, {}) or {}
                        if self.accept_file_commands else {})
        self.last_processed_seq = max_known_seq(prev_status, prev_command)
        self.current_seq: Optional[int] = None
        self.current_state = "STARTING"
        self.current_step = "initializing"
        self.last_error = ""
        self.stop_event = threading.Event()
        self.worker_thread: Optional[threading.Thread] = None

    # ----- 서브클래스 훅 -----
    @abstractmethod
    def setup(self) -> None:
        """하드웨어(로봇/그리퍼 등) 연결. loop() 시작 시 1회 호출."""

    @abstractmethod
    def execute_cycle(self, command: dict) -> None:
        """CYCLE_COMMANDS에 속한 명령 1건을 실행. 성공 시 정상 반환,
        실패 시 예외를 던지면 베이스가 ERROR 상태로 기록한다."""

    def teardown(self) -> None:
        """정리 (연결 해제 등). 기본 no-op."""

    def on_emergency_stop(self, command: dict) -> None:
        """비상정지 명령 처리. 기본 no-op (서브클래스가 로봇 정지 구현)."""

    def poll_extra(self) -> None:
        """매 루프 추가 폴링 (예: PLC-DIO 시작 이벤트). 기본 no-op."""

    def status_extra(self) -> dict:
        """상태 JSON에 병합할 추가 필드. 기본 빈 dict."""
        return {}

    # ----- 상태 -----
    def write_status(self, state: str, step: str,
                     command: Optional[dict] = None, error: str = "") -> None:
        self.current_state = state
        self.current_step = step
        self.last_error = error
        data = {
            "schema": self.STATUS_SCHEMA,
            "channel_id": self.channel_id,
            "worker": self.WORKER_NAME,
            "started_at": self.started_at,
            "state": state,
            "step": step,
            "last_processed_seq": self.last_processed_seq,
            "current_seq": self.current_seq,
            "updated_at": utc_ts(),
            "error": error,
        }
        data.update(self.status_extra())
        if command:
            data["command"] = {k: command.get(k) for k in
                                ("seq", "command", "source", "decision")}
        atomic_write_json(self.status_path, data)

    # ----- 명령 처리 -----
    def is_stale_command(self, command: dict) -> tuple[bool, str]:
        """명령이 만료(expires_at) 또는 노후(created_at age)됐는지."""
        now = time.time()
        try:
            expires_at = float(command.get("expires_at") or 0.0)
        except (TypeError, ValueError):
            expires_at = 0.0
        if expires_at > 0.0 and now > expires_at:
            return True, f"expired ({now - expires_at:.1f}s past expires_at)"
        try:
            created_at = float(command.get("created_at") or 0.0)
        except (TypeError, ValueError):
            created_at = 0.0
        if created_at > 0.0 and now - created_at > self.command_max_age_s:
            return True, (f"stale age={now - created_at:.1f}s "
                          f"> {self.command_max_age_s:.1f}s")
        return False, ""

    def handle_command(self, command: Any) -> None:
        """명령 JSON 1건을 디스패치. seq 중복/노후 명령은 무시."""
        if not isinstance(command, dict):
            return
        try:
            seq = int(command.get("seq") or 0)
        except (TypeError, ValueError):
            return
        if seq <= self.last_processed_seq:
            return

        cmd = command.get("command")
        try:
            if cmd == "emergency_stop":
                stale, reason = self.is_stale_command(command)
                if stale:
                    self._mark_ignored(seq, reason, command)
                    return
                self.current_seq = seq
                self.write_status("STOPPING", "emergency stop", command)
                self.on_emergency_stop(command)
                self.last_processed_seq = max(self.last_processed_seq, seq)
                self.write_status("STOPPED", "emergency stop handled", command)
                self.current_seq = None
            elif cmd in self.CYCLE_COMMANDS:
                stale, reason = self.is_stale_command(command)
                if stale:
                    self._mark_ignored(seq, reason, command)
                    return
                self.current_seq = seq
                self.execute_cycle(command)
                self.last_processed_seq = seq
                self.write_status("DONE", f"{cmd} complete", command)
                self.current_seq = None
            else:
                self._mark_ignored(seq, f"unknown command: {cmd}", command)
        except Exception as exc:
            self.last_processed_seq = seq
            self.write_status("ERROR", str(exc), command, error=str(exc))
            self.log.error("[%s] seq=%d ERROR: %s", self.channel_id, seq, exc)
            self.current_seq = None

    def _mark_ignored(self, seq: int, reason: str, command: dict) -> None:
        self.current_seq = seq
        self.last_processed_seq = seq
        self.write_status("IGNORED", reason, command, error=reason)
        self.log.info("[%s] seq=%d IGNORED: %s", self.channel_id, seq, reason)
        self.current_seq = None

    # ----- 생명주기 -----
    def loop(self) -> None:
        """워커 메인 루프. setup → 폴링 → teardown."""
        try:
            self.setup()
        except Exception as exc:
            self.write_status("ERROR", "setup failed", error=str(exc))
            self.log.error("[%s] setup 실패: %s", self.channel_id, exc)
            return
        self.write_status("READY", "waiting command")
        heartbeat_t = 0.0
        while not self.stop_event.is_set():
            if self.accept_file_commands:
                self.handle_command(read_json(self.command_path, {}))
            try:
                self.poll_extra()
            except Exception as exc:
                self.log.warning("[%s] poll_extra 오류: %s", self.channel_id, exc)
            if (time.time() - heartbeat_t > 1.0 and self.current_seq is None
                    and self.current_state not in {"ERROR", "STOPPED", "STOPPING"}):
                self.write_status("READY", "waiting command")
                heartbeat_t = time.time()
            self.stop_event.wait(self.poll_s)
        self.shutdown()

    def shutdown(self) -> None:
        self.write_status("STOPPED", "worker stopped")
        try:
            self.teardown()
        except Exception:
            pass

    def start_background(self) -> None:
        """워커를 데몬 스레드로 실행."""
        self.worker_thread = threading.Thread(target=self.loop, daemon=True)
        self.worker_thread.start()

    def request_stop(self) -> None:
        self.stop_event.set()
