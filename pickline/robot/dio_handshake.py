"""로봇 제어 — SmartDI/SmartDO 핸드셰이크 (PLC/CC-Link 인터록).

PLC와 로봇이 디지털 I/O로 작업을 주고받는 인터록 사이클을 구현한다:
- DI8 rising edge: "부품 도착" 신호 감지
- DI9/DI10: 주문 클래스 비트 (8pin/12pin)
- DO: BUSY / GOOD / REJECT / DONE / ERROR 출력

각 Indy 컨트롤러는 자기 SmartDI/SmartDO 네임스페이스를 가지므로
로봇별로 같은 인덱스를 재사용할 수 있다.

Intel5 robot_dio_handshake.py 에서 추출. 변경점:
- step27_file_ipc → pickline.infra.file_ipc
- print() → 주입 가능 logger
- DI/DO 인덱스·PLC 비트명은 모두 cfg dict로 주입 (프로젝트별 설정)
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

from pickline.infra.file_ipc import atomic_write_json, utc_ts

_log = logging.getLogger("pickline.robot.dio")

CLASS_NAMES = {0: "8pin", 1: "12pin"}


class DioOrderError(RuntimeError):
    """DI8 start 시 DI9/DI10 주문 비트가 유효하지 않을 때."""


@dataclass(frozen=True)
class DioStartEvent:
    """DI8 rising edge에서 디코드된 작업 시작 이벤트."""
    class_id: int
    class_name: str
    start_di_index: int
    order_8pin_di_index: int
    order_12pin_di_index: int
    raw_di8: bool
    raw_di9: bool
    raw_di10: bool


def _cfg_bool(cfg: dict, key: str, default: bool) -> bool:
    val = cfg.get(key, default)
    if isinstance(val, str):
        return val.strip().lower() not in {"0", "false", "no", "off", ""}
    return bool(val)


def _cfg_int(cfg: dict, key: str, default: int) -> int:
    return int(cfg.get(key, default))


def _optional_int(cfg: dict, key: str, default: Optional[int]) -> Optional[int]:
    val = cfg.get(key, default)
    if val is None or val == "":
        return None
    return int(val)


class RobotDioHandshake:
    """설정 기반 SmartDI/SmartDO 인터록 핸드셰이크.

    Args:
        robot: RobotController 구현체 (read_di/set_do/pump_joints 사용).
        cfg: 인덱스/PLC비트명/타이밍 설정 dict. 모든 키는 기본값 있음.
        channel_id: 로그/IPC 식별자 (예: "r1").
    """

    def __init__(self, robot: Any, cfg: dict, channel_id: str = "r?",
                 logger: Optional[logging.Logger] = None):
        self.robot = robot
        self.cfg = cfg
        self.channel_id = channel_id
        self.log = logger or _log
        self.enabled = _cfg_bool(cfg, "plc_dio_cycle_enabled", False)

        # DI 인덱스
        self.start_di_index = _cfg_int(cfg, "plc_start_di_index", 8)
        self.order_8pin_di_index = _cfg_int(cfg, "plc_order_8pin_di_index", 9)
        self.order_12pin_di_index = _cfg_int(cfg, "plc_order_12pin_di_index", 10)
        self.order_required = _cfg_bool(cfg, "plc_order_required", True)
        self.default_class_id = int(
            cfg.get("plc_default_class_id", cfg.get("prefer_class", 0)) or 0)
        if self.default_class_id not in CLASS_NAMES:
            self.default_class_id = 0

        # DO 인덱스
        self.good_do_index = _optional_int(cfg, "plc_good_do_index", 10)
        self.reject_do_index = _optional_int(cfg, "plc_reject_do_index", 11)
        self.done_do_index = _optional_int(cfg, "plc_done_do_index", 13)
        self.busy_do_index = _optional_int(cfg, "plc_busy_do_index", 14)
        self.error_do_index = _optional_int(cfg, "plc_error_do_index", 15)

        # 타이밍
        self.done_pulse_s = float(cfg.get("plc_done_pulse_s", 1.0) or 1.0)
        self.result_pulse_s = float(cfg.get("plc_result_pulse_s", self.done_pulse_s)
                                    or self.done_pulse_s)
        self.error_pulse_s = float(cfg.get("plc_error_pulse_s", self.done_pulse_s)
                                   or self.done_pulse_s)
        self.poll_s = float(cfg.get("plc_dio_poll_s", 0.1) or 0.1)
        self.start_debounce_s = float(cfg.get("plc_start_debounce_s", 0.05) or 0.05)
        self.clear_outputs_on_start = _cfg_bool(cfg, "plc_clear_outputs_on_start", True)
        self.skip_duplicate_do_writes = _cfg_bool(cfg, "plc_skip_duplicate_do_writes", True)
        self.result_latch_until_next_start = _cfg_bool(
            cfg, "plc_result_latch_until_next_start", True)
        self.error_latch = _cfg_bool(cfg, "plc_error_latch", True)
        self.emit_good_bit = _cfg_bool(cfg, "plc_emit_good_bit", True)
        self._do_state_cache: dict[int, bool] = {}

        # PLC 비트명 (로그/매핑 표시용)
        self.start_plc_bit = str(cfg.get("plc_start_plc_bit", "Y160"))
        self.good_plc_bit = str(cfg.get("plc_good_plc_bit", "X142"))
        self.reject_plc_bit = str(cfg.get("plc_reject_plc_bit", "X143"))
        self.done_plc_bit = str(cfg.get("plc_done_plc_bit", "X145"))
        self.busy_plc_bit = str(cfg.get("plc_busy_plc_bit", "X146"))
        self.error_plc_bit = str(cfg.get("plc_error_plc_bit", "X147"))

        self._last_start: Optional[bool] = None
        self.live_di_emit_s = float(cfg.get("plc_live_di_emit_s", 0.25) or 0.25)
        self._last_live_di_emit = 0.0
        self._dio_snapshot: dict[str, Any] = {
            "di8": None, "di9": None, "di10": None,
            "do10_state": None, "do11_state": None, "do13_state": None,
            "do14_state": None, "do15_state": None,
        }

        # 디지털 트윈용 IPC 미러 (best-effort, cfg에 file_ipc_dir 있을 때만).
        self._dio_state_path = None
        self._joint_state_path = None
        self._joint_seq = 0
        ipc_dir = cfg.get("file_ipc_dir")
        if ipc_dir:
            self._dio_state_path = Path(ipc_dir) / f"{channel_id}_dio_state.json"
            self._joint_state_path = Path(ipc_dir) / f"{channel_id}_joint_state.json"
            if robot is not None:
                try:
                    robot._joint_sink = self._emit_joints
                except Exception:
                    pass

    # --- IPC 미러 (best-effort, never raises into robot cycle) ---
    def _emit_joints(self, joints_rad: Any) -> None:
        if self._joint_state_path is None:
            return
        try:
            self._joint_seq += 1
            atomic_write_json(self._joint_state_path, {
                "schema": "intel5.robot_joint_state.v1",
                "seq": self._joint_seq,
                "channel_id": self.channel_id,
                "updated_at": utc_ts(),
                "joints_rad": [float(v) for v in list(joints_rad)[:6]],
            })
        except Exception as exc:
            self.log.debug("joint emit skipped: %r", exc)

    def _emit(self, event: str, **fields: Any) -> None:
        if self._dio_state_path is None:
            return
        try:
            for key in ("di8", "di9", "di10"):
                if key in fields:
                    self._dio_snapshot[key] = bool(fields[key])
            ts = utc_ts()
            payload = {
                "schema": "intel5.robot_dio_state.v1",
                "seq": int(ts * 1000),
                "channel_id": self.channel_id,
                "event": event,
                "updated_at": ts,
            }
            payload.update(self._dio_snapshot)
            payload.update(fields)
            atomic_write_json(self._dio_state_path, payload)
        except Exception as exc:
            self.log.debug("dio_state emit skipped: %r", exc)

    def _remember_do(self, index: int, state: bool) -> None:
        mapping = {
            self.good_do_index: "do10_state", self.reject_do_index: "do11_state",
            self.done_do_index: "do13_state", self.busy_do_index: "do14_state",
            self.error_do_index: "do15_state",
        }
        key = mapping.get(int(index))
        if key:
            self._dio_snapshot[key] = bool(state)

    def mapping_text(self) -> str:
        return (f"{self.start_plc_bit}->DI{self.start_di_index:02d} start, "
                f"DO{self.good_do_index}->{self.good_plc_bit} good, "
                f"DO{self.reject_do_index}->{self.reject_plc_bit} reject, "
                f"DO{self.done_do_index}->{self.done_plc_bit} done, "
                f"order={'required' if self.order_required else 'optional'}")

    # --- DI/DO 저수준 ---
    def _read_di(self, index: int) -> bool:
        return bool(self.robot.read_di(int(index)))

    def _set_do(self, index: Optional[int], state: bool, label: str) -> None:
        if index is None:
            return
        sb = bool(state)
        idx = int(index)
        if self.skip_duplicate_do_writes and self._do_state_cache.get(idx) == sb:
            return
        self.log.info("[%s] SmartDO_%02d %s (%s)",
                      self.channel_id, idx, "ON" if sb else "OFF", label)
        self.robot.set_do(idx, sb)
        self._do_state_cache[idx] = sb
        self._remember_do(idx, sb)
        self._emit("do_state", do_index=idx, do_state=sb, label=str(label))

    def _hold_then_release_do(self, index: Optional[int], hold_s: float,
                              label: str) -> None:
        if index is None:
            return
        hold_s = max(0.0, float(hold_s))
        idx = int(index)
        self.robot.set_do(idx, True)
        self._do_state_cache[idx] = True
        self._remember_do(idx, True)
        self._emit("do_state", do_index=idx, do_state=True, hold_s=hold_s, label=str(label))
        time.sleep(hold_s)
        self.robot.set_do(idx, False)
        self._do_state_cache[idx] = False
        self._remember_do(idx, False)
        self._emit("do_state", do_index=idx, do_state=False, hold_s=hold_s, label=str(label))

    # --- 시작 이벤트 ---
    def poll_start_event(self) -> Optional[DioStartEvent]:
        """DI8 rising edge에서만 이벤트 반환. 첫 호출은 엣지 검출기 prime용."""
        try:
            self.robot.pump_joints()
        except Exception:
            pass
        start_on = self._read_di(self.start_di_index)
        now = time.time()
        if now - self._last_live_di_emit >= self.live_di_emit_s:
            try:
                self._emit("di_snapshot", di8=start_on,
                           di9=self._read_di(self.order_8pin_di_index),
                           di10=self._read_di(self.order_12pin_di_index))
                self._last_live_di_emit = now
            except Exception:
                pass
        if self._last_start is None:
            self._last_start = start_on
            return None
        rising = start_on and not self._last_start
        self._last_start = start_on
        if not rising:
            return None
        if self.start_debounce_s > 0:
            time.sleep(self.start_debounce_s)
        ev = self.read_order_snapshot()
        self._emit("start", class_id=ev.class_id, class_name=ev.class_name,
                   di8=ev.raw_di8, di9=ev.raw_di9, di10=ev.raw_di10)
        return ev

    def read_order_snapshot(self) -> DioStartEvent:
        """현재 DI8/9/10을 읽어 주문 클래스를 디코드."""
        di8 = self._read_di(self.start_di_index)
        di9 = self._read_di(self.order_8pin_di_index)
        di10 = self._read_di(self.order_12pin_di_index)
        if di9 and not di10:
            class_id = 0
        elif di10 and not di9:
            class_id = 1
        elif not self.order_required:
            class_id = self.default_class_id
        elif di9 and di10:
            raise DioOrderError("invalid order: DI9=1 and DI10=1 both ON")
        else:
            raise DioOrderError("invalid order: DI9=0 and DI10=0 both OFF")
        return DioStartEvent(
            class_id=class_id, class_name=CLASS_NAMES[class_id],
            start_di_index=self.start_di_index,
            order_8pin_di_index=self.order_8pin_di_index,
            order_12pin_di_index=self.order_12pin_di_index,
            raw_di8=di8, raw_di9=di9, raw_di10=di10)

    # --- 출력 제어 ---
    def clear_cycle_outputs(self) -> None:
        self._set_do(self.good_do_index, False, "GOOD clear")
        self._set_do(self.reject_do_index, False, "REJECT clear")
        self._set_do(self.error_do_index, False, "ERROR clear")

    def clear_idle_outputs(self, source: str = "") -> None:
        for idx, label in ((self.good_do_index, "GOOD"), (self.reject_do_index, "REJECT"),
                           (self.done_do_index, "DONE"), (self.busy_do_index, "BUSY"),
                           (self.error_do_index, "ERROR")):
            self._set_do(idx, False, f"{label} clear {source}".strip())
        self._emit("clear_outputs", source=str(source))

    def clear_result_outputs(self, source: str = "") -> None:
        self._set_do(self.good_do_index, False, f"GOOD clear {source}".strip())
        self._set_do(self.reject_do_index, False, f"REJECT clear {source}".strip())

    def set_result_outputs(self, result: str, source: str = "") -> None:
        result = (result or "").upper()
        if result == "REJECT":
            self._set_do(self.good_do_index, False, "GOOD clear")
            self._set_do(self.reject_do_index, True, "REJECT")
        elif result == "GOOD":
            self._set_do(self.reject_do_index, False, "REJECT clear")
            if self.emit_good_bit:
                self._set_do(self.good_do_index, True, "GOOD")
        else:
            raise ValueError(f"unsupported result: {result!r}")

    def begin_cycle(self) -> None:
        if self.clear_outputs_on_start:
            self.clear_cycle_outputs()
        self._set_do(self.busy_do_index, True, "BUSY")
        self._emit("busy")

    def complete_cycle(self, result: str = "GOOD") -> None:
        result = (result or "GOOD").upper()
        result_index = self.reject_do_index if result == "REJECT" else self.good_do_index
        if self.result_latch_until_next_start:
            self.set_result_outputs(result, source="cycle")
        elif result == "REJECT" or self.emit_good_bit:
            self._hold_then_release_do(result_index, self.result_pulse_s, result)
        self._hold_then_release_do(self.done_do_index, self.done_pulse_s, "DONE")
        self._set_do(self.busy_do_index, False, "BUSY")
        self._emit("done", result=result)

    def fail_cycle(self, error: str = "") -> None:
        self._set_do(self.busy_do_index, False, "BUSY")
        if self.error_latch:
            self._set_do(self.error_do_index, True, f"ERROR {error}")
        else:
            self._hold_then_release_do(self.error_do_index, self.error_pulse_s,
                                       f"ERROR {error}")
        self._emit("error", error=str(error))
