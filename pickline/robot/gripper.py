"""로봇 제어 — 그리퍼/진공 추상화.

`Gripper` 포트 + `VacuumGripper` 구현체.
진공 흡착툴을 3가지 백엔드로 제어한다 (전략 패턴):
- plc              : Mitsubishi PLC 코일 (pymcprotocol MC Protocol)
- robot_do         : 로봇 Smart DO
- robot_endtool_do : 로봇 엔드툴 DO

Intel5 step10_vacuum.py 에서 추출. 진단용 CLI(--diag/--sweep)는 제외.

다른 그리퍼(평행 그리퍼 등)를 쓰려면 같은 `Gripper` 포트를 구현하면 된다.
"""

from __future__ import annotations

import logging
import time
from typing import Optional, Protocol, runtime_checkable

from pickline.shared.errors import RobotError

_log = logging.getLogger("pickline.robot.gripper")


@runtime_checkable
class Gripper(Protocol):
    """그리퍼 포트. 픽 사이클은 이 인터페이스에만 의존한다."""

    def engage(self) -> None:
        """물체를 잡는다 (진공 ON / 그리퍼 닫기)."""
        ...

    def release(self) -> None:
        """물체를 놓는다 (진공 OFF / 그리퍼 열기)."""
        ...

    def verify_grip(self, settle_ms: int = 200, timeout_ms: int = 800) -> bool:
        """잡기 성공 여부 (센서 확인). 성공 시 True."""
        ...

    def disconnect(self) -> None: ...


class VacuumGripper:
    """진공 흡착 그리퍼. Gripper 포트 구현체.

    Args:
        plc_ip/port: PLC 백엔드용 주소.
        vacuum_bit: 진공 솔레노이드 코일 (예: "M100", word 디바이스 "D2"도 가능).
        sensor_bit: 진공 압력 센서 입력 (예: "X10").
        backend: 'plc' | 'robot_do' | 'robot_endtool_do'.
        sensor_backend: 'plc' | 'none'. None이면 backend에 따라 자동.
        robot: robot_* 백엔드일 때 RobotController 구현체 주입.
        dry_run: 하드웨어 없이 동작.
    """

    def __init__(self, plc_ip: str = "192.168.0.20", port: int = 5007,
                 vacuum_bit: str = "M100", sensor_bit: str = "X10",
                 backend: str = "plc", sensor_backend: Optional[str] = None,
                 robot=None, robot_do_index: int = 2,
                 robot_endtool_do_index: int = 0, dry_run: bool = False,
                 logger: Optional[logging.Logger] = None):
        self.plc_ip = plc_ip
        self.port = port
        self.vacuum_bit = vacuum_bit
        self.sensor_bit = sensor_bit
        self.backend = (backend or "plc").replace("-", "_").lower()
        self.sensor_backend = (
            sensor_backend.replace("-", "_").lower() if sensor_backend
            else ("plc" if self.backend == "plc" else "none"))
        self.robot = robot
        self.robot_do_index = int(robot_do_index)
        self.robot_endtool_do_index = int(robot_endtool_do_index)
        self.dry_run = dry_run
        self.log = logger or _log
        self.plc = None
        self._dry_state_on = False
        self._commanded_on = False

        if self.backend not in ("plc", "robot_do", "robot_endtool_do"):
            raise ValueError(f"알 수 없는 vacuum backend: {backend}")
        needs_plc = self.backend == "plc" or self.sensor_backend == "plc"
        if not dry_run and needs_plc:
            self._connect()
        if not dry_run and self.backend.startswith("robot") and self.robot is None:
            raise ValueError("robot_* 백엔드는 robot=RobotController 주입 필요")

    def _connect(self) -> None:
        try:
            import pymcprotocol
        except ImportError as exc:
            raise RobotError("pymcprotocol 미설치 — pip install pickline[plc]") from exc
        try:
            self.plc = pymcprotocol.Type3E()
            self.plc.connect(self.plc_ip, self.port)
            self.log.info("PLC 연결 OK (%s:%s)", self.plc_ip, self.port)
        except Exception as exc:
            raise RobotError(f"PLC 연결 실패: {exc}") from exc

    @staticmethod
    def _is_word_device(name: str) -> bool:
        """D, W, R, ZR 등은 word register — bit write 불가."""
        return bool(name) and name.upper().startswith(("D", "W", "R", "ZR"))

    def _write_plc_vacuum(self, value: int) -> None:
        try:
            if self._is_word_device(self.vacuum_bit):
                self.plc.batchwrite_wordunits(headdevice=self.vacuum_bit, values=[value])
            else:
                self.plc.batchwrite_bitunits(headdevice=self.vacuum_bit, values=[value])
        except Exception as exc:
            raise RobotError(
                f"PLC vacuum write 실패 ({self.vacuum_bit}<-{value}): {exc}") from exc

    # --- Gripper 포트 ---
    def engage(self) -> None:
        """진공 ON."""
        if self.dry_run:
            self._dry_state_on = True
            self._commanded_on = True
            return
        if self.backend == "plc":
            self._write_plc_vacuum(1)
        elif self.backend == "robot_do":
            self.robot.set_do(self.robot_do_index, True)
        elif self.backend == "robot_endtool_do":
            self.robot.set_endtool_do(self.robot_endtool_do_index, True)
        self._commanded_on = True

    def release(self) -> None:
        """진공 OFF."""
        if self.dry_run:
            self._dry_state_on = False
            self._commanded_on = False
            return
        if self.backend == "plc":
            self._write_plc_vacuum(0)
        elif self.backend == "robot_do":
            self.robot.set_do(self.robot_do_index, False)
        elif self.backend == "robot_endtool_do":
            self.robot.set_endtool_do(self.robot_endtool_do_index, False)
        self._commanded_on = False

    def read_sensor(self) -> bool:
        """진공 압력 센서 ON 여부. True면 흡착 성공(충분한 진공압)."""
        if self.dry_run:
            return self._dry_state_on
        if self.sensor_backend == "none":
            return self._commanded_on
        if self.sensor_backend == "plc":
            try:
                bits = self.plc.batchread_bitunits(headdevice=self.sensor_bit, readsize=1)
                return bits[0] == 1
            except Exception as exc:
                raise RobotError(f"PLC vacuum sensor read 실패: {exc}") from exc
        raise ValueError(f"알 수 없는 sensor backend: {self.sensor_backend}")

    def verify_grip(self, settle_ms: int = 200, poll_hz: int = 50,
                    timeout_ms: int = 800) -> bool:
        """진공 ON 후 settle_ms 대기 → 센서 확인. timeout_ms까지 추가 폴링."""
        time.sleep(settle_ms / 1000.0)
        if self.sensor_backend == "none":
            return self._commanded_on
        if self.read_sensor():
            return True
        t0 = time.time()
        dt = 1.0 / poll_hz
        while (time.time() - t0) * 1000 < timeout_ms:
            if self.read_sensor():
                return True
            time.sleep(dt)
        return False

    def drop(self, blow_ms: int = 50) -> None:
        """진공 OFF + 짧은 블로우 대기."""
        self.release()
        if blow_ms > 0:
            time.sleep(blow_ms / 1000.0)

    def is_engaged(self) -> bool:
        return self._commanded_on

    def disconnect(self) -> None:
        if self.dry_run:
            return
        try:
            self.release()
        except Exception:
            pass
        if self.plc is not None:
            try:
                self.plc.close()
            except Exception:
                pass
