"""로봇 제어 — Indy7 컨트롤러 (RobotController 포트 구현체).

neuromeka IndyDCP3 / 레거시 indy_utils 두 SDK를 자동 감지해 추상화한다.
SDK 호출은 모두 이 모듈에 격리 — 상위 계층은 RobotController 포트만 본다.

Intel5 step9_robot.py 에서 추출. 변경점:
- 공개 API가 Pose6D 사용 (get_pose→Pose6D, move_l는 Pose6D|list 모두 허용)
- print() → 주입 가능한 logger
- RuntimeError → 도메인 예외 (RobotConnectionError/RobotMotionError)

단위 규약: 위치 mm, 회전 deg, 힘 N. dry_run=True면 하드웨어 없이 동작.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import List, Optional, Union

from pickline.shared.errors import RobotConnectionError, RobotMotionError
from pickline.shared.value_objects import Pose6D

_log = logging.getLogger("pickline.robot")

PoseLike = Union[Pose6D, List[float]]


def _to_list(pose: PoseLike) -> List[float]:
    return pose.as_list() if isinstance(pose, Pose6D) else list(pose)


class IndyController:
    """Indy7 + IndyDCP3 래퍼. RobotController 포트를 구현한다."""

    # 클래스 기본 안전 한계 (set_safety_limits로 override)
    _z_safe_max_mm: float = 1000.0
    _approach_uvw_deg: Optional[List[float]] = None

    def __init__(self, robot_ip: str = "192.168.0.10", dry_run: bool = False,
                 logger: Optional[logging.Logger] = None):
        self.robot_ip = robot_ip
        self.dry_run = dry_run
        self.log = logger or _log
        self.indy = None
        self._sdk: Optional[str] = None        # "neuromeka" | "indy_utils"
        self._neuromeka_io = None
        self._compliance_on = False
        self._connect_lock = threading.RLock()
        self._dry_pose = [0.0, 0.0, 500.0, 0.0, 180.0, 0.0]
        self._dry_force = [0.0] * 6
        self._dry_joints = [0.0] * 6
        # 디지털 트윈 미러용 관절 옵저버 (단일 스레드에서만 호출)
        self._joint_sink = None
        self._joint_pump_dt = 0.05
        self._joint_pump_last = 0.0
        if not dry_run:
            self._connect()

    # ----- 연결 -----
    def connect(self) -> bool:
        if self.dry_run:
            return True
        if self.indy is None:
            self._connect()
        return self.indy is not None

    def _connect(self) -> None:
        import os
        sdk_pref = os.environ.get("ROBOT_SDK", "auto").lower()

        if sdk_pref in ("auto", "neuromeka"):
            try:
                from neuromeka import IndyDCP3
                self.indy = IndyDCP3(self.robot_ip)
                try:
                    self.indy.get_robot_data()
                    self._sdk = "neuromeka"
                    self.log.info("neuromeka.IndyDCP3 OK (RTDE 활성, %s)", self.robot_ip)
                    return
                except Exception:
                    self.log.warning("neuromeka 연결됐으나 RTDE 비활성 — indy_utils fallback")
                    self.indy = None
            except Exception as exc:
                if sdk_pref == "neuromeka":
                    raise RobotConnectionError(f"neuromeka 강제 모드 실패: {exc}") from exc
                self.log.info("neuromeka 시도 실패: %s", exc)

        try:
            import os as _os
            import sys
            for candidate in (_os.path.expanduser("~/Desktop"),
                              _os.path.expanduser("~"),
                              _os.path.expanduser("~/Documents")):
                pkg = _os.path.join(candidate, "indy_utils", "indydcp_client.py")
                if _os.path.isfile(pkg) and candidate not in sys.path:
                    sys.path.insert(0, candidate)
                    break
            from indy_utils.indydcp_client import IndyDCPClient
            self.indy = IndyDCPClient(self.robot_ip, "NRMK-Indy7")
            if self.indy.connect() is False:
                raise RobotConnectionError(f"indy_utils TCP connect 실패: {self.robot_ip}:6066")
            try:
                self.indy.set_timeout_sec(3)
            except Exception:
                pass
            self.indy.get_robot_status()
            self._sdk = "indy_utils"
            self.log.info("indy_utils.IndyDCPClient OK (%s)", self.robot_ip)
            return
        except RobotConnectionError:
            raise
        except Exception as exc:
            self.log.info("indy_utils 시도 실패: %s", exc)

        raise RobotConnectionError(
            "Indy7 SDK 연결 실패 — neuromeka 또는 indy_utils 설치 확인")

    def reconnect(self, reason: str = "") -> bool:
        if self.dry_run:
            return True
        with self._connect_lock:
            old = self.indy
            if old is not None:
                for fn in ("disconnect", "shutdown"):
                    try:
                        getattr(old, fn)()
                        break
                    except Exception:
                        continue
            self.indy = None
            self._neuromeka_io = None
            self._sdk = None
            if reason:
                self.log.info("reconnect: %s", reason)
            self._connect()
        return True

    @staticmethod
    def _is_transport_error(exc: Exception) -> bool:
        return isinstance(exc, (BrokenPipeError, TimeoutError, OSError, ConnectionError))

    def _reconnect_after_transport_error(self, exc: Exception, op: str) -> bool:
        if not self._is_transport_error(exc):
            return False
        self.reconnect(f"{op} failed: {exc}")
        return True

    # ----- 상태 조회 -----
    def get_pose(self) -> Pose6D:
        """현재 EE pose. 단위 mm + deg로 통일."""
        if self.dry_run:
            return Pose6D.from_list(self._dry_pose)
        if self._sdk == "neuromeka":
            try:
                data = self.indy.get_robot_data()
                p = list(data.p) if hasattr(data, "p") else None
                if p and len(p) >= 6:
                    return Pose6D(p[0] * 1000, p[1] * 1000, p[2] * 1000,
                                  math.degrees(p[3]), math.degrees(p[4]),
                                  math.degrees(p[5]))
            except Exception as exc:
                self.log.warning("neuromeka pose 읽기 실패: %s", exc)
            return Pose6D(0, 0, 0)
        if self._sdk == "indy_utils":
            try:
                raw = self.indy.get_task_pos()
            except Exception as exc:
                if self._reconnect_after_transport_error(exc, "get_task_pos"):
                    raw = self.indy.get_task_pos()
                else:
                    raise
            return Pose6D(raw[0] * 1000, raw[1] * 1000, raw[2] * 1000,
                          raw[3], raw[4], raw[5])
        raise RobotConnectionError("SDK 미연결")

    def get_joint_pos(self) -> List[float]:
        """관절각 6개 (radian)."""
        if self.dry_run:
            return list(self._dry_joints)
        if self._sdk == "indy_utils":
            try:
                raw = self.indy.get_joint_pos()
            except Exception as exc:
                if self._reconnect_after_transport_error(exc, "get_joint_pos"):
                    raw = self.indy.get_joint_pos()
                else:
                    raise
            return [math.radians(float(v)) for v in list(raw)[:6]]
        if self._sdk == "neuromeka":
            try:
                data = self.indy.get_robot_data()
                q = list(data.q) if hasattr(data, "q") else None
                if q and len(q) >= 6:
                    return [float(v) for v in q[:6]]
            except Exception as exc:
                self.log.warning("neuromeka joint 읽기 실패: %s", exc)
        return [0.0] * 6

    def get_force(self) -> List[float]:
        """F/T 센서 [Fx,Fy,Fz,Tx,Ty,Tz]. API 없으면 0 반환."""
        if self.dry_run:
            return list(self._dry_force)
        names = ("get_ft_sensor_data", "get_ft_sensor", "get_force_sensor")
        for fn in names:
            try:
                return list(getattr(self.indy, fn)())
            except Exception:
                continue
        self.log.warning("SDK에 F/T 함수 없음 — 0 반환 (force-guided pick 불가)")
        return [0.0] * 6

    # ----- 설정 -----
    def set_safety_limits(self, z_safe_max_mm: Optional[float] = None,
                          approach_uvw_deg: Optional[List[float]] = None) -> None:
        if z_safe_max_mm is not None:
            self._z_safe_max_mm = float(z_safe_max_mm)
            self.log.info("z_safe_max_mm = %s", self._z_safe_max_mm)
        if approach_uvw_deg is not None:
            self._approach_uvw_deg = list(approach_uvw_deg)
            self.log.info("approach_uvw_deg fixed = %s", self._approach_uvw_deg)

    def set_tcp(self, tcp_mm_deg: List[float]) -> bool:
        if tcp_mm_deg is None or len(tcp_mm_deg) < 6:
            self.log.warning("set_tcp: invalid tcp %s — skipped", tcp_mm_deg)
            return False
        tcp = [float(v) for v in tcp_mm_deg[:6]]
        if self.dry_run:
            return True
        if self._sdk == "indy_utils":
            tcp_m = [tcp[0] / 1000, tcp[1] / 1000, tcp[2] / 1000, tcp[3], tcp[4], tcp[5]]
            try:
                self.indy.set_default_tcp(tcp_m)
            except Exception as exc:
                if self._reconnect_after_transport_error(exc, "set_default_tcp"):
                    self.indy.set_default_tcp(tcp_m)
                else:
                    raise
            return True
        if self._sdk == "neuromeka":
            for fn in ("set_tool_frame", "set_default_tcp", "set_tcp"):
                try:
                    getattr(self.indy, fn)(tcp)
                    return True
                except AttributeError:
                    continue
                except Exception as exc:
                    self.log.warning("neuromeka %s 실패: %s", fn, exc)
                    return False
        return False

    def _check_safety(self, pose: List[float], op: str) -> bool:
        if pose[2] > self._z_safe_max_mm:
            self.log.error("SAFETY VIOLATION: %s z=%.1f > z_safe_max=%.1f — REJECTED",
                           op, pose[2], self._z_safe_max_mm)
            return False
        return True

    # ----- 모션 -----
    def move_l(self, target: PoseLike, vel_mmps: float = 30.0, blending: float = 0.0,
               wait: bool = True, timeout_s: float = 30.0,
               skip_safety: bool = False) -> bool:
        """직선 이동. target은 Pose6D 또는 [x,y,z,rx,ry,rz] (mm+deg)."""
        pose = _to_list(target)
        if not skip_safety and not self._check_safety(pose, "move_l"):
            return False
        if self._approach_uvw_deg is not None:
            pose = list(pose[:3]) + list(self._approach_uvw_deg)
        if self.dry_run:
            self._dry_pose = list(pose)
            return True
        if self._sdk == "neuromeka":
            try:
                self.indy.movel(pose, vel=vel_mmps)
            except AttributeError:
                try:
                    self.indy.movetelel_abs(pose, vel=vel_mmps)
                except AttributeError:
                    self.indy.task_move_to(pose)
        elif self._sdk == "indy_utils":
            conv = [pose[0] / 1000, pose[1] / 1000, pose[2] / 1000,
                    pose[3], pose[4], pose[5]]
            self.indy.task_move_to(conv)
        if wait:
            self._wait_motion_done(timeout_s)
        return True

    def _wait_motion_done(self, timeout_s: float = 30.0, poll_hz: int = 20) -> None:
        if self.dry_run:
            return
        t0 = time.time()
        dt = 1.0 / poll_hz
        while time.time() - t0 < timeout_s:
            if self._is_motion_done():
                self._pump_joints()
                return
            self._pump_joints()
            time.sleep(dt)
        raise RobotMotionError(f"move timeout ({timeout_s}s)")

    def _is_motion_done(self) -> bool:
        if self.dry_run:
            return True
        if self._sdk == "neuromeka":
            for fn in ("is_target_reached", "is_in_motion"):
                try:
                    val = getattr(self.indy, fn)()
                    return bool(val) if fn == "is_target_reached" else not bool(val)
                except Exception:
                    continue
        elif self._sdk == "indy_utils":
            try:
                return self.indy.get_robot_status()["movedone"]
            except Exception as exc:
                self.log.warning("robot status read 실패: %s", exc)
                return False
        return False

    def pump_joints(self) -> None:
        """단일 스레드 관절 샘플 → 트윈 sink. best-effort, never raises."""
        self._pump_joints()

    def _pump_joints(self) -> None:
        sink = self._joint_sink
        if sink is None or self.dry_run:
            return
        now = time.time()
        if now - self._joint_pump_last < self._joint_pump_dt:
            return
        self._joint_pump_last = now
        try:
            sink(self.get_joint_pos())
        except Exception:
            pass

    def stop_motion(self) -> None:
        if self.dry_run:
            return
        for fn in ("stop_motion", "stop", "abort_motion"):
            try:
                getattr(self.indy, fn)()
                return
            except Exception:
                continue

    def go_home(self, wait: bool = True, timeout_s: float = 60.0,
                skip_safety: bool = False) -> bool:
        if self.dry_run:
            return True
        if not skip_safety:
            try:
                cur = self.get_pose()
                if cur.z > self._z_safe_max_mm:
                    self.log.warning("go_home: 현재 z=%.1f > %.1f, 위험 가능",
                                     cur.z, self._z_safe_max_mm)
            except Exception:
                pass
        for fn in ("home", "move_home", "go_home", "movej_home"):
            try:
                getattr(self.indy, fn)()
                if wait:
                    self._wait_motion_done(timeout_s)
                return True
            except Exception:
                continue
        raise RobotMotionError(f"{self._sdk} SDK에 home 함수 없음")

    def go_zero(self, wait: bool = True, timeout_s: float = 60.0) -> bool:
        if self.dry_run:
            return True
        for fn in ("zero", "go_zero", "move_zero", "movej_zero"):
            try:
                getattr(self.indy, fn)()
                if wait:
                    self._wait_motion_done(timeout_s)
                return True
            except Exception:
                continue
        raise RobotMotionError(f"{self._sdk} SDK에 zero 함수 없음")

    # ----- 디지털 I/O -----
    def _get_neuromeka_io_client(self):
        if self._neuromeka_io is not None:
            return self._neuromeka_io
        from neuromeka import IndyDCP3
        self._neuromeka_io = IndyDCP3(self.robot_ip)
        return self._neuromeka_io

    def set_do(self, address: int, state: bool) -> bool:
        import os
        address = int(address)
        sb = bool(state)
        si = 1 if sb else 0
        if self.dry_run:
            return True
        errors = []
        io_pref = os.environ.get("ROBOT_IO_SDK", "indy_utils").lower()
        if self._sdk == "indy_utils" and io_pref in ("auto", "indy_utils"):
            try:
                self.indy.set_do(address, si)
                return True
            except Exception as exc:
                if self._reconnect_after_transport_error(exc, "set_do"):
                    self.indy.set_do(address, si)
                    return True
                errors.append(f"indy_utils.set_do: {exc}")
        if io_pref in ("auto", "neuromeka"):
            try:
                self._get_neuromeka_io_client().set_do([(address, sb)])
                return True
            except Exception as exc:
                errors.append(f"neuromeka.io.set_do: {exc}")
        if self._sdk == "neuromeka" and io_pref in ("auto", "neuromeka"):
            try:
                self.indy.set_do([(address, sb)])
                return True
            except Exception as exc:
                errors.append(f"neuromeka.set_do: {exc}")
        raise RobotMotionError("robot DO write 실패 — " + " | ".join(errors))

    def get_di(self) -> List[int]:
        if self.dry_run:
            return [0] * 32
        if self._sdk == "indy_utils":
            try:
                vals = self.indy.get_di()
            except Exception as exc:
                if self._reconnect_after_transport_error(exc, "get_di"):
                    vals = self.indy.get_di()
                else:
                    raise
            return [int(v) for v in list(vals)]
        if self._sdk == "neuromeka":
            for fn in ("get_di", "get_dis", "get_smart_di", "get_smart_dis"):
                try:
                    return [int(v) for v in list(getattr(self.indy, fn)())]
                except Exception:
                    continue
        raise RobotMotionError("robot DI read 실패")

    def get_do(self) -> List[int]:
        if self.dry_run:
            return [0] * 32
        if self._sdk == "indy_utils":
            try:
                vals = self.indy.get_do()
            except Exception as exc:
                if self._reconnect_after_transport_error(exc, "get_do"):
                    vals = self.indy.get_do()
                else:
                    raise
            return [int(v) for v in list(vals)]
        if self._sdk == "neuromeka":
            for fn in ("get_do", "get_dos", "get_smart_do", "get_smart_dos"):
                try:
                    return [int(v) for v in list(getattr(self.indy, fn)())]
                except Exception:
                    continue
        raise RobotMotionError("robot DO read 실패")

    def read_di(self, address: int) -> bool:
        vals = self.get_di()
        address = int(address)
        if address < 0 or address >= len(vals):
            raise IndexError(f"SmartDI index out of range: {address}")
        return bool(vals[address])

    def pulse_do(self, address: int, pulse_s: float = 1.0) -> None:
        self.set_do(address, True)
        time.sleep(max(0.0, float(pulse_s)))
        self.set_do(address, False)

    def set_endtool_do(self, index: int, state: bool) -> bool:
        import os
        index = int(index)
        sb = bool(state)
        si = 1 if sb else 0
        if self.dry_run:
            return True
        errors = []
        io_pref = os.environ.get("ROBOT_IO_SDK", "indy_utils").lower()
        if self._sdk == "indy_utils" and io_pref in ("auto", "indy_utils"):
            try:
                self.indy.set_endtool_do(index, si)
                return True
            except Exception as exc:
                errors.append(f"indy_utils.set_endtool_do: {exc}")
        for use_io in (True, False):
            if use_io and io_pref not in ("auto", "neuromeka"):
                continue
            if not use_io and not (self._sdk == "neuromeka" and io_pref in ("auto", "neuromeka")):
                continue
            try:
                states = [False] * 8
                states[index] = sb
                client = self._get_neuromeka_io_client() if use_io else self.indy
                client.set_endtool_do([("0", states)])
                return True
            except Exception as exc:
                tag = "neuromeka.io" if use_io else "neuromeka"
                errors.append(f"{tag}.set_endtool_do: {exc}")
        raise RobotMotionError("robot endtool DO write 실패 — " + " | ".join(errors))

    # ----- 컴플라이언스 / 힘 제어 -----
    def set_compliance(self, stiffness: Optional[List[float]] = None,
                       damping: Optional[List[float]] = None) -> None:
        K = stiffness or [2000, 2000, 300, 10, 10, 80]
        D = damping or [2 * math.sqrt(max(k, 1e-6)) for k in K]
        if self.dry_run:
            self._compliance_on = True
            return
        if self._sdk == "neuromeka":
            for activate in ("set_task_compliance_ctrl", "enable_compliance_ctrl"):
                try:
                    getattr(self.indy, activate)(True)
                    break
                except Exception:
                    continue
            for setparam in ("set_task_compliance_param", "set_compliance"):
                try:
                    getattr(self.indy, setparam)(K, D)
                    break
                except Exception:
                    continue
        elif self._sdk == "indy_utils":
            self.indy.set_task_compliance_param(K)
            self.indy.set_task_compliance_ctrl(True)
        self._compliance_on = True

    def disable_compliance(self) -> None:
        if self.dry_run:
            self._compliance_on = False
            return
        if self._sdk == "neuromeka":
            for fn in ("set_task_compliance_ctrl", "disable_compliance_ctrl"):
                try:
                    getattr(self.indy, fn)(False)
                    break
                except Exception:
                    continue
        elif self._sdk == "indy_utils":
            self.indy.set_task_compliance_ctrl(False)
        self._compliance_on = False

    def force_guided_descent(self, target: PoseLike, axis: str = "z",
                             threshold_n: float = 5.0, vel_mmps: float = 5.0,
                             timeout_s: float = 8.0, poll_hz: int = 50,
                             max_safety_n: float = 30.0) -> bool:
        """force가 threshold_n에 닿을 때까지 천천히 하강. 접촉 시 True."""
        axis_idx = {"x": 0, "y": 1, "z": 2}[axis]
        if not self._compliance_on:
            self.log.warning("force_guided_descent: 컴플라이언스 OFF 상태 — 권장 안 함")
        pose = _to_list(target)
        if self.dry_run:
            time.sleep(0.1)
            self._dry_pose = list(pose)
            self._dry_force = [0, 0, threshold_n + 0.5, 0, 0, 0]
            return True
        self.move_l(pose, vel_mmps=vel_mmps, wait=False)
        t0 = time.time()
        dt = 1.0 / poll_hz
        while time.time() - t0 < timeout_s:
            f_axis = abs(self.get_force()[axis_idx])
            if f_axis > max_safety_n:
                self.stop_motion()
                self.log.error("emergency stop: |F%s|=%.1fN > %.1fN", axis, f_axis, max_safety_n)
                return False
            if f_axis > threshold_n:
                self.stop_motion()
                return True
            if self._is_motion_done():
                return False
            time.sleep(dt)
        self.stop_motion()
        return False

    # ----- 종료 -----
    def emergency_stop(self) -> None:
        try:
            self.stop_motion()
        except Exception:
            pass
        try:
            if self._compliance_on:
                self.disable_compliance()
        except Exception:
            pass

    def disconnect(self) -> None:
        if self.dry_run or self.indy is None:
            return
        try:
            if self._sdk == "indy_utils":
                self.indy.disconnect()
        except Exception:
            pass
