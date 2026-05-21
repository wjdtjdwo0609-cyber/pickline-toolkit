"""Phase 2 테스트 — VacuumGripper, RobotDioHandshake.

하드웨어 없이 dry_run + 가짜 로봇(FakeRobot)으로 검증.
"""

import pytest

from pickline.robot import (
    VacuumGripper, Gripper, RobotDioHandshake, DioStartEvent, DioOrderError,
)


class FakeRobot:
    """RobotController 일부를 흉내내는 테스트용 가짜 로봇.

    di: {index: 0/1} dict. 미지정 인덱스는 0.
    """
    def __init__(self, di=None):
        self.di = dict(di) if di else {}
        self.do_writes = []          # (index, state) 기록
        self.do_state = {}
    def read_di(self, address):
        return bool(self.di.get(int(address), 0))
    def set_do(self, address, state):
        self.do_writes.append((int(address), bool(state)))
        self.do_state[int(address)] = bool(state)
        return True
    def set_endtool_do(self, index, state):
        self.do_writes.append((f"endtool{index}", bool(state)))
        return True
    def pump_joints(self):
        pass


# --- VacuumGripper ---

def test_vacuum_gripper_implements_protocol():
    assert isinstance(VacuumGripper(backend="plc", dry_run=True), Gripper)

def test_vacuum_dry_run_engage_release():
    g = VacuumGripper(backend="plc", dry_run=True)
    g.engage()
    assert g.is_engaged() is True
    assert g.verify_grip(settle_ms=0) is True
    g.release()
    assert g.is_engaged() is False

def test_vacuum_robot_do_backend():
    robot = FakeRobot()
    g = VacuumGripper(backend="robot_do", robot=robot, robot_do_index=2, dry_run=True)
    # dry_run이라 robot 호출 없이 상태만 — 백엔드 검증은 아래 비-dry로
    g.engage(); g.release()

def test_vacuum_robot_do_backend_writes(monkeypatch):
    robot = FakeRobot()
    g = VacuumGripper(backend="robot_do", robot=robot, robot_do_index=5,
                      sensor_backend="none", dry_run=False)
    g.engage()
    assert (5, True) in robot.do_writes
    g.release()
    assert (5, False) in robot.do_writes

def test_vacuum_word_device_detection():
    assert VacuumGripper._is_word_device("D2") is True
    assert VacuumGripper._is_word_device("M100") is False

def test_vacuum_unknown_backend_raises():
    with pytest.raises(ValueError):
        VacuumGripper(backend="laser", dry_run=True)


# --- RobotDioHandshake ---

def test_dio_decode_8pin():
    robot = FakeRobot(di={8: 1, 9: 1, 10: 0})
    hs = RobotDioHandshake(robot, cfg={}, channel_id="r1")
    ev = hs.read_order_snapshot()
    assert isinstance(ev, DioStartEvent)
    assert ev.class_id == 0 and ev.class_name == "8pin"

def test_dio_decode_12pin():
    robot = FakeRobot(di={8: 1, 9: 0, 10: 1})
    hs = RobotDioHandshake(robot, cfg={}, channel_id="r2")
    assert hs.read_order_snapshot().class_id == 1

def test_dio_invalid_order_both_on():
    robot = FakeRobot(di={8: 1, 9: 1, 10: 1})
    hs = RobotDioHandshake(robot, cfg={}, channel_id="r1")
    with pytest.raises(DioOrderError):
        hs.read_order_snapshot()

def test_dio_order_optional_uses_default():
    robot = FakeRobot(di={8: 1, 9: 0, 10: 0})
    hs = RobotDioHandshake(robot, cfg={"plc_order_required": False,
                                       "plc_default_class_id": 1}, channel_id="r1")
    assert hs.read_order_snapshot().class_id == 1

def test_dio_rising_edge_only():
    robot = FakeRobot(di={8: 0, 9: 1, 10: 0})   # 8pin 주문 비트 ON
    hs = RobotDioHandshake(robot, cfg={"plc_start_debounce_s": 0}, channel_id="r1")
    assert hs.poll_start_event() is None      # prime
    assert hs.poll_start_event() is None      # 여전히 0
    robot.di[8] = 1
    ev = hs.poll_start_event()                # 0→1 rising
    assert ev is not None
    assert hs.poll_start_event() is None      # 계속 1 — 엣지 아님

def test_dio_cycle_outputs():
    robot = FakeRobot()
    hs = RobotDioHandshake(robot, cfg={"plc_done_pulse_s": 0,
                                       "plc_result_latch_until_next_start": True},
                           channel_id="r1")
    hs.begin_cycle()
    assert robot.do_state.get(14) is True            # BUSY ON
    hs.complete_cycle("GOOD")
    assert robot.do_state.get(14) is False           # BUSY OFF
    assert robot.do_state.get(10) is True            # GOOD latched
