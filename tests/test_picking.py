"""Phase 3 테스트 — PickCycle 유스케이스.

dry_run IndyController + dry_run VacuumGripper로 9단계 픽 시퀀스를 검증.
"""

import pytest

from pickline.robot import IndyController, VacuumGripper
from pickline.picking import PickCycle, PickConfig, PickResult
from pickline.shared.value_objects import PickTarget, Pose6D
from pickline.shared.events import EventBus, PickCompleted


def make_cycle(**cfg_kw):
    robot = IndyController(robot_ip="127.0.0.1", dry_run=True)
    gripper = VacuumGripper(backend="plc", dry_run=True)
    return PickCycle(robot, gripper, PickConfig(**cfg_kw))


def target(conf=0.9, cls=0):
    return PickTarget(class_id=cls, confidence=conf,
                      pick_xyz_mm=(300.0, 0.0, 200.0), pick_yaw_deg=15.0)


def test_pick_success_dry_run():
    result = make_cycle().run(target())
    assert isinstance(result, PickResult)
    assert result.success is True
    assert result.attempts == 1
    assert result.class_id == 0


def test_pick_rejects_low_confidence():
    result = make_cycle().run(target(conf=0.3))   # min 0.7 미달
    assert result.success is False
    assert "confidence" in result.reason


def test_pick_without_force_descent():
    result = make_cycle(use_force_descent=False).run(target())
    assert result.success is True


def test_pick_publishes_events():
    bus = EventBus()
    done = []
    bus.subscribe(PickCompleted, lambda e: done.append(e.success))
    robot = IndyController(robot_ip="127.0.0.1", dry_run=True)
    gripper = VacuumGripper(backend="plc", dry_run=True)
    PickCycle(robot, gripper, PickConfig(), event_bus=bus, channel_id="r1").run(target())
    assert done == [True]


def test_retry_offset_alternates():
    cyc = make_cycle(retry_offset_mm=5.0)
    t = target()
    assert cyc._apply_retry_offset(t, 0).pick_xyz_mm[0] == 300.0    # 그대로
    assert cyc._apply_retry_offset(t, 1).pick_xyz_mm[0] == 305.0    # +5
    assert cyc._apply_retry_offset(t, 2).pick_xyz_mm[0] == 295.0    # -5


def test_place():
    cyc = make_cycle()
    assert cyc.place(Pose6D(400, 100, 300, 0, 180, 0)) is True


def test_pick_fails_when_verify_fails(monkeypatch):
    cyc = make_cycle()
    # 흡착 검증이 항상 실패하도록 패치 → 모든 시도 실패
    monkeypatch.setattr(cyc.gripper, "verify_grip", lambda **kw: False)
    result = cyc.run(target())
    assert result.success is False
    assert result.attempts == cyc.cfg.max_retries + 1
    assert result.reason == "all_attempts_failed"
