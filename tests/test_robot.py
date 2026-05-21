"""Phase 2 로봇 모듈 테스트 — IndyController dry-run.

dry_run=True로 하드웨어/SDK 없이 인터페이스 계약을 검증한다.
"""

from pickline.robot import IndyController, RobotController
from pickline.shared.value_objects import Pose6D
from pickline.shared.errors import RobotMotionError
import pytest


def make() -> IndyController:
    return IndyController(robot_ip="127.0.0.1", dry_run=True)


def test_implements_protocol():
    # runtime_checkable Protocol — 구조적 타입 일치 확인
    assert isinstance(make(), RobotController)


def test_dry_run_connect():
    assert make().connect() is True


def test_get_pose_returns_pose6d():
    pose = make().get_pose()
    assert isinstance(pose, Pose6D)
    assert pose.z == 500.0   # dry 기본값


def test_move_l_accepts_pose6d_and_list():
    r = make()
    assert r.move_l(Pose6D(300, 0, 400, 0, 180, 0), vel_mmps=50) is True
    assert r.get_pose().x == 300
    assert r.move_l([350, 10, 420, 0, 180, 0]) is True
    assert r.get_pose().x == 350


def test_safety_limit_rejects_high_z():
    r = make()
    r.set_safety_limits(z_safe_max_mm=200)
    assert r.move_l([300, 0, 999, 0, 180, 0]) is False     # z 초과 → 거부
    assert r.move_l([300, 0, 150, 0, 180, 0]) is True       # 안전 → 허용
    assert r.move_l([300, 0, 999, 0, 180, 0], skip_safety=True) is True  # 우회


def test_approach_uvw_forced():
    r = make()
    r.set_safety_limits(approach_uvw_deg=[0, -180, 0])
    r.move_l([300, 0, 400, 11, 22, 33])     # 회전값 강제 교체돼야 함
    pose = r.get_pose()
    assert [pose.rx, pose.ry, pose.rz] == [0, -180, 0]


def test_dio_dry_run():
    r = make()
    assert r.set_do(13, True) is True
    assert len(r.get_di()) == 32
    r.pulse_do(13, pulse_s=0.0)


def test_force_guided_descent_dry_run():
    r = make()
    r.set_compliance()
    assert r.force_guided_descent([300, 0, 200, 0, 180, 0], threshold_n=5.0) is True
    r.disable_compliance()


def test_emergency_stop_safe():
    r = make()
    r.set_compliance()
    r.emergency_stop()       # 예외 없이 통과해야 함
    r.disconnect()
