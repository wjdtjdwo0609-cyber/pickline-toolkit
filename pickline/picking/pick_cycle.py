"""픽킹 — 픽 사이클 유스케이스 (application 계층).

`PickConfig` : 픽 동작 튜닝값 (프로젝트별로 이 값만 바꿔 재사용).
`PickCycle`  : robot + gripper 포트를 주입받아 픽앤플레이스 1회를 실행하는 유스케이스.

9단계 시퀀스 (Intel5 step11_pick_pipeline.py 기반):
    1. 빠른 접근 (물체 위 ~50mm, 고속)
    2. 느린 접근 (물체 위 ~10mm, 저속)
    3. 컴플라이언스 ON (Z 부드럽게, 면 따라가기)
    4. force-guided 하강 (표면에서 접촉 감지로 정지)
    5. 그리퍼 engage
    6. 흡착 검증 (센서 확인)
    7. 컴플라이언스 OFF
    8. 리프트 (물체 위 ~100mm)
    9. 실패 시 옆으로 옮겨 재시도 (최대 max_retries회)

PickCycle은 PickTarget(이미 로봇 좌표로 변환된 목표)을 받는다.
검출→좌표변환(Detection + Calibration → PickTarget)은 호출자/상위 계층의 몫.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import List, Optional

from pickline.shared.value_objects import Pose6D, PickTarget
from pickline.shared.events import EventBus, PickStarted, PickCompleted

_log = logging.getLogger("pickline.picking")


@dataclass
class PickConfig:
    """픽앤플레이스 1회의 모든 튜닝 파라미터.

    Intel5 step11_pick_pipeline.PickConfig + 워커 상수 통합.
    """
    # 접근/리프트 높이 (물체 z 기준 상대 mm)
    fast_approach_above_mm: float = 50.0
    slow_approach_above_mm: float = 10.0
    descent_below_mm: float = -5.0          # 표면 아래로 살짝 (force가 잡아줌)
    lift_above_mm: float = 100.0

    # 속도 (mm/s)
    fast_vel_mmps: float = 200.0
    slow_vel_mmps: float = 50.0
    descent_vel_mmps: float = 5.0

    # 힘 제어
    use_force_descent: bool = True
    force_threshold_n: float = 5.0
    descent_timeout_s: float = 8.0
    max_safety_n: float = 30.0
    compliance_stiffness: List[float] = field(
        default_factory=lambda: [2000, 2000, 300, 10, 10, 80])

    # 그리퍼/진공
    vacuum_settle_ms: int = 200
    vacuum_verify_timeout_ms: int = 800

    # 접근 자세 (deg) — 천장 카메라 기준 EE가 -Z로 하강
    approach_rx_deg: float = 180.0
    approach_ry_deg: float = 0.0

    # 재시도
    max_retries: int = 2
    retry_offset_mm: float = 5.0

    # 검출 게이트
    detection_min_confidence: float = 0.7

    # 안전
    z_safe_max_mm: float = 1000.0


@dataclass
class PickResult:
    """픽 사이클 1회 결과."""
    success: bool
    class_id: int = -1
    attempts: int = 0
    duration_s: float = 0.0
    reason: str = ""


class PickCycle:
    """픽앤플레이스 1회 유스케이스. robot + gripper 포트를 조합한다.

    사용:
        cycle = PickCycle(robot, gripper, PickConfig())
        result = cycle.run(pick_target)
    """

    def __init__(self, robot, gripper, config: Optional[PickConfig] = None,
                 event_bus: Optional[EventBus] = None,
                 logger: Optional[logging.Logger] = None,
                 channel_id: str = "default"):
        self.robot = robot
        self.gripper = gripper
        self.cfg = config or PickConfig()
        self.bus = event_bus
        self.log = logger or _log
        self.channel_id = channel_id

    def run(self, target: PickTarget) -> PickResult:
        """주어진 PickTarget에 대해 픽 시도 (재시도 포함)."""
        t0 = time.time()
        if target.confidence < self.cfg.detection_min_confidence:
            return PickResult(False, target.class_id, 0,
                              round(time.time() - t0, 3), "confidence 미달")

        if self.bus:
            self.bus.publish(PickStarted(channel_id=self.channel_id,
                                         class_id=target.class_id))

        for attempt in range(self.cfg.max_retries + 1):
            adjusted = self._apply_retry_offset(target, attempt)
            self.log.info("[%s] 픽 시도 %d/%d cls=%d",
                          self.channel_id, attempt + 1,
                          self.cfg.max_retries + 1, target.class_id)
            if self._attempt_single(adjusted):
                result = PickResult(True, target.class_id, attempt + 1,
                                    round(time.time() - t0, 3))
                self._publish_done(result)
                return result

        result = PickResult(False, target.class_id, self.cfg.max_retries + 1,
                            round(time.time() - t0, 3), "all_attempts_failed")
        self._publish_done(result)
        return result

    def _publish_done(self, result: PickResult) -> None:
        if self.bus:
            self.bus.publish(PickCompleted(
                channel_id=self.channel_id, class_id=result.class_id,
                success=result.success, duration_s=result.duration_s))

    def _apply_retry_offset(self, target: PickTarget, attempt: int) -> PickTarget:
        """재시도 시 X를 살짝 이동. 0: 그대로, 1: +offset, 2: -offset."""
        if attempt == 0:
            return target
        sign = 1 if attempt == 1 else -1
        dx = self.cfg.retry_offset_mm * sign
        x, y, z = target.pick_xyz_mm
        return PickTarget(
            class_id=target.class_id, confidence=target.confidence,
            pick_xyz_mm=(x + dx, y, z), pick_yaw_deg=target.pick_yaw_deg,
            depth_m=target.depth_m, pixel_center=target.pixel_center)

    def _attempt_single(self, target: PickTarget) -> bool:
        """9단계 픽 시퀀스 1회."""
        cfg = self.cfg
        x, y, z = target.pick_xyz_mm
        rx, ry, rz = cfg.approach_rx_deg, cfg.approach_ry_deg, target.pick_yaw_deg

        def pose(dz: float) -> Pose6D:
            return Pose6D(x, y, z + dz, rx, ry, rz)

        try:
            # 1~2. 빠른/느린 접근
            self.robot.move_l(pose(cfg.fast_approach_above_mm), vel_mmps=cfg.fast_vel_mmps)
            self.robot.move_l(pose(cfg.slow_approach_above_mm), vel_mmps=cfg.slow_vel_mmps)

            # 3~4. 컴플라이언스 + 하강
            if cfg.use_force_descent:
                self.robot.set_compliance(stiffness=cfg.compliance_stiffness)
                contact = self.robot.force_guided_descent(
                    pose(cfg.descent_below_mm), axis="z",
                    threshold_n=cfg.force_threshold_n, vel_mmps=cfg.descent_vel_mmps,
                    timeout_s=cfg.descent_timeout_s, max_safety_n=cfg.max_safety_n)
                if not contact:
                    self.robot.disable_compliance()
                    self.robot.move_l(pose(cfg.fast_approach_above_mm),
                                      vel_mmps=cfg.fast_vel_mmps)
                    return False
            else:
                self.robot.move_l(pose(0.0), vel_mmps=cfg.descent_vel_mmps)

            # 5. 그리퍼 engage
            self.gripper.engage()

            # 6. 흡착 검증
            verified = self.gripper.verify_grip(
                settle_ms=cfg.vacuum_settle_ms, timeout_ms=cfg.vacuum_verify_timeout_ms)
            if not verified:
                self.gripper.release()
                if cfg.use_force_descent:
                    self.robot.disable_compliance()
                self.robot.move_l(pose(cfg.fast_approach_above_mm),
                                  vel_mmps=cfg.fast_vel_mmps)
                return False

            # 7~8. 컴플라이언스 OFF + 리프트
            if cfg.use_force_descent:
                self.robot.disable_compliance()
            self.robot.move_l(pose(cfg.lift_above_mm), vel_mmps=cfg.fast_vel_mmps)
            return True

        except Exception as exc:
            self.log.error("[%s] 픽 시퀀스 오류: %s", self.channel_id, exc)
            try:
                self.robot.emergency_stop()
                self.gripper.release()
            except Exception:
                pass
            return False

    def place(self, place_pose: Pose6D, approach_above_mm: float = 50.0) -> bool:
        """집은 물체를 place_pose에 내려놓는다."""
        cfg = self.cfg
        try:
            self.robot.move_l(place_pose.offset_z(approach_above_mm),
                              vel_mmps=cfg.fast_vel_mmps)
            self.robot.move_l(place_pose, vel_mmps=cfg.slow_vel_mmps)
            self.gripper.release()
            time.sleep(0.2)
            self.robot.move_l(place_pose.offset_z(approach_above_mm),
                              vel_mmps=cfg.fast_vel_mmps)
            return True
        except Exception as exc:
            self.log.error("[%s] place 오류: %s", self.channel_id, exc)
            return False
