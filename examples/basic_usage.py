"""pickline 툴킷 기본 사용 예제 (Phase 0 추출 모듈).

실행: python examples/basic_usage.py
하드웨어 없이 동작 — 추출된 순수 모듈만 사용한다.
"""

from pickline.shared import Pose6D, EventBus, PickCompleted
from pickline.infra import atomic_write_json, read_json, select_device, device_info
from pickline.calibration import pose_to_matrix, invert_transform, apply_transform
from pickline.picking import PickConfig


def demo_value_objects():
    print("\n[1] 값 객체 — 로봇 자세")
    home = Pose6D(x=400, y=0, z=500, rx=0, ry=180, rz=0)
    approach = home.offset_z(50)        # 물체 위 50mm
    print(f"  home     = {home.as_list()}")
    print(f"  approach = {approach.as_list()}  (offset_z(+50))")


def demo_device():
    print("\n[2] 디바이스 자동 선택")
    print(f"  {device_info()}")
    print(f"  → 이 환경의 추론 디바이스: {select_device()}")


def demo_file_ipc(tmp="/tmp/pickline_demo_cmd.json"):
    print("\n[3] 파일 IPC — 프로세스 간 크래시 안전 통신")
    atomic_write_json(tmp, {"seq": 1, "action": "pick", "class_id": 0})
    cmd = read_json(tmp, default={})
    print(f"  기록 후 읽기: {cmd}")


def demo_transforms():
    print("\n[4] 좌표 변환 — 로봇 pose ↔ 4x4 행렬")
    T = pose_to_matrix([400, 0, 500, 0, 180, 0])   # mm+deg
    Tinv = invert_transform(T)
    pt = apply_transform(T, [0, 0, 0])              # 원점 변환
    print(f"  pose→T 위치(m)      = {T[:3, 3].round(3).tolist()}")
    print(f"  T로 원점 변환(m)     = {pt.round(3).tolist()}")
    print(f"  T @ inv(T) ≈ I      = {bool((T @ Tinv).round(6).tolist() == __import__('numpy').eye(4).tolist())}")


def demo_events():
    print("\n[5] 도메인 이벤트 — 컨텍스트 간 느슨한 결합")
    bus = EventBus()
    bus.subscribe(PickCompleted,
                  lambda e: print(f"  [구독자] 픽 완료 수신: ch={e.channel_id} success={e.success}"))
    bus.publish(PickCompleted(channel_id="r1", class_id=0, success=True, duration_s=2.4))


def demo_pick_config():
    print("\n[6] 픽 설정 — 프로젝트별로 이 값만 바꿔 재사용")
    cfg = PickConfig(z_pickup_mm=75, z_safe_max_mm=130, max_retries=3)
    print(f"  접근높이={cfg.z_approach_above_mm}mm  집기={cfg.z_pickup_mm}mm  "
          f"안전상한={cfg.z_safe_max_mm}mm  재시도={cfg.max_retries}")


if __name__ == "__main__":
    print("=" * 55)
    print("  pickline-toolkit — 기본 사용 예제")
    print("=" * 55)
    demo_value_objects()
    demo_device()
    demo_file_ipc()
    demo_transforms()
    demo_events()
    demo_pick_config()
    print("\n완료. 자세한 설계는 ARCHITECTURE.md 참고.\n")
