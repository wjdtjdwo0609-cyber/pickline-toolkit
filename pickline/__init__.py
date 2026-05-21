"""pickline — 비전 가이드 픽앤플레이스 라인 재사용 툴킷.

산업용 비전 + 로봇 픽앤플레이스 프로젝트를 위한 모듈형 라이브러리.
Intel5 릴레이 소켓 검사 라인 프로젝트에서 추출한 재사용 빌딩 블록.

DDD bounded context 구조:
    shared       — 공유 커널 (값 객체, 도메인 이벤트, 예외)
    infra        — 인프라 (파일 IPC, 디바이스 선택)
    vision       — 비전 (카메라, YOLO 검출)
    calibration  — 캘리브레이션 (좌표 변환, hand-eye, 평면 호모그래피)
    robot        — 로봇 제어 (Indy7/IndyDCP3 추상화, 그리퍼, DIO)
    picking      — 픽 사이클 오케스트레이션
    plc          — PLC 연동 (MC Protocol)
    measurement  — 측정 (로드셀, 무게 OCR)

설계와 추출 로드맵은 ARCHITECTURE.md 참고.
"""

__version__ = "0.1.0"
