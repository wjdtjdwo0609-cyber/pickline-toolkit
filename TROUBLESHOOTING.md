# Troubleshooting

Pickline Toolkit은 로봇 pick cycle, vision, PLC, calibration 유틸을 묶은 라이브러리입니다. 문제를 볼 때는 입력 데이터, 장비 연결, 도메인 예외를 분리해서 확인합니다.

## 빠른 점검

```bash
python -m pip install -e .
python -m pytest
```

패키지를 editable mode로 설치하면 예제와 테스트가 현재 소스 변경을 바로 반영합니다.

## 자주 나는 문제

| 증상 | 확인할 것 | 해결 |
| --- | --- | --- |
| import 실패 | editable install 여부, Python path | 저장소 루트에서 `python -m pip install -e .` 재실행 |
| calibration 결과가 튐 | 입력 좌표계, 단위, 샘플 수 | 카메라/로봇 좌표계를 다시 표시하고 이상치 샘플 제거 |
| vision 결과가 비어 있음 | 이미지 경로, 조명, threshold | 고정 fixture 이미지로 먼저 테스트하고 threshold를 로그로 남김 |
| PLC adapter 오류 | IP/port, register map, timeout | dry-run 또는 mock adapter로 도메인 로직을 먼저 검증 |
| 테스트가 장비 없어서 실패 | hardware test와 pure unit test 구분 | 장비 의존 테스트는 marker 또는 환경변수로 분리 |

## 도메인 예외 확인

- `PicklineError` 계층을 먼저 확인해 실패가 validation, calibration, IO, hardware 중 어디인지 분류합니다.
- 외부 장비 오류를 일반 `Exception`으로 삼키지 말고 adapter 경계에서 도메인 예외로 변환합니다.
- 재시도는 infrastructure adapter에서 처리하고 domain 계산은 deterministic하게 유지합니다.

## 이슈를 남길 때

- Python 버전
- 실행 명령
- 사용한 camera/robot/PLC adapter
- 입력 fixture 또는 register map
- 발생한 `PicklineError` 타입과 메시지
