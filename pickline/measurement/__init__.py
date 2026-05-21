"""측정 — 로드셀 / 무게 OCR (bounded context, 추출 예정).

이 컨텍스트는 아직 추출되지 않았다 (의도된 placeholder).

원본 Intel5에는 두 갈래가 있다:
- loadcell  : HX711 ADC 시리얼 / PLC 레지스터 기반 물리 저울.
              Intel5의 `loadcell/` 패키지가 이미 4계층 DDD로 완성돼 있어,
              그대로 `pickline/measurement/loadcell/`로 편입하면 된다.
- weight_ocr: 카메라로 저울 LCD 7세그먼트를 읽는 OCR.
              step32_ocr_weight_gui.py(3691줄) 안에 핵심 로직이 갇혀 있어
              SevenSegmentWeightReader / WeightOcrModel 추출이 필요하다.

자세한 계획은 저장소 루트의 ARCHITECTURE.md 3.8 절 참고.
"""

__all__: list[str] = []
