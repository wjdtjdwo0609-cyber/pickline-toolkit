"""Phase 4 테스트 — PlcClient (dry_run) + device_offset."""

from pickline.plc import PlcClient, device_offset


# --- device_offset ---

def test_device_offset_hex():
    assert device_offset("B260", 3) == "B263"
    assert device_offset("B25F", 1) == "B260"      # 16진수 자리올림
    assert device_offset("W100", 5) == "W105"

def test_device_offset_decimal():
    assert device_offset("D100", 5) == "D105"
    assert device_offset("M10", 9) == "M19"


# --- PlcClient dry_run ---

def test_dry_run_connect_close():
    with PlcClient("127.0.0.1", dry_run=True) as plc:
        assert plc.read_bit("B200") is False
        assert plc.read_word("D100") == 0
        plc.write_bit("B250", True)        # 예외 없이 통과
        plc.write_word("D200", 42)

def test_device_map_resolution():
    plc = PlcClient("127.0.0.1", dry_run=True,
                    device_map={"start_bit": "B200", "count_word": "D201"})
    assert plc.resolve("start_bit") == "B200"
    assert plc.resolve("count_word") == "D201"
    assert plc.resolve("B999") == "B999"   # 매핑에 없으면 그대로

def test_index_bits_dry_run():
    plc = PlcClient("127.0.0.1", dry_run=True,
                    device_map={"bad_base": "B310"})
    assert plc.read_index_bit("bad_base", 4) is False
    plc.write_index_bit("bad_base", 2, True)
    plc.clear_index_bits("bad_base", 5)    # 예외 없이 통과
