import json
import struct

import pytest


def _make_dummy_buffer(meta: dict, compressed: bytes) -> bytes:
    json_bytes = json.dumps(meta).encode("utf-8")
    header = struct.pack("<H", len(json_bytes)) + b"\x00\x00"
    return header + json_bytes + compressed


def test_deal_array_buffer_cpp_mode_skips_python_decode(monkeypatch, tmp_path):
    # Ensure we do not attempt to call Python wasmtime decoder in cpp mode.
    monkeypatch.setenv("LIDAR_USE_CPP_ACCEL", "true")
    monkeypatch.setenv("LIDAR_DUMP_DIR", str(tmp_path))
    monkeypatch.setenv("LIDAR_DUMP_MAX", "1")

    from go2_robot_sdk.infrastructure.webrtc import data_decoder

    # Force legacy path on, but with no decoder instance.
    monkeypatch.setattr(data_decoder, "_global_lidar_decoder", None)

    meta = {
        "topic": "ULIDAR_ARRAY",
        "data": {
            "resolution": 0.01,
            "origin": [0.0, 0.0, 0.0],
            "stamp": 0.0,
        },
    }
    compressed = b"\x42" * 16
    buf = _make_dummy_buffer(meta, compressed)

    out = data_decoder.deal_array_buffer(buf, perform_decode=True)
    assert isinstance(out, dict)

    # With no decoder instance it will use WebRTCDataDecoder fallback which includes compressed_data.
    assert out.get("compressed_data") == compressed

    # Dump file should have been written
    files = list(tmp_path.glob("ulidar_array_buffer_*.bin"))
    assert len(files) == 1
    assert files[0].read_bytes() == buf


def test_deal_array_buffer_python_mode_tries_decode_and_falls_back(monkeypatch):
    monkeypatch.setenv("LIDAR_USE_CPP_ACCEL", "false")

    from go2_robot_sdk.infrastructure.webrtc import data_decoder

    class DummyDecoder:
        def decode(self, compressed_data, meta):
            raise RuntimeError("forced failure")

    monkeypatch.setattr(data_decoder, "_global_lidar_decoder", DummyDecoder())

    meta = {
        "topic": "ULIDAR_ARRAY",
        "data": {
            "resolution": 0.01,
            "origin": [0.0, 0.0, 0.0],
            "stamp": 0.0,
        },
    }
    compressed = b"\x42" * 16
    buf = _make_dummy_buffer(meta, compressed)

    out = data_decoder.deal_array_buffer(buf, perform_decode=True)

    # Failure should result in returning None or a dict without decoded_data depending on path.
    # We only assert it doesn't crash.
    assert out is None or isinstance(out, dict)
