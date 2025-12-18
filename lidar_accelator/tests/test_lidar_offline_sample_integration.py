import os
import pathlib
import struct
import json

import numpy as np
import pytest


def _load_sample_path() -> pathlib.Path:
    p = os.getenv("LIDAR_SAMPLE_PATH", "").strip()
    if not p:
        pytest.skip("Set LIDAR_SAMPLE_PATH to a recorded ulidar array-buffer .bin")
    path = pathlib.Path(p)
    if not path.exists():
        pytest.skip(f"Sample file does not exist: {path}")
    return path


def _parse_ulidar_array_buffer(buf: bytes):
    if len(buf) < 4:
        raise ValueError("buffer too short")

    json_length = struct.unpack("<H", buf[:2])[0]
    if len(buf) < 4 + json_length:
        raise ValueError("buffer too short for json segment")

    json_segment = buf[4 : 4 + json_length]
    compressed = buf[4 + json_length :]

    metadata = json.loads(json_segment.decode("utf-8"))

    # There are two variants in the wild:
    # - metadata already contains 'data'
    # - metadata itself is the 'data'
    data = metadata.get("data", metadata)

    return data, compressed


def test_offline_sample_cpp_matches_python_end_to_end():
    try:
        import lidar_accelator
    except Exception:
        pytest.skip("lidar_accelator(pybind11) module is not available")

    # Python reference path: wasmtime decode -> positions/uvs -> update_meshes_for_cloud2
    from go2_robot_sdk.infrastructure.sensors.lidar_decoder import (
        get_voxel_decoder,
        update_meshes_for_cloud2,
    )

    path = _load_sample_path()
    buf = path.read_bytes()

    data, compressed = _parse_ulidar_array_buffer(buf)

    resolution = float(data.get("resolution", 0.01) or 0.01)
    origin = list(data.get("origin", [0.0, 0.0, 0.0]))

    intensity = float(os.getenv("LIDAR_INTENSITY_THRESHOLD", "0.0") or 0.0)
    dedup = os.getenv("LIDAR_DEDUPLICATE", "false").strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    downsample_step = int(os.getenv("LIDAR_DOWNSAMPLE_STEP", "1") or 1)
    max_points = int(os.getenv("LIDAR_MAX_POINTS", "0") or 0)

    decoder = get_voxel_decoder()
    decoded = decoder.decode(compressed, {"origin": origin, "resolution": resolution})

    py_points = update_meshes_for_cloud2(
        decoded["positions"],
        decoded["uvs"],
        resolution,
        origin,
        intensity,
        deduplicate=dedup,
        downsample_step=downsample_step,
        max_points=max_points,
        use_cpp_accel=False,
    )

    cpp_points = lidar_accelator.decode_and_process(
        compressed,
        resolution,
        origin,
        intensity,
        dedup,
        downsample_step,
        max_points,
    )

    assert isinstance(py_points, np.ndarray)
    assert isinstance(cpp_points, np.ndarray)
    assert py_points.shape[1] == 4
    assert cpp_points.shape[1] == 4

    assert cpp_points.dtype == np.float32

    # The point order can differ depending on dedup/unique implementation.
    # Sort rows lexicographically for a stable comparison.
    py_sorted = py_points[np.lexsort((py_points[:, 3], py_points[:, 2], py_points[:, 1], py_points[:, 0]))]
    cpp_sorted = cpp_points[np.lexsort((cpp_points[:, 3], cpp_points[:, 2], cpp_points[:, 1], cpp_points[:, 0]))]

    assert py_sorted.shape == cpp_sorted.shape
    assert np.allclose(py_sorted, cpp_sorted)
