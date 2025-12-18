import numpy as np
import pytest


def test_pack_xyzi_f32_to_bytes_matches_numpy():
    try:
        import lidar_accelator
    except Exception:
        pytest.skip("lidar_accelator(pybind11) module is not available")

    points = np.array(
        [
            [1.0, 2.0, 3.0, 0.1],
            [4.0, 5.0, 6.0, 0.2],
        ],
        dtype=np.float32,
    )

    expected = points.tobytes()
    got = lidar_accelator.pack_xyzi_f32_to_bytes(points)

    assert isinstance(got, (bytes, bytearray))
    assert got == expected
