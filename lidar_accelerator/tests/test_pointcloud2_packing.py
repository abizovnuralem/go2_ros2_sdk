import numpy as np
import pytest

from go2_robot_sdk.infrastructure.ros2.pointcloud2_packing import pack_xyzi_float32


def test_pack_xyzi_float32_happy_path():
    points = np.array(
        [
            [1.0, 2.0, 3.0, 0.1],
            [4.0, 5.0, 6.0, 0.2],
        ],
        dtype=np.float32,
    )

    packed = pack_xyzi_float32(points)

    assert packed["height"] == 1
    assert packed["width"] == 2
    assert packed["point_step"] == 16
    assert packed["row_step"] == 32
    assert packed["is_bigendian"] is False
    assert packed["is_dense"] is True
    assert isinstance(packed["data"], (bytes, bytearray))
    assert len(packed["data"]) == 2 * 16


def test_pack_xyzi_float32_invalid_shape_raises():
    points = np.zeros((10, 3), dtype=np.float32)
    with pytest.raises(ValueError):
        _ = pack_xyzi_float32(points)
