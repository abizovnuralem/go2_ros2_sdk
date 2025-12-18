import sys
import types

import numpy as np

from go2_robot_sdk.infrastructure.sensors.lidar_decoder import update_meshes_for_cloud2


def _make_dummy_cpp_module(return_value: np.ndarray):
    mod = types.ModuleType("lidar_accelator")

    def process_u8_to_xyzi_f32(
        positions,
        uvs,
        res,
        origin,
        intense_limiter,
        deduplicate,
        downsample_step,
        max_points,
    ):
        return return_value

    mod.process_u8_to_xyzi_f32 = process_u8_to_xyzi_f32
    return mod


def test_update_meshes_uses_cpp_when_enabled(monkeypatch):
    sentinel = np.array([[9.0, 9.0, 9.0, 0.9]], dtype=np.float32)
    monkeypatch.setitem(sys.modules, "lidar_accelator", _make_dummy_cpp_module(sentinel))

    positions = np.zeros((1, 3), dtype=np.uint8)
    uvs = np.zeros((1, 2), dtype=np.uint8)

    out = update_meshes_for_cloud2(
        positions,
        uvs,
        0.01,
        [0.0, 0.0, 0.0],
        0.0,
        deduplicate=False,
        downsample_step=1,
        max_points=0,
        use_cpp_accel=True,
    )

    assert out is sentinel


def test_update_meshes_falls_back_when_cpp_missing():
    positions = np.zeros((1, 3), dtype=np.uint8)
    uvs = np.zeros((1, 2), dtype=np.uint8)

    out = update_meshes_for_cloud2(
        positions,
        uvs,
        1.0,
        [0.0, 0.0, 0.0],
        0.0,
        deduplicate=False,
        downsample_step=1,
        max_points=0,
        use_cpp_accel=True,
    )

    assert isinstance(out, np.ndarray)
    assert out.shape[1] == 4
