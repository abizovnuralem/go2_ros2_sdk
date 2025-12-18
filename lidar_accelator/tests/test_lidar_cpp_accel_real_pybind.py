import numpy as np
import pytest

from go2_robot_sdk.infrastructure.sensors.lidar_decoder import update_meshes_for_cloud2


def test_cpp_accel_real_pybind_matches_python(caplog):
    try:
        import lidar_accelator
    except Exception:
        pytest.skip("lidar_accelator(pybind11) module is not available")

    assert hasattr(lidar_accelator, "__file__")
    assert str(lidar_accelator.__file__).endswith((".so", ".pyd"))

    positions = np.array([10, 20, 30, 40, 50, 60], dtype=np.uint8)
    uvs = np.array([10, 20, 30, 40], dtype=np.uint8)

    # 1) C++ 구현을 직접 호출(모듈이 실제로 동작함을 확인)
    out_cpp_direct = lidar_accelator.process_u8_to_xyzi_f32(
        positions,
        uvs,
        0.01,
        [0.0, 0.0, 0.0],
        0.0,
        False,
        1,
        0,
    )

    # 2) Python 구현과 결과 일치(기능 검증)
    out_py = update_meshes_for_cloud2(
        positions,
        uvs,
        0.01,
        [0.0, 0.0, 0.0],
        0.0,
        deduplicate=False,
        downsample_step=1,
        max_points=0,
        use_cpp_accel=False,
    )

    assert out_cpp_direct.shape == out_py.shape
    assert out_cpp_direct.dtype == np.float32
    assert np.allclose(out_cpp_direct, out_py)

    # 3) update_meshes_for_cloud2가 use_cpp_accel=True일 때 실제로 모듈 함수 attr를 호출하는지 확인
    sentinel = np.array([[9.0, 9.0, 9.0, 0.9]], dtype=np.float32)
    orig = lidar_accelator.process_u8_to_xyzi_f32
    try:
        lidar_accelator.process_u8_to_xyzi_f32 = lambda *args, **kwargs: sentinel
        out_switch = update_meshes_for_cloud2(
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
        assert out_switch is sentinel
    finally:
        lidar_accelator.process_u8_to_xyzi_f32 = orig
