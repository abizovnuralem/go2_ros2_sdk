import pytest


def test_decode_and_process_not_implemented():
    try:
        import lidar_accelerator
    except Exception:
        pytest.skip("lidar_accelerator(pybind11) module is not available")

    with pytest.raises(Exception):
        _ = lidar_accelerator.decode_and_process(
            b"",
            0.01,
            [0.0, 0.0, 0.0],
            0.0,
            False,
            1,
            0,
        )
