import numpy as np


def pack_xyzi_float32(points: np.ndarray) -> dict:
    points = np.ascontiguousarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 4:
        raise ValueError(f"Invalid point cloud array shape: {points.shape}")

    width = int(points.shape[0])
    point_step = 16

    return {
        "height": 1,
        "width": width,
        "is_bigendian": False,
        "point_step": point_step,
        "row_step": point_step * width,
        "is_dense": True,
        "data": points.tobytes(),
    }
