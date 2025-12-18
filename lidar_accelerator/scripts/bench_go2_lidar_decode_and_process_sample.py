import argparse
import json
import statistics
import struct
import time
from pathlib import Path

import numpy as np


def _parse_ulidar_array_buffer(buf: bytes):
    if len(buf) < 4:
        raise ValueError("buffer too short")

    json_length = struct.unpack("<H", buf[:2])[0]
    if len(buf) < 4 + json_length:
        raise ValueError("buffer too short for json segment")

    json_segment = buf[4 : 4 + json_length]
    compressed = buf[4 + json_length :]

    metadata = json.loads(json_segment.decode("utf-8"))
    data = metadata.get("data", metadata)
    return data, compressed


def _bench(fn, iters: int):
    times = []
    out = None
    for _ in range(iters):
        t0 = time.perf_counter()
        out = fn()
        t1 = time.perf_counter()
        times.append((t1 - t0) * 1000.0)
    return times, out


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("sample", type=str, help="Path to ulidar_array_buffer_*.bin")
    p.add_argument("--warmup", type=int, default=3)
    p.add_argument("--iters", type=int, default=20)
    p.add_argument("--intensity", type=float, default=0.0)
    p.add_argument("--deduplicate", action="store_true")
    p.add_argument("--downsample", type=int, default=1)
    p.add_argument("--max-points", type=int, default=0)
    p.add_argument("--csv", action="store_true")
    args = p.parse_args()

    import lidar_accelerator
    from go2_robot_sdk.infrastructure.sensors.lidar_decoder import (
        get_voxel_decoder,
        update_meshes_for_cloud2,
    )

    buf = Path(args.sample).read_bytes()
    data, compressed = _parse_ulidar_array_buffer(buf)

    resolution = float(data.get("resolution", 0.01) or 0.01)
    origin = list(data.get("origin", [0.0, 0.0, 0.0]))

    decoder = get_voxel_decoder()

    def python_end_to_end():
        decoded = decoder.decode(compressed, {"origin": origin, "resolution": resolution})
        return update_meshes_for_cloud2(
            decoded["positions"],
            decoded["uvs"],
            resolution,
            origin,
            float(args.intensity),
            deduplicate=bool(args.deduplicate),
            downsample_step=int(args.downsample),
            max_points=int(args.max_points),
            use_cpp_accel=False,
        )

    def cpp_end_to_end():
        return lidar_accelerator.decode_and_process(
            compressed,
            resolution,
            origin,
            float(args.intensity),
            bool(args.deduplicate),
            int(args.downsample),
            int(args.max_points),
        )

    for _ in range(int(args.warmup)):
        _ = python_end_to_end()
        _ = cpp_end_to_end()

    py_times, py_out = _bench(python_end_to_end, int(args.iters))
    cpp_times, cpp_out = _bench(cpp_end_to_end, int(args.iters))

    if py_out.shape != cpp_out.shape or not np.allclose(
        np.sort(py_out, axis=0), np.sort(cpp_out, axis=0)
    ):
        # Sorting per-column is not perfect, but catches obvious mismatches.
        raise SystemExit("Output mismatch (python vs cpp)")

    def summarize(label: str, times_ms):
        return {
            "label": label,
            "iters": len(times_ms),
            "mean_ms": statistics.mean(times_ms),
            "median_ms": statistics.median(times_ms),
            "min_ms": min(times_ms),
            "max_ms": max(times_ms),
        }

    rows = [summarize("python_end_to_end", py_times), summarize("cpp_end_to_end", cpp_times)]

    if args.csv:
        print("label,iters,mean_ms,median_ms,min_ms,max_ms")
        for r in rows:
            print(
                f"{r['label']},{r['iters']},{r['mean_ms']:.6f},{r['median_ms']:.6f},{r['min_ms']:.6f},{r['max_ms']:.6f}"
            )
    else:
        for r in rows:
            print(
                f"{r['label']}: iters={r['iters']} mean={r['mean_ms']:.3f}ms "
                f"median={r['median_ms']:.3f}ms min={r['min_ms']:.3f}ms max={r['max_ms']:.3f}ms"
            )
        print(f"speedup(cpp): {rows[0]['mean_ms'] / rows[1]['mean_ms']:.2f}x")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
