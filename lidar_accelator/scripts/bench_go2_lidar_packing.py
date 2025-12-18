import argparse
import statistics
import time

import numpy as np


def _bench(fn, iters: int):
    times = []
    last = None
    for _ in range(iters):
        t0 = time.perf_counter()
        last = fn()
        t1 = time.perf_counter()
        times.append((t1 - t0) * 1000.0)
    return times, last


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--points", type=int, default=30000)
    p.add_argument("--warmup", type=int, default=5)
    p.add_argument("--iters", type=int, default=50)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--csv", action="store_true")
    args = p.parse_args()

    rng = np.random.default_rng(args.seed)
    n = int(args.points)

    points = rng.standard_normal(size=(n, 4)).astype(np.float32)
    expected = points.tobytes()

    try:
        import lidar_accelator

        has_cpp = True
    except Exception:
        lidar_accelator = None
        has_cpp = False

    def py_tobytes():
        return points.tobytes()

    def cpp_bytes():
        return lidar_accelator.pack_xyzi_f32_to_bytes(points)

    for _ in range(int(args.warmup)):
        _ = py_tobytes()
        if has_cpp:
            _ = cpp_bytes()

    py_times, py_out = _bench(py_tobytes, int(args.iters))
    assert py_out == expected

    cpp_times = None
    cpp_out = None
    if has_cpp:
        cpp_times, cpp_out = _bench(cpp_bytes, int(args.iters))
        assert cpp_out == expected

    def summarize(label: str, times_ms):
        return {
            "label": label,
            "n": n,
            "iters": len(times_ms),
            "mean_ms": statistics.mean(times_ms),
            "median_ms": statistics.median(times_ms),
            "min_ms": min(times_ms),
            "max_ms": max(times_ms),
        }

    rows = [summarize("numpy_tobytes", py_times)]
    if has_cpp:
        rows.append(summarize("cpp_pack_bytes", cpp_times))

    if args.csv:
        print("label,points,iters,mean_ms,median_ms,min_ms,max_ms")
        for r in rows:
            print(
                f"{r['label']},{r['n']},{r['iters']},{r['mean_ms']:.6f},{r['median_ms']:.6f},{r['min_ms']:.6f},{r['max_ms']:.6f}"
            )
    else:
        for r in rows:
            print(
                f"{r['label']}: points={r['n']} iters={r['iters']} "
                f"mean={r['mean_ms']:.3f}ms median={r['median_ms']:.3f}ms "
                f"min={r['min_ms']:.3f}ms max={r['max_ms']:.3f}ms"
            )

        if has_cpp:
            print(f"speedup(cpp/np): {rows[0]['mean_ms'] / rows[1]['mean_ms']:.2f}x")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
