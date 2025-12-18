import argparse
import csv
import itertools
import os
import statistics
import time

import numpy as np

from go2_robot_sdk.infrastructure.sensors.lidar_decoder import update_meshes_for_cloud2


def _parse_bool(s: str) -> bool:
    return s.strip().lower() in ("1", "true", "yes", "on")


def _bench(fn, iters: int):
    times = []
    for _ in range(iters):
        t0 = time.perf_counter()
        out = fn()
        t1 = time.perf_counter()
        times.append((t1 - t0) * 1000.0)
    return times, out


def _parse_int_list(s: str):
    if not s:
        return []
    out = []
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        out.append(int(part))
    return out


def _parse_bool_list(s: str):
    if not s:
        return []
    out = []
    for part in s.split(","):
        part = part.strip().lower()
        if not part:
            continue
        out.append(part in ("1", "true", "yes", "on"))
    return out


def _summarize(times_ms):
    return {
        "iters": len(times_ms),
        "mean_ms": statistics.mean(times_ms),
        "median_ms": statistics.median(times_ms),
        "min_ms": min(times_ms),
        "max_ms": max(times_ms),
    }


def _run_one_case(
    *,
    n: int,
    warmup: int,
    iters: int,
    res: float,
    origin,
    intensity: float,
    dedup: bool,
    downsample: int,
    max_points: int,
    seed: int,
):
    rng = np.random.default_rng(seed)
    positions = rng.integers(0, 255, size=(n * 3,), dtype=np.uint8)
    uvs = rng.integers(0, 255, size=(n * 2,), dtype=np.uint8)

    try:
        import lidar_accelator

        has_cpp = True
    except Exception:
        lidar_accelator = None
        has_cpp = False

    def py_path():
        return update_meshes_for_cloud2(
            positions,
            uvs,
            res,
            list(origin),
            intensity,
            deduplicate=dedup,
            downsample_step=downsample,
            max_points=max_points,
            use_cpp_accel=False,
        )

    def cpp_direct():
        return lidar_accelator.process_u8_to_xyzi_f32(
            positions,
            uvs,
            res,
            list(origin),
            intensity,
            dedup,
            downsample,
            max_points,
        )

    def cpp_switch():
        return update_meshes_for_cloud2(
            positions,
            uvs,
            res,
            list(origin),
            intensity,
            deduplicate=dedup,
            downsample_step=downsample,
            max_points=max_points,
            use_cpp_accel=True,
        )

    for _ in range(int(warmup)):
        _ = py_path()
        if has_cpp:
            _ = cpp_direct()
            _ = cpp_switch()

    py_times, py_out = _bench(py_path, int(iters))
    py_stats = _summarize(py_times)

    cpp_stats = None
    switch_stats = None

    if has_cpp:
        cpp_times, cpp_out = _bench(cpp_direct, int(iters))
        switch_times, switch_out = _bench(cpp_switch, int(iters))
        cpp_stats = _summarize(cpp_times)
        switch_stats = _summarize(switch_times)

        if not np.allclose(cpp_out, py_out):
            raise SystemExit("cpp_direct output mismatch vs python")
        if not np.allclose(switch_out, py_out):
            raise SystemExit("cpp_switch output mismatch vs python")

    return has_cpp, py_stats, cpp_stats, switch_stats


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--points", type=int, default=3000)
    p.add_argument("--warmup", type=int, default=3)
    p.add_argument("--iters", type=int, default=20)
    p.add_argument("--res", type=float, default=0.01)
    p.add_argument("--origin", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    p.add_argument("--intensity", type=float, default=0.0)
    p.add_argument("--deduplicate", type=str, default="false")
    p.add_argument("--downsample", type=int, default=1)
    p.add_argument("--max-points", type=int, default=0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--csv", action="store_true")

    p.add_argument("--sweep", action="store_true")
    p.add_argument("--sweep-out", type=str, default="")
    p.add_argument("--points-list", type=str, default="")
    p.add_argument("--downsample-list", type=str, default="")
    p.add_argument("--max-points-list", type=str, default="")
    p.add_argument("--deduplicate-list", type=str, default="")
    args = p.parse_args()

    if args.sweep:
        points_list = _parse_int_list(args.points_list)
        if not points_list:
            points_list = [3000, 30000]

        down_list = _parse_int_list(args.downsample_list)
        if not down_list:
            down_list = [1, 32]

        max_list = _parse_int_list(args.max_points_list)
        if not max_list:
            max_list = [0, int(os.getenv("LIDAR_MAX_POINTS", "3000") or 3000)]

        dedup_list = _parse_bool_list(args.deduplicate_list)
        if not dedup_list:
            dedup_list = [False, True]

        out_f = None
        try:
            if args.sweep_out:
                out_f = open(args.sweep_out, "w", newline="")
                out = out_f
            else:
                out = None

            fp = out if out is not None else os.sys.stdout
            w = csv.writer(fp)
            w.writerow(
                [
                    "points",
                    "downsample",
                    "max_points",
                    "deduplicate",
                    "label",
                    "iters",
                    "mean_ms",
                    "median_ms",
                    "min_ms",
                    "max_ms",
                    "speedup_vs_python",
                ]
            )

            for n, downsample, max_points, dedup in itertools.product(
                points_list, down_list, max_list, dedup_list
            ):
                has_cpp, py_stats, cpp_stats, switch_stats = _run_one_case(
                    n=int(n),
                    warmup=int(args.warmup),
                    iters=int(args.iters),
                    res=float(args.res),
                    origin=list(args.origin),
                    intensity=float(args.intensity),
                    dedup=bool(dedup),
                    downsample=int(downsample),
                    max_points=int(max_points),
                    seed=int(args.seed),
                )

                w.writerow(
                    [
                        int(n),
                        int(downsample),
                        int(max_points),
                        bool(dedup),
                        "python",
                        py_stats["iters"],
                        f"{py_stats['mean_ms']:.6f}",
                        f"{py_stats['median_ms']:.6f}",
                        f"{py_stats['min_ms']:.6f}",
                        f"{py_stats['max_ms']:.6f}",
                        "1.0",
                    ]
                )

                if has_cpp and cpp_stats is not None:
                    w.writerow(
                        [
                            int(n),
                            int(downsample),
                            int(max_points),
                            bool(dedup),
                            "cpp_direct",
                            cpp_stats["iters"],
                            f"{cpp_stats['mean_ms']:.6f}",
                            f"{cpp_stats['median_ms']:.6f}",
                            f"{cpp_stats['min_ms']:.6f}",
                            f"{cpp_stats['max_ms']:.6f}",
                            f"{py_stats['mean_ms'] / cpp_stats['mean_ms']:.6f}",
                        ]
                    )

                if has_cpp and switch_stats is not None:
                    w.writerow(
                        [
                            int(n),
                            int(downsample),
                            int(max_points),
                            bool(dedup),
                            "cpp_via_switch",
                            switch_stats["iters"],
                            f"{switch_stats['mean_ms']:.6f}",
                            f"{switch_stats['median_ms']:.6f}",
                            f"{switch_stats['min_ms']:.6f}",
                            f"{switch_stats['max_ms']:.6f}",
                            f"{py_stats['mean_ms'] / switch_stats['mean_ms']:.6f}",
                        ]
                    )

        finally:
            if out_f is not None:
                out_f.close()

        return 0

    dedup = _parse_bool(args.deduplicate)
    n = int(args.points)

    has_cpp, py_stats, cpp_stats, switch_stats = _run_one_case(
        n=n,
        warmup=int(args.warmup),
        iters=int(args.iters),
        res=float(args.res),
        origin=list(args.origin),
        intensity=float(args.intensity),
        dedup=bool(dedup),
        downsample=int(args.downsample),
        max_points=int(args.max_points),
        seed=int(args.seed),
    )

    if args.csv:
        print("label,points,iters,mean_ms,median_ms,min_ms,max_ms")
        print(
            f"python,{n},{py_stats['iters']},{py_stats['mean_ms']:.6f},{py_stats['median_ms']:.6f},{py_stats['min_ms']:.6f},{py_stats['max_ms']:.6f}"
        )
        if has_cpp and cpp_stats is not None and switch_stats is not None:
            print(
                f"cpp_direct,{n},{cpp_stats['iters']},{cpp_stats['mean_ms']:.6f},{cpp_stats['median_ms']:.6f},{cpp_stats['min_ms']:.6f},{cpp_stats['max_ms']:.6f}"
            )
            print(
                f"cpp_via_switch,{n},{switch_stats['iters']},{switch_stats['mean_ms']:.6f},{switch_stats['median_ms']:.6f},{switch_stats['min_ms']:.6f},{switch_stats['max_ms']:.6f}"
            )
        return 0

    print(
        f"python: points={n} iters={py_stats['iters']} mean={py_stats['mean_ms']:.3f}ms "
        f"median={py_stats['median_ms']:.3f}ms min={py_stats['min_ms']:.3f}ms max={py_stats['max_ms']:.3f}ms"
    )
    if has_cpp and cpp_stats is not None and switch_stats is not None:
        print(
            f"cpp_direct: points={n} iters={cpp_stats['iters']} mean={cpp_stats['mean_ms']:.3f}ms "
            f"median={cpp_stats['median_ms']:.3f}ms min={cpp_stats['min_ms']:.3f}ms max={cpp_stats['max_ms']:.3f}ms"
        )
        print(
            f"cpp_via_switch: points={n} iters={switch_stats['iters']} mean={switch_stats['mean_ms']:.3f}ms "
            f"median={switch_stats['median_ms']:.3f}ms min={switch_stats['min_ms']:.3f}ms max={switch_stats['max_ms']:.3f}ms"
        )
        print(f"speedup(cpp_direct): {py_stats['mean_ms'] / cpp_stats['mean_ms']:.2f}x")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
