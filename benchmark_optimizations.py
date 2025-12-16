import time
import threading
import queue
import ctypes
import tracemalloc
import numpy as np
import statistics as stats

# ---------------------------
# Utilities
# ---------------------------
def now():
    return time.perf_counter()

def percentile(values, p):
    if not values:
        return float("nan")
    a = np.array(values, dtype=np.float64)
    return float(np.percentile(a, p))

def run_with_tracemalloc(fn):
    tracemalloc.start()
    t0 = now()
    out = fn()
    t1 = now()
    cur, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return out, (t1 - t0), peak

def fmt_bytes(n):
    for unit in ["B", "KB", "MB", "GB"]:
        if n < 1024:
            return f"{n:.1f}{unit}"
        n /= 1024
    return f"{n:.1f}TB"


def _fmt_cell(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        if np.isnan(v):
            return "nan"
        return f"{v:.2f}"
    return str(v)


def print_table(headers, rows):
    cols = [len(h) for h in headers]
    for r in rows:
        for i, h in enumerate(headers):
            cols[i] = max(cols[i], len(_fmt_cell(r.get(h))))

    def line(sep, fill):
        return sep + sep.join(fill * (w + 2) for w in cols) + sep

    def row(values, sep):
        parts = []
        for i, v in enumerate(values):
            parts.append(" " + _fmt_cell(v).ljust(cols[i]) + " ")
        return sep + sep.join(parts) + sep

    print(line("+", "-"))
    print(row(headers, "|"))
    print(line("+", "-"))
    for r in rows:
        print(row([r.get(h) for h in headers], "|"))
    print(line("+", "-"))

# ---------------------------
# 1) WebRTC 병목: 동기 처리 vs 비동기+Head-Drop
# ---------------------------
def heavy_decode_simulation(payload: bytes, work_ms=12):
    # CPU를 좀 쓰는 "디코딩 비슷한" 작업
    # payload 크기 영향을 주도록 numpy 연산 포함
    x = np.frombuffer(payload, dtype=np.uint8)
    _ = (x.sum() + x.mean())  # consume CPU a bit
    # 처리 시간이 일정 이상 걸리게 sleep(실제 decode 시간 가정)
    time.sleep(work_ms / 1000.0)
    return _

def bench_sync_processing(
    n_msgs=300,
    msg_bytes=80_000,
    period_ms=10,     # 수신 주기 (예: 100Hz면 10ms)
    work_ms=12        # 처리 시간이 더 길면 backlog 발생
):
    latencies = []
    t_next = now()
    for i in range(n_msgs):
        # "수신 시각"을 맞추기
        t_next += period_ms / 1000.0
        while now() < t_next:
            pass

        recv_t = now()
        payload = np.random.randint(0, 256, size=msg_bytes, dtype=np.uint8).tobytes()

        # 변경 전: 콜백(수신 스레드)에서 바로 decode
        heavy_decode_simulation(payload, work_ms=work_ms)

        done_t = now()
        latencies.append((done_t - recv_t) * 1000.0)
    return {
        "mode": "SYNC (decode in callback)",
        "n": n_msgs,
        "drop": 0,
        "lat_ms_avg": stats.mean(latencies),
        "lat_ms_p50": percentile(latencies, 50),
        "lat_ms_p95": percentile(latencies, 95),
        "throughput_msg_s": n_msgs / (sum(latencies) / 1000.0),
    }

def bench_async_head_drop(
    n_msgs=300,
    msg_bytes=80_000,
    period_ms=10,
    work_ms=12,
    maxsize=2
):
    q = queue.Queue(maxsize=maxsize)
    worker_running = True

    produced = 0
    processed = 0
    dropped = 0

    # latency: recv->finish (처리된 것만 측정)
    latencies = []
    # store recv timestamp per payload-id
    # (여기서는 payload 자체가 bytes라 ID가 없어서, (recv_t, payload) 튜플로 큐에 넣음)
    def worker():
        nonlocal processed, worker_running
        while worker_running:
            try:
                recv_t, payload = q.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                heavy_decode_simulation(payload, work_ms=work_ms)
                done_t = now()
                latencies.append((done_t - recv_t) * 1000.0)
                processed += 1
            finally:
                q.task_done()

    th = threading.Thread(target=worker, daemon=True)
    th.start()

    t_next = now()
    for i in range(n_msgs):
        t_next += period_ms / 1000.0
        while now() < t_next:
            pass

        recv_t = now()
        payload = np.random.randint(0, 256, size=msg_bytes, dtype=np.uint8).tobytes()
        produced += 1

        # 변경 후: 큐가 꽉 차면 오래된 프레임 drop (Head-Drop)
        if q.full():
            try:
                q.get_nowait()
                q.task_done()   # 중요: drop한 만큼 unfinished_tasks 맞춤
                dropped += 1
            except queue.Empty:
                pass

        try:
            q.put_nowait((recv_t, payload))
        except queue.Full:
            # 방어적: 이론상 거의 안 나야 함
            dropped += 1

    # 남은 처리 조금 기다림(실시간 시스템이면 굳이 다 처리 안 기다리고 종료해도 됨)
    q.join()
    worker_running = False

    # async는 "수신 스레드가 안 막힘"이 핵심이라, throughput은 produced 기준보다
    # processed 기준이 더 의미 있음
    elapsed_s = (n_msgs * period_ms) / 1000.0
    return {
        "mode": f"ASYNC + HEAD-DROP (maxsize={maxsize})",
        "n": produced,
        "processed": processed,
        "drop": dropped,
        "drop_pct": 100.0 * dropped / max(1, produced),
        "lat_ms_avg": stats.mean(latencies) if latencies else float("nan"),
        "lat_ms_p50": percentile(latencies, 50),
        "lat_ms_p95": percentile(latencies, 95),
        "processed_msg_s": processed / max(1e-9, elapsed_s),
    }

# ---------------------------
# 2) memmove vs Python loop copy
# ---------------------------
def bench_memcopy(size_bytes=2_000_000, iters=50):
    src = (np.random.randint(0, 256, size=size_bytes, dtype=np.uint8)).tobytes()

    # before: python loop
    def before():
        dst = bytearray(size_bytes)
        for _ in range(iters):
            # 1바이트씩 복사(의도적으로 느리게)
            for i, b in enumerate(src):
                dst[i] = b
        return dst[0]

    # after: ctypes.memmove
    def after():
        dst = (ctypes.c_ubyte * size_bytes)()
        dst_ptr = ctypes.addressof(dst)
        src_buf = (ctypes.c_ubyte * size_bytes).from_buffer_copy(src)
        src_ptr = ctypes.addressof(src_buf)
        for _ in range(iters):
            ctypes.memmove(dst_ptr, src_ptr, size_bytes)
        return dst[0]

    _, t_before, peak_before = run_with_tracemalloc(before)
    _, t_after, peak_after = run_with_tracemalloc(after)

    return {
        "memcopy_size": size_bytes,
        "iters": iters,
        "before_s": t_before,
        "after_s": t_after,
        "speedup_x": (t_before / t_after) if t_after > 0 else float("inf"),
        "peak_before": peak_before,
        "peak_after": peak_after,
    }

# ---------------------------
# 3) early filter+unique (uint8) vs float 변환 후 unique
# ---------------------------
def unique_before(positions_u8, uvs_u8, res, origin, intense_limiter):
    # 변경 전 스타일(개념): float 변환 후 unique
    pos = positions_u8.reshape(-1, 3).astype(np.float32) * res + origin
    uv = uvs_u8.reshape(-1, 2).astype(np.float32)
    intensities = np.min(uv, axis=1, keepdims=True)
    merged = np.hstack((pos, intensities))
    filtered = merged[merged[:, 3] > intense_limiter]
    uniq = np.unique(filtered, axis=0)
    return uniq

def unique_after(positions_u8, uvs_u8, res, origin, intense_limiter):
    # 변경 후 스타일(개념): uint8에서 필터+unique 후 float 변환
    pos_view = positions_u8.reshape(-1, 3)
    uv_view = uvs_u8.reshape(-1, 2)
    intens_u8 = np.min(uv_view, axis=1, keepdims=True)
    mask = intens_u8.flatten() > intense_limiter
    merged_u8 = np.hstack((pos_view[mask], intens_u8[mask]))
    uniq_u8 = np.unique(merged_u8, axis=0)
    if uniq_u8.size == 0:
        return np.empty((0, 4), dtype=np.float32)
    pos_f = uniq_u8[:, :3].astype(np.float32) * res + origin
    i_f = uniq_u8[:, 3:].astype(np.float32)
    return np.hstack((pos_f, i_f))

def bench_unique(n_points=200_000, iters=10):
    # uint8 원본(대용량)
    positions = np.random.randint(0, 256, size=(n_points, 3), dtype=np.uint8).reshape(-1)
    uvs = np.random.randint(0, 256, size=(n_points, 2), dtype=np.uint8).reshape(-1)
    res = 0.01
    origin = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    intense_limiter = 10  # uint8 기준

    def before():
        out = None
        for _ in range(iters):
            out = unique_before(positions, uvs, res, origin, intense_limiter)
        return out.shape[0]

    def after():
        out = None
        for _ in range(iters):
            out = unique_after(positions, uvs, res, origin, intense_limiter)
        return out.shape[0]

    n_before, t_before, peak_before = run_with_tracemalloc(before)
    n_after, t_after, peak_after = run_with_tracemalloc(after)

    return {
        "n_points": n_points,
        "iters": iters,
        "before_s": t_before,
        "after_s": t_after,
        "speedup_x": (t_before / t_after) if t_after > 0 else float("inf"),
        "peak_before": peak_before,
        "peak_after": peak_after,
        "out_points_before": n_before,
        "out_points_after": n_after,
    }

# ---------------------------
# 4) packing loop vs tobytes (PointCloud2 직렬화 회피 개념)
# ---------------------------
def pack_before(points_f32):
    # 느린 per-point packing 흉내 (python loop)
    import struct
    n = points_f32.shape[0]
    buf = bytearray(n * 16)  # 4 float32
    for i in range(n):
        struct.pack_into("<ffff", buf, i * 16,
                         float(points_f32[i, 0]),
                         float(points_f32[i, 1]),
                         float(points_f32[i, 2]),
                         float(points_f32[i, 3]))
    return buf

def pack_after(points_f32):
    # 변경 후 스타일: 한 번에 bytes
    return points_f32.astype(np.float32, copy=False).tobytes()

def bench_pack(n_points=200_000, iters=5):
    pts = np.random.randn(n_points, 4).astype(np.float32)

    def before():
        b = None
        for _ in range(iters):
            b = pack_before(pts)
        return len(b)

    def after():
        b = None
        for _ in range(iters):
            b = pack_after(pts)
        return len(b)

    s_before, t_before, peak_before = run_with_tracemalloc(before)
    s_after, t_after, peak_after = run_with_tracemalloc(after)
    return {
        "n_points": n_points,
        "iters": iters,
        "bytes": s_after,
        "before_s": t_before,
        "after_s": t_after,
        "speedup_x": (t_before / t_after) if t_after > 0 else float("inf"),
        "peak_before": peak_before,
        "peak_after": peak_after,
    }


def run_webrtc_case(case_name, period_ms, work_ms, maxsize=None, n_msgs=300, msg_bytes=80_000):
    rows = []

    r_sync, t_sync, p_sync = run_with_tracemalloc(
        lambda: bench_sync_processing(
            n_msgs=n_msgs,
            msg_bytes=msg_bytes,
            period_ms=period_ms,
            work_ms=work_ms,
        )
    )
    rows.append({
        "case": case_name,
        "mode": "SYNC",
        "period_ms": period_ms,
        "work_ms": work_ms,
        "maxsize": "-",
        "processed": r_sync.get("n"),
        "drop_pct": 0.0,
        "p50_ms": r_sync.get("lat_ms_p50"),
        "p95_ms": r_sync.get("lat_ms_p95"),
        "total_s": t_sync,
        "peak": fmt_bytes(p_sync),
    })

    r_async, t_async, p_async = run_with_tracemalloc(
        lambda: bench_async_head_drop(
            n_msgs=n_msgs,
            msg_bytes=msg_bytes,
            period_ms=period_ms,
            work_ms=work_ms,
            maxsize=maxsize if maxsize is not None else 2,
        )
    )
    rows.append({
        "case": case_name,
        "mode": "ASYNC",
        "period_ms": period_ms,
        "work_ms": work_ms,
        "maxsize": maxsize if maxsize is not None else 2,
        "processed": r_async.get("processed"),
        "drop_pct": r_async.get("drop_pct"),
        "p50_ms": r_async.get("lat_ms_p50"),
        "p95_ms": r_async.get("lat_ms_p95"),
        "total_s": t_async,
        "peak": fmt_bytes(p_async),
    })

    return rows


def run_webrtc_tuning_case(case_name, period_ms, work_ms, maxsizes, n_msgs=300, msg_bytes=80_000):
    rows = []
    for m in maxsizes:
        r_async, t_async, p_async = run_with_tracemalloc(
            lambda m=m: bench_async_head_drop(
                n_msgs=n_msgs,
                msg_bytes=msg_bytes,
                period_ms=period_ms,
                work_ms=work_ms,
                maxsize=m,
            )
        )
        rows.append({
            "case": case_name,
            "mode": "ASYNC",
            "period_ms": period_ms,
            "work_ms": work_ms,
            "maxsize": m,
            "processed": r_async.get("processed"),
            "drop_pct": r_async.get("drop_pct"),
            "p50_ms": r_async.get("lat_ms_p50"),
            "p95_ms": r_async.get("lat_ms_p95"),
            "total_s": t_async,
            "peak": fmt_bytes(p_async),
        })
    return rows

# ---------------------------
# Main report
# ---------------------------
def main():
    print("\n=== [1] WebRTC: Case Matrix (A/B/C) ===")
    webrtc_rows = []
    webrtc_rows += run_webrtc_case(
        case_name="A(normal)",
        period_ms=50,
        work_ms=12,
        maxsize=2,
    )
    webrtc_rows += run_webrtc_case(
        case_name="B(overload)",
        period_ms=10,
        work_ms=12,
        maxsize=2,
    )
    webrtc_rows += run_webrtc_tuning_case(
        case_name="C(tuning)",
        period_ms=10,
        work_ms=12,
        maxsizes=[2, 4, 8],
    )

    headers = [
        "case",
        "mode",
        "period_ms",
        "work_ms",
        "maxsize",
        "processed",
        "drop_pct",
        "p50_ms",
        "p95_ms",
        "total_s",
        "peak",
    ]
    print_table(headers, webrtc_rows)

    print("\n=== [2] Memory copy: python loop vs ctypes.memmove ===")
    mc = bench_memcopy(size_bytes=500_000, iters=10)
    print(f"- memcopy size={mc['memcopy_size']}B iters={mc['iters']}")
    print(f"  before={mc['before_s']:.3f}s peak={fmt_bytes(mc['peak_before'])}")
    print(f"  after ={mc['after_s']:.3f}s peak={fmt_bytes(mc['peak_after'])}")
    print(f"  speedup ~ {mc['speedup_x']:.1f}x")

    print("\n=== [3] LiDAR unique: float-unique vs uint8 early unique ===")
    uq = bench_unique(n_points=120_000, iters=5)
    print(f"- points={uq['n_points']} iters={uq['iters']}")
    print(f"  before={uq['before_s']:.3f}s peak={fmt_bytes(uq['peak_before'])} out={uq['out_points_before']}")
    print(f"  after ={uq['after_s']:.3f}s peak={fmt_bytes(uq['peak_after'])} out={uq['out_points_after']}")
    print(f"  speedup ~ {uq['speedup_x']:.1f}x")

    print("\n=== [4] Packing: per-point loop vs tobytes ===")
    pk = bench_pack(n_points=120_000, iters=3)
    print(f"- points={pk['n_points']} iters={pk['iters']} bytes={pk['bytes']}")
    print(f"  before={pk['before_s']:.3f}s peak={fmt_bytes(pk['peak_before'])}")
    print(f"  after ={pk['after_s']:.3f}s peak={fmt_bytes(pk['peak_after'])}")
    print(f"  speedup ~ {pk['speedup_x']:.1f}x")

    print("\n[NOTE]")
    print("- 이 결과는 절대값이 아니라 '상대 비교(전/후) 트렌드'를 보기 위한 것입니다.")
    print("- Jetson Orin에서는 메모리/CPU 특성이 달라서 speedup 배수가 더 커지거나 작아질 수 있습니다.")

if __name__ == '__main__':
    main()