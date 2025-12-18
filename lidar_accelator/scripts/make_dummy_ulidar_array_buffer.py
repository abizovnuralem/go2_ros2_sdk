import argparse
import json
import struct
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("out", type=str, help="Output .bin file path")
    p.add_argument("--resolution", type=float, default=0.01)
    p.add_argument("--origin", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    p.add_argument("--compressed-bytes", type=int, default=1024)
    args = p.parse_args()

    meta = {
        "topic": "ULIDAR_ARRAY",
        "data": {
            "resolution": float(args.resolution),
            "origin": list(args.origin),
            "stamp": 0.0,
            "width": [0, 0, 0],
            "src_size": int(args.compressed_bytes),
        },
    }

    json_bytes = json.dumps(meta).encode("utf-8")
    header = struct.pack("<H", len(json_bytes)) + b"\x00\x00"

    # Dummy bytes: this is NOT real libvoxel.wasm input.
    compressed = bytes([0x42]) * int(args.compressed_bytes)

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_bytes(header + json_bytes + compressed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
