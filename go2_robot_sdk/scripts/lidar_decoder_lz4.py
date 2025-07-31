import numpy as np
import lz4.block


def decompress(compressed_data: bytes, decomp_size: int) -> bytes:
    """Декомпрессия LZ4-буфера с указанием ожидаемого размера."""
    return lz4.block.decompress(compressed_data, uncompressed_size=decomp_size)


def bits_to_points(buf: bytes, origin, resolution: float = 0.05):
    """Преобразовать битовую карту высот в массив XYZ-точек (world-coords)."""
    buf = np.frombuffer(bytearray(buf), dtype=np.uint8)
    nonzero_indices = np.nonzero(buf)[0]
    points = []
    for n in nonzero_indices:
        byte_value = buf[n]
        z = n // 0x800
        n_slice = n % 0x800
        y = n_slice // 0x10
        x_base = (n_slice % 0x10) * 8
        for bit_pos in range(8):
            if byte_value & (1 << (7 - bit_pos)):
                x = x_base + bit_pos
                points.append((x, y, z))
    return np.array(points) * resolution + origin


class LidarDecoderLz4:
    """Декодер LZ4, совместимый по выходному формату с LibVoxelDecoder."""

    def decode(self, compressed_data: bytes, metadata: dict):
        # 1. Декомпрессия
        decompressed = decompress(compressed_data, metadata["src_size"])
        # 2. Массив точек в мировых координатах
        points_world = bits_to_points(
            decompressed,
            origin=np.array(metadata["origin"], dtype=np.float32),
            resolution=metadata["resolution"],
        )
        # 3. Перевод в воксельные индексы
        voxel_coords = ((points_world - metadata["origin"]) / metadata["resolution"]).astype(np.uint8)
        positions = voxel_coords.flatten()
        uvs = np.full(positions.size // 3 * 2, 255, dtype=np.uint8)
        indices = np.zeros(positions.size // 3 * 3, dtype=np.uint32)
        point_count = voxel_coords.shape[0]
        face_count = point_count
        return {
            "point_count": point_count,
            "face_count": face_count,
            "positions": positions,
            "uvs": uvs,
            "indices": indices,
            "points": points_world,
        } 