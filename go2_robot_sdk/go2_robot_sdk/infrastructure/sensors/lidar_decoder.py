# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
LiDAR data decoder for Go2 robot.
Handles decoding of compressed voxel map data from WebRTC stream.
"""

import ctypes
import numpy as np
import os
import math
from typing import Dict, Any

from wasmtime import Config, Engine, Store, Module, Instance, Func, FuncType, ValType
from ament_index_python import get_package_share_directory


def update_meshes_for_cloud2(
    positions: np.ndarray, 
    uvs: np.ndarray, 
    res: float, 
    origin: list, 
    intense_limiter: float
) -> np.ndarray:
    """
    Process LiDAR point cloud data for ROS2 PointCloud2 message.
    Optimized to perform filtering and unique operations on uint8 data before float conversion.
    """
    # Ensure inputs are numpy arrays (they should be from decode, but to be safe)
    # reshape without copy if possible
    pos_view = positions.reshape(-1, 3)
    uv_view = uvs.reshape(-1, 2)

    # Calculate intensities from UV values (uint8)
    # min(u, v)
    intensities = np.min(uv_view, axis=1, keepdims=True)

    # Filter out points below intensity threshold
    # intense_limiter is float, but data is uint8. 
    mask = intensities.flatten() > intense_limiter
    
    # Apply mask - this creates copies but reduces size significantly
    pos_filtered = pos_view[mask]
    int_filtered = intensities[mask]

    # Combine positions with intensities for unique check
    # Stack (N, 3) and (N, 1) -> (N, 4) uint8
    merged = np.hstack((pos_filtered, int_filtered))

    # Remove duplicate points
    # Running unique on uint8 is much faster than on float32
    unique_merged = np.unique(merged, axis=0)
    
    if unique_merged.size == 0:
        return np.empty((0, 4), dtype=np.float32)

    # Convert to float and apply transform only on unique points
    final_pos = unique_merged[:, :3].astype(np.float32)
    final_int = unique_merged[:, 3:].astype(np.float32)

    # Apply resolution scaling and origin offset
    final_pos *= res
    final_pos += origin

    # Combine back: x, y, z, intensity
    return np.hstack((final_pos, final_int))


class LidarDecoder:
    """Original WASM-based LiDAR decoder - the working implementation"""
    
    def __init__(self) -> None:
        config = Config()
        config.wasm_multi_value = True
        config.debug_info = True
        self.store = Store(Engine(config))

        libvoxel_path = os.path.join(
            get_package_share_directory('go2_robot_sdk'),
            "external_lib",
            'libvoxel.wasm')

        self.module = Module.from_file(self.store.engine, libvoxel_path)

        self.a_callback_type = FuncType([ValType.i32()], [ValType.i32()])
        self.b_callback_type = FuncType([ValType.i32(), ValType.i32(), ValType.i32()], [])

        a = Func(self.store, self.a_callback_type, self.adjust_memory_size)
        b = Func(self.store, self.b_callback_type, self.copy_memory_region)

        self.instance = Instance(self.store, self.module, [a, b])

        self.generate = self.instance.exports(self.store)["e"]
        self.malloc = self.instance.exports(self.store)["f"]
        self.free = self.instance.exports(self.store)["g"]
        self.wasm_memory = self.instance.exports(self.store)["c"]

        self.buffer = self.wasm_memory.data_ptr(self.store)
        self.memory_size = self.wasm_memory.data_len(self.store)

        self.buffer_ptr = int.from_bytes(self.buffer, "little")

        self.HEAP8 = (ctypes.c_int8 * self.memory_size).from_address(self.buffer_ptr)
        self.HEAP16 = (ctypes.c_int16 * (self.memory_size // 2)).from_address(self.buffer_ptr)
        self.HEAP32 = (ctypes.c_int32 * (self.memory_size // 4)).from_address(self.buffer_ptr)
        self.HEAPU8 = (ctypes.c_uint8 * self.memory_size).from_address(self.buffer_ptr)
        self.HEAPU16 = (ctypes.c_uint16 * (self.memory_size // 2)).from_address(self.buffer_ptr)
        self.HEAPU32 = (ctypes.c_uint32 * (self.memory_size // 4)).from_address(self.buffer_ptr)
        self.HEAPF32 = (ctypes.c_float * (self.memory_size // 4)).from_address(self.buffer_ptr)
        self.HEAPF64 = (ctypes.c_double * (self.memory_size // 8)).from_address(self.buffer_ptr)

        self.input = self.malloc(self.store, 61440)
        self.decompressBuffer = self.malloc(self.store, 80000)
        self.positions = self.malloc(self.store, 2880000)
        self.uvs = self.malloc(self.store, 1920000)
        self.indices = self.malloc(self.store, 5760000)
        self.decompressedSize = self.malloc(self.store, 4)
        self.faceCount = self.malloc(self.store, 4)
        self.pointCount = self.malloc(self.store, 4)
        self.decompressBufferSize = 80000

    def adjust_memory_size(self, t):
        return len(self.HEAPU8)

    def copy_within(self, target, start, end):
        sublist = self.HEAPU8[start:end]
        for i in range(len(sublist)):
            if target + i < len(self.HEAPU8):
                self.HEAPU8[target + i] = sublist[i]

    def copy_memory_region(self, t, n, a):
        self.copy_within(t, n, n + a)

    def get_value(self, t, n="i8"):
        if n.endswith("*"):
            n = "*"
        if n == "i1" or n == "i8":
            return self.HEAP8[t]
        elif n == "i16":
            return self.HEAP16[t >> 1]
        elif n == "i32" or n == "i64":
            return self.HEAP32[t >> 2]
        elif n == "float":
            return self.HEAPF32[t >> 2]
        elif n == "double":
            return self.HEAPF64[t >> 3]
        elif n == "*":
            return self.HEAPU32[t >> 2]
        else:
            raise ValueError(f"invalid type for getValue: {n}")

    def add_value_arr(self, start, value):
        # Optimized memory copy using ctypes.memmove
        # This replaces the slow python loop: for i, byte in enumerate(value): ...
        if start + len(value) <= self.memory_size:
            ctypes.memmove(self.buffer_ptr + start, value, len(value))
        else:
            raise ValueError("Not enough space to insert bytes at the specified index.")

    def decode(self, compressed_data, data):
        """Original decode method that actually works with the WASM module"""
        self.add_value_arr(self.input, compressed_data)

        some_v = math.floor(data["origin"][2] / data["resolution"])

        self.generate(
            self.store,
            self.input,
            len(compressed_data),
            self.decompressBufferSize,
            self.decompressBuffer,
            self.decompressedSize,
            self.positions,
            self.uvs,
            self.indices,
            self.faceCount,
            self.pointCount,
            some_v
        )

        self.get_value(self.decompressedSize, "i32")
        c = self.get_value(self.pointCount, "i32")
        u = self.get_value(self.faceCount, "i32")

        # Optimized extraction using ctypes.memmove directly to numpy arrays
        # This avoids creating large Python lists and bytearrays
        
        # Positions: u * 12 bytes
        pos_size = u * 12
        p = np.empty(pos_size, dtype=np.uint8)
        ctypes.memmove(p.ctypes.data, self.buffer_ptr + self.positions, pos_size)

        # UVs: u * 8 bytes
        uv_size = u * 8
        r = np.empty(uv_size, dtype=np.uint8)
        ctypes.memmove(r.ctypes.data, self.buffer_ptr + self.uvs, uv_size)

        # Indices: u * 24 bytes
        ind_size = u * 24
        # Note: Indices are read as uint8 bytes here, then cast to uint32 later?
        # Original code: indices_copy = bytearray(slice); o = np.frombuffer(indices_copy, dtype=np.uint32)
        # So we should read bytes first, then view as uint32?
        # Or read directly into uint32 array?
        # 24 bytes per face. If 3 indices (triangles)? 3 * 4 = 12 bytes? 
        # Or 4 indices (quads)? 4 * 4 = 16 bytes?
        # The original code reads u*24 bytes.
        # Let's stick to reading bytes to be safe and consistent with original logic.
        o_bytes = np.empty(ind_size, dtype=np.uint8)
        ctypes.memmove(o_bytes.ctypes.data, self.buffer_ptr + self.indices, ind_size)
        o = np.frombuffer(o_bytes, dtype=np.uint32)

        return {
            "point_count": c,
            "face_count": u,
            "positions": p,
            "uvs": r,
            "indices": o
        }


def get_voxel_decoder() -> LidarDecoder:
    """
    Get a LidarDecoder instance.
    
    Returns:
        Initialized LidarDecoder (the working implementation)
    """
    return LidarDecoder()


def decode_lidar_data(
    compressed_data: bytes,
    resolution: float = 0.01,
    origin: list = [0.0, 0.0, 0.0],
    intensity_threshold: float = 0.1
) -> np.ndarray:
    """
    High-level function to decode LiDAR data.
    
    Args:
        compressed_data: Compressed voxel map data
        resolution: Point cloud resolution
        origin: Origin offset
        intensity_threshold: Minimum intensity to include points
        
    Returns:
        Processed point cloud array
    """
    decoder = get_voxel_decoder()
    metadata = {
        "origin": origin,
        "resolution": resolution
    }
    
    result = decoder.decode(compressed_data, metadata)
    
    # Convert to expected format
    positions = result["positions"]
    uvs = result["uvs"]
    
    return update_meshes_for_cloud2(
        positions, uvs, resolution, origin, intensity_threshold
    ) 