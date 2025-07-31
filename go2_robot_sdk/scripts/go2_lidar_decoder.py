# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import ctypes
import numpy as np
import os
import math

from wasmtime import Config, Engine, Store, Module, Instance, Func, FuncType
from wasmtime import ValType
from ament_index_python import get_package_share_directory


def update_meshes_for_cloud2(positions, uvs, res, origin, intense_limiter):
    """Собирает облако точек с интенсивностью в формате `[x, y, z, i]`.

    Оптимизация:
    1. *In-place* умножение и смещение позиций без лишних копий.
    2. Расчёт интенсивности через ``np.minimum`` — быстрее, чем ``amin`` с
       сохранением оси.
    3. Фильтрация по маске до конкатенации массивов, сокращая объём
       обрабатываемых данных.
    4. Создание финального массива сразу нужного dtype/shape, без
       промежуточного ``hstack``.
    """

    # -- Подготовка позиций -------------------------------------------------
    pos = np.asarray(positions, dtype=np.float32).reshape(-1, 3)
    pos *= res                          # масштабируем
    pos += origin                       # смещаем

    # -- Интенсивность ------------------------------------------------------
    uv = np.asarray(uvs, dtype=np.float32).reshape(-1, 2)
    intensities = np.minimum(uv[:, 0], uv[:, 1])  # shape (N,)

    # -- Фильтрация по порогу ----------------------------------------------
    mask = intensities > intense_limiter
    if not np.any(mask):
        return np.empty((0, 4), dtype=np.float32)

    pos = pos[mask]
    intensities = intensities[mask]

    # -- Объединяем позиции и интенсивность --------------------------------
    result = np.empty((pos.shape[0], 4), dtype=np.float32)
    result[:, :3] = pos
    result[:, 3] = intensities

    # -- Удаляем дубликаты ---------------------------------------------------
    # np.unique быстрее на C-конт. массиве
    return np.unique(result, axis=0)


class LidarDecoder:
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
        # Copy the sublist for the specified range [start:end]
        sublist = self.HEAPU8[start:end]

        # Replace elements in the list starting from index 'target'
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
        if start + len(value) <= len(self.HEAPU8):
            for i, byte in enumerate(value):
                self.HEAPU8[start + i] = byte
        else:
            raise ValueError("Not enough space to insert bytes at the specified index.")

    def decode(self, compressed_data, data):
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

        positions_slice = self.HEAPU8[self.positions:self.positions + u * 12]
        positions_copy = bytearray(positions_slice)
        p = np.frombuffer(positions_copy, dtype=np.uint8)

        uvs_slice = self.HEAPU8[self.uvs:self.uvs + u * 8]
        uvs_copy = bytearray(uvs_slice)
        r = np.frombuffer(uvs_copy, dtype=np.uint8)

        indices_slice = self.HEAPU8[self.indices:self.indices + u * 24]
        indices_copy = bytearray(indices_slice)
        o = np.frombuffer(indices_copy, dtype=np.uint32)

        return {
            "point_count": c,
            "face_count": u,
            "positions": p,
            "uvs": r,
            "indices": o
        }
