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
from typing import Tuple, List, Optional

from wasmtime import Config, Engine, Store, Module, Instance, Func, FuncType, ValType
from ament_index_python import get_package_share_directory


def update_meshes_for_cloud2(
    positions: List[float], 
    uvs: List[float], 
    res: float, 
    origin: List[float], 
    intense_limiter: float
) -> np.ndarray:
    """
    Process LiDAR point cloud data for ROS2 PointCloud2 message.
    
    Args:
        positions: Raw position data from LiDAR
        uvs: UV coordinate data
        res: Resolution factor
        origin: Origin offset coordinates
        intense_limiter: Intensity threshold filter
        
    Returns:
        Processed point cloud array with x,y,z,intensity
    """
    # Convert positions to numpy array for vectorized operations
    position_array = np.array(positions).reshape(-1, 3).astype(np.float32)

    # Apply resolution scaling
    position_array *= res

    # Apply origin offset
    position_array += origin

    # Convert UV coordinates to numpy array
    uv_array = np.array(uvs, dtype=np.float32).reshape(-1, 2)

    # Calculate intensities from UV values
    intensities = np.min(uv_array, axis=1, keepdims=True)

    # Combine positions with intensities
    positions_with_intensities = np.hstack((position_array, intensities))

    # Filter out points below intensity threshold
    filtered_points = positions_with_intensities[
        positions_with_intensities[:, -1] > intense_limiter
    ]

    # Remove duplicate points
    unique_points = np.unique(filtered_points, axis=0)
    
    return unique_points


class VoxelDecoder:
    """WASM-based voxel map decoder for LiDAR data"""
    
    def __init__(self):
        self.engine = None
        self.store = None
        self.instance = None
        self.memory = None
        self.decoder_init_func = None
        self.decoder_decode_func = None
        self._initialize_wasm()
    
    def _initialize_wasm(self) -> None:
        """Initialize WASM engine and load decoder module"""
        try:
            # Configure WASM engine
            config = Config()
            self.engine = Engine(config)
            self.store = Store(self.engine)
            
            # Load WASM module
            wasm_path = os.path.join(
                get_package_share_directory('go2_robot_sdk'),
                'external_lib',
                'libvoxel.wasm'
            )
            
            if not os.path.exists(wasm_path):
                raise FileNotFoundError(f"WASM decoder not found: {wasm_path}")
            
            with open(wasm_path, 'rb') as wasm_file:
                wasm_bytes = wasm_file.read()
            
            module = Module(self.engine, wasm_bytes)
            self.instance = Instance(self.store, module, [])
            
            # Get memory and functions
            self.memory = self.instance.exports(self.store)["memory"]
            self.decoder_init_func = self.instance.exports(self.store)["decoder_init"]
            self.decoder_decode_func = self.instance.exports(self.store)["decoder_decode"]
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize WASM decoder: {e}")
    
    def decode_voxel_data(self, compressed_data: bytes) -> Tuple[List[float], List[float]]:
        """
        Decode compressed voxel data using WASM decoder.
        
        Args:
            compressed_data: Compressed voxel map bytes
            
        Returns:
            Tuple of (positions, uvs) lists
        """
        if not self.instance:
            raise RuntimeError("WASM decoder not initialized")
        
        try:
            data_length = len(compressed_data)
            
            # Allocate memory in WASM
            memory_data = self.memory.data_ptr(self.store)
            
            # Copy compressed data to WASM memory
            input_ptr = 1024  # Start offset in WASM memory
            ctypes.memmove(
                ctypes.c_void_p(memory_data + input_ptr),
                compressed_data,
                data_length
            )
            
            # Initialize decoder
            self.decoder_init_func(self.store)
            
            # Decode data
            result_ptr = self.decoder_decode_func(
                self.store, 
                input_ptr, 
                data_length
            )
            
            # Extract results from WASM memory
            positions, uvs = self._extract_decoded_data(memory_data, result_ptr)
            
            return positions, uvs
            
        except Exception as e:
            raise RuntimeError(f"Failed to decode voxel data: {e}")
    
    def _extract_decoded_data(
        self, 
        memory_data: int, 
        result_ptr: int
    ) -> Tuple[List[float], List[float]]:
        """Extract decoded position and UV data from WASM memory"""
        # This is a simplified version - actual implementation depends on
        # the specific WASM decoder output format
        positions = []
        uvs = []
        
        # Read result header to get data sizes
        header_ptr = ctypes.c_void_p(memory_data + result_ptr)
        
        # Extract positions and UVs based on decoder output format
        # Implementation details depend on the specific WASM module
        
        return positions, uvs


# Global decoder instance
_voxel_decoder: Optional[VoxelDecoder] = None


def get_voxel_decoder() -> VoxelDecoder:
    """Get singleton voxel decoder instance"""
    global _voxel_decoder
    if _voxel_decoder is None:
        _voxel_decoder = VoxelDecoder()
    return _voxel_decoder


def decode_lidar_data(
    compressed_data: bytes,
    resolution: float = 0.01,
    origin: List[float] = [0.0, 0.0, 0.0],
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
    positions, uvs = decoder.decode_voxel_data(compressed_data)
    
    return update_meshes_for_cloud2(
        positions, uvs, resolution, origin, intensity_threshold
    ) 