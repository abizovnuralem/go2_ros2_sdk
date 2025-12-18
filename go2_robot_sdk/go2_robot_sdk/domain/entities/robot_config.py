# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import dataclass
from typing import List


@dataclass
class RobotConfig:
    """Robot configuration parameters"""

    robot_ip_list: List[str]
    token: str
    conn_type: str
    enable_video: bool
    decode_lidar: bool
    use_cpp_lidar_accel: bool
    publish_raw_voxel: bool
    obstacle_avoidance: bool
    lidar_publish_rate: float
    lidar_downsample_step: int
    lidar_max_points: int
    lidar_deduplicate: bool
    lidar_intensity_threshold: float
    conn_mode: str  # 'single' or 'multi'

    @classmethod
    def from_params(
        cls,
        robot_ip: str,
        token: str,
        conn_type: str,
        enable_video: bool,
        decode_lidar: bool,
        publish_raw_voxel: bool,
        obstacle_avoidance: bool,
        use_cpp_lidar_accel: bool = False,
        lidar_publish_rate: float = 5.0,
        lidar_downsample_step: int = 4,
        lidar_max_points: int = 25000,
        lidar_deduplicate: bool = False,
        lidar_intensity_threshold: float = 0.0,
    ):
        """Создание конфигурации из параметров"""
        robot_ip_list = robot_ip.replace(" ", "").split(",")
        conn_mode = (
            "single"
            if (len(robot_ip_list) == 1 and conn_type != "cyclonedds")
            else "multi"
        )

        return cls(
            robot_ip_list=robot_ip_list,
            token=token,
            conn_type=conn_type,
            enable_video=enable_video,
            decode_lidar=decode_lidar,
            use_cpp_lidar_accel=use_cpp_lidar_accel,
            publish_raw_voxel=publish_raw_voxel,
            obstacle_avoidance=obstacle_avoidance,
            lidar_publish_rate=lidar_publish_rate,
            lidar_downsample_step=lidar_downsample_step,
            lidar_max_points=lidar_max_points,
            lidar_deduplicate=lidar_deduplicate,
            lidar_intensity_threshold=lidar_intensity_threshold,
            conn_mode=conn_mode,
        )
