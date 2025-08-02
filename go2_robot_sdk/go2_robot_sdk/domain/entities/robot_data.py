# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import dataclass
from typing import Dict, List, Optional, Any
import numpy as np


@dataclass
class RobotState:
    """Robot state information"""
    mode: int
    progress: float
    gait_type: int
    position: List[float]
    body_height: float
    velocity: List[float]
    range_obstacle: List[float]
    foot_force: List[float]
    foot_position_body: List[float]
    foot_speed_body: List[float]


@dataclass
class IMUData:
    """IMU sensor data"""
    quaternion: List[float]
    accelerometer: List[float]
    gyroscope: List[float]
    rpy: List[float]
    temperature: float


@dataclass
class OdometryData:
    """Odometry data"""
    position: Dict[str, float]  # x, y, z
    orientation: Dict[str, float]  # x, y, z, w (quaternion)


@dataclass
class JointData:
    """Joint data"""
    motor_state: List[Dict[str, float]]  # q, dq, ddq, tau


@dataclass
class LidarData:
    """LiDAR sensor data"""
    positions: np.ndarray
    uvs: np.ndarray
    resolution: float
    origin: List[float]
    stamp: float
    width: Optional[List[int]] = None
    src_size: Optional[int] = None
    compressed_data: Optional[bytes] = None


@dataclass
class CameraData:
    """Camera data"""
    image: np.ndarray
    height: int
    width: int
    encoding: str = "bgr8"


@dataclass
class RobotData:
    """Aggregated robot data container"""
    robot_id: str
    timestamp: float
    robot_state: Optional[RobotState] = None
    imu_data: Optional[IMUData] = None
    odometry_data: Optional[OdometryData] = None
    joint_data: Optional[JointData] = None
    lidar_data: Optional[LidarData] = None
    camera_data: Optional[CameraData] = None 