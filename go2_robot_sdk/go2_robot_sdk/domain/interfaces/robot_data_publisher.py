# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

from abc import ABC, abstractmethod
from ..entities.robot_data import RobotData


class IRobotDataPublisher(ABC):
    """Interface for publishing robot data to ROS2"""

    @abstractmethod
    def publish_odometry(self, robot_data: RobotData) -> None:
        """Publish odometry data"""
        pass

    @abstractmethod
    def publish_joint_state(self, robot_data: RobotData) -> None:
        """Publish joint states"""
        pass

    @abstractmethod
    def publish_robot_state(self, robot_data: RobotData) -> None:
        """Publish robot state and IMU data"""
        pass

    @abstractmethod
    def publish_lidar_data(self, robot_data: RobotData) -> None:
        """Publish LiDAR point cloud data"""
        pass

    @abstractmethod
    def publish_camera_data(self, robot_data: RobotData) -> None:
        """Publish camera image data"""
        pass

    @abstractmethod
    def publish_voxel_data(self, robot_data: RobotData) -> None:
        """Publish voxel map data"""
        pass 