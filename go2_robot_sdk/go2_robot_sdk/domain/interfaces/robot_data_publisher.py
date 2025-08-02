# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

from abc import ABC, abstractmethod
from ..entities.robot_data import RobotData


class IRobotDataPublisher(ABC):
    """Интерфейс для публикации данных робота"""

    @abstractmethod
    def publish_odometry(self, robot_data: RobotData) -> None:
        """Публикация данных одометрии"""
        pass

    @abstractmethod
    def publish_joint_state(self, robot_data: RobotData) -> None:
        """Публикация состояния суставов"""
        pass

    @abstractmethod
    def publish_robot_state(self, robot_data: RobotData) -> None:
        """Публикация состояния робота и IMU"""
        pass

    @abstractmethod
    def publish_lidar_data(self, robot_data: RobotData) -> None:
        """Публикация данных лидара"""
        pass

    @abstractmethod
    def publish_camera_data(self, robot_data: RobotData) -> None:
        """Публикация данных камеры"""
        pass

    @abstractmethod
    def publish_voxel_data(self, robot_data: RobotData) -> None:
        """Публикация voxel данных"""
        pass 