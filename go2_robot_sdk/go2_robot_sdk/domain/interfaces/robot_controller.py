# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

from abc import ABC, abstractmethod


class IRobotController(ABC):
    """Interface for robot control operations"""

    @abstractmethod
    def send_movement_command(self, robot_id: str, x: float, y: float, z: float) -> None:
        """Send movement command to robot"""
        pass

    @abstractmethod
    def send_stand_up_command(self, robot_id: str) -> None:
        """Send stand up command"""
        pass

    @abstractmethod
    def send_stand_down_command(self, robot_id: str) -> None:
        """Send stand down command"""
        pass

    @abstractmethod
    def send_webrtc_request(self, robot_id: str, api_id: int, parameter: str, topic: str) -> None:
        """Send WebRTC request"""
        pass 