# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

from abc import ABC, abstractmethod
from typing import Callable
from ..entities.robot_data import RobotData


class IRobotDataReceiver(ABC):
    """Interface for receiving data from robot via WebRTC"""

    @abstractmethod
    async def connect(self, robot_id: str) -> None:
        """Connect to robot"""
        pass

    @abstractmethod
    async def disconnect(self, robot_id: str) -> None:
        """Disconnect from robot"""
        pass

    @abstractmethod
    def set_data_callback(self, callback: Callable[[RobotData], None]) -> None:
        """Set callback for receiving data"""
        pass

    @abstractmethod
    def send_command(self, robot_id: str, command: str) -> None:
        """Send command to robot"""
        pass 