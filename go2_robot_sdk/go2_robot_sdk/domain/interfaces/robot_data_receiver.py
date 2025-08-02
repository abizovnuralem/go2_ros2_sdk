# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

from abc import ABC, abstractmethod
from typing import Callable
from ..entities.robot_data import RobotData


class IRobotDataReceiver(ABC):
    """Интерфейс для получения данных от робота"""

    @abstractmethod
    async def connect(self, robot_id: str) -> None:
        """Подключение к роботу"""
        pass

    @abstractmethod
    async def disconnect(self, robot_id: str) -> None:
        """Отключение от робота"""
        pass

    @abstractmethod
    def set_data_callback(self, callback: Callable[[RobotData], None]) -> None:
        """Установка callback для получения данных"""
        pass

    @abstractmethod
    def send_command(self, robot_id: str, command: str) -> None:
        """Отправка команды роботу"""
        pass 