# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

from abc import ABC, abstractmethod


class IRobotController(ABC):
    """Интерфейс для управления роботом"""

    @abstractmethod
    def send_movement_command(self, robot_id: str, x: float, y: float, z: float) -> None:
        """Отправка команды движения"""
        pass

    @abstractmethod
    def send_stand_up_command(self, robot_id: str) -> None:
        """Команда встать"""
        pass

    @abstractmethod
    def send_stand_down_command(self, robot_id: str) -> None:
        """Команда лечь"""
        pass

    @abstractmethod
    def send_webrtc_request(self, robot_id: str, api_id: int, parameter: str, topic: str) -> None:
        """Отправка WebRTC запроса"""
        pass 