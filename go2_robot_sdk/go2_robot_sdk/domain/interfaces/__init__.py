"""
Domain interfaces - абстракции для внешних зависимостей
"""
from .robot_data_publisher import IRobotDataPublisher
from .robot_data_receiver import IRobotDataReceiver
from .robot_controller import IRobotController

__all__ = ['IRobotDataPublisher', 'IRobotDataReceiver', 'IRobotController'] 