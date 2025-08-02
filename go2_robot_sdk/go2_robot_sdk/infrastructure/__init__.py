"""
Infrastructure layer - адаптеры для внешних систем
"""
from .ros2 import ROS2Publisher
from .webrtc import WebRTCAdapter

__all__ = ['ROS2Publisher', 'WebRTCAdapter'] 