"""
Domain math utilities - geometric and kinematic calculations
"""
from .geometry import Vector3, Quaternion
from .kinematics import get_robot_joints, HIP_LENGTH, THIGH_LENGTH, CALF_LENGTH

__all__ = [
    'Vector3', 'Quaternion', 'get_robot_joints', 
    'HIP_LENGTH', 'THIGH_LENGTH', 'CALF_LENGTH'
] 