# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Robot kinematics calculations for Go2 robot.
Contains inverse kinematics for leg joint calculations.
"""

import math
import numpy as np
from .geometry import Vector3

# URDF Go2 real physical dimensions (in meters)
HIP_LENGTH = 0.0955
THIGH_LENGTH = 0.213
CALF_LENGTH = 0.2135


def get_robot_joints(foot_position_value: list, foot_num: int) -> tuple:
    """
    Calculate joint angles for a leg given foot position.
    
    Args:
        foot_position_value: [x, y, z] position of the foot
        foot_num: Leg number (0=FL, 1=FR, 2=RL, 3=RR)
        
    Returns:
        Tuple of (hip_angle, thigh_angle, calf_angle) in radians
    """
    foot_position = Vector3(foot_position_value[0], foot_position_value[1], foot_position_value[2])

    # Base frame to hip joint offset
    base_tf_offset_hip_joint = Vector3(0.1934, 0.0465, 0)
    
    # Adjust for rear legs (negative x)
    if foot_num > 1:
        base_tf_offset_hip_joint.x = -base_tf_offset_hip_joint.x
    
    # Adjust for right side legs (negative y)
    if foot_num % 2 == 1:
        base_tf_offset_hip_joint.y = -base_tf_offset_hip_joint.y

    foot_position_distance = foot_position.distance_to(base_tf_offset_hip_joint)

    # Inverse kinematics using cosine law
    try:
        # Distance in the leg plane (excluding hip rotation)
        leg_reach = np.sqrt(foot_position_distance ** 2 - HIP_LENGTH ** 2)
        
        # Thigh angle calculation
        thigh_angle = np.arccos(
            (leg_reach ** 2 + THIGH_LENGTH ** 2 - CALF_LENGTH ** 2) / 
            (2 * leg_reach * THIGH_LENGTH)
        )
        
        # Calf angle calculation  
        calf_angle = (
            np.arccos((CALF_LENGTH ** 2 + THIGH_LENGTH ** 2 - leg_reach ** 2) /
                     (2 * CALF_LENGTH * THIGH_LENGTH)) - np.pi
        )
        
        # Foot position relative to hip
        relative_x = foot_position.x - base_tf_offset_hip_joint.x
        relative_y = foot_position.y - base_tf_offset_hip_joint.y

        # Adjust thigh angle based on foot height
        if foot_position.z < 0:
            final_thigh_angle = np.arcsin(-relative_x / leg_reach) + thigh_angle
        else:
            final_thigh_angle = -np.pi + np.arcsin(np.clip(relative_x / leg_reach, -1.0, 1.0)) + thigh_angle

        # Hip angle calculation
        horizontal_distance = np.sqrt(foot_position_distance ** 2 - relative_x ** 2)
        lateral_angle = np.arcsin(np.clip(relative_y / horizontal_distance, -1.0, 1.0))

        # Direction multiplier based on foot height
        direction = -1 if foot_position.z > 0 else 1

        # Hip angle depends on left/right side
        if foot_num % 2 == 0:  # Left side
            hip_angle = direction * (lateral_angle - np.arcsin(np.clip(HIP_LENGTH / horizontal_distance, -1.0, 1.0)))
        else:  # Right side
            hip_angle = direction * (lateral_angle + np.arcsin(np.clip(HIP_LENGTH / horizontal_distance, -1.0, 1.0)))

        # Check for NaN values
        if math.isnan(hip_angle + final_thigh_angle + calf_angle):
            return 0.0, 0.0, 0.0

        return hip_angle, final_thigh_angle, calf_angle
        
    except (ValueError, ZeroDivisionError):
        # Return safe default if calculation fails
        return 0.0, 0.0, 0.0 