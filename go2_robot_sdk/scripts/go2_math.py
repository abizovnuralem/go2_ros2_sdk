# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
DEPRECATED: This file has been moved to domain layer.
Use imports from go2_robot_sdk.domain.math instead.
This compatibility layer will be removed in future versions.
"""

import warnings
from go2_robot_sdk.domain.math import (
    Vector3, Quaternion, get_robot_joints,
    HIP_LENGTH, THIGH_LENGTH, CALF_LENGTH
)

# Issue deprecation warning
warnings.warn(
    "scripts.go2_math is deprecated. Use go2_robot_sdk.domain.math instead.",
    DeprecationWarning,
    stacklevel=2
)

# Re-export for backward compatibility
__all__ = [
    'Vector3', 'Quaternion', 'get_robot_joints',
    'HIP_LENGTH', 'THIGH_LENGTH', 'CALF_LENGTH'
]
