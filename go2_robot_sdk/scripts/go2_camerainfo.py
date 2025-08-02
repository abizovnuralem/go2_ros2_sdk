# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
DEPRECATED: This file has been moved to infrastructure layer.
Use imports from go2_robot_sdk.infrastructure.sensors instead.
This compatibility layer will be removed in future versions.
"""

import warnings
from go2_robot_sdk.infrastructure.sensors import load_camera_info

# Issue deprecation warning
warnings.warn(
    "scripts.go2_camerainfo is deprecated. Use go2_robot_sdk.infrastructure.sensors instead.",
    DeprecationWarning,
    stacklevel=2
)

# Re-export for backward compatibility
__all__ = ['load_camera_info']
