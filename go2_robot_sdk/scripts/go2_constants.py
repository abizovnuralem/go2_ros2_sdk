# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
DEPRECATED: This file has been moved to domain layer.
Use imports from go2_robot_sdk.domain instead.
This compatibility layer will be removed in future versions.
"""

import warnings
from go2_robot_sdk.domain.constants import ROBOT_CMD, RTC_TOPIC, DATA_CHANNEL_TYPE

# Issue deprecation warning
warnings.warn(
    "scripts.go2_constants is deprecated. Use go2_robot_sdk.domain.constants instead.",
    DeprecationWarning,
    stacklevel=2
)

# Re-export for backward compatibility
__all__ = ['ROBOT_CMD', 'RTC_TOPIC', 'DATA_CHANNEL_TYPE']
