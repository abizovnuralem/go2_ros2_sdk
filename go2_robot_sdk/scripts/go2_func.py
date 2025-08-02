# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
DEPRECATED: This file has been moved to application layer.
Use imports from go2_robot_sdk.application.utils instead.
This compatibility layer will be removed in future versions.
"""

import warnings
from go2_robot_sdk.application.utils import gen_command, gen_mov_command, generate_id

# Issue deprecation warning
warnings.warn(
    "scripts.go2_func is deprecated. Use go2_robot_sdk.application.utils instead.",
    DeprecationWarning,
    stacklevel=2
)

# Re-export for backward compatibility
__all__ = ['gen_command', 'gen_mov_command', 'generate_id']
