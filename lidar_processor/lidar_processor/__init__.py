# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
LiDAR Processor Package

This package provides ROS2 nodes for processing LiDAR data from Go2 robot,
including point cloud aggregation and 3D mapping capabilities.
"""

__version__ = "1.0.0"
__author__ = "brimo"
__license__ = "BSD-3-Clause"

__all__ = [
    'lidar_to_pointcloud_node',
    'pointcloud_aggregator_node',
] 