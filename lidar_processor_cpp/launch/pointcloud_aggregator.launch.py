#!/usr/bin/env python3

# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Launch file for PointCloud Aggregator processor (C++ version)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for pointcloud_aggregator_node"""
    
    # Declare launch arguments
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='20.0',
        description='Maximum range from robot center (meters)'
    )
    
    min_range_arg = DeclareLaunchArgument(
        'min_range',
        default_value='0.1',
        description='Minimum range from robot center (meters)'
    )
    
    height_filter_min_arg = DeclareLaunchArgument(
        'height_filter_min',
        default_value='-2.0',
        description='Minimum height filter (z-coordinate)'
    )
    
    height_filter_max_arg = DeclareLaunchArgument(
        'height_filter_max',
        default_value='3.0',
        description='Maximum height filter (z-coordinate)'
    )
    
    downsample_rate_arg = DeclareLaunchArgument(
        'downsample_rate',
        default_value='10',
        description='Downsample rate (keep every Nth point)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='5.0',
        description='Publishing rate in Hz'
    )
    
    # PointCloud Aggregator node
    pointcloud_aggregator_node = Node(
        package='lidar_processor_cpp',
        executable='pointcloud_aggregator_node',
        name='pointcloud_aggregator',
        output='screen',
        parameters=[{
            'max_range': LaunchConfiguration('max_range'),
            'min_range': LaunchConfiguration('min_range'),
            'height_filter_min': LaunchConfiguration('height_filter_min'),
            'height_filter_max': LaunchConfiguration('height_filter_max'),
            'downsample_rate': LaunchConfiguration('downsample_rate'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return LaunchDescription([
        max_range_arg,
        min_range_arg,
        height_filter_min_arg,
        height_filter_max_arg,
        downsample_rate_arg,
        publish_rate_arg,
        pointcloud_aggregator_node,
    ])