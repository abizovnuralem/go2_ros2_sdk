#!/usr/bin/env python3

# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Launch file for LiDAR to PointCloud processor (C++ version)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for lidar_to_pointcloud_node"""
    
    # Declare launch arguments
    robot_ip_lst_arg = DeclareLaunchArgument(
        'robot_ip_lst',
        default_value='[]',
        description='List of robot IP addresses'
    )
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='3d_map',
        description='Name of the map file to save'
    )
    
    map_save_arg = DeclareLaunchArgument(
        'map_save',
        default_value='true',
        description='Whether to save the map periodically'
    )
    
    save_interval_arg = DeclareLaunchArgument(
        'save_interval',
        default_value='10.0',
        description='Interval for saving map (seconds)'
    )
    
    max_points_arg = DeclareLaunchArgument(
        'max_points',
        default_value='1000000',
        description='Maximum number of points to keep in memory'
    )
    
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.01',
        description='Voxel size for downsampling when saving'
    )
    
    # LiDAR to PointCloud node
    lidar_to_pointcloud_node = Node(
        package='lidar_processor_cpp',
        executable='lidar_to_pointcloud_node',
        name='lidar_to_pointcloud',
        output='screen',
        parameters=[{
            'robot_ip_lst': LaunchConfiguration('robot_ip_lst'),
            'map_name': LaunchConfiguration('map_name'),
            'map_save': LaunchConfiguration('map_save'),
            'save_interval': LaunchConfiguration('save_interval'),
            'max_points': LaunchConfiguration('max_points'),
            'voxel_size': LaunchConfiguration('voxel_size'),
        }],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return LaunchDescription([
        robot_ip_lst_arg,
        map_name_arg,
        map_save_arg,
        save_interval_arg,
        max_points_arg,
        voxel_size_arg,
        lidar_to_pointcloud_node,
    ])