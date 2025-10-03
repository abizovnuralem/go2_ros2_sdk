#!/usr/bin/env python3

# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Combined launch file for complete LiDAR processing pipeline (C++ version)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for complete lidar processing pipeline"""
    
    # Declare launch arguments for lidar_to_pointcloud
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
    
    # Declare launch arguments for pointcloud_aggregator
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
    )
    
    return LaunchDescription([
        # Launch arguments
        robot_ip_lst_arg,
        map_name_arg,
        map_save_arg,
        save_interval_arg,
        max_points_arg,
        voxel_size_arg,
        max_range_arg,
        min_range_arg,
        height_filter_min_arg,
        height_filter_max_arg,
        downsample_rate_arg,
        publish_rate_arg,
        
        # Nodes
        lidar_to_pointcloud_node,
        pointcloud_aggregator_node,
    ])