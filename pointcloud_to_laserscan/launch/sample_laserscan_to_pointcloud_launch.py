from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-r', '10',
                '--qos-profile', 'sensor_data',
                [LaunchConfiguration(variable_name='scanner'), '/scan'],
                'sensor_msgs/msg/LaserScan', yaml.dump({
                    'header': {'frame_id': 'scan'}, 'angle_min': -1.0,
                    'angle_max': 1.0, 'angle_increment': 0.1, 'range_max': 10.0,
                    'ranges': [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
                })
            ],
            name='scan_publisher'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'map', '--child-frame-id', 'scan'
            ]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            remappings=[('scan_in', [LaunchConfiguration(variable_name='scanner'), '/scan']),
                        ('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud'])],
            parameters=[{'target_frame': 'scan', 'transform_tolerance': 0.01}]
        ),
    ])
