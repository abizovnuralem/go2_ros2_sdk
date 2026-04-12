# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            name='go2_driver_node',
            output='screen',
            parameters=[{
                'robot_ip': os.getenv('ROBOT_IP', ''),
                'token': os.getenv('ROBOT_TOKEN', ''),
                'conn_type': os.getenv('CONN_TYPE', 'webrtc')
            }],
        ),
    ])
