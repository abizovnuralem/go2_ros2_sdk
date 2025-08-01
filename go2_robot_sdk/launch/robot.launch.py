# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    with_rviz2 = LaunchConfiguration('rviz2', default='true')
    with_joystick = LaunchConfiguration('joystick', default='true')
    with_teleop = LaunchConfiguration('teleop', default='true')

    # Environment variables
    robot_token = os.getenv('ROBOT_TOKEN', '')
    robot_ip = os.getenv('ROBOT_IP', '')

    # Package directory
    pkg_dir = get_package_share_directory('go2_robot_sdk')
    
    # Config files
    rviz_config = os.path.join(pkg_dir, 'config', 'single_robot_conf.rviz')
    joy_params = os.path.join(pkg_dir, 'config', 'joystick.yaml')
    twist_mux_params = os.path.join(pkg_dir, 'config', 'twist_mux.yaml')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'go2.urdf')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }]
        ),

        # Pointcloud to laserscan converter
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', 'point_cloud2'),
                ('scan', 'scan')
            ],
            parameters=[{
                'target_frame': 'base_link',
                'max_height': 0.5
            }],
            output='screen'
        ),

        # Main Go2 driver
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            parameters=[{
                'robot_ip': robot_ip,
                'token': robot_token
            }]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(with_rviz2),
            arguments=['-d', rviz_config]
        ),

        # Joystick
        Node(
            package='joy',
            executable='joy_node',
            condition=IfCondition(with_joystick),
            parameters=[joy_params]
        ),

        # Teleop twist joy
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            condition=IfCondition(with_joystick),
            parameters=[twist_mux_params]
        ),

        # Twist mux
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            condition=IfCondition(with_teleop),
            parameters=[
                {'use_sim_time': use_sim_time},
                twist_mux_params
            ]
        ),
    ])
