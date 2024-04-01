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
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    no_rviz2 = LaunchConfiguration('no_rviz2', default='false')
    robot_ip = LaunchConfiguration('robot_ip', default=os.getenv('ROBOT_IP'))
    robot_token = LaunchConfiguration('robot_token', default=os.getenv('ROBOT_TOKEN',''))



    urdf_file_name = 'go2.urdf'
    urdf = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        "urdf",
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    
    foxglove_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'), 
        'launch', 
        'foxglove_bridge_launch.xml',
    )

    if not os.path.exists(foxglove_launch):
        foxglove_launch = os.path.join(
            get_package_share_directory('foxglove_bridge'), 
            'foxglove_bridge_launch.xml'
        )

    joy_params = os.path.join(
        get_package_share_directory('go2_robot_sdk'), 
        'config', 'joystick.yaml'
        )
    
    default_config_topics = os.path.join(get_package_share_directory('go2_robot_sdk'),
                                         'config', 'twist_mux.yaml')
    
    slam_toolbox_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'mapper_params_online_async.yaml'
    )

    nav2_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'nav2_params.yaml'
    )
    
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'config_topics',
            default_value=default_config_topics,
            description='Default topics config file'),

        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='cmd_vel',
            description='cmd vel output topic'),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'params_file': slam_toolbox_config,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'params_file': nav2_config,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/point_cloud2'),
            ],
            parameters=[{
                'target_frame': 'base_link',
            }]
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params]),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel', '/cmd_vel_joy')],),
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', LaunchConfiguration('cmd_vel_out'))},
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                LaunchConfiguration('config_topics')]
        ),
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            parameters=[{'robot_ip': robot_ip, 'token': robot_token}],
            ),
        Node(
            package='go2_robot_sdk',
            executable='go2_proc_text',
            ),
        Node(
            package='ros2_go2_video',
            executable='ros2_go2_video',
            parameters=[{'robot_ip': robot_ip, 'robot_token': robot_token}],
            ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            condition=UnlessCondition(no_rviz2),
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('go2_robot_sdk'), 'config', 'conf.rviz')]
        )
    ])