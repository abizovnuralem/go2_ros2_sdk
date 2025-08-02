# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def load_urdf(context, *args, **kwargs):
    # Get the URDF file name from context
    urdf_file_name = context.launch_configurations['urdf_file_name']

    # Get package directory
    pkg_dir = get_package_share_directory('go2_robot_sdk')

    # Create full path to URDF
    urdf_file_path = os.path.join(pkg_dir, 'urdf', urdf_file_name)

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', urdf_file_path]),
                value_type=str
            )
        }],
        on_exit=LaunchConfiguration('on_exit'),
    )

    return [robot_state_publisher_node]


def generate_launch_description():
    # Declare launch parameters
    robot_ip = LaunchConfiguration('robot_ip', default=os.getenv(
        'ROBOT_IP', os.getenv('GO2_IP', '')))
    enable_video = LaunchConfiguration('enable_video', default='true')
    urdf_file_name = LaunchConfiguration('urdf_file_name', default='go2.urdf')
    send_buffer_limit = LaunchConfiguration('send_buffer_limit', default='100000000')
    on_exit = LaunchConfiguration('on_exit', default='shutdown')
    elevenlabs_api_key = LaunchConfiguration(
        'elevenlabs_api_key', default=EnvironmentVariable(
            'ELEVENLABS_API_KEY', default_value=''))
    voice_name = LaunchConfiguration('voice_name', default='default')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'robot_ip',
            default_value=os.getenv('ROBOT_IP', os.getenv('GO2_IP', '')),
            description='IP address of the robot'
        ),
        DeclareLaunchArgument(
            'enable_video',
            default_value='true',
            description='Enable video streaming'
        ),
        DeclareLaunchArgument(
            'urdf_file_name',
            default_value='go2.urdf',
            description='Name of the URDF file'
        ),
        DeclareLaunchArgument(
            'on_exit',
            default_value='shutdown',
            description='Behavior when a node exits (shutdown will terminate all nodes)'
        ),
        DeclareLaunchArgument(
            'elevenlabs_api_key',
            default_value=EnvironmentVariable('ELEVENLABS_API_KEY', default_value=''),
            description='API key for ElevenLabs TTS service'
        ),
        DeclareLaunchArgument(
            'voice_name',
            default_value='default',
            description='Voice name for TTS'
        ),
        DeclareLaunchArgument(
            'obstacle_avoidance',
            default_value='false',
            description='Enable obstacle avoidance',
        ),
        DeclareLaunchArgument(
            'enable_foxglove_bridge',
            default_value='true',
            description='Enable Foxglove Bridge'
        ),



        # Group all nodes to ensure they share the same on_exit behavior
        GroupAction([
            # Go2 driver node with minimal parameters
            Node(
                package='go2_robot_sdk',
                executable='go2_driver_node',
                parameters=[{
                    'robot_ip': robot_ip,
                    'token': '',
                    'conn_type': 'webrtc',
                    'enable_video': enable_video,
                    'decode_lidar': False,
                    'publish_raw_voxel': True,
                    'obstacle_avoidance': LaunchConfiguration('obstacle_avoidance'),
                },
                    {
                    "qos_overrides": {
                        "/camera/image_raw": {
                            "publisher": {
                                "reliability": "reliable",
                                "history": "keep_last",
                                "depth": 1,
                            }
                        }
                    }
                }],
                remappings=[
                    ('cmd_vel_out', 'cmd_vel'),
                ],
                on_exit=on_exit,
            ),

            # Image compression node
            Node(
                package='image_transport',
                executable='republish',
                name='image_republisher',
                arguments=['raw', 'compressed'],
                remappings=[
                    ('in', 'camera/image_raw'),
                    ('out/compressed', 'camera/compressed'),
                ],
                on_exit=on_exit,
            ),

            # Foxglove Bridge node
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                parameters=[{
                    'send_buffer_limit': send_buffer_limit
                }],
                on_exit=on_exit,
                condition=IfCondition(LaunchConfiguration('enable_foxglove_bridge')),
            ),

            # TTS node
            Node(
                package='go2_robot_sdk',
                executable='tts_node',
                name='tts_node',
                parameters=[{
                    'elevenlabs_api_key': elevenlabs_api_key,
                    'voice_name': voice_name
                }],
                on_exit=on_exit,
            ),
        ]),

        OpaqueFunction(function=load_urdf),
    ])
