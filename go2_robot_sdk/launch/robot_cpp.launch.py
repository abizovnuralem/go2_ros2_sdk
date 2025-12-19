# Robot launch file with C++ lidar processing nodes

import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource


class Go2LaunchConfig:
    """Configuration container for Go2 robot launch parameters"""
    
    def __init__(self):
        # Environment variables
        self.robot_token = os.getenv('ROBOT_TOKEN', '')
        self.robot_ip = os.getenv('ROBOT_IP', '')
        self.robot_ip_list = self._parse_ip_list(self.robot_ip)
        self.map_name = os.getenv('MAP_NAME', '3d_map')
        self.save_map = os.getenv('MAP_SAVE', 'true')
        self.conn_type = os.getenv('CONN_TYPE', 'webrtc')
        
        # Derived configurations
        self.conn_mode = self._determine_connection_mode()
        self.rviz_config = self._get_rviz_config()
        self.urdf_file = self._get_urdf_file()
        
        # Package paths
        self.package_dir = get_package_share_directory('go2_robot_sdk')
        self.config_paths = self._get_config_paths()
        
        print(f"� Go2 Launch Configuration:")
        print(f"   Robot IPs: {self.robot_ip_list}")
        print(f"   Connection: {self.conn_type} ({self.conn_mode})")
        print(f"   URDF: {self.urdf_file}")
    
    def _parse_ip_list(self, robot_ip: str) -> List[str]:
        """Parse robot IP addresses from environment variable"""
        return robot_ip.replace(" ", "").split(",") if robot_ip else []
    
    def _determine_connection_mode(self) -> str:
        """Determine connection mode based on IP list and connection type"""
        return "single" if len(self.robot_ip_list) == 1 and self.conn_type != "cyclonedx" else "multi"
    
    def _get_rviz_config(self) -> str:
        """Get appropriate RViz configuration file"""
        if self.conn_type == 'cyclonedx':
            return "cyclonedx_config.rviz"
        elif self.conn_mode == 'single':
            return "single_robot_conf.rviz"
        else:
            return "multi_robot_conf.rviz"
    
    def _get_urdf_file(self) -> str:
        """Get appropriate URDF file"""
        return 'go2.urdf' if self.conn_mode == 'single' else 'multi_go2.urdf'
    
    def _get_config_paths(self) -> dict:
        """Get all configuration file paths"""
        return {
            'joystick': os.path.join(self.package_dir, 'config', 'joystick.yaml'),
            'twist_mux': os.path.join(self.package_dir, 'config', 'twist_mux.yaml'),
            'slam': os.path.join(self.package_dir, 'config', 'mapper_params_online_async.yaml'),
            'nav2': os.path.join(self.package_dir, 'config', 'nav2_params.yaml'),
            'rviz': os.path.join(self.package_dir, 'config', self.rviz_config),
            'urdf': os.path.join(self.package_dir, 'urdf', self.urdf_file),
        }


class Go2NodeFactory:
    """Factory for creating Go2 robot nodes"""
    
    def __init__(self, config: Go2LaunchConfig):
        self.config = config
    
    def create_launch_arguments(self) -> List[DeclareLaunchArgument]:
        """Create all launch arguments"""
        return [
            DeclareLaunchArgument('rviz2', default_value='true', description='Launch RViz2'),
            DeclareLaunchArgument('nav2', default_value='true', description='Launch Nav2'),
            DeclareLaunchArgument('map', default_value='', description='Full path to map file to load (for localization)'),
            DeclareLaunchArgument('foxglove', default_value='true', description='Launch Foxglove Bridge'),
            DeclareLaunchArgument('joystick', default_value='true', description='Launch joystick'),
            DeclareLaunchArgument('teleop', default_value='true', description='Launch teleoperation'),
            DeclareLaunchArgument('obstacle_avoidance', default_value='false', description='Enable obstacle avoidance'),
            DeclareLaunchArgument('log_level', default_value='warn', description='ROS 2 log level (debug|info|warn|error|fatal)'),
            DeclareLaunchArgument('lidar_publish_rate', default_value='5.0', description='LiDAR publish rate (Hz)'),
            DeclareLaunchArgument('lidar_downsample_step', default_value='4', description='LiDAR downsample step'),
            DeclareLaunchArgument('lidar_max_points', default_value='25000', description='LiDAR max points'),
            DeclareLaunchArgument('lidar_deduplicate', default_value='false', description='Deduplicate LiDAR points'),
            DeclareLaunchArgument('use_cpp_lidar_accel', default_value='true', description='Enable C++ LiDAR acceleration (pybind11)'),
            DeclareLaunchArgument('lidar_intensity_threshold', default_value='0.0', description='LiDAR intensity threshold'),
        ]
    
    def create_robot_state_nodes(self) -> List[Node]:
        """Create robot state publisher nodes"""
        nodes = []
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        log_level = LaunchConfiguration('log_level')
        
        if self.config.conn_mode == 'single':
            # Single robot configuration
            robot_desc = self._load_urdf_content(self.config.config_paths['urdf'])
            
            nodes.extend([
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='go2_robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'robot_description': robot_desc
                    }],
                    arguments=[
                        self.config.config_paths['urdf'],
                        '--ros-args',
                        '--log-level',
                        log_level,
                    ]
                ),
                self._create_pointcloud_to_laserscan_node()
            ])
        else:
            # Multi-robot configuration
            base_urdf = self._load_urdf_content(self.config.config_paths['urdf'])
            
            for i, _ in enumerate(self.config.robot_ip_list):
                robot_desc = base_urdf.format(robot_num=f"robot{i}")
                
                nodes.extend([
                    Node(
                        package='robot_state_publisher',
                        executable='robot_state_publisher',
                        name='go2_robot_state_publisher',
                        output='screen',
                        namespace=f"robot{i}",
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'robot_description': robot_desc
                        }],
                        arguments=[
                            self.config.config_paths['urdf'],
                            '--ros-args',
                            '--log-level',
                            log_level,
                        ]
                    ),
                    self._create_pointcloud_to_laserscan_node(f"robot{i}")
                ])
        
        return nodes
    
    def _load_urdf_content(self, urdf_path: str) -> str:
        """Load URDF file content"""
        with open(urdf_path, 'r') as file:
            return file.read()
    
    def _create_pointcloud_to_laserscan_node(self, namespace: str = None) -> Node:
        """Create pointcloud to laserscan conversion node"""
        log_level = LaunchConfiguration('log_level')
        if namespace:
            # Multi-robot setup
            return Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name=f'{namespace}_pointcloud_to_laserscan',
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[
                    ('cloud_in', f'{namespace}/point_cloud2'),
                    ('scan', f'{namespace}/scan'),
                ],
                parameters=[{
                    'target_frame': f'{namespace}/base_link',
                    'max_height': 0.1
                }],
                output='screen',
            )
        else:
            # Single robot setup
            return Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='go2_pointcloud_to_laserscan',
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[
                    ('cloud_in', 'point_cloud2'),
                    ('scan', 'scan'),
                ],
                parameters=[{
                    'target_frame': 'base_link',
                    'max_height': 0.5
                }],
                output='screen',
            )
    
    def create_core_nodes(self) -> List[Node]:
        """Create core Go2 robot nodes"""
        log_level = LaunchConfiguration('log_level')
        lidar_publish_rate = ParameterValue(LaunchConfiguration('lidar_publish_rate'), value_type=float)
        lidar_downsample_step = ParameterValue(LaunchConfiguration('lidar_downsample_step'), value_type=int)
        lidar_max_points = ParameterValue(LaunchConfiguration('lidar_max_points'), value_type=int)
        lidar_deduplicate = ParameterValue(LaunchConfiguration('lidar_deduplicate'), value_type=bool)
        use_cpp_lidar_accel = ParameterValue(LaunchConfiguration('use_cpp_lidar_accel'), value_type=bool)
        lidar_intensity_threshold = ParameterValue(
            LaunchConfiguration('lidar_intensity_threshold'), value_type=float
        )
        obstacle_avoidance = ParameterValue(LaunchConfiguration('obstacle_avoidance'), value_type=bool)
        return [
            # Main robot driver (clean architecture)
            Node(
                package='go2_robot_sdk',
                executable='go2_driver_node',
                name='go2_driver_node',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'robot_ip': self.config.robot_ip,
                    'token': self.config.robot_token,
                    'conn_type': self.config.conn_type,
                    'obstacle_avoidance': obstacle_avoidance,
                    'lidar_publish_rate': lidar_publish_rate,
                    'lidar_downsample_step': lidar_downsample_step,
                    'lidar_max_points': lidar_max_points,
                    'lidar_deduplicate': lidar_deduplicate,
                    'use_cpp_lidar_accel': use_cpp_lidar_accel,
                    'lidar_intensity_threshold': lidar_intensity_threshold,
                }],
            ),
            # LiDAR processing node (C++ implementation)
            Node(
                package='lidar_processor_cpp',
                executable='lidar_to_pointcloud_node',
                name='lidar_to_pointcloud',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'robot_ip_lst': self.config.robot_ip_list,
                    'map_name': self.config.map_name,
                    'map_save': self.config.save_map
                }],
            ),
            # Advanced point cloud aggregator (C++ implementation)
            Node(
                package='lidar_processor_cpp',
                executable='pointcloud_aggregator_node',
                name='pointcloud_aggregator',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'max_range': 20.0,
                    'min_range': 0.1,
                    'height_filter_min': -2.0,
                    'height_filter_max': 3.0,
                    'downsample_rate': 5,
                    'publish_rate': 10.0
                }],
            ),
            # TTS Node (new separate package)
            Node(
                package='speech_processor',
                executable='tts_node',
                name='tts_node',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'api_key': os.getenv('ELEVENLABS_API_KEY', ''),
                    'provider': 'elevenlabs',
                    'voice_name': 'XrExE9yKIg1WjnnlVkGX',
                    'local_playback': False,
                    'use_cache': True,
                    'audio_quality': 'standard'
                }],
            ),
        ]
    
    def create_teleop_nodes(self) -> List[Node]:
        """Create teleoperation and joystick nodes"""
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        with_joystick = LaunchConfiguration('joystick', default='true')
        with_teleop = LaunchConfiguration('teleop', default='true')
        log_level = LaunchConfiguration('log_level')
        
        return [
            # Joystick node
            Node(
                package='joy',
                executable='joy_node',
                condition=IfCondition(with_joystick),
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[self.config.config_paths['joystick']]
            ),
            # Teleop twist joy node
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='go2_teleop_node',
                condition=IfCondition(with_joystick),
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[self.config.config_paths['twist_mux']],
            ),
            # Twist multiplexer
            Node(
                package='twist_mux',
                executable='twist_mux',
                output='screen',
                condition=IfCondition(with_teleop),
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    self.config.config_paths['twist_mux']
                ],
            ),
        ]
    
    def create_visualization_nodes(self) -> List[Node]:
        """Create visualization nodes (RViz, Foxglove)"""
        with_rviz2 = LaunchConfiguration('rviz2', default='true')
        log_level = LaunchConfiguration('log_level')
        
        return [
            # RViz2
            Node(
                package='rviz2',
                executable='rviz2',
                condition=IfCondition(with_rviz2),
                name='go2_rviz2',
                output='screen',
                arguments=[
                    '-d',
                    self.config.config_paths['rviz'],
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
                parameters=[{'use_sim_time': False}]
            ),
        ]
    
    def create_include_launches(self) -> List[IncludeLaunchDescription]:
        """Create included launch descriptions"""
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        with_foxglove = LaunchConfiguration('foxglove', default='true')
        with_nav2 = LaunchConfiguration('nav2', default='true')
        
        map_file = LaunchConfiguration('map')
        
        # Check if map file is provided (Empty string means SLAM mode)
        has_map = PythonExpression(["'", map_file, "' != ''"])

        foxglove_launch = os.path.join(
            get_package_share_directory('foxglove_bridge'),
            'launch', 'foxglove_bridge_launch.xml'
        )
        
        return [
            # Foxglove Bridge
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(foxglove_launch),
                condition=IfCondition(with_foxglove),
            ),
            # SLAM Toolbox (Run if NO map is provided)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('slam_toolbox'),
                                'launch', 'online_async_launch.py')
                ]),
                condition=UnlessCondition(has_map),
                launch_arguments={
                    'slam_params_file': self.config.config_paths['slam'],
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
            # Nav2 (Localization Mode - Run if map IS provided)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('nav2_bringup'),
                                'launch', 'localization_launch.py')
                ]),
                condition=IfCondition(has_map),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': use_sim_time,
                    'params_file': self.config.config_paths['nav2'],
                }.items(),
            ),
            # Nav2 (Navigation Mode - Always run)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('nav2_bringup'),
                                'launch', 'navigation_launch.py')
                ]),
                condition=IfCondition(with_nav2),
                launch_arguments={
                    'params_file': self.config.config_paths['nav2'],
                    'use_sim_time': use_sim_time,
                    'map_subscribe_transient_local': 'true',
                }.items(),
            ),
        ]


def generate_launch_description():
    """Generate the launch description for Go2 robot system"""
    
    # Initialize configuration and factory
    config = Go2LaunchConfig()
    factory = Go2NodeFactory(config)
    
    # Create all components
    launch_args = factory.create_launch_arguments()
    robot_state_nodes = factory.create_robot_state_nodes()
    core_nodes = factory.create_core_nodes()
    teleop_nodes = factory.create_teleop_nodes()
    visualization_nodes = factory.create_visualization_nodes()
    include_launches = factory.create_include_launches()
    
    # Combine all elements
    launch_entities = (
        launch_args +
        robot_state_nodes +
        core_nodes +
        teleop_nodes +
        visualization_nodes +
        include_launches
    )
    
    return LaunchDescription(launch_entities)