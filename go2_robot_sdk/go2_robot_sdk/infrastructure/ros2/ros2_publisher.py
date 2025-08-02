
# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import logging

from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from go2_interfaces.msg import Go2State, IMU
from go2_interfaces.msg import VoxelMapCompressed
from sensor_msgs.msg import PointCloud2, PointField, JointState
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from ...domain.interfaces import IRobotDataPublisher
from ...domain.entities import RobotData, RobotConfig
from ..sensors.lidar_decoder import update_meshes_for_cloud2
from ..sensors.camera_config import load_camera_info

logger = logging.getLogger(__name__)


class ROS2Publisher(IRobotDataPublisher):
    """ROS2 adapter for publishing robot data"""

    def __init__(self, node: Node, config: RobotConfig, publishers: dict, broadcaster: TransformBroadcaster):
        self.node = node
        self.config = config
        self.publishers = publishers
        self.broadcaster = broadcaster
        self.bridge = CvBridge()
        self.camera_info = load_camera_info()

    def publish_odometry(self, robot_data: RobotData) -> None:
        """Publish odometry data"""
        if not robot_data.odometry_data:
            return

        try:
            robot_idx = int(robot_data.robot_id)
            
            # Publish transform
            self._publish_transform(robot_data, robot_idx)
            
            # Publish odometry topic
            self._publish_odometry_topic(robot_data, robot_idx)
            
        except Exception as e:
            logger.error(f"Error publishing odometry: {e}")

    def _publish_transform(self, robot_data: RobotData, robot_idx: int) -> None:
        """Publish TF transform"""
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.node.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'

        if self.config.conn_mode == 'single':
            odom_trans.child_frame_id = "base_link"
        else:
            odom_trans.child_frame_id = f"robot{robot_data.robot_id}/base_link"

        position = robot_data.odometry_data.position
        orientation = robot_data.odometry_data.orientation

        odom_trans.transform.translation.x = float(position['x'])
        odom_trans.transform.translation.y = float(position['y'])
        odom_trans.transform.translation.z = float(position['z']) + 0.07

        odom_trans.transform.rotation.x = float(orientation['x'])
        odom_trans.transform.rotation.y = float(orientation['y'])
        odom_trans.transform.rotation.z = float(orientation['z'])
        odom_trans.transform.rotation.w = float(orientation['w'])

        self.broadcaster.sendTransform(odom_trans)

    def _publish_odometry_topic(self, robot_data: RobotData, robot_idx: int) -> None:
        """Publish Odometry topic"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.node.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        if self.config.conn_mode == 'single':
            odom_msg.child_frame_id = "base_link"
        else:
            odom_msg.child_frame_id = f"robot{robot_data.robot_id}/base_link"

        position = robot_data.odometry_data.position
        orientation = robot_data.odometry_data.orientation

        odom_msg.pose.pose.position.x = float(position['x'])
        odom_msg.pose.pose.position.y = float(position['y'])
        odom_msg.pose.pose.position.z = float(position['z']) + 0.07

        odom_msg.pose.pose.orientation.x = float(orientation['x'])
        odom_msg.pose.pose.orientation.y = float(orientation['y'])
        odom_msg.pose.pose.orientation.z = float(orientation['z'])
        odom_msg.pose.pose.orientation.w = float(orientation['w'])

        self.publishers['odometry'][robot_idx].publish(odom_msg)

    def publish_joint_state(self, robot_data: RobotData) -> None:
        """Publish joint state data"""
        if not robot_data.joint_data:
            return

        try:
            robot_idx = int(robot_data.robot_id)
            joint_state = JointState()
            joint_state.header.stamp = self.node.get_clock().now().to_msg()

            # Define joint names
            if self.config.conn_mode == 'single':
                joint_state.name = [
                    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
                    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
                ]
            else:
                joint_state.name = [
                    f'robot{robot_data.robot_id}/FL_hip_joint', f'robot{robot_data.robot_id}/FL_thigh_joint', f'robot{robot_data.robot_id}/FL_calf_joint',
                    f'robot{robot_data.robot_id}/FR_hip_joint', f'robot{robot_data.robot_id}/FR_thigh_joint', f'robot{robot_data.robot_id}/FR_calf_joint',
                    f'robot{robot_data.robot_id}/RL_hip_joint', f'robot{robot_data.robot_id}/RL_thigh_joint', f'robot{robot_data.robot_id}/RL_calf_joint',
                    f'robot{robot_data.robot_id}/RR_hip_joint', f'robot{robot_data.robot_id}/RR_thigh_joint', f'robot{robot_data.robot_id}/RR_calf_joint'
                ]

            motor_state = robot_data.joint_data.motor_state
            joint_state.position = [
                motor_state[3]['q'], motor_state[4]['q'], motor_state[5]['q'],  # FL leg
                motor_state[0]['q'], motor_state[1]['q'], motor_state[2]['q'],  # FR leg
                motor_state[9]['q'], motor_state[10]['q'], motor_state[11]['q'], # RL leg
                motor_state[6]['q'], motor_state[7]['q'], motor_state[8]['q'],  # RR leg
            ]

            self.publishers['joint_state'][robot_idx].publish(joint_state)

        except Exception as e:
            logger.error(f"Error publishing joint state: {e}")

    def publish_robot_state(self, robot_data: RobotData) -> None:
        """Publish robot state and IMU data"""
        if not robot_data.robot_state:
            return

        try:
            robot_idx = int(robot_data.robot_id)

            # Publish Go2State
            go2_state = Go2State()
            state = robot_data.robot_state
            go2_state.mode = state.mode
            go2_state.progress = state.progress
            go2_state.gait_type = state.gait_type
            go2_state.position = list(map(float, state.position))
            go2_state.body_height = float(state.body_height)
            go2_state.velocity = state.velocity
            go2_state.range_obstacle = list(map(float, state.range_obstacle))
            go2_state.foot_force = state.foot_force
            go2_state.foot_position_body = list(map(float, state.foot_position_body))
            go2_state.foot_speed_body = list(map(float, state.foot_speed_body))
            
            self.publishers['robot_state'][robot_idx].publish(go2_state)

            # Publish IMU
            if robot_data.imu_data:
                imu = IMU()
                imu_data = robot_data.imu_data
                imu.quaternion = list(map(float, imu_data.quaternion))
                imu.accelerometer = list(map(float, imu_data.accelerometer))
                imu.gyroscope = list(map(float, imu_data.gyroscope))
                imu.rpy = list(map(float, imu_data.rpy))
                imu.temperature = imu_data.temperature
                
                self.publishers['imu'][robot_idx].publish(imu)

        except Exception as e:
            logger.error(f"Error publishing robot state: {e}")

    def publish_lidar_data(self, robot_data: RobotData) -> None:
        """Publish lidar data"""
        if not robot_data.lidar_data or not self.config.decode_lidar:
            return

        try:
            robot_idx = int(robot_data.robot_id)
            lidar = robot_data.lidar_data

            points = update_meshes_for_cloud2(
                lidar.positions,
                lidar.uvs,
                lidar.resolution,
                lidar.origin,
                0
            )

            point_cloud = PointCloud2()
            point_cloud.header = Header(frame_id="odom")
            point_cloud.header.stamp = self.node.get_clock().now().to_msg()
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            
            point_cloud = point_cloud2.create_cloud(point_cloud.header, fields, points)
            self.publishers['lidar'][robot_idx].publish(point_cloud)

        except Exception as e:
            logger.error(f"Error publishing lidar data: {e}")

    def publish_camera_data(self, robot_data: RobotData) -> None:
        """Publish camera data"""
        if not robot_data.camera_data:
            return

        try:
            robot_idx = int(robot_data.robot_id)
            camera = robot_data.camera_data

            # Convert to ROS Image
            ros_image = self.bridge.cv2_to_imgmsg(camera.image, encoding=camera.encoding)
            ros_image.header.stamp = self.node.get_clock().now().to_msg()

            # Camera info
            camera_info = self.camera_info[camera.height]
            camera_info.header.stamp = ros_image.header.stamp

            if self.config.conn_mode == 'single':
                camera_info.header.frame_id = 'front_camera'
                ros_image.header.frame_id = 'front_camera'
            else:
                camera_info.header.frame_id = f'robot{robot_data.robot_id}/front_camera'
                ros_image.header.frame_id = f'robot{robot_data.robot_id}/front_camera'

            # Publish
            self.publishers['camera'][robot_idx].publish(ros_image)
            self.publishers['camera_info'][robot_idx].publish(camera_info)

        except Exception as e:
            logger.error(f"Error publishing camera data: {e}")

    def publish_voxel_data(self, robot_data: RobotData) -> None:
        """Publish voxel data"""
        if not robot_data.lidar_data or not self.config.publish_raw_voxel:
            return

        try:
            robot_idx = int(robot_data.robot_id)
            lidar = robot_data.lidar_data

            voxel_msg = VoxelMapCompressed()
            voxel_msg.stamp = float(lidar.stamp)
            voxel_msg.frame_id = 'odom'
            voxel_msg.resolution = lidar.resolution
            voxel_msg.origin = lidar.origin
            voxel_msg.width = lidar.width or []
            voxel_msg.src_size = lidar.src_size or 0
            voxel_msg.data = lidar.compressed_data or b''

            self.publishers['voxel'][robot_idx].publish(voxel_msg)

        except Exception as e:
            logger.error(f"Error publishing voxel data: {e}") 