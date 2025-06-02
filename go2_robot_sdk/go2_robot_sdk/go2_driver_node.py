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


import json
import logging
import math
import os
import threading
import asyncio

from aiortc import MediaStreamTrack
from cv_bridge import CvBridge


from scripts.go2_constants import ROBOT_CMD, RTC_TOPIC
from scripts.go2_func import gen_command, gen_mov_command
from scripts.go2_lidar_decoder import update_meshes_for_cloud2
from scripts.go2_math import get_robot_joints
from scripts.go2_camerainfo import load_camera_info
from scripts.webrtc_driver import Go2Connection

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions
from rcl_interfaces.msg import SetParametersResult

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from go2_interfaces.msg import Go2State, IMU
from unitree_go.msg import LowState, VoxelMapCompressed, WebRtcReq
from sensor_msgs.msg import PointCloud2, PointField, JointState, Joy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo


logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RobotBaseNode(Node):

    def __init__(self):
        super().__init__('go2_driver_node')

        robot_ip = os.getenv('ROBOT_IP', os.getenv('GO2_IP', ''))
        token = os.getenv('ROBOT_TOKEN', os.getenv('GO2_TOKEN', ''))
        conn_type = os.getenv('CONN_TYPE', '')

        # Declare all parameters at once
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_ip', robot_ip),
                ('token', token),
                ('conn_type', conn_type),
                ('enable_video', True),
                ('decode_lidar', True),
                ('publish_raw_voxel', False),
                ('obstacle_avoidance', False),
            ]
        )

        self.add_on_set_parameters_callback(self.cb_set_parameters)

        self.robot_ip = self.get_parameter(
            'robot_ip').get_parameter_value().string_value
        self.token = self.get_parameter(
            'token').get_parameter_value().string_value
        self.robot_ip_lst = self.robot_ip.replace(" ", "").split(",")
        self.conn_type = self.get_parameter(
            'conn_type').get_parameter_value().string_value
        self.enable_video = self.get_parameter(
            'enable_video').get_parameter_value().bool_value
        self.decode_lidar = self.get_parameter(
            'decode_lidar').get_parameter_value().bool_value
        self.publish_raw_voxel = self.get_parameter(
            'publish_raw_voxel').get_parameter_value().bool_value
        self.obstacle_avoidance = self.get_parameter(
            'obstacle_avoidance').get_parameter_value().bool_value

        self.conn_mode = "single" if (
            len(self.robot_ip_lst) == 1 and self.conn_type != "cyclonedds") else "multi"

        self.get_logger().info(f"Received ip list: {self.robot_ip_lst}")
        self.get_logger().info(f"Connection type is {self.conn_type}")
        self.get_logger().info(f"Connection mode is {self.conn_mode}")
        self.get_logger().info(f"Enable video is {self.enable_video}")
        self.get_logger().info(f"Decode lidar is {self.decode_lidar}")
        self.get_logger().info(
            f"Publish raw voxel is {self.publish_raw_voxel}")
        self.get_logger().info(f"Obstacle avoidance is {self.obstacle_avoidance}")

        self.conn = {}
        qos_profile = QoSProfile(depth=10)
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.joint_pub = []
        self.go2_state_pub = []
        self.go2_lidar_pub = []
        self.go2_odometry_pub = []
        self.imu_pub = []
        self.img_pub = []
        self.camera_info_pub = []
        self.voxel_pub = []

        if self.conn_mode == 'single':
            self.joint_pub.append(self.create_publisher(
                JointState, 'joint_states', qos_profile))
            self.go2_state_pub.append(self.create_publisher(
                Go2State, 'go2_states', qos_profile))
            self.go2_lidar_pub.append(
                self.create_publisher(
                    PointCloud2,
                    'point_cloud2',
                    best_effort_qos,
                    qos_overriding_options=QoSOverridingOptions.with_default_policies()))
            self.go2_odometry_pub.append(
                self.create_publisher(Odometry, 'odom', qos_profile))
            self.imu_pub.append(self.create_publisher(IMU, 'imu', qos_profile))
            if self.enable_video:
                self.img_pub.append(
                    self.create_publisher(
                        Image,
                        'camera/image_raw',
                        best_effort_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))
                self.camera_info_pub.append(
                    self.create_publisher(
                        CameraInfo,
                        'camera/camera_info',
                        best_effort_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))
            if self.publish_raw_voxel:
                self.voxel_pub.append(
                    self.create_publisher(
                        VoxelMapCompressed,
                        '/utlidar/voxel_map_compressed',
                        best_effort_qos))

        else:
            for i in range(len(self.robot_ip_lst)):
                self.joint_pub.append(self.create_publisher(
                    JointState, f'robot{i}/joint_states', qos_profile))
                self.go2_state_pub.append(self.create_publisher(
                    Go2State, f'robot{i}/go2_states', qos_profile))
                self.go2_lidar_pub.append(
                    self.create_publisher(
                        PointCloud2,
                        f'robot{i}/point_cloud2',
                        best_effort_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))
                self.go2_odometry_pub.append(self.create_publisher(
                    Odometry, f'robot{i}/odom', qos_profile))
                self.imu_pub.append(self.create_publisher(
                    IMU, f'robot{i}/imu', qos_profile))
                if self.enable_video:
                    self.img_pub.append(
                        self.create_publisher(
                            Image,
                            f'robot{i}/camera/image_raw',
                            best_effort_qos,
                            qos_overriding_options=QoSOverridingOptions.with_default_policies()))
                    self.camera_info_pub.append(
                        self.create_publisher(
                            CameraInfo,
                            f'robot{i}/camera/camera_info',
                            best_effort_qos,
                            qos_overriding_options=QoSOverridingOptions.with_default_policies()))
                if self.publish_raw_voxel:
                    self.voxel_pub.append(
                        self.create_publisher(
                            VoxelMapCompressed,
                            f'robot{i}/utlidar/voxel_map_compressed',
                            best_effort_qos))

        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.bridge = CvBridge()
        self.camera_info = load_camera_info()

        self.robot_cmd_vel = {}
        self.robot_odom = {}
        self.robot_low_cmd = {}
        self.robot_sport_state = {}
        self.robot_lidar = {}
        self.webrtc_msgs = asyncio.Queue()

        self.joy_state = Joy()

        if self.conn_mode == 'single':
            self.create_subscription(
                Twist,
                'cmd_vel_out',
                lambda msg: self.cmd_vel_cb(msg, "0"),
                qos_profile)
            self.create_subscription(
                WebRtcReq,
                'webrtc_req',
                lambda msg: self.webrtc_req_cb(msg, "0"),
                qos_profile)
        else:
            for i in range(len(self.robot_ip_lst)):
                self.create_subscription(
                    Twist,
                    f'robot{str(i)}/cmd_vel_out',
                    lambda msg: self.cmd_vel_cb(msg, str(i)),
                    qos_profile)
                self.create_subscription(
                    WebRtcReq,
                    f'robot{str(i)}/webrtc_req',
                    lambda msg: self.webrtc_req_cb(msg, str(i)),
                    qos_profile)

        self.create_subscription(
            Joy,
            'joy',
            self.joy_cb,
            qos_profile)

        # Support for CycloneDDS (EDU version via ethernet)
        if self.conn_type == 'cyclonedds':
            self.create_subscription(
                LowState,
                'lowstate',
                self.publish_joint_state_cyclonedds,
                qos_profile)

            self.create_subscription(
                PoseStamped,
                '/utlidar/robot_pose',
                self.publish_body_poss_cyclonedds,
                qos_profile)

            self.create_subscription(
                PointCloud2,
                '/utlidar/cloud',
                self.publish_lidar_cyclonedds,
                qos_profile)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer_lidar = self.create_timer(0.5, self.timer_callback_lidar)

    def cb_set_parameters(self, params):
        result = SetParametersResult(successful=True)

        for p in params:
            if p.name == 'obstacle_avoidance':
                self.get_logger().info(f'New obstacle_avoidance value: {p.value.bool_value}')
                self.obstacle_avoidance = p.value.bool_value
                self.set_is_remote_command_from_api()

                result.successful = True
                result.reason = 'Updated obstacle_avoidance'
                break

        return result

    def timer_callback(self):
        if self.conn_type == 'webrtc':
            self.publish_odom_webrtc()
            self.publish_odom_topic_webrtc()
            self.publish_robot_state_webrtc()
            self.publish_joint_state_webrtc()

    def timer_callback_lidar(self):
        if self.conn_type == 'webrtc' and self.decode_lidar:
            self.publish_lidar_webrtc()

        # Publish raw voxel data
        if self.conn_type == 'webrtc' and self.publish_raw_voxel:
            self.publish_voxel_webrtc()

    def cmd_vel_cb(self, msg, robot_num):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z

        # Allow omni-directional movement
        if True or x != 0.0 or y != 0.0 or z != 0.0:
            self.robot_cmd_vel[robot_num] = gen_mov_command(
                round(x, 2), round(y, 2), round(z, 2), self.obstacle_avoidance)

            self.get_logger().info(
                f"Received cmd_vel: {self.robot_cmd_vel[robot_num]}")

    def webrtc_req_cb(self, msg, robot_num):
        parameter_str = msg.parameter
        try:
            parameter = "" if parameter_str == "" else json.loads(parameter_str)
        except ValueError as e:
            self.get_logger().error(f"Invalid JSON in WebRTC request: {e}")
            parameter = parameter_str
        payload = gen_command(msg.api_id, parameter, msg.topic, msg.id)
        self.get_logger().info(f"Received WebRTC request: {payload[:50]}")
        self.webrtc_msgs.put_nowait(payload)

    def joy_cb(self, msg):
        self.joy_state = msg

    def set_is_remote_command_from_api(self):
        payload = gen_command(
            1004, {"is_remote_commands_from_api": self.obstacle_avoidance},
            RTC_TOPIC['OBSTACLE_AVOIDANCE'])
        self.get_logger().info(f"Chaning remote command from api: {payload[:50]}")
        self.webrtc_msgs.put_nowait(payload)

    def publish_body_poss_cyclonedds(self, msg):
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = "robot0/base_link"
        odom_trans.transform.translation.x = msg.pose.position.x
        odom_trans.transform.translation.y = msg.pose.position.y
        odom_trans.transform.translation.z = msg.pose.position.z + 0.07
        odom_trans.transform.rotation.x = msg.pose.orientation.x
        odom_trans.transform.rotation.y = msg.pose.orientation.y
        odom_trans.transform.rotation.z = msg.pose.orientation.z
        odom_trans.transform.rotation.w = msg.pose.orientation.w
        self.broadcaster.sendTransform(odom_trans)

    def publish_joint_state_cyclonedds(self, msg):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'robot0/FL_hip_joint',
            'robot0/FL_thigh_joint',
            'robot0/FL_calf_joint',
            'robot0/FR_hip_joint',
            'robot0/FR_thigh_joint',
            'robot0/FR_calf_joint',
            'robot0/RL_hip_joint',
            'robot0/RL_thigh_joint',
            'robot0/RL_calf_joint',
            'robot0/RR_hip_joint',
            'robot0/RR_thigh_joint',
            'robot0/RR_calf_joint',
        ]
        joint_state.position = [
            msg.motor_state[3].q, msg.motor_state[4].q, msg.motor_state[5].q,
            msg.motor_state[0].q, msg.motor_state[1].q, msg.motor_state[2].q,
            msg.motor_state[9].q, msg.motor_state[10].q, msg.motor_state[11].q,
            msg.motor_state[6].q, msg.motor_state[7].q, msg.motor_state[8].q,
        ]
        self.joint_pub[0].publish(joint_state)

    def publish_lidar_cyclonedds(self, msg):
        msg.header = Header(frame_id="robot0/radar")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.go2_lidar_pub[0].publish(msg)

    def joy_cmd(self, robot_num):
        if robot_num in self.conn and robot_num in self.robot_cmd_vel and self.robot_cmd_vel[
                robot_num] is not None:
            self.get_logger().info("Move")
            self.conn[robot_num].data_channel.send(
                self.robot_cmd_vel[robot_num])
            self.robot_cmd_vel[robot_num] = None

        if robot_num in self.conn and self.joy_state.buttons and self.joy_state.buttons[1]:
            self.get_logger().info("Stand down")
            stand_down_cmd = gen_command(ROBOT_CMD["StandDown"])
            self.conn[robot_num].data_channel.send(stand_down_cmd)

        if robot_num in self.conn and self.joy_state.buttons and self.joy_state.buttons[0]:
            self.get_logger().info("Stand up")
            stand_up_cmd = gen_command(ROBOT_CMD["StandUp"])
            self.conn[robot_num].data_channel.send(stand_up_cmd)
            move_cmd = gen_command(ROBOT_CMD['BalanceStand'])
            self.conn[robot_num].data_channel.send(move_cmd)

    def on_validated(self, robot_num):
        if robot_num in self.conn:
            for topic in RTC_TOPIC.values():
                self.conn[robot_num].data_channel.send(
                    json.dumps({"type": "subscribe", "topic": topic}))

    async def on_video_frame(self, track: MediaStreamTrack, robot_num):
        logger.info(f"Video frame received for robot {robot_num}")

        while True:
            frame = await track.recv()
            img = frame.to_ndarray(format="bgr24")

            logger.debug(
                f"Shape: {img.shape}, Dimensions: {img.ndim}, Type: {img.dtype}, Size: {img.size}")

            # Convert the OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()

            # Set the timestamp for both image and camera info
            camera_info = self.camera_info[ros_image.height]
            camera_info.header.stamp = ros_image.header.stamp

            if self.conn_mode == 'single':
                camera_info.header.frame_id = 'front_camera'
                ros_image.header.frame_id = 'front_camera'
            else:
                camera_info.header.frame_id = f'robot{str(robot_num)}/front_camera'
                ros_image.header.frame_id = f'robot{str(robot_num)}/front_camera'

            # Publish image and camera info
            self.img_pub[robot_num].publish(ros_image)
            self.camera_info_pub[robot_num].publish(camera_info)
            await asyncio.sleep(0)

    def on_data_channel_message(self, _, msg, robot_num):

        if msg.get('topic') == RTC_TOPIC["ULIDAR_ARRAY"]:
            self.robot_lidar[robot_num] = msg

        if msg.get('topic') == RTC_TOPIC['ROBOTODOM']:
            self.robot_odom[robot_num] = msg

        if msg.get('topic') == RTC_TOPIC['LF_SPORT_MOD_STATE']:
            self.robot_sport_state[robot_num] = msg

        if msg.get('topic') == RTC_TOPIC['LOW_STATE']:
            self.robot_low_cmd[robot_num] = msg

    def publish_odom_webrtc(self):
        for i in range(len(self.robot_odom)):
            if self.robot_odom[str(i)]:
                odom_trans = TransformStamped()
                odom_trans.header.stamp = self.get_clock().now().to_msg()
                odom_trans.header.frame_id = 'odom'

                if self.conn_mode == 'single':
                    odom_trans.child_frame_id = "base_link"
                else:
                    odom_trans.child_frame_id = f"robot{str(i)}/base_link"

                pose = self.robot_odom[str(i)]['data']['pose']
                position = pose['position']
                orientation = pose['orientation']

                pos_vals = [position['x'], position['y'], position['z']]
                rot_vals = [orientation['x'], orientation['y'], orientation['z'], orientation['w']]

                # Check all are numbers and finite
                if not all(
                    isinstance(v, (int, float)) and math.isfinite(v)
                    for v in pos_vals + rot_vals
                ):
                    continue
                try:
                    odom_trans.transform.translation.x = float(position['x'])
                    odom_trans.transform.translation.y = float(position['y'])
                    odom_trans.transform.translation.z = float(position['z']) + 0.07

                    odom_trans.transform.rotation.x = float(orientation['x'])
                    odom_trans.transform.rotation.y = float(orientation['y'])
                    odom_trans.transform.rotation.z = float(orientation['z'])
                    odom_trans.transform.rotation.w = float(orientation['w'])

                    self.broadcaster.sendTransform(odom_trans)
                except Exception as e:
                    self.get_logger().error(
                        f"Error in publish_odom_webrtc: {e} for robot {i}")

    def publish_odom_topic_webrtc(self):
        for i in range(len(self.robot_odom)):
            if self.robot_odom.get(str(i)):
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = 'odom'

                if self.conn_mode == 'single':
                    odom_msg.child_frame_id = "base_link"
                else:
                    odom_msg.child_frame_id = f"robot{str(i)}/base_link"

                pose = self.robot_odom[str(i)]['data']['pose']
                position = pose['position']
                orientation = pose['orientation']

                pos_vals = [position['x'], position['y'], position['z']]
                rot_vals = [orientation['x'],
                            orientation['y'],
                            orientation['z'],
                            orientation['w']]

                if not all(
                    isinstance(v, (int, float)) and math.isfinite(v)
                    for v in pos_vals + rot_vals
                ):
                    continue

                try:
                    odom_msg.pose.pose.position.x = float(position['x'])
                    odom_msg.pose.pose.position.y = float(position['y'])
                    odom_msg.pose.pose.position.z = float(position['z']) + 0.07

                    odom_msg.pose.pose.orientation.x = float(orientation['x'])
                    odom_msg.pose.pose.orientation.y = float(orientation['y'])
                    odom_msg.pose.pose.orientation.z = float(orientation['z'])
                    odom_msg.pose.pose.orientation.w = float(orientation['w'])

                    self.go2_odometry_pub[i].publish(odom_msg)

                except Exception as e:
                    self.get_logger().error(
                        f"Error in publish_odom_topic_webrtc: {e} for robot {i}")

    def publish_lidar_webrtc(self):
        for i in range(len(self.robot_lidar)):
            if self.robot_lidar[str(i)]:
                points = update_meshes_for_cloud2(
                    self.robot_lidar[str(i)]["decoded_data"]["positions"],
                    self.robot_lidar[str(i)]["decoded_data"]["uvs"],
                    self.robot_lidar[str(i)]['data']['resolution'],
                    self.robot_lidar[str(i)]['data']['origin'],
                    0
                )
                point_cloud = PointCloud2()
                point_cloud.header = Header(frame_id="odom")
                point_cloud.header.stamp = self.get_clock().now().to_msg()
                fields = [
                    PointField(name='x', offset=0,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12,
                               datatype=PointField.FLOAT32, count=1),
                ]
                point_cloud = point_cloud2.create_cloud(
                    point_cloud.header, fields, points)
                self.go2_lidar_pub[i].publish(point_cloud)

    def publish_voxel_webrtc(self):
        for i in range(len(self.robot_lidar)):
            if self.robot_lidar[str(i)]:
                voxel_msg = VoxelMapCompressed()
                voxel_msg.stamp = float(
                    self.robot_lidar[str(i)]['data']['stamp'])
                voxel_msg.frame_id = 'odom'

                # Example data: {"type":"msg","topic":"rt/utlidar/voxel_map_compressed",
                # "data":{"stamp":1.709106e+09,"frame_id":"odom","resolution":0.050000,
                # "src_size":77824,"origin":[1.675000,5.325000,-0.575000],"width":[128,128,38]}}
                voxel_msg.resolution = self.robot_lidar[str(
                    i)]['data']['resolution']
                voxel_msg.origin = self.robot_lidar[str(i)]['data']['origin']
                voxel_msg.width = self.robot_lidar[str(i)]['data']['width']
                voxel_msg.src_size = self.robot_lidar[str(
                    i)]['data']['src_size']
                voxel_msg.data = self.robot_lidar[str(i)]['compressed_data']

                self.voxel_pub[i].publish(voxel_msg)

    def publish_joint_state_webrtc(self):

        for i in range(len(self.robot_sport_state)):
            if self.robot_sport_state[str(i)]:
                try:
                    joint_state = JointState()
                    joint_state.header.stamp = self.get_clock().now().to_msg()

                    foot_data = self.robot_sport_state[str(i)]["data"]["foot_position_body"]
                    # Check if all foot data is float, if not skip
                    if not all(isinstance(x, float) and math.isfinite(x) for x in foot_data):
                        continue
                    # Front Left
                    fl_foot_pos_array = list(map(float, foot_data[3:6]))
                    FL_hip_joint, FL_thigh_joint, FL_calf_joint = get_robot_joints(
                        fl_foot_pos_array, 0
                    )

                    # Front Right
                    fr_foot_pos_array = list(map(float, foot_data[0:3]))
                    FR_hip_joint, FR_thigh_joint, FR_calf_joint = get_robot_joints(
                        fr_foot_pos_array, 1
                    )

                    # Rear Left
                    rl_foot_pos_array = list(map(float, foot_data[9:12]))
                    RL_hip_joint, RL_thigh_joint, RL_calf_joint = get_robot_joints(
                        rl_foot_pos_array, 2
                    )

                    # Rear Right
                    rr_foot_pos_array = list(map(float, foot_data[6:9]))
                    RR_hip_joint, RR_thigh_joint, RR_calf_joint = get_robot_joints(
                        rr_foot_pos_array, 3
                    )
                    # Joint names
                    if self.conn_mode == 'single':
                        joint_state.name = [
                            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
                            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
                        ]
                    else:
                        joint_state.name = [
                            f'robot{i}/FL_hip_joint',
                            f'robot{i}/FL_thigh_joint',
                            f'robot{i}/FL_calf_joint',
                            f'robot{i}/FR_hip_joint',
                            f'robot{i}/FR_thigh_joint',
                            f'robot{i}/FR_calf_joint',
                            f'robot{i}/RL_hip_joint',
                            f'robot{i}/RL_thigh_joint',
                            f'robot{i}/RL_calf_joint',
                            f'robot{i}/RR_hip_joint',
                            f'robot{i}/RR_thigh_joint',
                            f'robot{i}/RR_calf_joint'
                        ]

                    joint_state.position = [
                        FL_hip_joint, FL_thigh_joint, FL_calf_joint,
                        FR_hip_joint, FR_thigh_joint, FR_calf_joint,
                        RL_hip_joint, RL_thigh_joint, RL_calf_joint,
                        RR_hip_joint, RR_thigh_joint, RR_calf_joint,
                    ]

                    self.joint_pub[i].publish(joint_state)

                except Exception as e:
                    self.get_logger().error(
                        f"Error in publish_joint_state_webrtc: {e} for robot {i}"
                    )

    def publish_webrtc_commands(self, robot_num):
        while True:
            try:
                message = self.webrtc_msgs.get_nowait()
                try:
                    self.conn[robot_num].data_channel.send(message)
                finally:
                    self.webrtc_msgs.task_done()
            except asyncio.QueueEmpty:
                break

    def publish_robot_state_webrtc(self):
        for i in range(len(self.robot_sport_state)):
            if self.robot_sport_state[str(i)]:
                try:
                    data = self.robot_sport_state[str(i)]["data"]

                    # Check required lists for float validity
                    if not all(
                        isinstance(x, float) and math.isfinite(x)
                        for x in data["position"]
                    ):
                        continue
                    if not all(
                        isinstance(x, float) and math.isfinite(x)
                        for x in data["range_obstacle"]
                    ):
                        continue
                    if not all(
                        isinstance(x, float) and math.isfinite(x)
                        for x in data["foot_position_body"]
                    ):
                        continue
                    if not all(
                        isinstance(x, float) and math.isfinite(x)
                        for x in data["foot_speed_body"]
                    ):
                        continue
                    if (
                        not isinstance(data["body_height"], (int, float)) or
                        not math.isfinite(data["body_height"])
                    ):
                        continue

                    go2_state = Go2State()
                    go2_state.mode = data["mode"]
                    go2_state.progress = data["progress"]
                    go2_state.gait_type = data["gait_type"]
                    go2_state.position = list(map(float, data["position"]))
                    go2_state.body_height = float(data["body_height"])
                    go2_state.velocity = data["velocity"]
                    go2_state.range_obstacle = list(map(float, data["range_obstacle"]))
                    go2_state.foot_force = data["foot_force"]
                    go2_state.foot_position_body = list(map(float, data["foot_position_body"]))
                    go2_state.foot_speed_body = list(map(float, data["foot_speed_body"]))
                    self.go2_state_pub[i].publish(go2_state)

                    imu_data = data["imu_state"]
                    # Check all IMU fields
                    if not all(
                        isinstance(x, float) and math.isfinite(x)
                        for x in imu_data["quaternion"]
                    ):
                        continue
                    if not all(
                        isinstance(x, float) and math.isfinite(x)
                        for x in imu_data["accelerometer"]
                    ):
                        continue
                    if not all(
                        isinstance(x, float) and math.isfinite(x)
                        for x in imu_data["gyroscope"]
                    ):
                        continue
                    if not all(
                        isinstance(x, float) and math.isfinite(x)
                        for x in imu_data["rpy"]
                    ):
                        continue

                    imu = IMU()
                    imu.quaternion = list(map(float, imu_data["quaternion"]))
                    imu.accelerometer = list(map(float, imu_data["accelerometer"]))
                    imu.gyroscope = list(map(float, imu_data["gyroscope"]))
                    imu.rpy = list(map(float, imu_data["rpy"]))
                    imu.temperature = imu_data["temperature"]
                    self.imu_pub[i].publish(imu)
                except Exception as e:
                    self.get_logger().error(
                        f"Error in publish_robot_state_webrtc: {e} for robot {i}")


async def run(conn, robot_num, node):
    """
    Standalone run function to handle robot connection.

    Args:
        conn: The robot connection object
        robot_num: The robot number as a string
        node: The RobotBaseNode instance
    """
    node.conn[robot_num] = conn
    if node.conn_type == 'webrtc':
        try:
            await node.conn[robot_num].connect()
            # await
            # node.conn[robot_num].data_channel.disableTrafficSaving(True)
        except Exception as e:
            node.get_logger().error(
                f"Failed to connect to robot {robot_num} - exiting: {e}")
            # Signal that a critical error occurred by raising an exception
            raise RuntimeError(f"Failed to connect to robot {robot_num}") from e

    try:
        while True:
            if node.conn_type == 'webrtc':
                node.joy_cmd(robot_num)
                node.publish_webrtc_commands(robot_num)
            await asyncio.sleep(0.1)
    except Exception as e:
        node.get_logger().error(
            f"Error in run loop for robot {robot_num}: {e}")
        # Raise the exception to signal task failure
        raise


async def spin(node: Node):
    """Spin the node in a separate thread with proper context management."""
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Create a future that will be completed when we want to stop spinning
    event_loop = asyncio.get_event_loop()
    stop_future = event_loop.create_future()

    try:
        # Wait until the future is completed or cancelled
        await stop_future
    except asyncio.CancelledError:
        # Handle cancellation
        pass
    finally:
        # Shutdown the executor and join the thread
        executor.shutdown()
        spin_thread.join(timeout=1.0)  # Add timeout to avoid hanging


async def start_node():
    # Create the node
    base_node = RobotBaseNode()

    # Setup node spinning in a separate thread
    spin_task = asyncio.create_task(spin(base_node))

    # Track all robot connection tasks
    robot_tasks = []

    # Function to handle errors in any task
    def handle_error(e, task_name="unknown"):
        base_node.get_logger().error(f"Error in {task_name}: {e}")
        # Cancel the spin task to initiate shutdown
        if not spin_task.done():
            spin_task.cancel()
        # Cancel all robot tasks
        for task in robot_tasks:
            if not task.done():
                task.cancel()

    # Start connections to robots
    try:
        for i in range(len(base_node.robot_ip_lst)):
            conn = Go2Connection(
                robot_ip=base_node.robot_ip_lst[i],
                robot_num=str(i),
                token=base_node.token,
                on_validated=base_node.on_validated,
                on_message=base_node.on_data_channel_message,
                on_video_frame=base_node.on_video_frame if base_node.enable_video else None,
                decode_lidar=base_node.decode_lidar,
            )

            # Start the robot connection and add to our list
            # Use the standalone run function instead of a method on the node
            run_task = asyncio.create_task(run(conn, str(i), base_node))
            robot_tasks.append(run_task)

            # Define a unique callback for each task that can reference the
            # robot number
            robot_num = str(i)

            def create_callback(robot_id):
                def callback(task):
                    try:
                        task.result()  # Will raise exception if one occurred
                    except asyncio.CancelledError:
                        # Normal during shutdown
                        pass
                    except Exception as e:
                        handle_error(e, f"robot {robot_id}")
                return callback

            # Add the callback to the task
            run_task.add_done_callback(create_callback(robot_num))

        # Wait for any task to complete or fail
        done, pending = await asyncio.wait(
            [spin_task] + robot_tasks,
            return_when=asyncio.FIRST_COMPLETED
        )

        # Check if any task completed with an exception
        for task in done:
            try:
                task.result()
            except asyncio.CancelledError:
                # Normal during shutdown
                pass
            except Exception as e:
                handle_error(e, "completed task")

    except Exception as e:
        handle_error(e, "setup phase")
    finally:
        # Ensure clean shutdown
        if not spin_task.done():
            spin_task.cancel()

        for task in robot_tasks:
            if not task.done():
                task.cancel()

        # Wait for all tasks to finish
        try:
            # Use a timeout to avoid hanging indefinitely
            await asyncio.wait([spin_task] + robot_tasks, timeout=2.0)
        except Exception:
            # Ignore any errors during shutdown
            pass


def main():
    """Main entry point with proper initialization and cleanup."""
    # Initialize ROS
    rclpy.init()

    try:
        # Get the event loop
        loop = asyncio.get_event_loop()

        # Run the main coroutine
        loop.run_until_complete(start_node())
    except KeyboardInterrupt:
        print("Node terminated by keyboard interrupt")
    except Exception as e:
        print(f"Fatal error in node execution: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean shutdown
        try:
            # Close remaining tasks
            pending = asyncio.all_tasks(loop)
            if pending:
                print(f"Cancelling {len(pending)} pending tasks...")
                for task in pending:
                    task.cancel()
                # Wait briefly for tasks to acknowledge cancellation
                loop.run_until_complete(asyncio.gather(
                    *pending, return_exceptions=True))
        except Exception as e:
            print(f"Error during cleanup: {e}")

        # Finally, close the loop and shutdown ROS
        loop.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
