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
import numpy as np
import time

from aiortc import MediaStreamTrack
from cv_bridge import CvBridge


from scripts.go2_constants import ROBOT_CMD, RTC_TOPIC
from scripts.go2_func import gen_command, gen_mov_command
from scripts.go2_camerainfo import load_camera_info
from scripts.webrtc_driver import Go2Connection

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions
from rcl_interfaces.msg import SetParametersResult

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped
from go2_interfaces.msg import Go2State, IMU
from unitree_go.msg import VoxelMapCompressed, WebRtcReq
from sensor_msgs.msg import PointCloud2, PointField, JointState, Joy
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


        # Declare all parameters at once
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_ip', robot_ip),
                ('token', token),
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

        if len(self.robot_ip_lst) > 1:
            self.get_logger().warning(
                "Multi mode is no longer supported. Only the first IP will be used: "
                f"{self.robot_ip_lst[0]}"
            )
            self.robot_ip_lst = [self.robot_ip_lst[0]]

        self.enable_video = self.get_parameter('enable_video').get_parameter_value().bool_value
        self.decode_lidar = self.get_parameter('decode_lidar').get_parameter_value().bool_value
        self.publish_raw_voxel = self.get_parameter('publish_raw_voxel').get_parameter_value().bool_value
        self.obstacle_avoidance = self.get_parameter('obstacle_avoidance').get_parameter_value().bool_value
        
        # Log configuration info
        self.get_logger().info(f"Received ip list: {self.robot_ip_lst}")
        self.get_logger().info(f"Enable video is {self.enable_video}")
        self.get_logger().info(f"Decode lidar is {self.decode_lidar}")
        self.get_logger().info(f"Publish raw voxel is {self.publish_raw_voxel}")
        self.get_logger().info(f"Obstacle avoidance is {self.obstacle_avoidance}")
        
        self.conn = None  # Single robot connection
        qos_profile = QoSProfile(depth=10)
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers - single robot only
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.go2_state_pub = self.create_publisher(Go2State, 'go2_states', qos_profile)
        self.go2_lidar_pub = self.create_publisher(
            PointCloud2,
            'point_cloud2',
            best_effort_qos,
            qos_overriding_options=QoSOverridingOptions.with_default_policies())
        self.go2_odometry_pub = self.create_publisher(Odometry, 'odom', qos_profile)
        self.imu_pub = self.create_publisher(IMU, 'imu', qos_profile)
        
        if self.enable_video:
            self.img_pub = self.create_publisher(
                Image,
                'camera/image_raw',
                best_effort_qos,
                qos_overriding_options=QoSOverridingOptions.with_default_policies())
            self.camera_info_pub = self.create_publisher(
                CameraInfo,
                'camera/camera_info',
                best_effort_qos,
                qos_overriding_options=QoSOverridingOptions.with_default_policies())
        else:
            self.img_pub = None
            self.camera_info_pub = None
            
        if self.publish_raw_voxel:
            self.voxel_pub = self.create_publisher(
                VoxelMapCompressed,
                '/utlidar/voxel_map_compressed',
                best_effort_qos)
        else:
            self.voxel_pub = None

        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.bridge = CvBridge()
        self.camera_info = load_camera_info()

        # Single robot data - no dictionaries needed
        self.cmd_vel = None
        self.odom = None
        self.low_cmd = None
        self.sport_state = None
        self.lidar = None
        self.webrtc_msgs = asyncio.Queue()

        self.joy_state = Joy()

        # Subscriptions - single robot only
        self.create_subscription(
            Twist,
            'cmd_vel_out',
            self.cmd_vel_cb,
            qos_profile)
        self.create_subscription(
            WebRtcReq,
            'webrtc_req',
            self.webrtc_req_cb,
            qos_profile)

        self.create_subscription(
            Joy,
            'joy',
            self.joy_cb,
            qos_profile)


    def cb_set_parameters(self, params):
        result = SetParametersResult(successful=True)

        try:
            for p in params:
                if p.name == 'obstacle_avoidance':
                    self.get_logger().info(f'New obstacle_avoidance value: {p.value}')
                    self.obstacle_avoidance = p.value
                    
                    try:
                        self.set_is_remote_command_from_api()
                    except Exception as e:
                        self.get_logger().error(f"Failed to call is_remote_commands_from_api:{e}")
                        result.successful = False
                        result.reason = str(e)
                        break
                    
                    result.successful = True
                    result.reason = 'Updated obstacle_avoidance'
                    break    
        except Exception as e:
            self.get_logger().error(f"Error while setting parameters: {e}")
            result.successful = False
            result.reason = str(e)
        return result

    def _process_lidar_msg(self):
        
        if not self.lidar:
            return

        decoded = self.lidar["decoded_data"]
        cloud_array = None

        if "points" in decoded and decoded["points"] is not None:
            pts = decoded["points"].astype(np.float32)
        else:
            self.get_logger().warning("No point data in decoded lidar message – skipping frame")
            return

        cloud_array = pts  # shape (N,3)

        pc_msg = PointCloud2()
        pc_msg.header = Header(frame_id="odom")
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.height = 1
        pc_msg.width = cloud_array.shape[0]
        pc_msg.point_step = 12
        pc_msg.row_step = pc_msg.point_step * pc_msg.width
        pc_msg.is_dense = True
        pc_msg.is_bigendian = False
        pc_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.data = cloud_array.tobytes(order='C')
        self.go2_lidar_pub.publish(pc_msg)

        if self.publish_raw_voxel:
            voxel_msg = VoxelMapCompressed()
            voxel_msg.stamp = float(self.lidar["data"]["stamp"])
            voxel_msg.frame_id = "odom"
            voxel_msg.resolution = self.lidar["data"]["resolution"]
            voxel_msg.origin = self.lidar["data"]["origin"]
            voxel_msg.width = self.lidar["data"]["width"]
            voxel_msg.src_size = self.lidar["data"]["src_size"]
            voxel_msg.data = self.lidar["compressed_data"]
            self.voxel_pub.publish(voxel_msg)


    def cmd_vel_cb(self, msg):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z

        # Allow omni-directional movement
        if True or x != 0.0 or y != 0.0 or z != 0.0:
            self.cmd_vel = gen_mov_command(
                round(x, 2), round(y, 2), round(z, 2), self.obstacle_avoidance)

            self.get_logger().info(
                f"Received cmd_vel: {self.cmd_vel}")

    def webrtc_req_cb(self, msg):
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
        if not hasattr(self, "webrtc_msgs") or self.webrtc_msgs is None:
            self.get_logger().warn("WebRTC message queue is not initialized.")
            return
        try:
            payload = gen_command(
                1004, {"is_remote_commands_from_api": self.obstacle_avoidance},
                RTC_TOPIC['OBSTACLES_AVOID'])
            self.get_logger().info(f"Chaning remote command from api: {payload[:50]}")
            self.webrtc_msgs.put_nowait(payload)
        except Exception as e:
            self.get_logger().error(f"Failed to set remote command from API: {e}")

    def joy_cmd(self):
        if self.conn and self.cmd_vel is not None:
            self.get_logger().info("Move")
            if self.obstacle_avoidance:
                self.set_is_remote_command_from_api()
                    
            self.conn.data_channel.send(
                self.cmd_vel)
            self.cmd_vel = None

        if self.conn and self.joy_state.buttons and self.joy_state.buttons[1]:
            self.get_logger().info("Stand down")
            stand_down_cmd = gen_command(ROBOT_CMD["StandDown"])
            self.conn.data_channel.send(stand_down_cmd)

        if self.conn and self.joy_state.buttons and self.joy_state.buttons[0]:
            self.get_logger().info("Stand up")
            stand_up_cmd = gen_command(ROBOT_CMD["StandUp"])
            self.conn.data_channel.send(stand_up_cmd)
            move_cmd = gen_command(ROBOT_CMD['BalanceStand'])
            self.conn.data_channel.send(move_cmd)

    def on_validated(self):
        if self.conn:
            for topic in RTC_TOPIC.values():
                self.conn.data_channel.send(
                    json.dumps({"type": "subscribe", "topic": topic}))

    async def on_video_frame(self, track: MediaStreamTrack):
        logger.info(f"Video frame received for robot")

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

            camera_info.header.frame_id = 'front_camera'
            ros_image.header.frame_id = 'front_camera'

            # Publish image and camera info
            self.img_pub.publish(ros_image)
            self.camera_info_pub.publish(camera_info)
            await asyncio.sleep(0)

    def on_data_channel_message(self, _, msg):

        if msg.get('topic') == RTC_TOPIC["ULIDAR_ARRAY"]:
            self.lidar = msg
            
            if self.decode_lidar:
                self._process_lidar_msg()

            if self.publish_raw_voxel:
                self.publish_voxel_webrtc()

        if msg.get('topic') == RTC_TOPIC['ROBOTODOM']:
            self.odom = msg
            self.publish_odom_webrtc()
            self.publish_odom_topic_webrtc()

        if msg.get('topic') == RTC_TOPIC['LF_SPORT_MOD_STATE']:
            self.sport_state = msg
            self.publish_robot_state_webrtc()

        if msg.get('topic') == RTC_TOPIC['LOW_STATE']:
            self.low_cmd = msg
            self.publish_joint_state_webrtc()

        now = time.time()
        prev_ul = getattr(self, '_prev_ulidar', None)
        if prev_ul is not None:
            dt = now - prev_ul
            if dt > 0:
                self.get_logger().debug(f'ULIDAR in {1/dt:0.1f} Hz')
        self._prev_ulidar = now

    def publish_odom_webrtc(self):
        if self.odom:
            odom_trans = TransformStamped()
            odom_trans.header.stamp = self.get_clock().now().to_msg()
            odom_trans.header.frame_id = 'odom'

            odom_trans.child_frame_id = "base_link"

            pose = self.odom['data']['pose']
            position = pose['position']
            orientation = pose['orientation']

            pos_vals = [position['x'], position['y'], position['z']]
            rot_vals = [orientation['x'], orientation['y'], orientation['z'], orientation['w']]

            # Check all are numbers and finite
            if not all(
                isinstance(v, (int, float)) and math.isfinite(v)
                for v in pos_vals + rot_vals
            ):
                return
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
                self.get_logger().error(f"Error in publish_odom_webrtc: {e}")

    def publish_odom_topic_webrtc(self):
        if self.odom:
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'

            odom_msg.child_frame_id = "base_link"

            pose = self.odom['data']['pose']
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
                return

            try:
                odom_msg.pose.pose.position.x = float(position['x'])
                odom_msg.pose.pose.position.y = float(position['y'])
                odom_msg.pose.pose.position.z = float(position['z']) + 0.07

                odom_msg.pose.pose.orientation.x = float(orientation['x'])
                odom_msg.pose.pose.orientation.y = float(orientation['y'])
                odom_msg.pose.pose.orientation.z = float(orientation['z'])
                odom_msg.pose.pose.orientation.w = float(orientation['w'])

                self.go2_odometry_pub.publish(odom_msg)

            except Exception as e:
                self.get_logger().error(f"Error in publish_odom_topic_webrtc: {e}")


    def publish_voxel_webrtc(self):
        if self.lidar:
            voxel_msg = VoxelMapCompressed()
            voxel_msg.stamp = float(
                self.lidar['data']['stamp'])
            voxel_msg.frame_id = 'odom'

            # Example data: {"type":"msg","topic":"rt/utlidar/voxel_map_compressed",
            # "data":{"stamp":1.709106e+09,"frame_id":"odom","resolution":0.050000,
            # "src_size":77824,"origin":[1.675000,5.325000,-0.575000],"width":[128,128,38]}}
            voxel_msg.resolution = self.lidar['data']['resolution']
            voxel_msg.origin = self.lidar['data']['origin']
            voxel_msg.width = self.lidar['data']['width']
            voxel_msg.src_size = self.lidar['data']['src_size']
            voxel_msg.data = self.lidar['compressed_data']

            self.voxel_pub.publish(voxel_msg)

    def publish_joint_state_webrtc(self):
        # Modified to iterate over self.robot_low_cmd instead of self.robot_sport_state
        if self.low_cmd:
            # Extract data message for easy reading.
            low_state_data = self.low_cmd['data']

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()

            # Define joint names for single robot
            joint_state.name = [
                'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
                'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            ]
            
            # Use the same logic to access motor data (motor_state) from the cyclone_dds function.
            # Modify the access method to fit the JSON structure coming from WebRTC.
            motor_state = low_state_data['motor_state']
            joint_state.position = [
                motor_state[3]['q'], motor_state[4]['q'], motor_state[5]['q'],  # FL leg
                motor_state[0]['q'], motor_state[1]['q'], motor_state[2]['q'],  # FR leg
                motor_state[9]['q'], motor_state[10]['q'], motor_state[11]['q'], # RL leg
                motor_state[6]['q'], motor_state[7]['q'], motor_state[8]['q'],  # RR leg
            ]

            # Publish joint data.
            try:
                self.joint_pub.publish(joint_state)
            except Exception as e:
                self.get_logger().error(f"Error in publish_joint_state_webrtc: {e}")

    def publish_webrtc_commands(self):
        while True:
            try:
                message = self.webrtc_msgs.get_nowait()
                try:
                    self.conn.data_channel.send(message)
                finally:
                    self.webrtc_msgs.task_done()
            except asyncio.QueueEmpty:
                break

    def publish_robot_state_webrtc(self):
        if self.sport_state:
            try:
                data = self.sport_state["data"]

                # Check required lists for float validity
                if not all(
                    isinstance(x, float) and math.isfinite(x)
                    for x in data["position"]
                ):
                    return
                if not all(
                    isinstance(x, float) and math.isfinite(x)
                    for x in data["range_obstacle"]
                ):
                    return
                if not all(
                    isinstance(x, float) and math.isfinite(x)
                    for x in data["foot_position_body"]
                ):
                    return
                if not all(
                    isinstance(x, float) and math.isfinite(x)
                    for x in data["foot_speed_body"]
                ):
                    return
                if (
                    not isinstance(data["body_height"], (int, float)) or
                    not math.isfinite(data["body_height"])
                ):
                    return

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
                self.go2_state_pub.publish(go2_state)

                imu_data = data["imu_state"]
                # Check all IMU fields
                if not all(
                    isinstance(x, float) and math.isfinite(x)
                    for x in imu_data["quaternion"]
                ):
                    return
                if not all(
                    isinstance(x, float) and math.isfinite(x)
                    for x in imu_data["accelerometer"]
                ):
                    return
                if not all(
                    isinstance(x, float) and math.isfinite(x)
                    for x in imu_data["gyroscope"]
                ):
                    return
                if not all(
                    isinstance(x, float) and math.isfinite(x)
                    for x in imu_data["rpy"]
                ):
                    return

                imu = IMU()
                imu.quaternion = list(map(float, imu_data["quaternion"]))
                imu.accelerometer = list(map(float, imu_data["accelerometer"]))
                imu.gyroscope = list(map(float, imu_data["gyroscope"]))
                imu.rpy = list(map(float, imu_data["rpy"]))
                imu.temperature = imu_data["temperature"]
                self.imu_pub.publish(imu)
            except Exception as e:
                self.get_logger().error(f"Error in publish_robot_state_webrtc: {e}")


async def run(conn, node):
    """
    Standalone run function to handle robot connection.

    Args:
        conn: The robot connection object
        robot_num: The robot number as a string
        node: The RobotBaseNode instance
    """
    node.conn = conn

    try:
        await node.conn.connect()
        
        # Disable traffic saving for better lidar data transmission
        try:
            await node.conn.disableTrafficSaving(True)
            node.get_logger().info(f"Traffic saving disabled for robot")
        except Exception as e:
            node.get_logger().warning(f"Failed to disable traffic saving for robot: {e}")
            
    except Exception as e:
        node.get_logger().error(
            f"Failed to connect to robot- exiting: {e}")
        # Signal that a critical error occurred by raising an exception
        raise RuntimeError(f"Failed to connect to robot") from e

    try:
        while True:
            node.joy_cmd()
            node.publish_webrtc_commands()
            await asyncio.sleep(0.1)
    except Exception as e:
        node.get_logger().error(
            f"Error in run loop for robot: {e}")
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

    # Start connection to single robot
    try:
        conn = Go2Connection(
            robot_ip=base_node.robot_ip_lst[0],  # Use first (and only) robot IP
            token=base_node.token,
            on_validated=base_node.on_validated,
            on_message=base_node.on_data_channel_message,
            on_video_frame=base_node.on_video_frame if base_node.enable_video else None,
            decode_lidar=base_node.decode_lidar,
        )

        # Start the robot connection
        run_task = asyncio.create_task(run(conn, base_node))
        robot_tasks.append(run_task)

        def robot_callback(task):
            try:
                task.result()  # Will raise exception if one occurred
            except asyncio.CancelledError:
                # Normal during shutdown
                pass
            except Exception as e:
                handle_error(e, f"robot connection")

        # Add the callback to the task
        run_task.add_done_callback(robot_callback)

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
