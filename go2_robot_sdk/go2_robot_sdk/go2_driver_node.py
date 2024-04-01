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
import os
import threading
import asyncio

from scripts.go2_constants import ROBOT_CMD, RTC_TOPIC
from scripts.go2_func import gen_command, gen_mov_command
from scripts.go2_lidar_decoder import update_meshes_for_cloud2
from scripts.go2_math import get_robot_joints
from scripts.webrtc_driver import Go2Connection

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Twist, TransformStamped
from go2_interfaces.msg import Go2State, IMU
from sensor_msgs.msg import PointCloud2, PointField, JointState, Joy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String


logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RobotBaseNode(Node):

    def __init__(self):
        super().__init__('go2_driver_node')

        self.declare_parameter('robot_ip', os.getenv('ROBOT_IP', os.getenv('GO2_IP')))
        self.declare_parameter('token', os.getenv('ROBOT_TOKEN', os.getenv('GO2_TOKEN','')))
        
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.token = self.get_parameter('token').get_parameter_value().string_value

        self.conn = None
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.go2_state_pub = self.create_publisher(Go2State, 'go2_states', qos_profile)
        self.go2_lidar_pub = self.create_publisher(PointCloud2, 'point_cloud2', qos_profile)
        
        self.imu_pub = self.create_publisher(IMU, 'imu', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.robot_cmd_vel = None
        self.robot_odom = None
        self.robot_low_cmd = None
        self.robot_sport_state = None
        self.robot_lidar = None
        self.robot_command_queue = []
        self.joy_state = Joy()
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_cb,
            qos_profile)
        
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_cb,
            qos_profile)
        
        self.command_sub = self.create_subscription(
            String,
            'command',
            self.command_cb,
            qos_profile)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer_lidar = self.create_timer(0.5, self.timer_callback_lidar)

    def timer_callback(self):
        self.publish_odom()
        self.publish_robot_state()
        self.publish_joint_state()

    def timer_callback_lidar(self):
        self.publish_lidar()

    def cmd_vel_cb(self, msg):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z
        if x > 0.0 or y > 0.0 or z > 0.0:
            self.robot_cmd_vel = gen_mov_command(x, y, z)

    def joy_cb(self, msg):
        self.joy_state = msg

    def command_cb(self, msg):
        self.get_logger().info(f"Received command: {msg.data}")
        self.robot_command_queue.append(msg)

    def joy_cmd(self):
        if self.robot_cmd_vel:
            self.get_logger().info("Attack!")
            self.conn.data_channel.send(self.robot_cmd_vel)
            self.robot_cmd_vel = None

        if len(self.robot_command_queue) > 0:
            msg = self.robot_command_queue[0]
            robot_cmd = gen_command(ROBOT_CMD[msg.data])
            self._logger.info(f"Sending command: {ROBOT_CMD[msg.data]}")
            self.conn.data_channel.send(robot_cmd)
            self.robot_command_queue.pop(0)

        if self.joy_state.buttons and self.joy_state.buttons[1]:
            self.get_logger().info("Stand down")
            stand_down_cmd = gen_command(ROBOT_CMD["StandDown"])
            self.conn.data_channel.send(stand_down_cmd)

        if self.joy_state.buttons and self.joy_state.buttons[0]:
            self.get_logger().info("Stand up")
            stand_up_cmd = gen_command(ROBOT_CMD["StandUp"])
            self.conn.data_channel.send(stand_up_cmd)
            balance_stand_cmd = gen_command(ROBOT_CMD['BalanceStand'])
            self.conn.data_channel.send(balance_stand_cmd)

    def on_validated(self):
        for topic in RTC_TOPIC.values():
            self.conn.data_channel.send(json.dumps({"type": "subscribe", "topic": topic}))
        
    def on_data_channel_message(self, _, msg):
        
        if msg.get('topic') == RTC_TOPIC["ULIDAR_ARRAY"]:
            self.robot_lidar = msg

        if msg.get('topic') == RTC_TOPIC['ROBOTODOM']:
            self.robot_odom = msg

        if msg.get('topic') == RTC_TOPIC['LF_SPORT_MOD_STATE']:
            self.robot_sport_state = msg
            
        if msg.get('topic') == RTC_TOPIC['LOW_STATE']:
            self.robot_low_cmd = msg

    def publish_odom(self):
        if self.robot_odom:
            odom_trans = TransformStamped()
            odom_trans.header.stamp = self.get_clock().now().to_msg()
            odom_trans.header.frame_id = 'odom'
            odom_trans.child_frame_id = 'base_link'
            odom_trans.transform.translation.x = self.robot_odom['data']['pose']['position']['x']
            odom_trans.transform.translation.y = self.robot_odom['data']['pose']['position']['y']
            odom_trans.transform.translation.z = self.robot_odom['data']['pose']['position']['z'] + 0.07
            odom_trans.transform.rotation.x = self.robot_odom['data']['pose']['orientation']['x']
            odom_trans.transform.rotation.y = self.robot_odom['data']['pose']['orientation']['y']
            odom_trans.transform.rotation.z = self.robot_odom['data']['pose']['orientation']['z']
            odom_trans.transform.rotation.w = self.robot_odom['data']['pose']['orientation']['w']
            self.broadcaster.sendTransform(odom_trans)

    def publish_lidar(self):
        if self.robot_lidar:
            points = update_meshes_for_cloud2(
                self.robot_lidar["decoded_data"]["positions"], 
                self.robot_lidar["decoded_data"]["uvs"], 
                self.robot_lidar['data']['resolution'], 
                self.robot_lidar['data']['origin'],
                0
                )
            point_cloud = PointCloud2()
            point_cloud.header = Header(frame_id="odom")
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            point_cloud = point_cloud2.create_cloud(point_cloud.header, fields, points)
            self.go2_lidar_pub.publish(point_cloud)

    def publish_joint_state(self): 
        if self.robot_sport_state:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()

            fl_foot_pos_array = [
                self.robot_sport_state["data"]["foot_position_body"][3], 
                self.robot_sport_state["data"]["foot_position_body"][4], 
                self.robot_sport_state["data"]["foot_position_body"][5]
            ]

            FL_hip_joint, FL_thigh_joint, FL_calf_joint = get_robot_joints(
                fl_foot_pos_array,
                0
                )
            
            fr_foot_pos_array = [
                self.robot_sport_state["data"]["foot_position_body"][0], 
                self.robot_sport_state["data"]["foot_position_body"][1], 
                self.robot_sport_state["data"]["foot_position_body"][2]
            ]

            FR_hip_joint, FR_thigh_joint, FR_calf_joint = get_robot_joints(
                fr_foot_pos_array,
                1
                )
            
            rl_foot_pos_array = [
                self.robot_sport_state["data"]["foot_position_body"][9], 
                self.robot_sport_state["data"]["foot_position_body"][10], 
                self.robot_sport_state["data"]["foot_position_body"][11]
            ]

            RL_hip_joint, RL_thigh_joint, RL_calf_joint = get_robot_joints(
                rl_foot_pos_array,
                2
                )
            
            rr_foot_pos_array = [
                self.robot_sport_state["data"]["foot_position_body"][6], 
                self.robot_sport_state["data"]["foot_position_body"][7], 
                self.robot_sport_state["data"]["foot_position_body"][8]
            ]

            RR_hip_joint, RR_thigh_joint, RR_calf_joint = get_robot_joints(
                rr_foot_pos_array,
                3
                )
            
            joint_state.name = [
                'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
                'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
                ]
            joint_state.position = [
                FL_hip_joint, FL_thigh_joint, FL_calf_joint,
                FR_hip_joint, FR_thigh_joint, FR_calf_joint,
                RL_hip_joint, RL_thigh_joint, RL_calf_joint,
                RR_hip_joint, RR_thigh_joint, RR_calf_joint,
                ]
            self.joint_pub.publish(joint_state) 

    def publish_robot_state(self):
        if self.robot_sport_state:
            go2_state = Go2State()
            go2_state.mode = self.robot_sport_state["data"]["mode"]
            go2_state.progress = self.robot_sport_state["data"]["progress"]
            go2_state.gait_type = self.robot_sport_state["data"]["gait_type"]
            go2_state.position = list(map(float,self.robot_sport_state["data"]["position"]))
            go2_state.body_height = float(self.robot_sport_state["data"]["body_height"])
            go2_state.velocity = self.robot_sport_state["data"]["velocity"]
            go2_state.range_obstacle = list(map(float, self.robot_sport_state["data"]["range_obstacle"]))
            go2_state.foot_force = self.robot_sport_state["data"]["foot_force"]
            go2_state.foot_position_body = list(map(float,self.robot_sport_state["data"]["foot_position_body"]))
            go2_state.foot_speed_body = list(map(float, self.robot_sport_state["data"]["foot_speed_body"]))
            self.go2_state_pub.publish(go2_state) 

            imu = IMU()
            imu.quaternion = list(map(float,self.robot_sport_state["data"]["imu_state"]["quaternion"]))
            imu.accelerometer = list(map(float,self.robot_sport_state["data"]["imu_state"]["accelerometer"]))
            imu.gyroscope = list(map(float,self.robot_sport_state["data"]["imu_state"]["gyroscope"]))
            imu.rpy = list(map(float,self.robot_sport_state["data"]["imu_state"]["rpy"]))
            imu.temperature = self.robot_sport_state["data"]["imu_state"]["temperature"]
            self.imu_pub.publish(imu) 

    async def run(self, conn):
        self.conn = conn
        await self.conn.connect()
        self.get_logger().info(f"Connected to {os.environ.get('ROBOT_IP')}")

        while True:
            self.joy_cmd()
            await asyncio.sleep(0.1)


async def spin(node: Node):
    cancel = node.create_guard_condition(lambda: None)
    def _spin(node: Node,
              future: asyncio.Future,
              event_loop: asyncio.AbstractEventLoop):
        while not future.cancelled():
            rclpy.spin_once(node)
        if not future.cancelled():
            event_loop.call_soon_threadsafe(future.set_result, None)
    event_loop = asyncio.get_event_loop()
    spin_task = event_loop.create_future()
    spin_thread = threading.Thread(target=_spin, args=(node, spin_task, event_loop))
    spin_thread.start()
    try:
        await spin_task
    except asyncio.CancelledError:
        cancel.trigger()
    spin_thread.join()
    node.destroy_guard_condition(cancel)


async def start_node():
    base_node = RobotBaseNode()
    conn = Go2Connection(
        robot_ip=base_node.robot_ip,
        token=base_node.token,
        on_validated=base_node.on_validated,
        on_message=base_node.on_data_channel_message,

    )
    spin_task = asyncio.get_event_loop().create_task(spin(base_node))
    sleep_task = asyncio.get_event_loop().create_task(base_node.run(conn))
    await asyncio.wait([spin_task, sleep_task], return_when=asyncio.FIRST_COMPLETED)

def main():
    rclpy.init()
    asyncio.get_event_loop().run_until_complete(start_node())
    asyncio.get_event_loop().close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()