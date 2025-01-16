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


"""
this driver node only exposes ros2 topics over webrtc
"""

import json
import logging
import os
import threading
import asyncio

from aiortc import MediaStreamTrack
from cv_bridge import CvBridge


from scripts_go2.go2_constants import ROBOT_CMD, RTC_TOPIC
from scripts_go2.webrtc_driver import Go2Connection

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from go2_interfaces.msg import Go2State, IMU
from unitree_go.msg import LowState
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

        self.declare_parameter('robot_ip', os.getenv(
            'ROBOT_IP', os.getenv('GO2_IP')))
        self.declare_parameter('token', os.getenv(
            'ROBOT_TOKEN', os.getenv('GO2_TOKEN', '')))
        self.declare_parameter('conn_type', os.getenv(
            'CONN_TYPE', os.getenv('CONN_TYPE', '')))

        self.robot_ip = self.get_parameter(
            'robot_ip').get_parameter_value().string_value
        self.token = self.get_parameter(
            'token').get_parameter_value().string_value
        self.robot_ip_lst = self.robot_ip.replace(" ", "").split(",")
        self.conn_type = self.get_parameter(
            'conn_type').get_parameter_value().string_value

        self.conn_mode = "single" if len(self.robot_ip_lst) == 1 else "multi"

        self.get_logger().info(f"Received ip list: {self.robot_ip_lst}")
        self.get_logger().info(f"Connection type is {self.conn_type}")

        self.get_logger().info(f"Connection mode is {self.conn_mode}")

        self.conn = {}

    def on_validated(self, robot_num):
        if robot_num in self.conn:
            for topic in RTC_TOPIC.values():
                self.conn[robot_num].data_channel.send(
                    json.dumps({"type": "subscribe", "topic": topic}))

    async def run(self, conn, robot_num):
        self.conn[robot_num] = conn

        if self.conn_type == 'webrtc':
            await self.conn[robot_num].connect()
            # await self.conn[robot_num].data_channel.disableTrafficSaving(True)

        while True:
        #     self.joy_cmd(robot_num)
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
    spin_thread = threading.Thread(
        target=_spin, args=(node, spin_task, event_loop))
    spin_thread.start()
    try:
        await spin_task
    except asyncio.CancelledError:
        cancel.trigger()
    spin_thread.join()
    node.destroy_guard_condition(cancel)


async def start_node():
    base_node = RobotBaseNode()
    spin_task = asyncio.get_event_loop().create_task(spin(base_node))
    sleep_task_lst = []

    for i in range(len(base_node.robot_ip_lst)):

        conn = Go2Connection(
            robot_ip=base_node.robot_ip_lst[i],
            robot_num=str(i),
            token=base_node.token,
            on_validated=base_node.on_validated,
            # on_message=base_node.on_data_channel_message,
            # on_video_frame=base_node.on_video_frame,
        )

        sleep_task_lst.append(asyncio.get_event_loop(
        ).create_task(base_node.run(conn, str(i))))

    await asyncio.wait([spin_task, *sleep_task_lst], return_when=asyncio.FIRST_COMPLETED)


def main():
    rclpy.init()
    asyncio.get_event_loop().run_until_complete(start_node())
    asyncio.get_event_loop().close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
