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


# WEB_RTC_COMM WAS ORIGINALY FORKED from https://github.com/tfoldi/go2-webrtc/tree/master
# Big thanks for your passion! @tfoldi (Földi Tamás)

import base64
import datetime
import hashlib
import json
import random
import rclpy
import aiohttp
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription
)
import asyncio
from aiortc.contrib.media import MediaBlackhole

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# PUT YOUR IP
ROBOT_IP = "192.168.31.20" 
# PUT YOUR TOKEN
ROBOT_TOKEN = ""
JOY_SENSITIVITY = 0.3


ROBOT_CMD = {
    "Damp": 1001,
    "BalanceStand": 1002,
    "StopMove": 1003,
    "StandUp": 1004,
    "StandDown": 1005,
    "RecoveryStand": 1006,
    "Euler": 1007,
    "Move": 1008,
    "Sit": 1009,
    "RiseSit": 1010,
    "SwitchGait": 1011,
    "Trigger": 1012,
    "BodyHeight": 1013,
    "FootRaiseHeight": 1014,
    "SpeedLevel": 1015,
    "Hello": 1016,
    "Stretch": 1017,
    "TrajectoryFollow": 1018,
    "ContinuousGait": 1019,
    "Content": 1020,
    "Wallow": 1021,
    "Dance1": 1022,
    "Dance2": 1023,
    "GetBodyHeight": 1024,
    "GetFootRaiseHeight": 1025,
    "GetSpeedLevel": 1026,
    "SwitchJoystick": 1027,
    "Pose": 1028,
    "Scrape": 1029,
    "FrontFlip": 1030,
    "FrontJump": 1031,
    "FrontPounce": 1032,
    "WiggleHips": 1033,
    "GetState": 1034,
    "EconomicGait": 1035,
    "FingerHeart": 1036,
}

RTC_TOPIC = {
    "LOW_STATE": "rt/lf/lowstate",
    "MULTIPLE_STATE": "rt/multiplestate",
    "FRONT_PHOTO_REQ": "rt/api/videohub/request",
    "ULIDAR_SWITCH": "rt/utlidar/switch",
    "ULIDAR": "rt/utlidar/voxel_map",
    "ULIDAR_ARRAY": "rt/utlidar/voxel_map_compressed",
    "ULIDAR_STATE": "rt/utlidar/lidar_state",
    "ROBOTODOM": "rt/utlidar/robot_pose",
    "UWB_REQ": "rt/api/uwbswitch/request",
    "UWB_STATE": "rt/uwbstate",
    "LOW_CMD": "rt/lowcmd",
    "WIRELESS_CONTROLLER": "rt/wirelesscontroller",
    "SPORT_MOD": "rt/api/sport/request",
    "SPORT_MOD_STATE": "rt/sportmodestate",
    "LF_SPORT_MOD_STATE": "rt/lf/sportmodestate",
    "BASH_REQ": "rt/api/bashrunner/request",
    "SELF_TEST": "rt/selftest",
    "GRID_MAP": "rt/mapping/grid_map",
    "SERVICE_STATE": "rt/servicestate",
    "GPT_FEEDBACK": "rt/gptflowfeedback",
    "VUI": "rt/api/vui/request",
    "OBSTACLES_AVOID": "rt/api/obstacles_avoid/request",
    "SLAM_QT_COMMAND": "rt/qt_command",
    "SLAM_ADD_NODE": "rt/qt_add_node",
    "SLAM_ADD_EDGE": "rt/qt_add_edge",
    "SLAM_QT_NOTICE": "rt/qt_notice",
    "SLAM_PC_TO_IMAGE_LOCAL": "rt/pctoimage_local",
    "SLAM_ODOMETRY": "rt/lio_sam_ros2/mapping/odometry",
    "ARM_COMMAND": "rt/arm_Command",
    "ARM_FEEDBACK": "rt/arm_Feedback",
    "AUDIO_HUB_REQ": "rt/api/audiohub/request",
    "AUDIO_HUB_PLAY_STATE": "rt/audiohub/player/state",
}


def generate_id():
    return int(datetime.datetime.now().timestamp() * 1000 % 2147483648) + random.randint(0, 999)


def gen_command(cmd: int):
    command = {
                "type": "msg", 
                "topic": "rt/api/sport/request", 
                "data": {
                    "header":
                        {
                            "identity":
                                {
                                    "id": generate_id(), 
                                    "api_id": cmd
                                }
                        },
                    "parameter": json.dumps(cmd)          
                    }
                }
    command = json.dumps(command)
    return command


def gen_mov_command(x: float, y: float, z: float):

    command = {
        "type": "msg", 
        "topic": "rt/api/sport/request", 
        "data": {
            "header":
                {
                    "identity":
                        {
                            "id": generate_id(), 
                            "api_id": 1008
                        }
                },
            "parameter": json.dumps(
                {
                    "x": x, 
                    "y": y, 
                    "z": z
                })
                
            }
        }
    command = json.dumps(command)
    return command


class Go2ConnectionAsync(Node):
    def __init__(self, robot_ip, token):

        super().__init__('go2_driver')
        
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_cb,
            10)
        self.robot_validation = "PENDING"
        self.pc = RTCPeerConnection()
        self.robot_ip = robot_ip
        self.token = token
        self.audio_track = MediaBlackhole()
        self.video_track = MediaBlackhole()
        self.data_channel = self.pc.createDataChannel("data", id=0)
        self.data_channel.on("open", self.on_data_channel_open)
        self.data_channel.on("message", self.on_data_channel_message)
        self.pc.on("track", self.on_track)
        self.pc.on("connectionstatechange", self.on_connection_state_change)

    def on_connection_state_change(self):
        self.get_logger().info(f"Connection state is {self.pc.connectionState}")

    def on_track(self, track):
        self.get_logger().info(f"Receiving {track.kind}")
        if track.kind == "audio":
            pass
        elif track.kind == "video":
            pass
    
    def on_data_channel_open(self):
        self.get_logger().info("Data channel is open")

    def on_data_channel_message(self, message):
        if self.data_channel.readyState != "open":
            self.data_channel._setReadyState("open")

        message = json.loads(message)

        if message.get("type") == "validation":
            self.validate_robot_conn(message)

        
        if message.get("type") == "validation" and message.get("data") == "Validation Ok.":
            self.data_channel.send('{"type": "subscribe", "topic": "rt/lf/lowstate" }')

        if message.get("type") == "msg" and message.get("topic") == "rt/lf/lowstate":
            # update joint_state
            joint_state = JointState()
            
            FL_hip_joint = message["data"]["motor_state"][0]["q"]
            FL_thigh_joint = message["data"]["motor_state"][1]["q"]
            FL_calf_joint = message["data"]["motor_state"][2]["q"]
            
            FR_hip_joint = message["data"]["motor_state"][3]["q"]
            FR_thigh_joint = message["data"]["motor_state"][4]["q"]
            FR_calf_joint = message["data"]["motor_state"][5]["q"]

            RL_hip_joint = message["data"]["motor_state"][6]["q"]
            RL_thigh_joint = message["data"]["motor_state"][7]["q"]
            RL_calf_joint = message["data"]["motor_state"][8]["q"]

            RR_hip_joint = message["data"]["motor_state"][9]["q"]
            RR_thigh_joint = message["data"]["motor_state"][10]["q"]
            RR_calf_joint = message["data"]["motor_state"][11]["q"] 

            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
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


    async def generate_offer(self):
        await self.audio_track.start()
        await self.video_track.start()
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)
        return offer.sdp


    async def set_answer(self, sdp):
        answer = RTCSessionDescription(sdp, type="answer")
        await self.pc.setRemoteDescription(answer)

    async def connect(self):
        offer = await self.generate_offer()
        async with aiohttp.ClientSession() as session:
            url = f"http://{self.robot_ip}:8081/offer"
            headers = {"content-type": "application/json"}
            data = {
                "sdp": offer,
                "id": "STA_localNetwork",
                "type": "offer",
                "token": self.token,
            }
            async with session.post(url, json=data, headers=headers) as resp:
                if resp.status == 200:
                    answer_data = await resp.json()
                    answer_sdp = answer_data.get("sdp")
                    await self.set_answer(answer_sdp)
                else:
                    self.get_logger().info("Failed to get answer from server")


    def validate_robot_conn(self, message):
        if message.get("data") == "Validation Ok.":
            self.robot_validation = "SUCCESS"
        else:
            self.publish(
                "",
                self.encrypt_key(message.get("data")),
                "validation",
            )

    def publish(self, topic, data, msg_type):
        if self.data_channel.readyState != "open":
            self.get_logger().info(
                f"Data channel is not open. State is {self.data_channel.readyState}",
            )
            return
        
        payload = {
            "type": msg_type,
            "topic": topic,
            "data": data,
        }

        payload_dumped = json.dumps(payload)
        self.get_logger().info(f"-> Sending message {payload_dumped}")
        self.data_channel.send(payload_dumped)

    async def run(self):
        await self.connect()
        self.get_logger().info("Connected to Go2 !!!")

        while True:
            self.get_logger().info("111")
            # if self.joy_state.buttons:
            #     self.get_logger().info(self.joy_state.buttons[0])

            # if self.joy_state.buttons and self.joy_state.buttons[6]:
            #     self.get_logger().info("Stand down")
            #     stand_down_cmd = gen_command(ROBOT_CMD['StandDown'])
            #     self.data_channel.send(stand_down_cmd['topic'], stand_down_cmd['data'], stand_down_cmd['type'])

            # if self.joy_state.buttons and self.joy_state.buttons[0]:
            #     self.get_logger().info("Stand up")
            #     stand_up_cmd = gen_command(ROBOT_CMD['StandUp'])
            #     self.data_channel.send(stand_up_cmd['topic'], stand_up_cmd['data'], stand_up_cmd['type'])
            #     balance_stand_cmd = gen_command(ROBOT_CMD['BalanceStand'])
            #     self.data_channel.send(balance_stand_cmd['topic'], balance_stand_cmd['data'], balance_stand_cmd['type'])

            await asyncio.sleep(0.1)

    @staticmethod
    def hex_to_base64(hex_str):
        bytes_array = bytes.fromhex(hex_str)
        return base64.b64encode(bytes_array).decode("utf-8")

    @staticmethod
    def encrypt_key(key):
        prefixed_key = f"UnitreeGo2_{key}"
        encrypted = Go2ConnectionAsync.encrypt_by_md5(prefixed_key)
        return Go2ConnectionAsync.hex_to_base64(encrypted)

    @staticmethod
    def encrypt_by_md5(input_str):
        hash_obj = hashlib.md5()
        hash_obj.update(input_str.encode("utf-8"))
        return hash_obj.hexdigest()



def main():
    rclpy.init()

    robot_node = Go2ConnectionAsync(
        ROBOT_IP,
        ROBOT_TOKEN,
    )
    loop = asyncio.get_event_loop()
    loop.run_until_complete(robot_node.run())

    

if __name__ == '__main__':
    main()
    