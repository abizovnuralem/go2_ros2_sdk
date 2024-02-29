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
import logging
import os
import random
import struct
import threading
import asyncio
import math
import ctypes
import time
import numpy as np

from wasmtime import Config, Engine, Store, Module, Instance, Func, FuncType
from wasmtime import ValType
from ament_index_python import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import aiohttp
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription
)
from aiortc.contrib.media import MediaBlackhole

from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Twist, TransformStamped
from go2_interfaces.msg import Go2State, IMU
from sensor_msgs.msg import PointCloud2, PointField, JointState, Joy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


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


def update_meshes_for_cloud2(positions, uvs, res, origin):
    # Convert the list of positions to a NumPy array for vectorized operations
    position_array = np.array(positions).reshape(-1, 3).astype(np.float32)

    # Do some resolution for each point
    position_array *= res

    # Recalculate origin
    position_array += origin

    # Convert the list of uvs to a NumPy array
    uv_array = np.array(uvs, dtype=np.float32).reshape(-1, 2)

    # Calculate intensities for unique positions based on their UV values
    intensities = np.min(uv_array, axis=1, keepdims=True)

    # Merge uvs with points
    positions_with_uvs = np.hstack((position_array, intensities))

    # Remove duplicated points by creating a set of tuples
    positions_with_uvs = np.unique(positions_with_uvs, axis=0)
    return positions_with_uvs


class LidarDecoder:
    def __init__(self) -> None:

        config = Config()
        config.wasm_multi_value = True
        config.debug_info = True
        self.store = Store(Engine(config))

        libvoxel_path = os.path.join(
            get_package_share_directory('go2_robot_sdk'),
            "external_lib",
            'libvoxel.wasm')
        
        self.module = Module.from_file(self.store.engine, libvoxel_path)

        self.a_callback_type = FuncType([ValType.i32()], [ValType.i32()])
        self.b_callback_type = FuncType([ValType.i32(), ValType.i32(), ValType.i32()], [])

        a = Func(self.store, self.a_callback_type, self.adjust_memory_size)
        b = Func(self.store, self.b_callback_type, self.copy_memory_region)

        self.instance = Instance(self.store, self.module, [a, b])

        self.generate = self.instance.exports(self.store)["e"]
        self.malloc = self.instance.exports(self.store)["f"]
        self.free = self.instance.exports(self.store)["g"]
        self.wasm_memory = self.instance.exports(self.store)["c"]

        self.buffer = self.wasm_memory.data_ptr(self.store)
        self.memory_size = self.wasm_memory.data_len(self.store)

        self.buffer_ptr = int.from_bytes(self.buffer, "little")

        self.HEAP8 = (ctypes.c_int8 * self.memory_size).from_address(self.buffer_ptr)
        self.HEAP16 = (ctypes.c_int16 * (self.memory_size // 2)).from_address(self.buffer_ptr)
        self.HEAP32 = (ctypes.c_int32 * (self.memory_size // 4)).from_address(self.buffer_ptr)
        self.HEAPU8 = (ctypes.c_uint8 * self.memory_size).from_address(self.buffer_ptr)
        self.HEAPU16 = (ctypes.c_uint16 * (self.memory_size // 2)).from_address(self.buffer_ptr)
        self.HEAPU32 = (ctypes.c_uint32 * (self.memory_size // 4)).from_address(self.buffer_ptr)
        self.HEAPF32 = (ctypes.c_float * (self.memory_size // 4)).from_address(self.buffer_ptr)
        self.HEAPF64 = (ctypes.c_double * (self.memory_size // 8)).from_address(self.buffer_ptr)

        self.input = self.malloc(self.store, 61440)
        self.decompressBuffer = self.malloc(self.store, 80000)
        self.positions = self.malloc(self.store, 2880000)
        self.uvs = self.malloc(self.store, 1920000)
        self.indices = self.malloc(self.store, 5760000)
        self.decompressedSize = self.malloc(self.store, 4)
        self.faceCount = self.malloc(self.store, 4)
        self.pointCount = self.malloc(self.store, 4)
        self.decompressBufferSize = 80000

    def adjust_memory_size(self, t):
        return len(self.HEAPU8)

    def copy_within(self, target, start, end):
        # Copy the sublist for the specified range [start:end]
        sublist = self.HEAPU8[start:end]

        # Replace elements in the list starting from index 'target'
        for i in range(len(sublist)):
            if target + i < len(self.HEAPU8):
                self.HEAPU8[target + i] = sublist[i]
    
    def copy_memory_region(self, t, n, a):
        self.copy_within(t, n, n + a)

    def get_value(self, t, n="i8"):
        if n.endswith("*"):
            n = "*"
        if n == "i1" or n == "i8":
            return self.HEAP8[t]
        elif n == "i16":
            return self.HEAP16[t >> 1]
        elif n == "i32" or n == "i64":
            return self.HEAP32[t >> 2]
        elif n == "float":
            return self.HEAPF32[t >> 2]
        elif n == "double":
            return self.HEAPF64[t >> 3]
        elif n == "*":
            return self.HEAPU32[t >> 2]
        else:
            raise ValueError(f"invalid type for getValue: {n}")
        
    def add_value_arr(self, start, value):
        if start + len(value) <= len(self.HEAPU8):
            for i, byte in enumerate(value):
                self.HEAPU8[start + i] = byte
        else:
            raise ValueError("Not enough space to insert bytes at the specified index.")

    def decode(self, compressed_data, data):
        self.add_value_arr(self.input, compressed_data)

        some_v = math.floor(data["origin"][2] / data["resolution"])

        self.generate(
            self.store,
            self.input,
            len(compressed_data),
            self.decompressBufferSize,
            self.decompressBuffer,
            self.decompressedSize, 
            self.positions,
            self.uvs,
            self.indices,          
            self.faceCount,
            self.pointCount,        
            some_v
        )

        self.get_value(self.decompressedSize, "i32")
        c = self.get_value(self.pointCount, "i32")
        u = self.get_value(self.faceCount, "i32")

        positions_slice = self.HEAPU8[self.positions:self.positions + u * 12]
        positions_copy = bytearray(positions_slice)
        p = np.frombuffer(positions_copy, dtype=np.uint8)

        uvs_slice = self.HEAPU8[self.uvs:self.uvs + u * 8]
        uvs_copy = bytearray(uvs_slice)
        r = np.frombuffer(uvs_copy, dtype=np.uint8)

        indices_slice = self.HEAPU8[self.indices:self.indices + u * 24]
        indices_copy = bytearray(indices_slice)
        o = np.frombuffer(indices_copy, dtype=np.uint32)

        return {
            "point_count": c,
            "face_count": u,
            "positions": p,
            "uvs": r,
            "indices": o
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


class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def set_from_axis_angle(self, n, o):
        s = o / 2
        c = math.sin(s)
        self.x = n.x * c
        self.y = n.y * c
        self.z = n.z * c
        self.w = math.cos(s)
        return self.x, self.y, self.z, self.w
    
    def invert(self):
        self.x *= -1
        self.y *= -1
        self.z *= -1
        return self.x, self.y, self.z, self.w
      
class Vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def add(self, n):
        self.x += n.x
        self.y += n.y
        self.z += n.z
        return self.x, self.y, self.z
    
    def clone(self):
        new_vector = Vector3(self.x, self.y, self.z)
        return new_vector
      
    def apply_quaternion(self, quaternion):
        o = self.x
        s = self.y
        c = self.z
        u = quaternion.x
        l = quaternion.y
        f = quaternion.z
        _ = quaternion.w
        g = _ * o + l * c - f * s
        v = _ * s + f * o - u * c
        T = _ * c + u * s - l * o
        E = -u * o - l * s - f * c

        self.x = g * _ + E * -u + v * -f - T * -l
        self.y = v * _ + E * -l + T * -u - g * -f
        self.z = T * _ + E * -f + g * -l - v * -u
        return self.x, self.y, self.z
    
    def negate(self): 
        self.x = -self.x
        self.y = -self.y
        self.z = -self.z
        return self.x, self.y, self.z
    
    def distance_to(self, n):
        return math.sqrt(self.distance_to_squared(n))
      
    def distance_to_squared(self, n):
        o = self.x - n.x
        s = self.y - n.y
        c = self.z - n.z
        return o * o + s * s + c * c
      
    def apply_axis_angle(self, n, o):
        quaternion = Quaternion(0, 0, 0, 1)
        return self.apply_quaternion(quaternion.set_from_axis_angle(n, o))


def get_joints_go2(footPositionValue, foot_num):
    # URDF GO2 real values
    hip_length = 0.0955
    thigh_length = 0.213
    calf_length = 0.2135

    foot_position = Vector3(footPositionValue[0], footPositionValue[1], footPositionValue[2])

    base_tf_offset_hip_joint = Vector3(0.1934, 0.0465, 0)
    if foot_num > 1:
        base_tf_offset_hip_joint.x = -base_tf_offset_hip_joint.x
    if foot_num % 2 == 1:
        base_tf_offset_hip_joint.y = -base_tf_offset_hip_joint.y
    
    foot_position_distance = foot_position.distance_to(base_tf_offset_hip_joint)

    # cos theory
    E = np.sqrt(foot_position_distance ** 2 - hip_length ** 2)
    y = np.arccos((E ** 2 + thigh_length ** 2 - calf_length ** 2) / (2 * E * thigh_length))
    S = np.arccos((calf_length ** 2 + thigh_length ** 2 - E ** 2) / (2 * calf_length * thigh_length)) - np.pi
    C = foot_position.x - base_tf_offset_hip_joint.x
    R = foot_position.y - base_tf_offset_hip_joint.y

    if foot_position.z < 0:
        A = np.arcsin(-C / E) + y
    else:
        A = -np.pi + np.arcsin(C / E) + y

    O = np.sqrt(foot_position_distance ** 2 - C ** 2)
    L = np.arcsin(R / O)

    if foot_position.z > 0:
        P = -1
    else:
        P = 1

    if foot_num % 2 == 0:
        J = P * (L - np.arcsin(hip_length / O))

    else:
        J = P * (L + np.arcsin(hip_length / O)) 

    if math.isnan(J + A + S):
        logger.info("something is wrong with kinematics")
        return 0, 0, 0

    return J, A, S


class Go2ConnectionAsync():
    def __init__(
            self, 
            robot_ip=None, 
            on_validated=None, 
            on_message=None, 
            on_open=None
            ):

        self.pc = RTCPeerConnection()
        self.robot_ip = robot_ip
        self.robot_validation = "PENDING"
        self.on_validated = on_validated
        self.on_message = on_message
        self.on_open = on_open
        
        self.audio_track = MediaBlackhole()
        self.video_track = MediaBlackhole()
        
        self.data_channel = self.pc.createDataChannel("data", id=0)
        self.data_channel.on("open", self.on_data_channel_open)
        self.data_channel.on("message", self.on_data_channel_message)
        
        self.pc.on("track", self.on_track)
        self.pc.on("connectionstatechange", self.on_connection_state_change)

    def on_connection_state_change(self):
        logger.info(f"Connection state is {self.pc.connectionState}")

    def on_track(self, track):
        logger.info(f"Receiving {track.kind}")
        if track.kind == "audio":
            pass
        elif track.kind == "video":
            pass
    
    async def generate_offer(self):
        await self.audio_track.start()
        await self.video_track.start()
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)
        return offer.sdp
    
    async def set_answer(self, sdp):
        answer = RTCSessionDescription(sdp, type="answer")
        await self.pc.setRemoteDescription(answer)
    
    def on_data_channel_open(self):
        logger.info("Data channel is open")
        if self.on_open:
            self.on_open()
    
    def on_data_channel_message(self, msg):

        logger.debug("Received message: %s", msg)

        if self.data_channel.readyState != "open":
            self.data_channel._setReadyState("open")

        try:
            if isinstance(msg, str):
                msgobj = json.loads(msg)
                if msgobj.get("type") == "validation":
                    self.validate_robot_conn(msgobj)
            elif isinstance(msg, bytes):
                msgobj = Go2ConnectionAsync.deal_array_buffer(msg)

            if self.on_message:
                self.on_message(msg, msgobj)

        except json.JSONDecodeError:
            pass

    async def connect(self):
        offer = await self.generate_offer()
        async with aiohttp.ClientSession() as session:
            url = f"http://{self.robot_ip}:8081/offer"
            headers = {"content-type": "application/json"}
            data = {
                "sdp": offer,
                "id": "STA_localNetwork",
                "type": "offer",
                "token": "",
            }
            async with session.post(url, json=data, headers=headers) as resp:
                if resp.status == 200:
                    answer_data = await resp.json()
                    answer_sdp = answer_data.get("sdp")
                    await self.set_answer(answer_sdp)
                else:
                    logger.info("Failed to get answer from server")

    def validate_robot_conn(self, message):
        if message.get("data") == "Validation Ok.":
            self.validation_result = "SUCCESS"
            if self.on_validated:
                self.on_validated()
        else:
            self.publish(
                "",
                self.encrypt_key(message.get("data")),
                "validation",
            )

    def publish(self, topic, data, msg_type):
        if self.data_channel.readyState != "open":
            logger.info(
                f"Data channel is not open. State is {self.data_channel.readyState}",
            )
            return
        
        payload = {
            "type": msg_type,
            "topic": topic,
            "data": data,
        }

        payload_dumped = json.dumps(payload)
        logger.info(f"-> Sending message {payload_dumped}")
        self.data_channel.send(payload_dumped)

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
    
    @staticmethod
    def deal_array_buffer(n):

        if isinstance(n, bytes):
            length = struct.unpack("H", n[:2])[0]
            json_segment = n[4 : 4 + length]
            compressed_data = n[4 + length :]
            json_str = json_segment.decode("utf-8")
            obj = json.loads(json_str)
            decoder = LidarDecoder()
            decoded_data = decoder.decode(compressed_data, obj['data'])
            obj["decoded_data"] = decoded_data
            return obj
        return None

class RobotBaseNode(Node):

    def __init__(self):
        super().__init__('go2_driver')
        
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
        self.robot_cmd_vel = gen_mov_command(x, y, z)

    def joy_cb(self, msg):
        self.joy_state = msg

    def joy_cmd(self):

        if self.robot_cmd_vel:
            self.get_logger().info("Attack!")
            self.conn.data_channel.send(self.robot_cmd_vel)
            self.robot_cmd_vel = None


        if self.joy_state.buttons and self.joy_state.buttons[1]:
            self.get_logger().info("Stand down")
            stand_down_cmd = gen_command(1005)
            self.conn.data_channel.send(stand_down_cmd)

        if self.joy_state.buttons and self.joy_state.buttons[0]:
            self.get_logger().info("Stand up")
            stand_up_cmd = gen_command(1004)
            self.conn.data_channel.send(stand_up_cmd)

    def on_validated(self):
        self.conn.data_channel.send('{"type": "subscribe", "topic": "rt/lf/lowstate"}')
        self.conn.data_channel.send('{"type": "subscribe", "topic": "rt/utlidar/robot_pose"}')
        self.conn.data_channel.send('{"type": "subscribe", "topic": "rt/lf/sportmodestate"}')
        self.conn.data_channel.send('{"type": "subscribe", "topic": "rt/utlidar/foot_position"}')
        self.conn.data_channel.send('{"type": "subscribe", "topic": "rt/utlidar/voxel_map_compressed"}')

    def on_data_channel_message(self, _, msg):
        if msg.get('topic') == "rt/utlidar/voxel_map_compressed":
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
            odom_trans.child_frame_id = 'base'
            odom_trans.transform.translation.x = self.robot_odom['data']['pose']['position']['x']
            odom_trans.transform.translation.y = self.robot_odom['data']['pose']['position']['y']
            odom_trans.transform.translation.z = self.robot_odom['data']['pose']['position']['z']
            odom_trans.transform.rotation.x = self.robot_odom['data']['pose']['orientation']['x']
            odom_trans.transform.rotation.y = self.robot_odom['data']['pose']['orientation']['y']
            odom_trans.transform.rotation.z = self.robot_odom['data']['pose']['orientation']['z']
            odom_trans.transform.rotation.w = self.robot_odom['data']['pose']['orientation']['w']
            self.broadcaster.sendTransform(odom_trans)

    def publish_slow_joint_state(self):
        if self.robot_low_cmd:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()

            FL_hip_joint = self.robot_low_cmd["data"]["motor_state"][0]["q"]
            FL_thigh_joint = self.robot_low_cmd["data"]["motor_state"][1]["q"]
            FL_calf_joint = self.robot_low_cmd["data"]["motor_state"][2]["q"]

            FR_hip_joint = self.robot_low_cmd["data"]["motor_state"][3]["q"]
            FR_thigh_joint = self.robot_low_cmd["data"]["motor_state"][4]["q"]
            FR_calf_joint = self.robot_low_cmd["data"]["motor_state"][5]["q"]

            RL_hip_joint = self.robot_low_cmd["data"]["motor_state"][6]["q"]
            RL_thigh_joint = self.robot_low_cmd["data"]["motor_state"][7]["q"]
            RL_calf_joint = self.robot_low_cmd["data"]["motor_state"][8]["q"]

            RR_hip_joint = self.robot_low_cmd["data"]["motor_state"][9]["q"]
            RR_thigh_joint = self.robot_low_cmd["data"]["motor_state"][10]["q"]
            RR_calf_joint = self.robot_low_cmd["data"]["motor_state"][11]["q"]
            
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

    def publish_lidar(self):
        if self.robot_lidar:
            points = update_meshes_for_cloud2(
                self.robot_lidar["decoded_data"]["positions"], 
                self.robot_lidar["decoded_data"]["uvs"], 
                self.robot_lidar['data']['resolution'], 
                self.robot_lidar['data']['origin']
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

            FL_hip_joint, FL_thigh_joint, FL_calf_joint = get_joints_go2(
                fl_foot_pos_array,
                0
                )
            
            fr_foot_pos_array = [
                self.robot_sport_state["data"]["foot_position_body"][0], 
                self.robot_sport_state["data"]["foot_position_body"][1], 
                self.robot_sport_state["data"]["foot_position_body"][2]
            ]

            FR_hip_joint, FR_thigh_joint, FR_calf_joint = get_joints_go2(
                fr_foot_pos_array,
                1
                )
            
            rl_foot_pos_array = [
                self.robot_sport_state["data"]["foot_position_body"][9], 
                self.robot_sport_state["data"]["foot_position_body"][10], 
                self.robot_sport_state["data"]["foot_position_body"][11]
            ]

            RL_hip_joint, RL_thigh_joint, RL_calf_joint = get_joints_go2(
                rl_foot_pos_array,
                2
                )
            
            rr_foot_pos_array = [
                self.robot_sport_state["data"]["foot_position_body"][6], 
                self.robot_sport_state["data"]["foot_position_body"][7], 
                self.robot_sport_state["data"]["foot_position_body"][8]
            ]

            RR_hip_joint, RR_thigh_joint, RR_calf_joint = get_joints_go2(
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
            go2_state.position = self.robot_sport_state["data"]["position"]
            go2_state.body_height = self.robot_sport_state["data"]["body_height"]
            go2_state.velocity = self.robot_sport_state["data"]["velocity"]
            go2_state.range_obstacle = list(map(float, self.robot_sport_state["data"]["range_obstacle"]))
            go2_state.foot_force = self.robot_sport_state["data"]["foot_force"]
            go2_state.foot_position_body = self.robot_sport_state["data"]["foot_position_body"]
            go2_state.foot_speed_body = self.robot_sport_state["data"]["foot_speed_body"]
            self.go2_state_pub.publish(go2_state) 

            imu = IMU()
            imu.quaternion = self.robot_sport_state["data"]["imu_state"]["quaternion"]
            imu.accelerometer = list(map(float,self.robot_sport_state["data"]["imu_state"]["accelerometer"]))
            imu.gyroscope = list(map(float,self.robot_sport_state["data"]["imu_state"]["gyroscope"]))
            imu.rpy = self.robot_sport_state["data"]["imu_state"]["rpy"]
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


async def execute_all():
    base_node = RobotBaseNode()
    conn = Go2ConnectionAsync(
        os.environ.get('ROBOT_IP'),
        on_validated=base_node.on_validated,
        on_message=base_node.on_data_channel_message,

    )
    spin_task = asyncio.get_event_loop().create_task(spin(base_node))
    sleep_task = asyncio.get_event_loop().create_task(base_node.run(conn))
    await asyncio.wait([spin_task, sleep_task], return_when=asyncio.FIRST_COMPLETED)

def main():
    rclpy.init()
    asyncio.get_event_loop().run_until_complete(execute_all())
    asyncio.get_event_loop().close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()