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

import aiohttp
import base64
import hashlib
import json
import logging
import struct

from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaBlackhole

from scripts.go2_lidar_decoder import LidarDecoder


logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


decoder = LidarDecoder()


class Go2Connection():
    def __init__(
            self, 
            robot_ip=None,
            token="",
            on_validated=None, 
            on_message=None, 
            on_open=None
            ):
        
        

        self.pc = RTCPeerConnection()
        self.robot_ip = robot_ip
        self.token = token
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
                msgobj = Go2Connection.deal_array_buffer(msg)

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
                "token": self.token,
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
        encrypted = Go2Connection.encrypt_by_md5(prefixed_key)
        return Go2Connection.hex_to_base64(encrypted)

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
            decoded_data = decoder.decode(compressed_data, obj['data'])
            obj["decoded_data"] = decoded_data
            return obj
        return None