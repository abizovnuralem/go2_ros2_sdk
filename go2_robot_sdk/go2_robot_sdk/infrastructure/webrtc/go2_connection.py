# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Full Go2 WebRTC connection implementation with clean architecture.
Handles WebRTC peer connection and data channel communication with Go2 robot.
Originally forked from https://github.com/tfoldi/go2-webrtc and 
https://github.com/legion1581/go2_webrtc_connect
Big thanks to @tfoldi (Földi Tamás) and @legion1581 (The RoboVerse Discord Group)
"""

import asyncio
import json
import logging
import base64
from typing import Callable, Optional, Any, Dict, Union
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack

from .crypto.encryption import CryptoUtils, ValidationCrypto, PathCalculator, EncryptionError
from .http_client import HttpClient, WebRTCHttpError
from .data_decoder import WebRTCDataDecoder, DataDecodingError
from cryptography.hazmat.primitives.ciphers.aead import AESGCM

logger = logging.getLogger(__name__)


class Go2ConnectionError(Exception):
    """Custom exception for Go2 connection errors"""
    pass


class Go2Connection:
    """Full WebRTC connection to Go2 robot with encryption and proper signaling"""
    
    def __init__(
        self,
        robot_ip: str,
        robot_num: int,
        token: str = "",
        on_validated: Optional[Callable] = None,
        on_message: Optional[Callable] = None,
        on_open: Optional[Callable] = None,
        on_video_frame: Optional[Callable] = None,
        decode_lidar: bool = True,
    ):
        self.pc = RTCPeerConnection()
        self.robot_ip = robot_ip
        self.robot_num = str(robot_num)
        self.token = token
        self.robot_validation = "PENDING"
        self.validation_result = "PENDING"
        
        # Callbacks
        self.on_validated = on_validated
        self.on_message = on_message
        self.on_open = on_open
        self.on_video_frame = on_video_frame
        self.decode_lidar = decode_lidar
        
        # Initialize components
        self.http_client = HttpClient(timeout=10.0)
        self.data_decoder = WebRTCDataDecoder(enable_lidar_decoding=decode_lidar)
        
        # Setup data channel
        self.data_channel = self.pc.createDataChannel("data", id=0)
        self.data_channel.on("open", self.on_data_channel_open)
        self.data_channel.on("message", self.on_data_channel_message)
        
        # Setup peer connection events
        self.pc.on("track", self.on_track)
        self.pc.on("connectionstatechange", self.on_connection_state_change)
        
        # Add video transceiver if video callback provided
        if self.on_video_frame:
            self.pc.addTransceiver("video", direction="recvonly")
    
    def on_connection_state_change(self) -> None:
        """Handle peer connection state changes"""
        logger.info(f"Connection state is {self.pc.connectionState}")
        
        # Note: Validation is handled after successful WebRTC connection
        # in the original implementation, not here
    
    def on_data_channel_open(self) -> None:
        """Handle data channel open event"""
        logger.info("Data channel is open")
        
        # Force data channel to open state if needed (workaround)
        if self.data_channel.readyState != "open":
            self.data_channel._setReadyState("open")
        
        if self.on_open:
            self.on_open()
    
    def on_data_channel_message(self, message: Union[str, bytes]) -> None:
        """Handle incoming data channel messages"""
        try:
            logger.debug(f"Received message: {message}")
            
            # Ensure data channel is marked as open
            if self.data_channel.readyState != "open":
                self.data_channel._setReadyState("open")
            
            msgobj = None
            
            if isinstance(message, str):
                # Text message - likely JSON
                try:
                    msgobj = json.loads(message)
                    if msgobj.get("type") == "validation":
                        self.validate_robot_conn(msgobj)
                except json.JSONDecodeError:
                    logger.warning("Failed to decode JSON message")
                    
            elif isinstance(message, bytes):
                # Binary message - likely compressed data
                msgobj = legacy_deal_array_buffer(message, perform_decode=self.decode_lidar)
            
            # Forward message to callback
            if self.on_message:
                self.on_message(message, msgobj, self.robot_num)
                
        except Exception as e:
            logger.error(f"Error processing data channel message: {e}")
    
    async def on_track(self, track: MediaStreamTrack) -> None:
        """Handle incoming media tracks (video)"""
        logger.info("Receiving video")
        
        if track.kind == "video" and self.on_video_frame:
            try:
                await self.on_video_frame(track, self.robot_num)
            except Exception as e:
                logger.error(f"Error in video frame callback: {e}")
    
    def validate_robot_conn(self, message: Dict[str, Any]) -> None:
        """Handle robot validation response"""
        try:
            if message.get("data") == "Validation Ok.":
                # Turn on video
                self.publish("", "on", "vid")
                
                self.validation_result = "SUCCESS"
                self.robot_validation = "OK"
                
                if self.on_validated:
                    self.on_validated(self.robot_num)
                    
                logger.info("Robot validation successful")
            else:
                # Send encrypted validation response
                validation_key = message.get("data", "")
                encrypted_key = ValidationCrypto.encrypt_key(validation_key)
                self.publish("", encrypted_key, "validation")
                
        except Exception as e:
            logger.error(f"Error in robot validation: {e}")
    
    def publish(self, topic: str, data: Any, msg_type: str = "msg") -> None:
        """
        Publish message to data channel.
        
        Args:
            topic: Message topic
            data: Message data  
            msg_type: Message type
        """
        try:
            if self.data_channel.readyState != "open":
                logger.warning(f"Data channel is not open. State is {self.data_channel.readyState}")
                return
            
            payload = {
                "type": msg_type,
                "topic": topic,
                "data": data
            }
            
            payload_str = json.dumps(payload)
            logger.info(f"-> Sending message {payload_str}")
            self.data_channel.send(payload_str)
            
        except Exception as e:
            logger.error(f"Failed to publish message: {e}")
    
    async def disableTrafficSaving(self, switch: bool) -> bool:
        """
        Disable traffic saving mode for better data transmission.
        Should be turned on when subscribed to ulidar topic.
        
        Args:
            switch: True to disable traffic saving, False to enable
            
        Returns:
            True if successful
        """
        try:
            data = {
                "req_type": "disable_traffic_saving",
                "instruction": "on" if switch else "off"
            }
            
            self.publish("", data, "rtc_inner_req")
            logger.info(f"DisableTrafficSaving: {data['instruction']}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to set traffic saving: {e}")
            return False

    #decrypt RSA key from firmware version >=1.1.8
    def decrypt_con_notify_data(self, encrypted_b64: str) -> str:
        key = bytes([232, 86, 130, 189, 22, 84, 155, 0, 142, 4, 166, 104, 43, 179, 235, 227])
        data = base64.b64decode(encrypted_b64)
        if len(data) < 28:
            raise ValueError("Decryption failed: input data too short")
        tag = data[-16:]
        nonce = data[-28:-16]
        ciphertext = data[:-28]
        
        aesgcm = AESGCM(key) 
        plaintext = aesgcm.decrypt(nonce, ciphertext + tag, None)
        return plaintext.decode('utf-8')
	 

    async def connect(self) -> None:
        """Establish WebRTC connection to robot with full encryption"""
        try:
            logger.info("Trying to send SDP using full encrypted method...")
            
            # Step 1: Create WebRTC offer
            offer = await self.pc.createOffer()
            await self.pc.setLocalDescription(offer)
            
            sdp_offer = self.pc.localDescription
            sdp_offer_json = {
                "id": "STA_localNetwork",
                "sdp": sdp_offer.sdp,
                "type": sdp_offer.type,
                "token": self.token
            }
            
            new_sdp = json.dumps(sdp_offer_json)
            
            # Step 2: Get robot's public key
            try:
                response = self.http_client.get_robot_public_key(self.robot_ip)
                if not response:
                    raise Go2ConnectionError("Failed to get public key response")
                
                # Decode the response text from base64
                decoded_response = base64.b64decode(response.text).decode('utf-8')
                decoded_json = json.loads(decoded_response)
                
                # Extract the 'data1' and 'data2' fields from the JSON
                data1 = decoded_json.get('data1')
                data2 = decoded_json.get('data2')
                if not data1:
                    raise Go2ConnectionError("No data1 field in public key response")

                if data2 == 2:
                    data1 = self.decrypt_con_notify_data(data1)
                # Extract the public key from 'data1'
                public_key_pem = data1[10:len(data1)-10]
                path_ending = PathCalculator.calc_local_path_ending(data1)
                
                logger.info(f"Extracted path ending: {path_ending}")
                
            except (WebRTCHttpError, EncryptionError) as e:
                raise Go2ConnectionError(f"Failed to get robot public key: {e}")
            
            # Step 3: Encrypt and send SDP
            try:
                # Generate AES key
                aes_key = CryptoUtils.generate_aes_key()
                
                # Load Public Key
                public_key = CryptoUtils.rsa_load_public_key(public_key_pem)
                
                # Encrypt the SDP and AES key
                encrypted_body = {
                    "data1": CryptoUtils.aes_encrypt(new_sdp, aes_key),
                    "data2": CryptoUtils.rsa_encrypt(aes_key, public_key),
                }
                
                # Send the encrypted data
                response = self.http_client.send_encrypted_sdp(
                    self.robot_ip, path_ending, encrypted_body
                )
                
                if not response:
                    raise Go2ConnectionError("Failed to send encrypted SDP")
                
                # Decrypt the response
                decrypted_response = CryptoUtils.aes_decrypt(response.text, aes_key)
                peer_answer = json.loads(decrypted_response)
                
                # Set remote description
                answer = RTCSessionDescription(
                    sdp=peer_answer['sdp'], 
                    type=peer_answer['type']
                )
                await self.pc.setRemoteDescription(answer)
                
                logger.info(f"Successfully established WebRTC connection to robot {self.robot_num}")
                
            except (WebRTCHttpError, EncryptionError) as e:
                raise Go2ConnectionError(f"Failed to complete encrypted handshake: {e}")
            
        except Go2ConnectionError:
            raise
        except Exception as e:
            raise Go2ConnectionError(f"Unexpected error during connection: {e}")
    
    async def disconnect(self) -> None:
        """Close WebRTC connection and cleanup resources"""
        try:
            # Close peer connection
            await self.pc.close()
            
            # Close HTTP client
            self.http_client.close()
            
            logger.info(f"Disconnected from robot {self.robot_num}")
            
        except Exception as e:
            logger.error(f"Error disconnecting: {e}")
    
    def __del__(self):
        """Cleanup on object destruction"""
        try:
            if hasattr(self, 'http_client'):
                self.http_client.close()
        except:
            pass


# Static methods for backward compatibility
Go2Connection.hex_to_base64 = ValidationCrypto.hex_to_base64
Go2Connection.encrypt_key = ValidationCrypto.encrypt_key
Go2Connection.encrypt_by_md5 = ValidationCrypto.encrypt_by_md5

# Use the legacy deal_array_buffer function for full compatibility
from .data_decoder import deal_array_buffer as legacy_deal_array_buffer
Go2Connection.deal_array_buffer = staticmethod(legacy_deal_array_buffer) 
