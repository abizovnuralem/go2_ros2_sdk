#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright (c) 2024, The RoboVerse community
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
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

import base64
from datetime import datetime
import io
import json
import os
import time
import hashlib

from pydub import AudioSegment
from pydub.playback import play
import rclpy
from rclpy.node import Node
import requests
from std_msgs.msg import String
from scripts.go2_constants import RTC_TOPIC
from unitree_go.msg import WebRtcReq

# flake8: noqa: Q000


class TTSNode(Node):

    def __init__(self):
        super().__init__("tts_node")

        # Initialize parameters
        self.declare_parameter("elevenlabs_api_key", "")
        self.declare_parameter("local_playback", False)  # Default to robot playback
        self.declare_parameter("voice_name", "XrExE9yKIg1WjnnlVkGX")
        self.declare_parameter("use_cache", True)  # Enable caching by default

        self.api_key = self.get_parameter("elevenlabs_api_key").value
        self.local_playback = self.get_parameter("local_playback").value
        self.voice_name = self.get_parameter("voice_name").value
        self.use_cache = self.get_parameter("use_cache").value

        if not self.api_key:
            self.get_logger().error("ElevenLabs API key not provided!")
            return

        # Create subscription for TTS requests
        self.subscription = self.create_subscription(
            String, "/tts", self.tts_callback, 10
        )

        # Create publisher for robot audio hub requests
        self.audio_pub = self.create_publisher(WebRtcReq, "/webrtc_req", 10)

        # Create cache directory
        self.cache_dir = "tts_cache"
        os.makedirs(self.cache_dir, exist_ok=True)

        self.get_logger().info(
            f'TTS Node initialized ({"local" if self.local_playback else "robot"} playback, cache {"enabled" if self.use_cache else "disabled"})'
        )

    def tts_callback(self, msg):
        """Handle incoming TTS requests."""
        try:
            self.get_logger().info(
                f'Received TTS request: "{msg.data}" with voice: {self.voice_name}'
            )

            # Check cache first if enabled
            audio_data = None
            cache_hit = False
            cache_path = None

            if self.use_cache:
                cache_path = self._get_cache_path(msg.data, self.voice_name)
                if os.path.exists(cache_path):
                    self.get_logger().info(f"Cache hit: {cache_path}")
                    with open(cache_path, "rb") as f:
                        audio_data = f.read()
                    cache_hit = True

            # If not in cache, generate speech
            if not cache_hit:
                audio_data = self.generate_speech(msg.data, self.voice_name)

                # Save to cache if enabled
                if audio_data and self.use_cache and cache_path:
                    with open(cache_path, "wb") as f:
                        f.write(audio_data)
                    self.get_logger().info(f"Saved to cache: {cache_path}")

            if audio_data:
                if self.local_playback:
                    # Play locally
                    self.play_audio(audio_data)
                else:
                    # Convert MP3 to WAV for robot playback (in memory)
                    wav_data = self.convert_mp3_to_wav(audio_data)
                    # Send to robot
                    self.play_on_robot(wav_data)

                self.get_logger().info(
                    f"Successfully processed TTS request. {'Using cached audio' if cache_hit else 'Generated new audio'}"
                )
            else:
                self.get_logger().error("Failed to generate speech")

        except Exception as e:
            self.get_logger().error(f"Error processing TTS request: {str(e)}")

    def _get_cache_path(self, text, voice_name):
        """Generate a cache file path based on text and voice name."""
        # Create a hash of the text and voice to use as the filename
        text_hash = hashlib.md5(f"{text}_{voice_name}".encode()).hexdigest()
        return os.path.join(self.cache_dir, f"{text_hash}.mp3")

    def generate_speech(self, text, voice_name):
        """Generate speech using ElevenLabs API."""
        url = f"https://api.elevenlabs.io/v1/text-to-speech/{voice_name}"

        headers = {
            "Accept": "audio/mpeg",
            "Content-Type": "application/json",
            "xi-api-key": self.api_key,
        }

        data = {
            "text": text,
            "model_id": "eleven_turbo_v2_5",
            "voice_settings": {"stability": 0.5, "similarity_boost": 0.5},
        }

        try:
            self.get_logger().info(f"Calling ElevenLabs API for voice: {voice_name}")
            response = requests.post(url, json=data, headers=headers)
            response.raise_for_status()
            return response.content

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"API request failed: {str(e)}")
            return None

    def convert_mp3_to_wav(self, audio_data):
        """Convert MP3 audio data to WAV format in memory."""
        try:
            # Convert MP3 to WAV in memory
            audio = AudioSegment.from_mp3(io.BytesIO(audio_data))
            wav_io = io.BytesIO()
            audio.export(wav_io, format="wav")
            return wav_io.getvalue()

        except Exception as e:
            self.get_logger().error(f"Error converting MP3 to WAV: {str(e)}")
            return None

    def play_audio(self, audio_data):
        """Play audio locally using pydub."""
        try:
            audio = AudioSegment.from_mp3(io.BytesIO(audio_data))
            play(audio)
        except Exception as e:
            self.get_logger().error(f"Error playing audio: {str(e)}")

    def split_into_chunks(self, data, chunk_size=16 * 1024):
        """Split data into chunks of specified size."""
        return [
            data[i: i + chunk_size] for i in range(0, len(data), chunk_size)  # noqa: E203
        ]

    def play_on_robot(self, wav_data):
        """Send audio to robot's audio hub in chunks."""
        try:
            b64_encoded = base64.b64encode(wav_data).decode("utf-8")
            chunks = self.split_into_chunks(b64_encoded)
            total_chunks = len(chunks)

            self.get_logger().info(f"Sending audio in {total_chunks} chunks")

            # Start audio
            start_req = WebRtcReq()
            start_req.api_id = 4001
            start_req.priority = 0
            start_req.parameter = ""
            start_req.topic = RTC_TOPIC["AUDIO_HUB_REQ"]

            self.audio_pub.publish(start_req)

            time.sleep(0.1)

            # Send WAV data in chunks
            for chunk_idx, chunk in enumerate(chunks, 1):
                wav_req = WebRtcReq()
                wav_req.api_id = 4003
                wav_req.priority = 0
                wav_req.topic = RTC_TOPIC["AUDIO_HUB_REQ"]

                audio_block = {
                    "current_block_index": chunk_idx,
                    "total_block_number": total_chunks,
                    "block_content": chunk,
                }
                wav_req.parameter = json.dumps(audio_block)

                self.audio_pub.publish(wav_req)
                self.get_logger().info(
                    f"Sent chunk {chunk_idx}/{total_chunks} ({len(chunk)} bytes)"
                )

                # Add a small delay between chunks to prevent flooding
                # time.sleep(0.01)
                time.sleep(0.15)

            # Wait until playback finished
            audio = AudioSegment.from_wav(io.BytesIO(wav_data))
            duration_ms = len(audio)
            duration_s = duration_ms / 1000.0

            self.get_logger().info(
                f"Waiting for audio playback ({duration_s:.2f} seconds)..."
            )
            time.sleep(duration_s + 1)

            # End audio
            end_req = WebRtcReq()
            end_req.api_id = 4002
            end_req.priority = 0
            end_req.parameter = ""
            end_req.topic = RTC_TOPIC["AUDIO_HUB_REQ"]

            self.audio_pub.publish(end_req)

            self.get_logger().info("Completed sending audio to robot")

        except Exception as e:
            self.get_logger().error(f"Error sending audio to robot: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
