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

import logging
import os
import time
import cv2
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile
import subprocess
import threading


logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def build_camera_rust_stream():
    webrtc_rc_exists = os.path.isdir(
        get_package_share_directory('go2_robot_sdk') + "/external_lib/target")
    
    if not webrtc_rc_exists:
        go2webrtc_rc_path = os.path.join(
            get_package_share_directory('go2_robot_sdk'),
            "external_lib",
            'camera_webrtc_rc')
        logger.info(f"Building go2webrtc-rc at {go2webrtc_rc_path}")
        cmd = f"cd {go2webrtc_rc_path}; cargo build --release"
        subprocess.call(cmd, shell=True)
    else:
        logging.info("go2webrtc-rc already built..")


class Go2CameraStream(Node):
    def __init__(self):
        super().__init__('go2_camera_stream')
        build_camera_rust_stream()
        go2webrtc_rc_path = os.path.join(
            get_package_share_directory('go2_robot_sdk'),
            "external_lib/camera_webrtc_rc/target/release",
            'go2webrtc-rc')
                
        time.sleep(2)
        # Start the external RUST webrtc-rc command inside a thread
        self.external_command_thread = threading.Thread(
            target=self.run_external_command,
            args=(go2webrtc_rc_path, os.environ.get('ROBOT_IP'))
        )
        self.external_command_thread.start()

        self.get_logger().info(f"Connected to go2 camera: {os.environ.get('ROBOT_IP')}")
        
        qos_profile = QoSProfile(depth=10)
        self.image_pub = self.create_publisher(Image, '/go2_camera/color/image', qos_profile)
        self.cv_bridge = CvBridge()

        gst_pipeline = (
            "udpsrc port=4002 "
            "caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! "
            "appsink"
        )

        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video stream.')
            return
        # Capture images at a rate of 15fps
        timer_period = 1/15  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to get a frame.')
            return

        self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))

    def run_external_command(self, go2webrtc_rc_path, robot_ip):
        subprocess.run([go2webrtc_rc_path, "--robot", robot_ip])

def main():
    rclpy.init()
    robot_node = Go2CameraStream()
    try:
        rclpy.spin(robot_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_node.destroy_node()
        robot_node.external_command_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    