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

from go2_robot_sdk.webrtc_driver import RTC_TOPIC, gen_mov_command
import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.node import Node


class Go2(Node):
    def __init__(self):

        super().__init__('go2_main_node')
        self.robot_cmd_vel = None
        self.joy_state = Joy()
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_cb,
            10)
        
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_cb,
            10)
        
        timer_period = 0.5  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.rtc_topic_subs = RTC_TOPIC.values()

    def timer_callback(self):
        self.get_logger().info('Publishing')

    def cmd_vel_cb(self, msg):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z
        self.robot_cmd_vel = gen_mov_command(x, y, z)

    def joy_cb(self, msg):
        self.joy_state = msg



def main():
    rclpy.init()
    robot_node = Go2()
    rclpy.spin(robot_node)


if __name__ == '__main__':
    main()
    