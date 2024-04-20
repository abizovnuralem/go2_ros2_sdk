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


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String
import os


class Go2ProcText(Node):

    def __init__(self):
        super().__init__('go2_proc_text')

        text_file_name = '/tmp/robot_cmd'

        # remove all contents of the file
        with open(text_file_name, 'w') as file:
            pass

        # touch the file
        os.utime(text_file_name, None)

        self.tmp_file = open(text_file_name, 'r')

        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(String, 'command', qos_profile)

        self.text_read_timer = self.create_timer(0.2, self.update)

    def update(self):
        line = self.tmp_file.readline()

        # TODO: replace this with chatGPT to process natural language
        # and translate it into a queue of robot actions
        if "sit" in line.lower():
            msg = String()
            msg.data = "Sit"
            self.publisher_.publish(msg)

        if "stand" in line.lower():
            msg = String()
            msg.data = "RecoveryStand"
            self.publisher_.publish(msg)
            msg.data = "StandUp"
            self.publisher_.publish(msg)
            msg.data = "BalanceStand"
            self.publisher_.publish(msg)

        if "shake hand" in line.lower():
            msg = String()
            msg.data = "Hello"
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    go2_proc_text = Go2ProcText()

    rclpy.spin(go2_proc_text)

    go2_proc_text.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()