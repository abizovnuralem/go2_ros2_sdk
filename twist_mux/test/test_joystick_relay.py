#! /usr/bin/python3
# -*- coding: utf-8 -*-

# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

import launch_testing
import rclpy
import threading

from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Bool
from twist_mux_msgs.action import JoyPriority, JoyTurbo

MAX_VEL = 1.0
MIN_VEL = 0.5
STEPS = 4
INIT_STEP = 1.0
PRIORITY = True


def generate_test_description():

    joystick_relay = Node(
      package='twist_mux',
      executable='joystick_relay.py',
      output='screen',
      parameters=[{
          'priority': PRIORITY,
          'turbo.linear_forward_min': MIN_VEL,
          'turbo.linear_forward_max': MAX_VEL,
          'turbo.linear_lateral_min': MIN_VEL,
          'turbo.linear_lateral_max': MAX_VEL,
          'turbo.linear_backward_min': MIN_VEL,
          'turbo.linear_backward_max': MAX_VEL,
          'turbo.angular_min': MIN_VEL,
          'turbo.angular_max': MAX_VEL,
          'turbo.steps': STEPS,
          'turbo.init_step': INIT_STEP
      }])

    return LaunchDescription([
        joystick_relay,
        launch_testing.actions.ReadyToTest(),
    ]), {'joystick_relay': joystick_relay}


class TestJoystickRelay(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        rclpy.init()
        self.node = rclpy.create_node('test_joystick_relay')
        self.pub = self.node.create_publisher(Twist, 'joy_vel_in', rclpy.qos.QoSProfile(depth=10))

    @classmethod
    def tearDownClass(self):
        self.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def setUp(self):
        self.sub = self.node.create_subscription(
            Twist, 'joy_vel_out', self._sub_callback, QoSProfile(depth=1))
        self.timer = self.node.create_timer(0.1, self._pub_callback)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.exec_thread = threading.Thread(target=self.executor.spin)
        self.exec_thread.start()
        self._reset_and_wait_twist()

    @classmethod
    def tearDown(self):
        self.timer.destroy()
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.exec_thread.join()

    @classmethod
    def _pub_callback(self):
        self.pub.publish(self.twist_msg)

    @classmethod
    def _sub_callback(self, msg):
        self.current_twist = msg

    @classmethod
    def _get_current_vel(self, commanded_vel, current_step):
        vel = (MIN_VEL + (MAX_VEL - MIN_VEL) / (STEPS - 1) * (current_step - 1))
        if (vel < MIN_VEL):
            vel = MIN_VEL
        elif (vel > MAX_VEL):
            vel = MAX_VEL
        return vel * commanded_vel

    @classmethod
    def _reset_and_wait_twist(self):
        self.twist_msg = Twist()
        self.current_twist = Twist()

        # wait until all is reset
        while (self.current_twist.linear.x != 0.0 or
               self.current_twist.linear.y != 0.0 or
               self.current_twist.angular.z != 0.0):
            pass

    @classmethod
    def _set_and_wait_twist(self, linear_x, linear_y, angular_z):
        self.twist_msg.linear.x = linear_x
        self.twist_msg.linear.y = linear_y
        self.twist_msg.angular.z = angular_z

        # wait until all is set
        while (self.current_twist.linear.x == 0.0 or
               self.current_twist.linear.y == 0.0 or
               self.current_twist.angular.z == 0.0):
            pass

    def test_joy_priority_action(self):
        priority = None

        def _priority_cb(msg):
            nonlocal priority
            priority = msg

        def wait_and_check_priority(value):
            nonlocal priority
            while (priority is None):
                pass
            self.assertEqual(priority.data, value)
            priority = None

        self.node.create_subscription(Bool, 'joy_priority', _priority_cb, QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, depth=1))
        wait_and_check_priority(PRIORITY)  # default one

        client = ActionClient(self.node, JoyPriority, 'joy_priority_action')
        client.wait_for_server()

        client.send_goal_async(JoyPriority.Goal())
        wait_and_check_priority(not PRIORITY)

        client.send_goal_async(JoyPriority.Goal())
        wait_and_check_priority(PRIORITY)

    def test_joy_vel(self):
        self._set_and_wait_twist(0.5, 0.8, 1.0)

        self.assertEqual(
            self.current_twist.linear.x,
            self._get_current_vel(self.twist_msg.linear.x, INIT_STEP))
        self.assertEqual(
            self.current_twist.linear.y,
            self._get_current_vel(self.twist_msg.linear.y, INIT_STEP))
        self.assertEqual(
            self.current_twist.angular.z,
            self._get_current_vel(self.twist_msg.angular.z, INIT_STEP))

    def test_turbo_increase(self):
        # Increase
        increase_client = ActionClient(self.node, JoyTurbo, 'joy_turbo_increase')
        increase_client.wait_for_server()
        increase_client.send_goal(JoyTurbo.Goal())

        self._set_and_wait_twist(0.5, 0.8, 1.0)

        self.assertEqual(
            self.current_twist.linear.x,
            self._get_current_vel(self.twist_msg.linear.x, INIT_STEP + 1))
        self.assertEqual(
            self.current_twist.linear.y,
            self._get_current_vel(self.twist_msg.linear.y, INIT_STEP + 1))
        self.assertEqual(
            self.current_twist.angular.z,
            self._get_current_vel(self.twist_msg.angular.z, INIT_STEP + 1))

    def test_turbo_decrease(self):
        # Decrease
        decrease_client = ActionClient(self.node, JoyTurbo, 'joy_turbo_decrease')
        decrease_client.wait_for_server()
        decrease_client.send_goal(JoyTurbo.Goal())

        self._set_and_wait_twist(0.5, 0.8, 1.0)

        self.assertEqual(
            self.current_twist.linear.x,
            self._get_current_vel(self.twist_msg.linear.x, INIT_STEP - 1))
        self.assertEqual(
            self.current_twist.linear.y,
            self._get_current_vel(self.twist_msg.linear.y, INIT_STEP - 1))
        self.assertEqual(
            self.current_twist.angular.z,
            self._get_current_vel(self.twist_msg.angular.z, INIT_STEP - 1))

    def test_turbo_angular_increase(self):

        # Angular increase
        client = ActionClient(self.node, JoyTurbo, 'joy_turbo_angular_increase')
        client.wait_for_server()
        client.send_goal(JoyTurbo.Goal())

        self._set_and_wait_twist(0.5, 0.8, 1.0)

        self.assertEqual(
            self.current_twist.linear.x,
            self._get_current_vel(self.twist_msg.linear.x, INIT_STEP))
        self.assertEqual(
            self.current_twist.linear.y,
            self._get_current_vel(self.twist_msg.linear.y, INIT_STEP))
        self.assertEqual(
            self.current_twist.angular.z,
            self._get_current_vel(self.twist_msg.angular.z, INIT_STEP + 1))

    def test_turbo_angular_decrease(self):
        # Angular decrease
        client = ActionClient(self.node, JoyTurbo, 'joy_turbo_angular_decrease')
        client.wait_for_server()
        client.send_goal(JoyTurbo.Goal())

        self._set_and_wait_twist(0.5, 0.8, 1.0)

        self.assertEqual(
            self.current_twist.linear.x,
            self._get_current_vel(self.twist_msg.linear.x, INIT_STEP))
        self.assertEqual(
            self.current_twist.linear.y,
            self._get_current_vel(self.twist_msg.linear.y, INIT_STEP))
        self.assertEqual(
            self.current_twist.angular.z,
            self._get_current_vel(self.twist_msg.angular.z, INIT_STEP - 1))

    def test_joy_turbo_reset(self):
        # Decrease first to see the effect of the reset
        decrease_client = ActionClient(self.node, JoyTurbo, 'joy_turbo_decrease')
        decrease_client.wait_for_server()
        decrease_client.send_goal(JoyTurbo.Goal())

        self._set_and_wait_twist(0.5, 0.8, 1.0)

        self.assertEqual(
            self.current_twist.linear.x,
            self._get_current_vel(self.twist_msg.linear.x, INIT_STEP - 1))
        self.assertEqual(
            self.current_twist.linear.y,
            self._get_current_vel(self.twist_msg.linear.y, INIT_STEP - 1))
        self.assertEqual(
            self.current_twist.angular.z,
            self._get_current_vel(self.twist_msg.angular.z, INIT_STEP - 1))

        self._reset_and_wait_twist()

        # Reset
        reset_client = ActionClient(self.node, JoyTurbo, 'joy_turbo_reset')
        reset_client.wait_for_server()
        reset_client.send_goal(JoyTurbo.Goal())

        self._set_and_wait_twist(0.5, 0.8, 1.0)

        self.assertEqual(
            self.current_twist.linear.x,
            self._get_current_vel(self.twist_msg.linear.x, INIT_STEP))
        self.assertEqual(
            self.current_twist.linear.y,
            self._get_current_vel(self.twist_msg.linear.y, INIT_STEP))
        self.assertEqual(
            self.current_twist.angular.z,
            self._get_current_vel(self.twist_msg.angular.z, INIT_STEP))


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, joystick_relay, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, process=joystick_relay)
