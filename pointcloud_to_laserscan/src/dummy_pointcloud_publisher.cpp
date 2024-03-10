/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Open Source Robotics Foundation, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("dummy_pointcloud_publisher");
  auto pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS());

  sensor_msgs::msg::PointCloud2 dummy_cloud;
  sensor_msgs::PointCloud2Modifier modifier(dummy_cloud);
  modifier.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(node->declare_parameter("cloud_size", 100));
  std::mt19937 gen(node->declare_parameter("cloud_seed", 0));
  double extent = node->declare_parameter("cloud_extent", 10.0);
  std::uniform_real_distribution<float> distribution(-extent / 2, extent / 2);
  sensor_msgs::PointCloud2Iterator<float> it_x(dummy_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(dummy_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(dummy_cloud, "z");
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    *it_x = distribution(gen);
    *it_y = distribution(gen);
    *it_z = distribution(gen);
  }
  dummy_cloud.header.frame_id = node->declare_parameter("cloud_frame_id", "");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  rclcpp::Rate rate(1.0);
  while (rclcpp::ok()) {
    dummy_cloud.header.stamp = node->get_clock()->now();
    pub->publish(dummy_cloud);
    executor.spin_some();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
