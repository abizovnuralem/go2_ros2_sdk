/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Eurotec, Netherlands
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

/*
 * Author: Rein Appeldoorn
 */

#ifndef POINTCLOUD_TO_LASERSCAN__LASERSCAN_TO_POINTCLOUD_NODE_HPP_
#define POINTCLOUD_TO_LASERSCAN__LASERSCAN_TO_POINTCLOUD_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pointcloud_to_laserscan/visibility_control.h"

namespace pointcloud_to_laserscan
{
typedef tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> MessageFilter;

//! \brief The PointCloudToLaserScanNodelet class to process incoming laserscans into pointclouds.
//!
class LaserScanToPointCloudNode : public rclcpp::Node
{
public:
  POINTCLOUD_TO_LASERSCAN_PUBLIC
  explicit LaserScanToPointCloudNode(const rclcpp::NodeOptions & options);

  ~LaserScanToPointCloudNode() override;

private:
  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

  void subscriptionListenerThreadLoop();

  std::unique_ptr<tf2_ros::Buffer> tf2_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_;
  std::unique_ptr<MessageFilter> message_filter_;
  std::thread subscription_listener_thread_;
  std::atomic_bool alive_{true};

  laser_geometry::LaserProjection projector_;

  // ROS Parameters
  int input_queue_size_;
  std::string target_frame_;
  double tolerance_;
};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN__LASERSCAN_TO_POINTCLOUD_NODE_HPP_
