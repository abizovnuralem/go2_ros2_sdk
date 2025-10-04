// Copyright (c) 2024, RoboVerse community
// SPDX-License-Identifier: BSD-3-Clause

#ifndef LIDAR_PROCESSOR_CPP__POINTCLOUD_AGGREGATOR_NODE_HPP_
#define LIDAR_PROCESSOR_CPP__POINTCLOUD_AGGREGATOR_NODE_HPP_

#include <memory>
#include <vector>
#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl_conversions/pcl_conversions.h"

namespace lidar_processor_cpp
{

struct AggregatorConfig
{
  double max_range;          // Maximum range from robot center
  double min_range;          // Minimum range from robot center
  double height_filter_min;  // Minimum height (z-coordinate)
  double height_filter_max;  // Maximum height (z-coordinate)
  int downsample_rate;       // Keep every Nth point
  double publish_rate;       // Hz
};

class StatisticalFilter
{
public:
  StatisticalFilter(int k_neighbors = 20, double std_ratio = 2.0);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

private:
  int k_neighbors_;
  double std_ratio_;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter_;
};

class PointCloudAggregatorNode : public rclcpp::Node
{
public:
  PointCloudAggregatorNode();

private:
  void declareParameters();
  AggregatorConfig loadConfiguration();
  void setupSubscriptions();
  void setupPublishers();
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr applyFilters(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);
  void publishCallback();
  void logConfiguration();

  AggregatorConfig config_;
  std::unique_ptr<StatisticalFilter> statistical_filter_;
  
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> aggregated_clouds_;
  std::chrono::steady_clock::time_point last_publish_time_;
  mutable std::mutex clouds_mutex_;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace lidar_processor_cpp

#endif  // LIDAR_PROCESSOR_CPP__POINTCLOUD_AGGREGATOR_NODE_HPP_