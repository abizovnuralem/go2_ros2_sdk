// Copyright (c) 2024, RoboVerse community
// SPDX-License-Identifier: BSD-3-Clause

#include "lidar_processor_cpp/pointcloud_aggregator_node.hpp"
#include <algorithm>
#include <cmath>

namespace lidar_processor_cpp
{

StatisticalFilter::StatisticalFilter(int k_neighbors, double std_ratio)
  : k_neighbors_(k_neighbors), std_ratio_(std_ratio)
{
  sor_filter_.setMeanK(k_neighbors_);
  sor_filter_.setStddevMulThresh(std_ratio_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalFilter::filterPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{
  if (input_cloud->points.size() < static_cast<size_t>(k_neighbors_)) {
    return input_cloud;
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  sor_filter_.setInputCloud(input_cloud);
  sor_filter_.filter(*filtered_cloud);
  
  return filtered_cloud;
}

PointCloudAggregatorNode::PointCloudAggregatorNode()
  : Node("pointcloud_aggregator")
{
  // Declare parameters
  declareParameters();
  config_ = loadConfiguration();
  
  // Initialize filters
  statistical_filter_ = std::make_unique<StatisticalFilter>(20, 2.0);
  
  // Initialize timing
  last_publish_time_ = std::chrono::steady_clock::now();
  
  // Setup subscriptions and publishers
  setupSubscriptions();
  setupPublishers();
  
  // Publish timer
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / config_.publish_rate),
    std::bind(&PointCloudAggregatorNode::publishCallback, this)
  );
  
  RCLCPP_INFO(this->get_logger(), "🔄 PointCloud Aggregator Node initialized");
  logConfiguration();
}

void PointCloudAggregatorNode::declareParameters()
{
  this->declare_parameter("max_range", 20.0);
  this->declare_parameter("min_range", 0.1);
  this->declare_parameter("height_filter_min", -2.0);
  this->declare_parameter("height_filter_max", 3.0);
  this->declare_parameter("downsample_rate", 10);
  this->declare_parameter("publish_rate", 5.0);
}

AggregatorConfig PointCloudAggregatorNode::loadConfiguration()
{
  AggregatorConfig config;
  
  config.max_range = this->get_parameter("max_range").as_double();
  config.min_range = this->get_parameter("min_range").as_double();
  config.height_filter_min = this->get_parameter("height_filter_min").as_double();
  config.height_filter_max = this->get_parameter("height_filter_max").as_double();
  config.downsample_rate = this->get_parameter("downsample_rate").as_int();
  config.publish_rate = this->get_parameter("publish_rate").as_double();
  
  return config;
}

void PointCloudAggregatorNode::setupSubscriptions()
{
  auto qos = rclcpp::QoS(5)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);
  
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/pointcloud/aggregated",
    qos,
    std::bind(&PointCloudAggregatorNode::pointcloudCallback, this, std::placeholders::_1)
  );
}

void PointCloudAggregatorNode::setupPublishers()
{
  auto qos = rclcpp::QoS(5)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);
  
  filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pointcloud/filtered", qos
  );
  
  downsampled_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pointcloud/downsampled", qos
  );
}

void PointCloudAggregatorNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  try {
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    if (cloud->points.empty()) {
      return;
    }
    
    // Apply filters
    auto filtered_cloud = applyFilters(cloud);
    
    // Store for aggregation
    if (!filtered_cloud->points.empty()) {
      std::lock_guard<std::mutex> lock(clouds_mutex_);
      aggregated_clouds_.push_back(filtered_cloud);
      
      // Keep only recent data (last 10 seconds worth)
      size_t max_clouds = static_cast<size_t>(config_.publish_rate * 10);
      if (aggregated_clouds_.size() > max_clouds) {
        aggregated_clouds_.erase(
          aggregated_clouds_.begin(),
          aggregated_clouds_.begin() + (aggregated_clouds_.size() - max_clouds)
        );
      }
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudAggregatorNode::applyFilters(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{
  if (input_cloud->points.empty()) {
    return input_cloud;
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  // Apply range and height filters
  for (const auto& point : input_cloud->points) {
    // Skip invalid points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    
    // Range filter (XY distance only)
    double distance = std::sqrt(point.x * point.x + point.y * point.y);
    if (distance < config_.min_range || distance > config_.max_range) {
      continue;
    }
    
    // Height filter
    if (point.z < config_.height_filter_min || point.z > config_.height_filter_max) {
      continue;
    }
    
    filtered_cloud->points.push_back(point);
  }
  
  // Update cloud properties
  filtered_cloud->width = filtered_cloud->points.size();
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = true;
  
  // Statistical outlier removal (only if enough points)
  if (filtered_cloud->points.size() > 100) {
    filtered_cloud = statistical_filter_->filterPoints(filtered_cloud);
  }
  
  return filtered_cloud;
}

void PointCloudAggregatorNode::publishCallback()
{
  try {
    std::lock_guard<std::mutex> lock(clouds_mutex_);
    
    if (aggregated_clouds_.empty()) {
      return;
    }
    
    // Combine all stored point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (const auto& cloud : aggregated_clouds_) {
      *combined_cloud += *cloud;
    }
    
    if (combined_cloud->points.empty()) {
      return;
    }
    
    // Update cloud properties
    combined_cloud->width = combined_cloud->points.size();
    combined_cloud->height = 1;
    combined_cloud->is_dense = true;
    
    // Create header
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "base_link";
    
    // Publish filtered cloud
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*combined_cloud, filtered_msg);
    filtered_msg.header = header;
    filtered_pub_->publish(filtered_msg);
    
    // Create and publish downsampled version
    if (config_.downsample_rate > 1) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      
      for (size_t i = 0; i < combined_cloud->points.size(); i += config_.downsample_rate) {
        downsampled_cloud->points.push_back(combined_cloud->points[i]);
      }
      
      downsampled_cloud->width = downsampled_cloud->points.size();
      downsampled_cloud->height = 1;
      downsampled_cloud->is_dense = true;
      
      sensor_msgs::msg::PointCloud2 downsampled_msg;
      pcl::toROSMsg(*downsampled_cloud, downsampled_msg);
      downsampled_msg.header = header;
      downsampled_pub_->publish(downsampled_msg);
    }
    
    // Log statistics periodically
    auto current_time = std::chrono::steady_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(
      current_time - last_publish_time_).count();
    
    if (time_diff >= 5) {  // Every 5 seconds
      size_t total_points = combined_cloud->points.size();
      size_t downsampled_points = total_points / std::max(1, config_.downsample_rate);
      
      RCLCPP_INFO(this->get_logger(),
        "📊 Processed: %zu points, Clouds: %zu, Downsampled: %zu",
        total_points, aggregated_clouds_.size(), downsampled_points);
      
      last_publish_time_ = current_time;
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing processed data: %s", e.what());
  }
}

void PointCloudAggregatorNode::logConfiguration()
{
  RCLCPP_INFO(this->get_logger(), "⚙️  Aggregator Configuration:");
  RCLCPP_INFO(this->get_logger(), "   Range filter: %.1f - %.1fm", 
    config_.min_range, config_.max_range);
  RCLCPP_INFO(this->get_logger(), "   Height filter: %.1f - %.1fm", 
    config_.height_filter_min, config_.height_filter_max);
  RCLCPP_INFO(this->get_logger(), "   Downsample rate: 1/%d", config_.downsample_rate);
  RCLCPP_INFO(this->get_logger(), "   Publish rate: %.1f Hz", config_.publish_rate);
}

}  // namespace lidar_processor_cpp

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<lidar_processor_cpp::PointCloudAggregatorNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "Error running aggregator: " << e.what() << std::endl;
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}