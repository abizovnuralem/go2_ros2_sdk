// Copyright (c) 2024, RoboVerse community
// SPDX-License-Identifier: BSD-3-Clause

#include "lidar_processor_cpp/lidar_to_pointcloud_node.hpp"
#include <algorithm>
#include <chrono>

namespace lidar_processor_cpp
{

PointCloudAggregator::PointCloudAggregator(const LidarConfig& config)
  : config_(config), points_changed_(false)
{
}

void PointCloudAggregator::addPoints(const std::vector<Point3D>& new_points)
{
  std::lock_guard<std::mutex> lock(points_mutex_);
  
  for (const auto& point : new_points) {
    // Round points to reduce memory usage
    Point3D rounded_point(
      std::round(point.x * 1000.0f) / 1000.0f,
      std::round(point.y * 1000.0f) / 1000.0f,
      std::round(point.z * 1000.0f) / 1000.0f
    );
    points_.insert(rounded_point);
  }
  
  // Memory management - remove oldest points if exceeding limit
  if (static_cast<int>(points_.size()) > config_.max_points) {
    std::vector<Point3D> points_vector(points_.begin(), points_.end());
    
    // Sort by distance from origin, keep closest points
    std::sort(points_vector.begin(), points_vector.end(),
      [](const Point3D& a, const Point3D& b) {
        float dist_a = a.x * a.x + a.y * a.y + a.z * a.z;
        float dist_b = b.x * b.x + b.y * b.y + b.z * b.z;
        return dist_a < dist_b;
      });
    
    points_.clear();
    for (int i = 0; i < config_.max_points && i < static_cast<int>(points_vector.size()); ++i) {
      points_.insert(points_vector[i]);
    }
  }
  
  points_changed_ = true;
}

std::vector<Point3D> PointCloudAggregator::getPointsCopy() const
{
  std::lock_guard<std::mutex> lock(points_mutex_);
  return std::vector<Point3D>(points_.begin(), points_.end());
}

bool PointCloudAggregator::hasChanges() const
{
  return points_changed_.load();
}

void PointCloudAggregator::markSaved()
{
  points_changed_ = false;
}

int PointCloudAggregator::getPointCount() const
{
  std::lock_guard<std::mutex> lock(points_mutex_);
  return static_cast<int>(points_.size());
}

LidarToPointCloudNode::LidarToPointCloudNode()
  : Node("lidar_to_pointcloud")
{
  // Declare and get parameters
  declareParameters();
  config_ = loadConfiguration();
  
  // Initialize components
  aggregator_ = std::make_unique<PointCloudAggregator>(config_);
  
  // Setup subscriptions and publishers
  setupSubscriptions();
  setupPublishers();
  
  // Setup timers
  if (config_.save_map) {
    save_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(config_.save_interval),
      std::bind(&LidarToPointCloudNode::saveMapCallback, this)
    );
  }
  
  // Log configuration
  logConfiguration();
}

void LidarToPointCloudNode::declareParameters()
{
  this->declare_parameter("robot_ip_lst", std::vector<std::string>{});
  this->declare_parameter("map_name", "3d_map");
  this->declare_parameter("map_save", "true");
  this->declare_parameter("save_interval", 10.0);
  this->declare_parameter("max_points", 1000000);
  this->declare_parameter("voxel_size", 0.01);
}

LidarConfig LidarToPointCloudNode::loadConfiguration()
{
  LidarConfig config;
  
  config.robot_ip_list = this->get_parameter("robot_ip_lst").as_string_array();
  config.map_name = this->get_parameter("map_name").as_string();
  std::string save_map_str = this->get_parameter("map_save").as_string();
  config.save_map = (save_map_str == "true");
  config.save_interval = this->get_parameter("save_interval").as_double();
  config.max_points = this->get_parameter("max_points").as_int();
  config.voxel_size = this->get_parameter("voxel_size").as_double();
  
  return config;
}

void LidarToPointCloudNode::setupSubscriptions()
{
  // Setup QoS profile for high-frequency data
  auto qos = rclcpp::QoS(1)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);
  
  if (config_.robot_ip_list.size() == 1) {
    // Single robot mode
    auto subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/robot0/point_cloud2",
      qos,
      std::bind(&LidarToPointCloudNode::lidarCallback, this, std::placeholders::_1)
    );
    subscriptions_.push_back(subscription);
  } else {
    // Multi-robot mode
    for (size_t i = 0; i < config_.robot_ip_list.size(); ++i) {
      std::string topic = "/robot" + std::to_string(i) + "/point_cloud2";
      auto subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic,
        qos,
        std::bind(&LidarToPointCloudNode::lidarCallback, this, std::placeholders::_1)
      );
      subscriptions_.push_back(subscription);
    }
  }
}

void LidarToPointCloudNode::setupPublishers()
{
  auto qos = rclcpp::QoS(1)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);
  
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pointcloud/aggregated", qos
  );
}

void LidarToPointCloudNode::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  try {
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    // Extract points
    std::vector<Point3D> points;
    points.reserve(cloud->points.size());
    
    for (const auto& pcl_point : cloud->points) {
      if (std::isfinite(pcl_point.x) && std::isfinite(pcl_point.y) && std::isfinite(pcl_point.z)) {
        points.emplace_back(pcl_point.x, pcl_point.y, pcl_point.z);
      }
    }
    
    // Add to aggregator
    aggregator_->addPoints(points);
    
    // Publish aggregated point cloud
    publishAggregatedPointcloud(msg->header);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing LiDAR data: %s", e.what());
  }
}

void LidarToPointCloudNode::publishAggregatedPointcloud(const std_msgs::msg::Header& header)
{
  try {
    auto points = aggregator_->getPointsCopy();
    if (points.empty()) {
      return;
    }
    
    // Create PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(points.size());
    
    for (const auto& point : points) {
      cloud->points.emplace_back(point.x, point.y, point.z);
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    
    // Convert to ROS message
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(*cloud, pointcloud_msg);
    pointcloud_msg.header = header;
    
    pointcloud_pub_->publish(pointcloud_msg);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing point cloud: %s", e.what());
  }
}

void LidarToPointCloudNode::saveMapCallback()
{
  try {
    if (!aggregator_->hasChanges()) {
      return;
    }
    
    auto points = aggregator_->getPointsCopy();
    if (points.empty()) {
      return;
    }
    
    // Create PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(points.size());
    
    for (const auto& point : points) {
      cloud->points.emplace_back(point.x, point.y, point.z);
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    
    // Apply voxel downsampling for saving
    if (config_.voxel_size > 0) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      voxel_filter.setInputCloud(cloud);
      voxel_filter.setLeafSize(config_.voxel_size, config_.voxel_size, config_.voxel_size);
      voxel_filter.filter(*downsampled_cloud);
      cloud = downsampled_cloud;
    }
    
    // Save to file
    std::string map_filename = config_.map_name + ".ply";
    if (pcl::io::savePLYFileBinary(map_filename, *cloud) == 0) {
      int point_count = static_cast<int>(cloud->points.size());
      int total_points = aggregator_->getPointCount();
      aggregator_->markSaved();
      
      RCLCPP_INFO(this->get_logger(),
        "💾 Saved map: %s (%d downsampled / %d total points)",
        map_filename.c_str(), point_count, total_points);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", map_filename.c_str());
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error saving map: %s", e.what());
  }
}

void LidarToPointCloudNode::logConfiguration()
{
  RCLCPP_INFO(this->get_logger(), "🗺️  LiDAR Processor Configuration:");
  
  std::string robot_ips = "[";
  for (size_t i = 0; i < config_.robot_ip_list.size(); ++i) {
    robot_ips += config_.robot_ip_list[i];
    if (i < config_.robot_ip_list.size() - 1) robot_ips += ", ";
  }
  robot_ips += "]";
  
  RCLCPP_INFO(this->get_logger(), "   Robot IPs: %s", robot_ips.c_str());
  RCLCPP_INFO(this->get_logger(), "   Map name: %s", config_.map_name.c_str());
  RCLCPP_INFO(this->get_logger(), "   Save map: %s", config_.save_map ? "true" : "false");
  
  if (config_.save_map) {
    RCLCPP_INFO(this->get_logger(), "   Save interval: %.1fs", config_.save_interval);
    RCLCPP_INFO(this->get_logger(), "   Max points: %d", config_.max_points);
    RCLCPP_INFO(this->get_logger(), "   Voxel size: %.3fm", config_.voxel_size);
  }
}

}  // namespace lidar_processor_cpp

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<lidar_processor_cpp::LidarToPointCloudNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "Error running lidar processor: " << e.what() << std::endl;
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}