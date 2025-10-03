// Copyright (c) 2024, RoboVerse community
// SPDX-License-Identifier: BSD-3-Clause

#ifndef LIDAR_PROCESSOR_CPP__LIDAR_TO_POINTCLOUD_NODE_HPP_
#define LIDAR_PROCESSOR_CPP__LIDAR_TO_POINTCLOUD_NODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <unordered_set>
#include <mutex>
#include <atomic>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/ply_io.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

namespace lidar_processor_cpp
{

struct LidarConfig
{
  std::vector<std::string> robot_ip_list;
  std::string map_name;
  bool save_map;
  double save_interval;  // seconds
  int max_points;        // Maximum points to keep in memory
  double voxel_size;     // Voxel downsampling size
};

struct Point3D
{
  float x, y, z;
  
  Point3D(float x_val, float y_val, float z_val) 
    : x(x_val), y(y_val), z(z_val) {}
  
  bool operator==(const Point3D& other) const 
  {
    return std::abs(x - other.x) < 1e-6 && 
           std::abs(y - other.y) < 1e-6 && 
           std::abs(z - other.z) < 1e-6;
  }
};

struct Point3DHash
{
  std::size_t operator()(const Point3D& p) const 
  {
    // Round to 3 decimal places for hashing
    int x_int = static_cast<int>(std::round(p.x * 1000));
    int y_int = static_cast<int>(std::round(p.y * 1000));
    int z_int = static_cast<int>(std::round(p.z * 1000));
    
    return std::hash<long long>{}(
      (static_cast<long long>(x_int) << 32) | 
      (static_cast<long long>(y_int) << 16) | 
      static_cast<long long>(z_int)
    );
  }
};

class PointCloudAggregator
{
public:
  explicit PointCloudAggregator(const LidarConfig& config);
  
  void addPoints(const std::vector<Point3D>& new_points);
  std::vector<Point3D> getPointsCopy() const;
  bool hasChanges() const;
  void markSaved();
  int getPointCount() const;

private:
  LidarConfig config_;
  std::unordered_set<Point3D, Point3DHash> points_;
  mutable std::mutex points_mutex_;
  std::atomic<bool> points_changed_;
};

class LidarToPointCloudNode : public rclcpp::Node
{
public:
  LidarToPointCloudNode();

private:
  void declareParameters();
  LidarConfig loadConfiguration();
  void setupSubscriptions();
  void setupPublishers();
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishAggregatedPointcloud(const std_msgs::msg::Header& header);
  void saveMapCallback();
  void logConfiguration();

  LidarConfig config_;
  std::unique_ptr<PointCloudAggregator> aggregator_;
  
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::TimerBase::SharedPtr save_timer_;
};

}  // namespace lidar_processor_cpp

#endif  // LIDAR_PROCESSOR_CPP__LIDAR_TO_POINTCLOUD_NODE_HPP_