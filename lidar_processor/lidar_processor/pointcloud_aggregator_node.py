# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
PointCloud Aggregator Node

Advanced point cloud processing with filtering, downsampling, and visualization.
"""

from typing import List
from dataclasses import dataclass
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np


@dataclass 
class AggregatorConfig:
    """Configuration for point cloud aggregator"""
    max_range: float = 20.0        # Maximum range from robot center
    min_range: float = 0.1         # Minimum range from robot center
    height_filter_min: float = -2.0  # Minimum height (z-coordinate)
    height_filter_max: float = 3.0   # Maximum height (z-coordinate)
    downsample_rate: int = 10       # Keep every Nth point
    publish_rate: float = 5.0       # Hz


class StatisticalFilter:
    """Statistical outlier removal filter"""
    
    def __init__(self, k_neighbors: int = 20, std_ratio: float = 2.0):
        self.k_neighbors = k_neighbors
        self.std_ratio = std_ratio
    
    def filter_points(self, points: np.ndarray) -> np.ndarray:
        """Remove statistical outliers from point cloud"""
        if len(points) < self.k_neighbors:
            return points
        
        # Calculate distances to k nearest neighbors for each point
        distances = []
        for i, point in enumerate(points):
            # Calculate distances to all other points
            dists = np.linalg.norm(points - point, axis=1)
            # Get k+1 nearest (including self) and take mean of k neighbors
            nearest_dists = np.partition(dists, self.k_neighbors)[:self.k_neighbors+1]
            distances.append(np.mean(nearest_dists[1:]))  # Exclude self (distance=0)
        
        # Filter based on statistical threshold
        distances = np.array(distances)
        mean_dist = np.mean(distances)
        std_dist = np.std(distances)
        threshold = mean_dist + self.std_ratio * std_dist
        
        # Keep points within threshold
        mask = distances <= threshold
        return points[mask]


class PointCloudAggregatorNode(Node):
    """Advanced point cloud aggregation and processing node"""
    
    def __init__(self):
        super().__init__('pointcloud_aggregator')
        
        # Declare parameters
        self._declare_parameters()
        self.config = self._load_configuration()
        
        # Initialize filters
        self.statistical_filter = StatisticalFilter()
        
        # Setup QoS
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Storage for aggregated points
        self.aggregated_points: List[np.ndarray] = []
        self.last_publish_time = time.time()
        
        # Setup subscriptions and publishers
        self._setup_subscriptions()
        self._setup_publishers()
        
        # Publish timer
        self.publish_timer = self.create_timer(
            1.0 / self.config.publish_rate,
            self._publish_callback
        )
        
        self.get_logger().info("🔄 PointCloud Aggregator Node initialized")
        self._log_configuration()
    
    def _declare_parameters(self) -> None:
        """Declare node parameters"""
        self.declare_parameter('max_range', 20.0)
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('height_filter_min', -2.0)
        self.declare_parameter('height_filter_max', 3.0)
        self.declare_parameter('downsample_rate', 10)
        self.declare_parameter('publish_rate', 5.0)
    
    def _load_configuration(self) -> AggregatorConfig:
        """Load configuration from parameters"""
        return AggregatorConfig(
            max_range=self.get_parameter('max_range').get_parameter_value().double_value,
            min_range=self.get_parameter('min_range').get_parameter_value().double_value,
            height_filter_min=self.get_parameter('height_filter_min').get_parameter_value().double_value,
            height_filter_max=self.get_parameter('height_filter_max').get_parameter_value().double_value,
            downsample_rate=self.get_parameter('downsample_rate').get_parameter_value().integer_value,
            publish_rate=self.get_parameter('publish_rate').get_parameter_value().double_value
        )
    
    def _setup_subscriptions(self) -> None:
        """Setup subscriptions to raw point clouds"""
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pointcloud/aggregated',
            self._pointcloud_callback,
            self.qos_profile
        )
    
    def _setup_publishers(self) -> None:
        """Setup publishers for processed data"""
        self.filtered_pub = self.create_publisher(
            PointCloud2,
            '/pointcloud/filtered',
            self.qos_profile
        )
        
        self.downsampled_pub = self.create_publisher(
            PointCloud2,
            '/pointcloud/downsampled',
            self.qos_profile
        )
    
    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        """Process incoming aggregated point cloud"""
        try:
            # Extract points
            points = []
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])
            
            if not points:
                return
            
            points_array = np.array(points)
            
            # Apply filters
            filtered_points = self._apply_filters(points_array)
            
            # Store for aggregation
            if len(filtered_points) > 0:
                self.aggregated_points.append(filtered_points)
                
                # Keep only recent data (last 10 seconds worth)
                max_clouds = int(self.config.publish_rate * 10)
                if len(self.aggregated_points) > max_clouds:
                    self.aggregated_points = self.aggregated_points[-max_clouds:]
            
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")
    
    def _apply_filters(self, points: np.ndarray) -> np.ndarray:
        """Apply all filtering operations"""
        if len(points) == 0:
            return points
        
        # Range filter
        distances = np.linalg.norm(points[:, :2], axis=1)  # XY distance only
        range_mask = (distances >= self.config.min_range) & (distances <= self.config.max_range)
        points = points[range_mask]
        
        if len(points) == 0:
            return points
        
        # Height filter
        height_mask = (points[:, 2] >= self.config.height_filter_min) & \
                     (points[:, 2] <= self.config.height_filter_max)
        points = points[height_mask]
        
        if len(points) == 0:
            return points
        
        # Statistical outlier removal (only if enough points)
        if len(points) > 100:
            points = self.statistical_filter.filter_points(points)
        
        return points
    
    def _publish_callback(self) -> None:
        """Periodic publishing of processed data"""
        try:
            if not self.aggregated_points:
                return
            
            # Combine all stored point clouds
            all_points = np.vstack(self.aggregated_points)
            
            if len(all_points) == 0:
                return
            
            # Create header
            header = self.get_clock().now().to_msg()
            header.frame_id = "base_link"
            
            # Publish filtered cloud
            filtered_msg = point_cloud2.create_cloud_xyz32(header, all_points.tolist())
            self.filtered_pub.publish(filtered_msg)
            
            # Create and publish downsampled version
            if self.config.downsample_rate > 1:
                downsampled_points = all_points[::self.config.downsample_rate]
                downsampled_msg = point_cloud2.create_cloud_xyz32(header, downsampled_points.tolist())
                self.downsampled_pub.publish(downsampled_msg)
            
            # Log statistics periodically
            current_time = time.time()
            if current_time - self.last_publish_time > 5.0:  # Every 5 seconds
                self.get_logger().info(
                    f"📊 Processed: {len(all_points):,} points, "
                    f"Clouds: {len(self.aggregated_points)}, "
                    f"Downsampled: {len(all_points)//max(1, self.config.downsample_rate):,}"
                )
                self.last_publish_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Error publishing processed data: {e}")
    
    def _log_configuration(self) -> None:
        """Log current configuration"""
        self.get_logger().info("⚙️  Aggregator Configuration:")
        self.get_logger().info(f"   Range filter: {self.config.min_range:.1f} - {self.config.max_range:.1f}m")
        self.get_logger().info(f"   Height filter: {self.config.height_filter_min:.1f} - {self.config.height_filter_max:.1f}m")
        self.get_logger().info(f"   Downsample rate: 1/{self.config.downsample_rate}")
        self.get_logger().info(f"   Publish rate: {self.config.publish_rate:.1f} Hz")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = PointCloudAggregatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running aggregator: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 