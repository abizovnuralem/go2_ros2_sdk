# LiDAR Processor C++

High-performance C++ implementation of LiDAR data processing nodes for the Go2 robot. This package provides optimized C++ versions of the Python LiDAR processing nodes with identical functionality, topics, and parameters.

## Features

- **High Performance**: Optimized C++ implementation using PCL (Point Cloud Library)
- **Memory Efficient**: Advanced point cloud aggregation with memory management
- **Thread Safe**: Thread-safe operations for concurrent data processing
- **Statistical Filtering**: Built-in outlier removal using statistical analysis
- **Compatible**: Drop-in replacement for Python lidar_processor nodes

## Nodes

### 1. lidar_to_pointcloud_node

Processes raw LiDAR data and aggregates point clouds with optional map saving.

**Topics:**
- **Subscribed:**
  - `/robot0/point_cloud2` (single robot mode)
  - `/robot{i}/point_cloud2` (multi-robot mode)
- **Published:**
  - `/pointcloud/aggregated` - Aggregated point cloud data

**Parameters:**
- `robot_ip_lst` (string_array): List of robot IP addresses
- `map_name` (string): Name of the map file to save (default: "3d_map")
- `map_save` (string): Whether to save map periodically (default: "true")
- `save_interval` (double): Map saving interval in seconds (default: 10.0)
- `max_points` (int): Maximum points to keep in memory (default: 1000000)
- `voxel_size` (double): Voxel size for downsampling when saving (default: 0.01)

### 2. pointcloud_aggregator_node

Advanced point cloud processing with filtering, downsampling, and statistical outlier removal.

**Topics:**
- **Subscribed:**
  - `/pointcloud/aggregated` - Raw aggregated point clouds
- **Published:**
  - `/pointcloud/filtered` - Filtered point clouds
  - `/pointcloud/downsampled` - Downsampled point clouds

**Parameters:**
- `max_range` (double): Maximum range from robot center in meters (default: 20.0)
- `min_range` (double): Minimum range from robot center in meters (default: 0.1)
- `height_filter_min` (double): Minimum height filter (z-coordinate) (default: -2.0)
- `height_filter_max` (double): Maximum height filter (z-coordinate) (default: 3.0)
- `downsample_rate` (int): Keep every Nth point for downsampling (default: 10)
- `publish_rate` (double): Publishing rate in Hz (default: 5.0)

## Building

```bash
# From your ROS2 workspace
colcon build --packages-select lidar_processor_cpp
```

## Usage

### Single Node Launch

Launch individual nodes:

```bash
# Launch LiDAR to PointCloud processor
ros2 launch lidar_processor_cpp lidar_to_pointcloud.launch.py

# Launch PointCloud Aggregator
ros2 launch lidar_processor_cpp pointcloud_aggregator.launch.py
```

### Complete Pipeline Launch

Launch the complete processing pipeline:

```bash
ros2 launch lidar_processor_cpp lidar_processing_pipeline.launch.py
```

### Custom Parameters

Launch with custom parameters:

```bash
ros2 launch lidar_processor_cpp lidar_processing_pipeline.launch.py \
    robot_ip_lst:="['192.168.1.100']" \
    map_name:="my_custom_map" \
    max_range:=15.0 \
    publish_rate:=10.0
```

## Dependencies

- **ROS2 Dependencies:**
  - `rclcpp`
  - `sensor_msgs`
  - `geometry_msgs`
  - `std_msgs`

- **PCL Dependencies:**
  - `pcl_ros`
  - `pcl_conversions`
  - `libpcl-dev`

- **System Dependencies:**
  - `tf2`
  - `tf2_ros`

## Performance Optimizations

1. **Memory Management**: Efficient point cloud storage using hash sets
2. **Statistical Filtering**: PCL-based outlier removal
3. **Voxel Downsampling**: Memory-efficient point cloud downsampling
4. **Thread Safety**: Mutex-protected shared data structures
5. **SIMD Operations**: Leveraging PCL's optimized algorithms

## Compatibility

This C++ implementation maintains full compatibility with the Python version:
- Identical topic names and message types
- Same parameter names and default values
- Compatible launch file structure
- Same filtering and processing algorithms

## License

BSD-3-Clause