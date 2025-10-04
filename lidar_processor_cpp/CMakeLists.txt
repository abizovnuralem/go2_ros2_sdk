cmake_minimum_required(VERSION 3.8)
project(lidar_processor_cpp)

# Set CMake policy to use <PackageName>_ROOT variables
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

# Compiler definitions
add_definitions(${PCL_DEFINITIONS})

# Common dependencies for all targets
set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
)

# Add executables
add_executable(lidar_to_pointcloud_node src/lidar_to_pointcloud_node.cpp)
ament_target_dependencies(lidar_to_pointcloud_node ${dependencies})
target_link_libraries(lidar_to_pointcloud_node ${PCL_LIBRARIES})

add_executable(pointcloud_aggregator_node src/pointcloud_aggregator_node.cpp)
ament_target_dependencies(pointcloud_aggregator_node ${dependencies})
target_link_libraries(pointcloud_aggregator_node ${PCL_LIBRARIES})

# Install targets
install(TARGETS
  lidar_to_pointcloud_node
  pointcloud_aggregator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY
  include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()