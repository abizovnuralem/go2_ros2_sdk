cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan)

find_package(ament_cmake REQUIRED)

find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_library(laserscan_to_pointcloud SHARED
  src/laserscan_to_pointcloud_node.cpp)
target_compile_definitions(laserscan_to_pointcloud
  PRIVATE "POINTCLOUD_TO_LASERSCAN_BUILDING_DLL")
target_include_directories(laserscan_to_pointcloud PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(laserscan_to_pointcloud
  laser_geometry::laser_geometry
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_components::component
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
)
rclcpp_components_register_node(laserscan_to_pointcloud
  PLUGIN "pointcloud_to_laserscan::LaserScanToPointCloudNode"
  EXECUTABLE laserscan_to_pointcloud_node)

add_library(pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp)
target_compile_definitions(pointcloud_to_laserscan
  PRIVATE "POINTCLOUD_TO_LASERSCAN_BUILDING_DLL")
target_include_directories(pointcloud_to_laserscan PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(pointcloud_to_laserscan
  laser_geometry::laser_geometry
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_components::component
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
)
rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)

add_executable(dummy_pointcloud_publisher
  src/dummy_pointcloud_publisher.cpp
)
target_link_libraries(dummy_pointcloud_publisher
  rclcpp::rclcpp
  "${sensor_msgs_TARGETS}"
)

install(TARGETS
  laserscan_to_pointcloud
  pointcloud_to_laserscan
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(TARGETS
  dummy_pointcloud_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  laser_geometry
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

ament_package()
