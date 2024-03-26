cmake_minimum_required(VERSION 3.14)
project(go2_robot_sim)

# Default settings
set(ROBOT_TYPE "Go1")         # The type of robot, support Go1 and A1 currently
set(PLATFORM "amd64")         # The platform to compile, support amd64 and arm64

set(SIMULATION ON)         # Use Gazebo or not, ON or OFF
set(REAL_ROBOT OFF)        # Link real robot or not, ON or OFF
set(DEBUG OFF)             # Use debug functions or not, ON or OFF

# Check required variables
if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()

if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

# Set definitions based on robot type
if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

# Simulation and real robot cannot both be set
if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
endif()

set(CMAKE_CXX_STANDARD 17)   # ROS 2 typically uses C++17
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Boost is used for shared pointers before C++11
find_package(Boost REQUIRED)

if(SIMULATION)
    find_package(gazebo_ros_pkgs REQUIRED)
endif()

# Definitions for different configurations
add_definitions(-DRUN_ROS)
if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python COMPONENTS Interpreter Development NumPy)
endif()

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
)

# Source files
file(GLOB_RECURSE SRC_LIST
    "src/*.cpp"
    "src/*.cc"
)

add_executable(junior_ctrl src/main.cpp ${SRC_LIST})

ament_target_dependencies(junior_ctrl
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

if(DEBUG)
    target_include_directories(junior_ctrl PRIVATE ${Python_INCLUDE_DIRS} ${Python_NumPy_INCLUDE_DIRS})
    target_link_libraries(junior_ctrl Python::Python Python::NumPy)
endif()

# Real robot linking
if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        # Include directories and link libraries for A1
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
        # Include directories and link libraries for Go1
        # You'd fill this in based on how the SDK should be linked in ROS2.
    endif()
endif()

# Additional libraries
target_link_libraries(junior_ctrl -pthread)

if(SIMULATION)
    ament_target_dependencies(junior_ctrl gazebo_ros_pkgs)
endif()

install(TARGETS
    junior_ctrl
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
