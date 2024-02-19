# go2_ros2_sdk
Unitee go2 ROS 2 drivers are implemented using the go2-WebRTC interface originally designed by @tfoldi (https://github.com/tfoldi/go2-webrtc)

This project enables ROS2 SDK functionality for your Unithree GO2 AIR/PRO

Current state:
1. URDF (Done)
2. GO2_joint_states sync (Done)

Soon:
1. IMU sync
2. Camera stream
3. Lidar stream
4. Joystick control

## Getting started

install ROS 2 (tested on iron)

clone this rep and build it
```
git clone https://github.com/abizovnuralem/go2_ros2_sdk.git
colcon build
```

## Usage
```
source install/setup.bash
ros2 launch go2_robot_sdk robot.launch.py
```
