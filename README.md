# go2_ros2_sdk
Unitree go2 ROS 2 drivers are implemented using the go2-WebRTC interface originally designed by @tfoldi (https://github.com/tfoldi/go2-webrtc)

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

don't forget to put your GO2 in Wifi-mode only and get robot IP and TOKEN (HOW TO in https://github.com/tfoldi/go2-webrtc repo)
PUT these data into webrtc_driver.py

## Usage
```
source install/setup.bash
ros2 launch go2_robot_sdk robot.launch.py
```
