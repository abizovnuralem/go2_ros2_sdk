# go2_ros2_sdk
Unitree go2 ROS 2 drivers are implemented using the go2-WebRTC interface originally designed by @tfoldi (https://github.com/tfoldi/go2-webrtc)

This project enables ROS2 SDK functionality for your Unitree GO2 AIR/PRO/EDU

Also you can get foot force sensors feadback from GO2 PRO enabled.

![alt text](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/go2.gif?raw=true)

Current state:
1. URDF ( :white_check_mark: )
2. Joint states sync in real time ( :white_check_mark: )
3. IMU sync in real time ( :white_check_mark: )
4. Joystick control in real time ( :white_check_mark: )
6. Go2 topics info in real time ( :white_check_mark: )
7. Foot force sensors info in real time ( :white_check_mark: )
8. Camera stream
9. Lidar stream

## Topic
Real time Go2 Air/PRO ROS2 topics
![alt text](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/topics_2.png?raw=true)
![alt text](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/topics_1.png?raw=true)


## Getting started

install ROS 2 (tested on ROS2 Iron with Ubuntu 22.04 and ROS2 Humble with MacOS)

clone this rep and build it (put go2_interfaces and go2_robot_sdk to src folder of your own ros2_ws repo)
```
git clone https://github.com/abizovnuralem/go2_ros2_sdk.git
pip install -r requirements.txt
colcon build
```

don't forget to setup your GO2-robot in Wifi-mode only and get IP
then

```
export ROBOT_IP="Your robot ip"
```

## Usage
```
source install/setup.bash
ros2 launch go2_robot_sdk robot.launch.py
```


