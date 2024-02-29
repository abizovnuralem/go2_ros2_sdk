# go2_ros2_sdk
Unitree go2 ROS 2 sdk are implemented using the go2-WebRTC interface originally designed by @tfoldi (https://github.com/tfoldi/go2-webrtc)

This project enables ROS2 SDK functionality for your Unitree GO2 AIR/PRO/EDU

Also you can get foot force sensors feadback from GO2 PRO enabled.

Real time Go2 Air/PRO/EDU joints sync:

![alt text](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/go2.gif?raw=true)

Go2 Air/PRO/EDU lidar point cloud:

![alt text](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/go2_lidar_1.gif?raw=true)

![alt text](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/go2_lidar_3.gif?raw=true)


Current state:
1. URDF ( :white_check_mark: )
2. Joint states sync in real time ( :white_check_mark: )
3. IMU sync in real time ( :white_check_mark: )
4. Joystick control in real time ( :white_check_mark: )
6. Go2 topics info in real time ( :white_check_mark: )
7. Foot force sensors info in real time ( :white_check_mark: )
8. Lidar stream ( :white_check_mark: ) **
9. Camera stream ***
10. Integrate Foxglove
11. Integrate Object detection
12. Integrate AutoPilot

## Topic
Real time Go2 Air/PRO ROS2 topics

![alt text](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/topics_2.png?raw=true)

![alt text](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/topics_1.png?raw=true)


## System requirements
Tested systems and ROS2 distro
|systems|ROS2 distro|
|--|--|
|Ubuntu 22.04|humble|
|Ubuntu 22.04|iron|

clone this rep and build it (put go2_interfaces and go2_robot_sdk to src folder of your own ros2_ws repo)
```
git clone https://github.com/abizovnuralem/go2_ros2_sdk.git
pip install -r requirements.txt
colcon build
```

don't forget to setup your GO2-robot in Wifi-mode and get IP
then

```
export ROBOT_IP="Your robot ip"
```

## Usage
```
source install/setup.bash
ros2 launch go2_robot_sdk robot.launch.py
```

## Development

To contribute or modify the project, refer to these resources for implementing additional features or improving the existing codebase. PRs are welcome!
** We need to optimize the performance of LiDAR and point cloud synchronization.
*** We need to integrate the camera stream.

## Thanks
Special thanks to @legion1581, @tfoldi, @budavariam, @alex.lin and TheRoboVerse community!


