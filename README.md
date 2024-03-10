# Welcome to the Unitree Go2 ROS2 SDK Project!

We are delighted to present you with our integration of the Unitree Go2 ROS2 SDK, leveraging the innovative go2-WebRTC interface, originally designed by the talented @tfoldi. You can explore and utilize his groundbreaking work at go2-webrtc on GitHub.

This resourceful project is here to empower your Unitree GO2 AIR/PRO/EDU robots with ROS2 SDK capabilities. We're thrilled to offer an enhanced level of control and interaction, enabling you to take your robotics projects to new heights.

## Exciting Features:

:sparkles: Full ROS2 SDK support for your Unitree GO2

:robot: Compatible with AIR, PRO, and EDU variants

:footprints: Access to foot force sensors feedback (available on GO2 AIR/PRO)

## Your feedback and support mean the world to us. 

If you're as enthusiastic about this project as we are, please consider giving it a :star: star on our GitHub repository. 

Your encouragement fuels our passion and helps us develop our RoadMap further. We welcome any help or suggestions you can offer!

Together, let's push the boundaries of what's possible with the Unitree Go2 and ROS2!

Real time Go2 Air/PRO/EDU joints sync:

![alt text](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/go2.gif?raw=true)

Go2 Air/PRO/EDU lidar point cloud:

![alt text](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/go2_lidar_3.gif?raw=true)

![alt text](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/go2_lidar_1.gif?raw=true)


## Project RoadMap:
1. URDF :white_check_mark: 
2. Joint states sync in real time :white_check_mark: 
3. IMU sync in real time :white_check_mark: 
4. Joystick control in real time :white_check_mark: 
6. Go2 topics info in real time :white_check_mark: 
7. Foot force sensors info in real time :white_check_mark: 
8. Lidar stream :white_check_mark: 
9. Camera stream 
10. Foxglove bridge :white_check_mark:
11. SLAM
12. Object detection
13. AutoPilot

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


## Foxglove

![alt text](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/foxglove.gif?raw=true)

To use Foxglove, you need to install Foxglove Studio:
```
sudo snap install foxglove-studio
```

1. Open Foxglove Studio and press "Open Connection".
2. In the "Open Connection" settings, choose "Foxglove WebSocket" and use the default configuration ws://localhost:8765, then press "Open".
3. (Optional) You can also import a default layout view from the foxglove.json file located inside this repository.


## Development

To contribute or modify the project, refer to these resources for implementing additional features or improving the existing codebase. PRs are welcome!


## Thanks
Special thanks to @legion1581, @tfoldi, @budavariam, @alex.lin and TheRoboVerse community!


## License

This project is licensed under the BSD 2-clause License - see the [LICENSE](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/LICENSE) file for details.