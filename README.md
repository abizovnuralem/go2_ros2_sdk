# Welcome to the Unitree Go2 ROS2 SDK Project!

![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)

We are delighted to present you with our integration of the Unitree Go2 ROS2 SDK, leveraging the innovative go2-WebRTC interface, originally designed by the talented @tfoldi. You can explore and utilize his groundbreaking work at go2-webrtc on GitHub.

This resourceful project is here to empower your Unitree GO2 AIR/PRO/EDU robots with ROS2 SDK capabilities. We're thrilled to offer an enhanced level of control and interaction, enabling you to take your robotics projects to new heights.

## Project RoadMap:
1. URDF :white_check_mark: 
2. Joint states sync in real time :white_check_mark: 
3. IMU sync in real time :white_check_mark: 
4. Joystick control in real time :white_check_mark: 
6. Go2 topics info in real time :white_check_mark: 
7. Foot force sensors info in real time :white_check_mark: 
8. Lidar stream (added pointCloud2) :white_check_mark: 
9. Camera stream :white_check_mark:
10. Foxglove bridge :white_check_mark:
11. Laser Scan :white_check_mark:
12. SLAM (slam_toolbox) :white_check_mark:
13. Navigation (nav2) :white_check_mark:
14. Object detection
15. AutoPilot

## Your feedback and support mean the world to us. 

If you're as enthusiastic about this project as we are, please consider giving it a :star: star on our GitHub repository. 

Your encouragement fuels our passion and helps us develop our RoadMap further. We welcome any help or suggestions you can offer!

Together, let's push the boundaries of what's possible with the Unitree Go2 and ROS2!

## Exciting Features:

:sparkles: Full ROS2 SDK support for your Unitree GO2

:robot: Compatible with AIR, PRO, and EDU variants

:footprints: Access to foot force sensors feedback (available on GO2 PRO/EDU)


Real time Go2 Air/PRO/EDU joints sync:

<p align="center">
<img width="1280" height="640" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/bf3f5a83-f02b-4c78-a7a1-b379ce057492" alt='Go2 joints sync'>
</p>

Go2 Air/PRO/EDU lidar point cloud:

<p align="center">
<img width="1280" height="640" src="https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/go2_lidar_3.gif?raw=true" alt='Go2 point cloud'>
</p>

<p align="center">
<img width="1280" height="640" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/9c1c3826-f875-4da1-a650-747044e748e1" alt='Go2 point cloud'>
</p>

## Topic
Real time Go2 Air/PRO ROS2 topics

<p align="left">
<img width="731" height="383" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/4d36c2c0-bd21-4af7-925e-b8fd6db68e61" alt='Go2 topic list'>
</p>

<p align="left">
<img width="446" height="1045" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/671af655-877b-4f36-89d3-91730674526d" alt='Go2 topic list'>
</p>

## System requirements
Tested systems and ROS2 distro
|systems|ROS2 distro|Build status
|--|--|--|
|Ubuntu 22.04|iron|![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)
|Ubuntu 22.04|humble|![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)
|Ubuntu 22.04|rolling|![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)

A single workspace can contain as many packages as you want, each in their own folder. You can also have packages of different build types in one workspace (CMake, Python, etc.). You cannot have nested packages.

Best practice is to have a src folder within your workspace, and to create your packages in there. This keeps the top level of the workspace “clean”.

Your workspace should look like:
```
workspace_folder/
    src/
      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/

      py_package_2/
          package.xml
          resource/py_package_2
          setup.cfg
          setup.py
          py_package_2/
```

clone this repo to src folder of your own ros2_ws repo

```
git clone --recurse-submodules https://github.com/abizovnuralem/go2_ros2_sdk.git

cd go2_ros2_sdk
sudo apt install python3-pip clang
pip install -r requirements.txt
cd ..
mkdir -p ros2_ws/src
copy all files inside go2_ros2_sdk folder to ros2_ws/src folder

```
install rust language support in your system https://www.rust-lang.org/tools/install 

cargo should work in terminal
```
cargo --version
```

Build it

You need to install ros2 and rosdep package first.

https://docs.ros.org/en/humble/Installation.html


```
source /opt/ros/$ROS_DISTRO/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Usage
don't forget to setup your GO2-robot in Wifi-mode and get IP then

```
export ROBOT_IP="Your robot ip"
cd ros2_ws
source install/setup.bash
ros2 launch go2_robot_sdk robot.launch.py
```

## Foxglove

<p align="center">
<img width="1200" height="630" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/f0920d6c-5b7a-4718-b781-8cfa03a88095" alt='Foxglove bridge'>
</p>

To use Foxglove, you need to install Foxglove Studio:
```
sudo snap install foxglove-studio
```

1. Open Foxglove Studio and press "Open Connection".
2. In the "Open Connection" settings, choose "Foxglove WebSocket" and use the default configuration ws://localhost:8765, then press "Open".
3. (Optional) You can also import a default layout view from the foxglove.json file located inside this repository.

## SLAM

<p align="center">
<img width="1280" height="640" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/59f33599-a54c-4cff-8ac2-6859a05ccb8a" alt='Slam'>
</p>

## Development

To contribute or modify the project, refer to these resources for implementing additional features or improving the existing codebase. PRs are welcome!

## Thanks

Special thanks to @legion1581, @tfoldi, @budavariam, @alex.lin and TheRoboVerse community!

## License

This project is licensed under the BSD 2-clause License - see the [LICENSE](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/LICENSE) file for details.
