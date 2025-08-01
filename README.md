![Ros2 SDK](https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/49edebbe-11b6-49c6-b82d-bc46257674bd)

# Welcome to the Unitree Go2 ROS2 SDK Project!

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-orange.svg)](https://www.microsoft.com/en-us/)
![ROS2 Build](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)
[![License](https://img.shields.io/badge/license-BSD--2-yellow.svg)](https://opensource.org/licenses/BSD-2-Clause)


We are happy to present you our integration of the Unitree Go2 with ROS2 over Wi-Fi, that was designed by the talented [@tfoldi](https://github.com/tfoldi). You can explore his groundbreaking work at [go2-webrtc](https://github.com/tfoldi/go2-webrtc).

This repo will empower your Unitree GO2 AIR/PRO/EDU robots with ROS2 capabilities, using both WebRTC (Wi-Fi) and CycloneDDS (Ethernet) protocols.

If you are using WebRTC (Wi-Fi) protocol, close the connection with a mobile app before connecting to the robot.

## Project RoadMap:

1. URDF :white_check_mark: 
2. Joint states sync in real time :white_check_mark: 
3. IMU sync in real time :white_check_mark: 
4. Joystick control in real time :white_check_mark: 
6. Go2 topics info in real time :white_check_mark: 
7. Foot force sensors info in real time :white_check_mark: 
8. Lidar stream (added pointCloud2) :white_check_mark: 
9. Camera stream :white_check_mark:


## Your feedback and support mean the world to us. 

If you're as enthusiastic about this project as we are, please consider giving it a :star: star!!!

Your encouragement fuels our passion and helps us develop our RoadMap further. We welcome any help or suggestions you can offer!

Together, let's push the boundaries of what's possible with the Unitree Go2 and ROS2!

## Exciting Features:

:sparkles: Full ROS2 SDK support for your Unitree GO2

:robot: Compatible with AIR, PRO, and EDU variants

:footprints: Access to foot force sensors feedback (available on some GO2 PRO models or EDU)


## Real time Go2 Air/PRO/EDU joints sync:

<p align="center">
<img width="1280" height="640" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/bf3f5a83-f02b-4c78-a7a1-b379ce057492" alt='Go2 joints sync'>
</p>

## Go2 Air/PRO/EDU lidar point cloud:

<p align="center">
<img width="1280" height="640" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/9c1c3826-f875-4da1-a650-747044e748e1" alt='Go2 point cloud'>
</p>


## System requirements

Tested systems and ROS2 distro
|systems|ROS2 distro|Build status
|--|--|--|
|Ubuntu 22.04|iron|![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)
|Ubuntu 22.04|humble|![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)
|Ubuntu 22.04|rolling|![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)

## Installation

```shell
mkdir -p ros2_ws
cd ros2_ws
git clone --recurse-submodules https://github.com/abizovnuralem/go2_ros2_sdk.git src
sudo apt install ros-$ROS_DISTRO-image-tools
sudo apt install ros-$ROS_DISTRO-vision-msgs

sudo apt install python3-pip clang portaudio19-dev
cd src
pip install -r requirements.txt
cd ..
```
Pay attention to any error messages. If `pip install` does not complete cleanly, various features will not work. For example, `open3d` does not yet support `python3.12` and therefore you will need to set up a 3.11 `venv` first etc.

Build `go2_ros_sdk`. You need to have `ros2` and `rosdep` installed. If you do not, follow these [instructions](https://docs.ros.org/en/humble/Installation.html). Then:
```shell
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Running via Docker
Can set environment variables beforehand, hardcoded in docker/docker-compose.yaml, or as shown below. 

Run:
```shell
cd docker
ROBOT_IP=<ROBOT_IP> CONN_TYPE=<webrtc/cyclonedds> docker-compose up --build
```

## Usage

Don't forget to set up your Go2 robot in Wifi-mode and obtain the IP. You can use the mobile app to get it. Go to Device -> Data -> Automatic Machine Inspection and look for STA Network: wlan0.

```shell
source install/setup.bash
export ROBOT_IP="robot_ip"
ros2 launch go2_robot_sdk robot.launch.py
```

The `robot.launch.py` code starts many services/nodes simultaneously, including 

* robot_state_publisher
* ros2_go2_video (front color camera)
* go2_robot_sdk/go2_driver_node
* rviz2
* `joy` (ROS2 Driver for Generic Joysticks and Game Controllers)
* `teleop_twist_joy` (facility for tele-operating Twist-based ROS2 robots with a standard joystick. Converts joy messages to velocity commands)       
* `twist_mux` (twist_multiplexer with source prioritization)        


When you run `robot.launch.py`, `rviz` will fire up, lidar data will begin to accumulate, the front color camera data will be displayed too (typically after 4 seconds), and your dog will be waiting for commands from your joystick (e.g. a X-box controller). You can then steer the dog through your house, e.g., and collect LIDAR mapping data. 


## WebRTC Topic Interface

The SDK provides a WebRTC topic interface that allows sending various commands to the robot. This is particularly useful for non-movement actions such as turning on headlights, playing sounds, and other robot control functions.

To send commands via the WebRTC topic:

```bash
# Basic command structure
ros2 topic pub /webrtc_req unitree_go/msg/WebRtcReq "{api_id: <API_ID>, parameter: '<PARAMETER>', topic: '<TOPIC>', priority: <0|1>}" --once

# Example: Send a handshake command
ros2 topic pub /webrtc_req unitree_go/msg/WebRtcReq "{api_id: 1016, topic: 'rt/api/sport/request'}" --once
```

## WSL 2

If you are running ROS2 under WSL2 - you may need to configure Joystick\Gamepad to navigate the robot.

1. Step 1 - share device with WSL2

    Follow steps here https://learn.microsoft.com/en-us/windows/wsl/connect-usb to share your console device with WSL2

2. Step 2 - Enable WSL2 joystick drivers

    WSL2 does not come by default with the modules for joysticks. Build WSL2 Kernel with the joystick drivers. Follow the instructions here: https://github.com/dorssel/usbipd-win/wiki/WSL-support#building-your-own-wsl-2-kernel-with-additional-drivers  If you're comfortable with WSl2, skip the export steps and start at `Install prerequisites.`

    Before buiding, edit `.config` file and update the CONFIG_ values listed in this GitHub issue: https://github.com/microsoft/WSL/issues/7747#issuecomment-1328217406

2. Step 3 - Give permissions to /dev/input devices

    Once you've finished the guides under Step 3 - you should be able to see your joystick device under /dev/input

    ```bash
    ls /dev/input
    by-id  by-path  event0  js0
    ```

    By default /dev/input/event* will only have root permissions, so joy node won't have access to the joystick

    Create a file `/etc/udev/rules.d/99-userdev-input.rules` with the following content:
    `KERNEL=="event*", SUBSYSTEM=="input", RUN+="/usr/bin/setfacl -m u:YOURUSERNAME:rw $env{DEVNAME}"`

    Run as root: `udevadm control --reload-rules && udevadm trigger`

    https://askubuntu.com/a/609678

3. Step 3 - verify that joy node is able to see the device properly. 

    Run `ros2 run joy joy_enumerate_devices`

    ```
    ID : GUID                             : GamePad : Mapped : Joystick Device Name
    -------------------------------------------------------------------------------
    0 : 030000005e040000120b000007050000 :    true :  false : Xbox Series X Controller
    ```

## Thanks

Special thanks to:
1. @tfoldi (Tamas) for his idea and talent to create a webrtc connection method between python and unitree GO2;
2. @budavariam for helping with lidar issues;
3. @legion1581 for a new webrtc method, that is working with 1.1.1 firmware update;
4. @alex.lin for his passion in ros1 ingration;
5. @alansrobotlab for his passion in robotics and helping me to debug new webrtc method;
6. @giangalv (Gianluca Galvagn) for helping me debug new issues with webrtc;
7. Many many other open source contributors! and TheRoboVerse community!


## License

This project is licensed under the BSD 2-clause License - see the [LICENSE](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/LICENSE) file for details.
