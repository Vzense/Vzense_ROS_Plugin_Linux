# Vzense_ROS_Plugin_Linux
The goal of this project is to help developers easily use the Vzense TOF camera in ROS.

## Requirements

- ROS: melodic and newer
- OS: Ubuntu 18.04
- vzense linux base sdk:3.4.4

Note: Before using, please refer to https://github.com/Vzense/Vzense_SDK_Linux to update the vzense linux sdk to the latest version

## Usage 

```bash
roslaunch vzense_camera vzense_camera.launch
roslaunch vzense_camera vzense_pointCloudxyz.launch
roslaunch vzense_camera vzense_pointCloudxyzrgb.launch
```

## Install

```bash
cd ~/catkin_ws/src
git clone https://github.com/Vzense/Vzense_ROS_Plugin_Linux
cd ..
sudo cp src/Vzense_ROS_Plugin_Linux/dependencies/Lib/libImgPreProcess.so /usr/lib
catkin build vzense_camera
source devel/setup.bash
```
## Supported Devices

- DCAM710
- DCAM800
- DCAM800LITE
- DCAM305
- DCAM500
- DCAM550
- DCAM560

