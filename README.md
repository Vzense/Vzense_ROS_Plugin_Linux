# Vzense camera Driver

## Usage 

```bash
roslaunch vzense_camera vzense_camera.launch
```

## Install

```bash
cd ~/catkin_ws/src
git clone https://github.com/Vzense/BaseSDK_ROS_wrapper
cd ..
sudo cp src/BaseSDK_ROS_wrapper/dependencies/Lib/libImgPreProcess.so /usr/lib
catkin build vzense_camera
source devel/setup.bash
```

## Requirements

- ROS: melodic and newer
- OS: Ubuntu 18.04
- vzense linux sdk:3.1.1

