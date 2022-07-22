#include <iostream>
#include "ros/ros.h"
#include <vzense_manager.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "vzense_manager");

    int32_t device_index = ros::param::param<int32_t>("~device_index", 0);
    PsDataMode dataMode = (PsDataMode)ros::param::param<int32_t>("~dataMode", 0);
    PsDepthRange depth_range = (PsDepthRange)ros::param::param<int32_t>("~depth_range", 0);
    PsResolution rgb_resolution = (PsResolution)ros::param::param<int>("~rgb_resolution", PsResolution::PsRGB_Resolution_640_480);

    VzenseManager manager = VzenseManager(device_index);
    manager.set_DataMode(dataMode);
    manager.set_depthRange(depth_range);
    manager.set_rgbResolution(rgb_resolution);
    manager.run();

    return 0;
}
