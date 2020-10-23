#ifndef VZENSE_MANAGER_H
#define VZENSE_MANAGER_H


#include <csignal>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include "Vzense_api2.h"
#include <dynamic_reconfigure/server.h>

using namespace std;
using namespace cv;


class VzenseManager {
public:
    explicit VzenseManager(int32_t device_index = 0, const std::string &topic_name = "Vzense");

    void set_rgbResolution(const PsResolution resolutionIndex);
    void set_depthRange(const PsDepthRange range);
    void set_backgroundThreshold(const uint16_t threshold);
    void set_DataMode(const PsDataMode dataMode);
    void run();

private:
    static void sigsegv_handler(int sig);
    void checkPsReturnStatus(PsReturnStatus status, const std::string &message_on_fail);
    void set_sensor_intrinsics();
    bool fillImagePtr(const ros::Time& time, const PsFrameType type, sensor_msgs::CameraInfoPtr& cameraInfoPtr, sensor_msgs::ImagePtr& imagePtr);

    std::string camera_name_;
    ros::NodeHandle color_nh_, depth_nh_, alignedDepth_nh_, ir_nh_;
    std::shared_ptr<image_transport::ImageTransport> color_it_, depth_it_, alignedDepth_it_, ir_it_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> color_info_, depth_info_, alignedDepth_info_, ir_info_;

    image_transport::CameraPublisher color_pub_, depth_pub_, alignedDepth_pub_, ir_pub_;

    int32_t device_index_, data_mode_ = PsDepthAndRGB_30;
    uint16_t depth_threshold_, slope_;
    PsDepthRange depth_range_;
    int rgb_width_, rgb_height_;
    PsDeviceHandle deviceHandle_;
    unsigned int sessionIndex_;  
    PsCameraParameters depth_intrinsics_{}, color_intrinsics_{};
    PsCameraExtrinsicParameters extrinsics_{};
};


#endif //VZENSE_MANAGER_H
