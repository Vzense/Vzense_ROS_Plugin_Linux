#include "vzense_manager.hpp"

VzenseManager::VzenseManager(int32_t device_index, const string &camera_name) :
        color_nh_(camera_name + "/color"),
        depth_nh_(camera_name + "/depth"),
        ir_nh_(camera_name + "/ir"),
        alignedDepth_nh_(camera_name + "/aligned_depth_to_color"),
        camera_name_(camera_name),
        color_info_(new camera_info_manager::CameraInfoManager(color_nh_)),
        depth_info_(new camera_info_manager::CameraInfoManager(depth_nh_)),
        ir_info_(new camera_info_manager::CameraInfoManager(ir_nh_)),
        alignedDepth_info_(new camera_info_manager::CameraInfoManager(alignedDepth_nh_)),
        color_it_(new image_transport::ImageTransport(color_nh_)),
        depth_it_(new image_transport::ImageTransport(depth_nh_)),
        ir_it_(new image_transport::ImageTransport(ir_nh_)),
        alignedDepth_it_(new image_transport::ImageTransport(alignedDepth_nh_)),
        depth_threshold_(20),
        depth_range_(PsNearRange),
        rgb_width_(-1),
        rgb_height_(-1),
        slope_(1450),
        deviceHandle_(0),
        sessionIndex_(0)
{
    signal(SIGSEGV, VzenseManager::sigsegv_handler);

    // Initialise the API
    checkPsReturnStatus(Ps2_Initialize(), "Ps2_Initialize failed!");

    // Get number of available devices
    uint32_t device_count = 0;
GET:
    int checkDeviceSec = 0;
	PsReturnStatus status = Ps2_GetDeviceCount(&device_count);
	if (status != PsReturnStatus::PsRetOK || device_count < 1)
	{
        ROS_INFO("check device cost:%d second.", checkDeviceSec++);
        ros::Duration(1).sleep();
		goto GET;	
	}
    ROS_INFO("Get device count: %d", device_count);

    // Verify device index selection
    this->device_index_ = device_index;
    if (this->device_index_ < 0
        || this->device_index_ >= device_count)
        throw std::runtime_error(
                "Device index outside of available devices range 0-" + std::to_string(device_count));
    
    PsDeviceInfo* pPsDeviceInfo = new PsDeviceInfo;
    status =  Ps2_GetDeviceInfo(pPsDeviceInfo, device_index_);
    ROS_INFO("pPsDeviceInfo.uri:%s", pPsDeviceInfo->uri);

    // Attempt to open the device
    checkPsReturnStatus(Ps2_OpenDevice(pPsDeviceInfo->uri, &deviceHandle_), "OpenDevice failed!");

    ROS_INFO("Successfully connected to device %d", this->device_index_);

    status= Ps2_StartStream(deviceHandle_, sessionIndex_);
    ROS_INFO_STREAM( "Start Depth Frame status: " << status);

    // Set synchronisation between rgb and depth
    checkPsReturnStatus(Ps2_SetSynchronizeEnabled(deviceHandle_, sessionIndex_, true),
                      "Could not enable sensor synchronisation between rgb and depth sensors");
    ROS_INFO("Enabled sensor synchronisation for device %d", this->device_index_);

    checkPsReturnStatus(Ps2_SetRGBDistortionCorrectionEnabled(deviceHandle_, sessionIndex_, true),
                      "Could not enable rgb distortion correction");
    checkPsReturnStatus(Ps2_SetDepthDistortionCorrectionEnabled(deviceHandle_, sessionIndex_, true),
                      "Could not enable depth distortion correction");
    Ps2_SetSpatialFilterEnabled(deviceHandle_, sessionIndex_, true);
        ROS_INFO("Enabled distortion correction for device %d", this->device_index_);
}

void VzenseManager::set_rgbResolution(const PsResolution resolutionIndex) {
    const int valid_resolutions[6][2] = {{1920, 1080}, {1280, 720}, {640,  480}, {640,  360},{1600,  1200},{800,  600}};

    PsResolution invalidResolutionIndex = (PsResolution)(resolutionIndex%4);
    std::string message("Resolution " + to_string(valid_resolutions[invalidResolutionIndex][0]) + "x" + to_string(valid_resolutions[invalidResolutionIndex][1]));


    if(PsRetOK != Ps2_SetRGBResolution(deviceHandle_, sessionIndex_, invalidResolutionIndex))
    {
        ROS_WARN("SetRGBResolution %s is failed.",message.c_str());
        return;
    }
                       
    this->rgb_width_ = valid_resolutions[invalidResolutionIndex][0];
    this->rgb_height_ = valid_resolutions[invalidResolutionIndex][1];

    message += " has been set for device " + to_string(this->device_index_);
    ROS_INFO(message.c_str());
}

void VzenseManager::set_depthRange(const PsDepthRange range) {
    // Set depth range
    Ps2_SetDepthRange(deviceHandle_, sessionIndex_, range);
    PsMeasuringRange measuringRange = {0};
    Ps2_GetMeasuringRange(deviceHandle_, sessionIndex_, range, &measuringRange);
    switch (range%3)
    {
    case 0:
        this->slope_ = measuringRange.effectDepthMaxNear;
        break;
    case 1:
        this->slope_ = measuringRange.effectDepthMaxMid;
        break;
    case 2:
        this->slope_ = measuringRange.effectDepthMaxFar;
        break;
    default:
        break;
    }
    ROS_INFO("Set depth range %d at max distance %.2fm", range, (double) this->slope_ / 1000.0);
}

void VzenseManager::set_backgroundThreshold(const uint16_t threshold) {
    this->depth_threshold_ = threshold;
    checkPsReturnStatus(Ps2_SetThreshold(deviceHandle_, sessionIndex_, this->depth_threshold_), "Could not set depth threshold");
    ROS_INFO("Enabled depth confidence threshold at %d% (device %d)", this->depth_threshold_, this->device_index_);
}

void VzenseManager::set_DataMode(const PsDataMode dataMode){
    // Set to dataMode
    if(PsWDR_Depth == dataMode)
    {

        PsWDROutputMode wdrOutputMode = {PsWDRTotalRange_Two, PsNearRange, 1, PsMidRange, 1};
        Ps2_SetWDROutputMode(deviceHandle_, sessionIndex_, &wdrOutputMode);
        checkPsReturnStatus(Ps2_SetDataMode(deviceHandle_, sessionIndex_, (PsDataMode) dataMode),
                      "Set DataMode Failed failed!");
        Ps2_SetWDRStyle(deviceHandle_, sessionIndex_, PsWDR_FUSION);
    }
    else
    {
        checkPsReturnStatus(Ps2_SetDataMode(deviceHandle_, sessionIndex_, (PsDataMode) dataMode),
                      "Set DataMode Failed failed!");
    }

    this->data_mode_ = dataMode;
    // Enable depth to rgb frame alignment in the API，then alignedDepth becomes available
    if( PsDepthAndRGB_30 == dataMode || PsDepthAndIRAndRGB_30 == dataMode )
    {
        checkPsReturnStatus(Ps2_SetMapperEnabledRGBToDepth(deviceHandle_, sessionIndex_, true),
                            "Could not enable depth to rgb alignment");
        ROS_INFO("Enabled depth to rgb alignment for device %d", this->device_index_);
    }
    
    ROS_INFO("Set capture mode to %d for device %d", dataMode, this->device_index_);
}

void VzenseManager::run() {
    // Initialise ROS nodes
    set_sensor_intrinsics();
    sensor_msgs::CameraInfoPtr color_ci(new sensor_msgs::CameraInfo(color_info_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr depth_ci(new sensor_msgs::CameraInfo(depth_info_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr alignedDepth_ci(new sensor_msgs::CameraInfo(alignedDepth_info_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr ir_ci(new sensor_msgs::CameraInfo(ir_info_->getCameraInfo()));

    this->color_pub_ = this->color_it_->advertiseCamera("image_raw", 30);
    this->depth_pub_ = this->depth_it_->advertiseCamera("image_raw", 30);
    this->ir_pub_ = this->ir_it_->advertiseCamera("image_raw", 30);
    this->alignedDepth_pub_ = this->alignedDepth_it_->advertiseCamera("image_raw", 30);

    // Containers for frames
    PsReturnStatus status;
    ros::Time now = ros::Time::now();
    int missed_frames = 0;
    sensor_msgs::ImagePtr depth_msg;
    sensor_msgs::ImagePtr ir_msg;
    sensor_msgs::ImagePtr color_msg;
    sensor_msgs::ImagePtr alignedDetph_msg;

    while (ros::ok()) {
        // Get next frame set
        PsFrameReady psReadFrame = {0};
        PsReturnStatus status = Ps2_ReadNextFrame(deviceHandle_, sessionIndex_, &psReadFrame);
        if (status != PsRetOK)
        {
            continue;
        }

        now = ros::Time::now();

        if (1 == psReadFrame.depth)
        {
            psReadFrame.depth = (true == fillImagePtr(now, PsDepthFrame, depth_ci, depth_msg)) ? 1 : 0;
        }

        if (1 == psReadFrame.ir)
        {
            psReadFrame.ir = (true == fillImagePtr(now, PsIRFrame, ir_ci, ir_msg)) ? 1 : 0;           
        }

        if (1 == psReadFrame.rgb)
        {
            psReadFrame.rgb = (true == fillImagePtr(now, PsRGBFrame, color_ci, color_msg)) ? 1 : 0;           
        }

        if (1 == psReadFrame.mappedDepth)
        {
            psReadFrame.mappedDepth = (true == fillImagePtr(now, PsMappedDepthFrame, alignedDepth_ci, alignedDetph_msg)) ? 1 : 0;            
        }

        //Ps2_SetWDRStyle(deviceHandle_, sessionIndex_, PsWDR_FUSION)
        if (PsWDR_Depth == data_mode_ && 1 == psReadFrame.wdrDepth)
        {
            psReadFrame.depth = (true == fillImagePtr(now, PsWDRDepthFrame, depth_ci, depth_msg)) ? 1 : 0;            
        }
        
        if (1 == psReadFrame.depth)
        {
            this->depth_pub_.publish(depth_msg, depth_ci);
        }

        if (1 == psReadFrame.ir)
        {
            this->ir_pub_.publish(ir_msg, ir_ci);
        }

        if (1 == psReadFrame.rgb)
        {
            this->color_pub_.publish(color_msg, color_ci);
        }

        if (1 == psReadFrame.mappedDepth)
        {
            this->alignedDepth_pub_.publish(alignedDetph_msg, alignedDepth_ci);
        }

        // Process any callbacks
        ros::spinOnce();
    }

    status = Ps2_StopStream(deviceHandle_, sessionIndex_);
    ROS_INFO_STREAM( "Stop Depth Frame status: " << status);
    status = Ps2_CloseDevice(&deviceHandle_);
    ROS_INFO_STREAM( "CloseDevice status: " << status);
    status = Ps2_Shutdown();
    ROS_INFO_STREAM( "Shutdown status: " << status );
}

bool VzenseManager::fillImagePtr(const ros::Time& time, const PsFrameType type, sensor_msgs::CameraInfoPtr& cameraInfoPtr, sensor_msgs::ImagePtr& imagePtr)
{
    bool ret = false;

    PsFrame frame = {0};
    Ps2_GetFrame(deviceHandle_, sessionIndex_, type, &frame);
    
    if (frame.pFrameData != NULL)
    {
        int cvMatType = CV_16UC1;
        std::string imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
        switch (type)
        {
        case PsIRFrame:
        case PsDepthFrame:
        case PsWDRDepthFrame:
        case PsMappedDepthFrame:
            cvMatType = CV_16UC1;
            imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
            break;
        case PsRGBFrame:
            cvMatType = CV_8UC3;
            imageEncodeType = sensor_msgs::image_encodings::BGR8;
            break;
        default:
            return ret;
        }

        cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
        cameraInfoPtr->height = frame.height;
        cameraInfoPtr->width = frame.width;
        cameraInfoPtr->header.stamp = time;
        imagePtr = cv_bridge::CvImage(cameraInfoPtr->header, imageEncodeType, mat).toImageMsg();
        ret = true;
    }

    return ret;
}

void VzenseManager::sigsegv_handler(int sig) {
    signal(SIGSEGV, SIG_DFL);
    ROS_ERROR("Segmentation fault, stopping camera driver (%d).", sig);    
    ros::shutdown();
}

void VzenseManager::checkPsReturnStatus(PsReturnStatus status, const std::string &message_on_fail) {
    if (status == PsReturnStatus::PsRetOK)
        return;
    ROS_ERROR(message_on_fail.c_str());
    if (0 != deviceHandle_)
    {
        Ps2_StopStream(deviceHandle_, sessionIndex_);
        ROS_INFO_STREAM("Stop Depth Frame status: " << status);
        Ps2_CloseDevice(&deviceHandle_);
        ROS_INFO_STREAM("CloseDevice status: " << status);
    }
    Ps2_Shutdown();
    ROS_INFO_STREAM("Shutdown status: " << status);

    throw std::runtime_error(message_on_fail);
}

void VzenseManager::set_sensor_intrinsics() {
    std::string camera_frame(this->camera_name_ + "_frame"), 
                color_frame(this->camera_name_ + "_color_frame"),
                aligned_frame(this->camera_name_ + "_aligned_depth_to_color_frame"),
                depth_frame(this->camera_name_ + "_depth_frame"),
                ir_frame(this->camera_name_ + "_ir_frame");

    // Get camera parameters (extrinsic)
    checkPsReturnStatus(Ps2_GetCameraExtrinsicParameters(deviceHandle_, sessionIndex_, &this->extrinsics_),
                      "Could not get extrinsics!");

    // Setup tf broadcaster
    static tf2_ros::StaticTransformBroadcaster tf_broadcaster;
    ros::Time now = ros::Time::now();

    // PsCameraExtrinsicParameters to ROS transform
    tf::Transform transform;
    tf::Matrix3x3 rotation_matrix(extrinsics_.rotation[0], extrinsics_.rotation[1], extrinsics_.rotation[2],
                                  extrinsics_.rotation[3], extrinsics_.rotation[4], extrinsics_.rotation[5],
                                  extrinsics_.rotation[6], extrinsics_.rotation[7], extrinsics_.rotation[8]);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    // Publish static TFs
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = now;
    msg.transform.rotation.w = 1.0;

    // Camera base to Color Frame
    msg.header.frame_id = camera_frame;
    msg.child_frame_id = color_frame;
    tf_broadcaster.sendTransform(msg);

    // Color Frame to Aligned Frame
    msg.header.frame_id = color_frame;
    msg.child_frame_id = depth_frame;
    tf_broadcaster.sendTransform(msg);

    // Color Frame to Depth Frame
    msg.transform.translation.x = extrinsics_.translation[0] / 1000;
    msg.transform.translation.y = extrinsics_.translation[1] / 1000;
    msg.transform.translation.z = extrinsics_.translation[2] / 1000;
    msg.transform.rotation = orientation;
    msg.header.frame_id = color_frame;
    msg.child_frame_id = aligned_frame;
    tf_broadcaster.sendTransform(msg);

    // Get camera parameters (intrinsic)
    checkPsReturnStatus(Ps2_GetCameraParameters(deviceHandle_, sessionIndex_, PsDepthSensor, &this->depth_intrinsics_),
                      "Could not get depth intrinsics!");
    checkPsReturnStatus(Ps2_GetCameraParameters(deviceHandle_, sessionIndex_, PsRgbSensor, &this->color_intrinsics_),
                      "Could not get rgb intrinsics!");

    // Initialise camera info messages
    sensor_msgs::CameraInfo info_msg;
    info_msg.distortion_model = "plumb_bob";
    info_msg.header.frame_id = color_frame;
    info_msg.D = {color_intrinsics_.k1, color_intrinsics_.k2, color_intrinsics_.p1, color_intrinsics_.p2,
                  color_intrinsics_.k3};
    info_msg.K = {color_intrinsics_.fx, 0, color_intrinsics_.cx, 
                    0, color_intrinsics_.fy, color_intrinsics_.cy,
                    0, 0, 1};
    info_msg.P = {color_intrinsics_.fx, 0, color_intrinsics_.cx,
                    0, 0, color_intrinsics_.fy,
                    color_intrinsics_.cy, 0, 0, 
                    0, 1, 0};
    info_msg.R.fill(0);
    info_msg.R[0] = 1;
    info_msg.R[4] = 1;
    info_msg.R[8] = 1;
    color_info_->setCameraInfo(info_msg);
    alignedDepth_info_->setCameraInfo(info_msg);

    info_msg.header.frame_id = depth_frame;
    info_msg.D = {depth_intrinsics_.k1, depth_intrinsics_.k2, depth_intrinsics_.p1, depth_intrinsics_.p2,
                  depth_intrinsics_.k3};
    info_msg.K = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx,
                    0, depth_intrinsics_.fy, depth_intrinsics_.cy,
                    0, 0, 1};
    info_msg.P = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx,
                    0, 0, depth_intrinsics_.fy,
                    depth_intrinsics_.cy, 0, 0,
                    0, 1, 0};
    
    depth_info_->setCameraInfo(info_msg);
    info_msg.header.frame_id = ir_frame;
    ir_info_->setCameraInfo(info_msg);

    ROS_INFO("Successfully received intrinsic and extrinsic parameters for device %d", this->device_index_);
}
