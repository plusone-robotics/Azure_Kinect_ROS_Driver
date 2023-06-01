// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

#ifndef K4A_EXPOSURE_CALIBRATION_NODE_H
#define K4A_EXPOSURE_CALIBRATION_NODE_H

// Library headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/client.h>
#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>
#include <mutex>

// Project headers
#include "azure_kinect_ros_driver/AzureKinectParamsConfig.h"
#include "azure_kinect_ros_driver/k4a_update_exposure.h"
#include "azure_kinect_ros_driver/k4a_auto_tune_exposure.h"
#include "azure_kinect_ros_driver/k4aCameraExposureServiceErrorCode.h"

class K4AExposureCalibration
{
public:
    K4AExposureCalibration();
    K4AExposureCalibration(ros::NodeHandle& nh, image_transport::ImageTransport& it);
    void init(ros::NodeHandle& nh, image_transport::ImageTransport& it);
    bool k4aUpdateExposure(int req_exposure, int& error_code, std::string& res_msg);
    bool k4aAutoTuneExposure(int target_blue_value, int& final_exposure, int& error_code, std::string& res_msg);
private:
    void p2Callback(const sensor_msgs::PointCloud2& msg);
    void rgbRawImageCallback(const sensor_msgs::ImageConstPtr& msg);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber subPC;
    image_transport::Subscriber subRGBRaw;

    // allocate memory space to store latest image
    cv::Mat latest_k4a_image;
    cv::Mat* latest_k4a_image_ptr;
    cv_bridge::CvImageConstPtr k4aCvImagePtr;
    std::mutex latest_k4a_image_mutex;
    azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode k4a_error_code;

    bool k4aUpdateExposureCallback(azure_kinect_ros_driver::k4a_update_exposure::Request &req,
                                   azure_kinect_ros_driver::k4a_update_exposure::Response &res);
    bool k4aAutoTuneExposureCallback(azure_kinect_ros_driver::k4a_auto_tune_exposure::Request &req,
                                     azure_kinect_ros_driver::k4a_auto_tune_exposure::Response &res);
};

#endif // K4A_EXPOSURE_CALIBRATION_NODE_H