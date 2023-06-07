// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

#pragma once

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

/**
 * @brief Class that allows the user to tune exposure in real time from the command line
 */
class K4AExposureCalibration
{
public:
    /**
     * @brief initializes calibrator with a provided node handle
     * @details Subscribes the node to /points2 and /rgb/raw/image.
     *          Advertises the k4a_update_exposure service and the k4a_auto_tune_exposure service.
     * @param[in] nh ROS node handle
     */
    K4AExposureCalibration(ros::NodeHandle& nh);

    /**
     * @brief call k4a_nodelet_manager/set_parameters to update exposure value
     * @param[in] req_exposure new exposure
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     * @return true if exposure is successfully updated
     */
    bool k4aUpdateExposure(int req_exposure, int& error_code, std::string& res_msg);
    
    /**
     * @brief auto tune exposure with given target blue value
     * @param[in] target_blue_value requested blue value to tune exposure to
     * @param[out] final_exposure exposure camera is set to after call
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     * @return true if auto tuning exposure is successfully completed
     */
    bool k4aAutoTuneExposure(int target_blue_value, int& final_exposure, int& error_code, std::string& res_msg);
    
    /**
     * @brief check if dynamic_reconfigure response has correct updated exposure
     * @param[in] requested_exposure exposure originally requested in k4aUpdateExposure
     * @param[out] updated_exposure exposure that was returned by the dynamic_reconfigure call
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     * @return true if requested_exposure == updated_exposure
     */
    bool k4aCameraExposureUpdateCheck(int requested_exposure, int updated_exposure, int& error_code, std::string& res_msg);
    
    /**
     * @brief check if requested_exposure is in appropriate bounds
     * @param[in] requested_exposure exposure originally requested in k4aUpdateExposure
     * @param[out] updated_exposure exposure that was returned by the dynamic_reconfigure call
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     * @return true if requested_exposure == updated_exposure
     */
    bool k4aCameraExposureBoundsCheck(int requested_exposure, int& error_code, std::string& res_msg);
    
    /**
     * @brief check if target_blue_value has been achieved
     * @details When conducting the auto tune loop, this method compares the average blue
     *          value of the image at a particular exposure to the desired target average
     *          blue value provided by the user in k4aAutoTuneExposure
     * @param[in] target_blue_val target blue value originally requested in k4aAutoTuneExposure
     * @param[in] current_average_blue_value the average blue value of the image at a point in time
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     */
    bool k4aTargetBlueCheck(int target_blue_val, int current_avg_blue_value, int& error_code, std::string& res_msg);
    
    /**
     * @brief check if target_blue_value is in appropriate range
     * @param[in] target_blue_value target blue value originally requested in k4aAutoTuneExposure
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     */
    bool k4aBlueBoundsCheck(int target_blue_value, int& error_code, std::string& res_msg);
    
    /**
     * @brief did the node receive an image at all?
     * @param[in] mat current OpenCV mat stored in latest_k4a_image for k4aAutoTuneExposure loop
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     */
    bool k4aImagePopulatedCheck(cv::Mat& mat, int& error_code, std::string& res_msg);

private:
    /**
     * @brief callback for /rgb/raw/image subscription
     * @details This callback handles updating the current image for the node.
     *          It converts the ROS image message to an OpenCV mat for auto tuning calculations.
     * @param[out] msg message received from image transport subscriber subscribed to /rgb/raw/image
     */
    void rgbRawImageCallback(const sensor_msgs::ImageConstPtr& msg);

    /**
     * @brief callback for k4aUpdateExposure
     * @details This callback handles requests to update the camera exposure from the
     *          k4a_update_exposure service.
     * @param[in] req request received from calling k4a_update_exposure
     * @param[out] res response sent from k4a_update_exposure
     */
    bool k4aUpdateExposureCallback(azure_kinect_ros_driver::k4a_update_exposure::Request &req,
                                   azure_kinect_ros_driver::k4a_update_exposure::Response &res);
    
    /**
     * @brief callback for k4aAutoTuneExposure
     * @details This callback handles requests to update the camera exposure from the
     *          k4a_auto_tune_exposure service.
     * @param[in] req request received from calling k4a_auto_tune_exposure
     * @param[out] res response sent from k4a_auto_tune_exposure
     */
    bool k4aAutoTuneExposureCallback(azure_kinect_ros_driver::k4a_auto_tune_exposure::Request &req,
                                     azure_kinect_ros_driver::k4a_auto_tune_exposure::Response &res);

    // private members
    ros::NodeHandle nh_;
    image_transport::Subscriber subRGBRaw;
    ros::ServiceServer update_exposure_service;
    ros::ServiceServer auto_tune_exposure_service;

    // allocate memory space to store latest image
    cv::Mat latest_k4a_image; /** @brief latest image*/
    cv::Mat* const latest_k4a_image_ptr = &latest_k4a_image; /** @brief pointer to latest image*/
    cv_bridge::CvImageConstPtr k4aCvImagePtr; /** @brief pointer to convert ROS image to OpenCV mat*/
    std::mutex latest_k4a_image_mutex; /** @brief mutex to protect latest image*/
    azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode k4a_error_code; /** @brief error codes*/

    // config file info
    // TODO: PULL THESE VALUES FROM CONFIG FILE
    const int MIN_EXPOSURE = 488;
    const int MAX_EXPOSURE = 1000000;
    const int DEFAULT_EXPOSURE = 15625;
    const int EXPOSURE_INC = 250;
    const int MIN_BLUE = 0;
    const int MAX_BLUE = 255;
};
