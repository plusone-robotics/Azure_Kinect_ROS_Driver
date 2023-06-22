// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

#pragma once

// Library headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <dynamic_reconfigure/client.h>
#include <k4a/k4a.h>
#include <mutex>

// Project headers
#include "azure_kinect_ros_driver/AzureKinectParamsConfig.h"
#include "azure_kinect_ros_driver/k4a_update_exposure.h"
#include "azure_kinect_ros_driver/k4a_update_white_balance.h"
#include "azure_kinect_ros_driver/k4a_auto_tune_exposure.h"
#include "azure_kinect_ros_driver/k4a_sgd_tune.h"
#include "azure_kinect_ros_driver/k4aCameraExposureServiceErrorCode.h"

/**
 * @brief Class that allows the user to tune camera params in real time from the command line
 */
class K4APORCalibration
{
public:
    /**
     * @brief default constructor and destructor
     * @details These are required in order for catkin testing to work properly.
     */
    K4APORCalibration();
    ~K4APORCalibration();

    /**
     * @brief initializes calibrator with a provided node handle
     * @details Subscribes the node to /rgb/raw/image.
     *          Advertises:
                    k4a_update_exposure service
                    k4a_update_white_balance service
                    k4a_auto_tune_exposure service
                    k4a_sgd_tune service
     * @param[in] nh ROS node handle
     */
    K4APORCalibration(ros::NodeHandle& nh);

    /**
     * @brief call k4a_nodelet_manager/set_parameters to update exposure value
     * @param[in] req_exposure new exposure
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     * @return true if exposure is successfully updated
     */
    bool k4aUpdateExposure(const uint32_t req_exposure, int8_t& error_code, std::string& res_msg);

    /**
     * @brief call k4a_nodelet_manager/set_parameters to update white balance value
     * @param[in] req_white_balance new white balance
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     * @return true if white balance is successfully updated
     */
    bool k4aUpdateWhiteBalance(const uint16_t req_white_balance, int8_t& error_code, std::string& res_msg);
    
    /**
     * @brief auto tune exposure with given target blue value
     * @param[in] target_blue_value requested blue value to tune exposure to
     * @param[out] final_exposure exposure camera is set to after call
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     * @return true if auto tuning exposure is successfully completed
     */
    bool k4aAutoTuneExposure(const uint8_t target_blue_value, uint32_t& final_exposure, int8_t& error_code, std::string& res_msg);

    /**
     * @brief auto tune exposure and white balance with given target bgr and white values
     * @param[in] target_blue_value requested blue value to tune params to
     * @param[in] target_green_value requested green value to tune params to
     * @param[in] target_red_value requested red value to tune params to
     * @param[in] target_white_value requested white value to tune params to
     * @param[out] final_exposure exposure camera is set to after call
     * @param[out] final_white_balance white balance camera is set to after call
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     * @return true if auto tuning is successfully completed
     */
    bool k4aSGDTune(const float target_blue_value,
                    const float target_green_value,
                    const float target_red_value,
                    const float target_white_value,
                    uint32_t& final_exposure,
                    uint16_t& final_white_balance,
                    int8_t& error_code,
                    std::string& res_msg);

    /**
     * @brief check if dynamic_reconfigure response has correctly updated exposure
     * @param[in] requested_exposure exposure originally requested in k4aUpdateExposure
     * @param[in] updated_exposure exposure that was returned by the dynamic_reconfigure call
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     * @return true if requested_exposure == updated_exposure
     */
    bool k4aCameraExposureUpdateCheck(const uint32_t requested_exposure, uint32_t updated_exposure, int8_t& error_code, std::string& res_msg);
    
    /**
     * @brief check if requested_exposure is in appropriate bounds
     * @param[in] requested_exposure exposure originally requested in k4aUpdateExposure
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     * @return true if requested_exposure is within appropriate exposure bounds
     */
    bool k4aCameraExposureBoundsCheck(const uint32_t requested_exposure, int8_t& error_code, std::string& res_msg);

    /**
     * @brief check if dynamic_reconfigure response has correctly updated white balance
     * @param[in] requested_white_balance white balance originally requested in k4aUpdateWhiteBalance
     * @param[in] updated_white_balance white balance that was returned by the dynamic_reconfigure call
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     * @return true if requested_white_balance == updated_white_balance
     */
    bool k4aCameraWhiteBalanceUpdateCheck(const uint16_t requested_white_balance, uint16_t updated_white_balance, int8_t& error_code, std::string& res_msg);
    
    /**
     * @brief check if requested_white_balance is in appropriate bounds
     * @param[in] requested_white_balance white balance originally requested in k4aUpdateWhiteBalance
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     * @return true if requested_white_balance is within appropriate white balance bounds
     */
    bool k4aCameraWhiteBalanceBoundsCheck(const uint16_t requested_white_balance, int8_t& error_code, std::string& res_msg);

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
    bool k4aTargetBlueCheck(const uint8_t target_blue_val, uint8_t current_avg_blue_value, int8_t& error_code, std::string& res_msg);
    
    /**
     * @brief did the node receive an image at all?
     * @param[in] mat current OpenCV mat stored in latest_k4a_image for k4aAutoTuneExposure loop
     * @param[out] error_code error code included in response
     * @param[out] res_msg human-readable error message included in response
     */
    bool k4aImagePopulatedCheck(cv::Mat& mat, int8_t& error_code, std::string& res_msg);

private:
    /**
     * @brief callback for /rgb/raw/image subscription
     * @details This callback handles updating the current image for the node.
     *          It converts the ROS image message to an OpenCV mat for auto tuning calculations.
     * @param[in] msg message received from image transport subscriber subscribed to /rgb/raw/image
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
     * @brief callback for k4aUpdateWhiteBalance
     * @details This callback handles requests to update the camera white balance from the
     *          k4a_update_white_balance service.
     * @param[in] req request received from calling k4a_update_white_balance
     * @param[out] res response sent from k4a_update_white_balance
     */
    bool k4aUpdateWhiteBalanceCallback(azure_kinect_ros_driver::k4a_update_white_balance::Request &req,
                                       azure_kinect_ros_driver::k4a_update_white_balance::Response &res);

    /**
     * @brief callback for k4aAutoTuneExposure
     * @details This callback handles requests to update the camera exposure from the
     *          k4a_auto_tune_exposure service.
     * @param[in] req request received from calling k4a_auto_tune_exposure
     * @param[out] res response sent from k4a_auto_tune_exposure
     */
    bool k4aAutoTuneExposureCallback(azure_kinect_ros_driver::k4a_auto_tune_exposure::Request &req,
                                     azure_kinect_ros_driver::k4a_auto_tune_exposure::Response &res);

    /**
     * @brief callback for k4aSGDTune
     * @details This callback handles requests to update the camera settings from the
     *          k4a_sgd_tune service. Currently supports updating exposure_time and
                white_balance.
     * @param[in] req request received from calling k4a_sgd_tune
     * @param[out] res response sent from k4a_sgd_tune
     */
    bool k4aSGDTuneCallback(azure_kinect_ros_driver::k4a_sgd_tune::Request &req,
                            azure_kinect_ros_driver::k4a_sgd_tune::Response &res);

    /**
     * @brief calculate RMSE for SGD tuning
     * @param[in] current current average value
     * @param[in] target target average value
     * @param[out] rmse RMSE calculated
     */
    float k4aRMSE(const float current, const float target);

    // private members
    ros::NodeHandle nh_;
    image_transport::Subscriber subRGBRaw_;
    ros::ServiceServer update_exposure_service_;
    ros::ServiceServer update_white_balance_service_;
    ros::ServiceServer auto_tune_exposure_service_;
    ros::ServiceServer sgd_tune_service_;

    // allocate memory space to store latest image
    cv::Mat latest_k4a_image_; /** @brief latest image*/
    cv::Mat* const latest_k4a_image_ptr_ = &latest_k4a_image_; /** @brief pointer to latest image*/
    cv::Mat latest_k4a_image_hls_; /** @brief latest image converted to hls*/
    cv::Mat* const latest_k4a_image_hls__ptr_ = &latest_k4a_image_hls_; /** @brief pointer to latest hls image*/
    cv_bridge::CvImageConstPtr k4aCvImagePtr_; /** @brief pointer to convert ROS image to OpenCV mat*/
    std::mutex latest_k4a_image_mutex_; /** @brief mutex to protect latest image*/
    azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode k4a_error_code_; /** @brief error codes*/

    // config file info
    // TODO: PULL THESE VALUES FROM CONFIG FILE "azure_kinect_ros_driver/AzureKinectParamsConfig.h"
    const uint32_t[12] EXPOSURES_ = { 488, 977, 1953, 3906, 7813, 15625, 31250, 62500, 125000, 250000, 500000, 1000000 };
    const uint32_t MIN_EXPOSURE_ = EXPOSURES_[0];
    const uint32_t MAX_EXPOSURE_ = EXPOSURES_[11];
    const uint32_t DEFAULT_EXPOSURE_ = EXPOSURES_[5];

    const uint16_t MIN_WHITE_BALANCE_ = 2500;
    const uint16_t MAX_WHITE_BALANCE_ = 12500;
    const uint16_t DEFAULT_WHITE_BALANCE_ = 4500;
    const uint16_t WHITE_BALANCE_INC_ = 10;

    const double LEARNING_RATE_ = 0.001; // learning rate for SGD
    const uint16_t MAX_ITERATIONS_ = 5000; // max iterations for SGD
};
