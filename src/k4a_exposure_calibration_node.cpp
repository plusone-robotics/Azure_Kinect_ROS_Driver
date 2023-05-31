// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Library headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>
#include <mutex>

// Associated headers
#include "azure_kinect_ros_driver/AzureKinectParamsConfig.h"
#include "azure_kinect_ros_driver/k4a_update_exposure.h"
#include "azure_kinect_ros_driver/k4a_auto_tune_exposure.h"
#include "azure_kinect_ros_driver/k4aCameraExposureServiceErrorCode.h"

// allocate memory space to store latest image
cv::Mat latest_k4a_image;
cv::Mat* latest_k4a_image_ptr = &latest_k4a_image;
cv_bridge::CvImageConstPtr CvImagePtr;
// see sensor_manager.cpp lines 464/2018
std::mutex latest_k4a_image_mutex;

// call k4a_nodelet_manager/set_parameters to update exposure value
bool k4aUpdateExposure(int reqExposure)
{
  ROS_INFO("Updating exposure_time to: [%d]", reqExposure);

  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::IntParameter int_param;
  dynamic_reconfigure::Config conf;

  int_param.name = "exposure_time";
  int_param.value = reqExposure;
  conf.ints.push_back(int_param);
  srv_req.config = conf;
  ros::service::call("/k4a_nodelet_manager/set_parameters", srv_req, srv_resp);
  // check to make sure parameter was actually set
  std::string param_name;
  if(srv_resp.config.ints("exposure_time") == reqExposure)
  {
    ROS_INFO("Exposure successfully updated to [%d]", srv_resp.config.ints("exposure_time"));
    return true;
  }
  else
  {
    ROS_INFO("exposure_time update failed");
    return false;
  }
}

// auto tune exposure with given target blue value
bool k4aAutoTuneExposure(int target_blue_value, int& final_exposure, int& error_code, std::string& res_msg)
{
  ROS_INFO("Starting K4A auto exposure tuning...");

  // exposure loop
  int total_blue = 0;
  for(int exp=488; exp<1000000; exp+=500)
  {
    // split OpenCV mat into three color channels
    cv::Mat color_channels[3];
    (
    std::lock_guard<std::mutex> lock(latest_k4a_image_mutex);
    cv::split(*latest_k4a_image_ptr, color_channels);
    )
    int rows = latest_k4a_image.rows;
    int cols = latest_k4a_image.cols;
    int pixel_count = rows * cols;
    if(rows <= 0 || cols <= 0)
    {
      std::string error_msg = "Failed to retrieve latest image in k4aAutoTuneExposure";
      ROS_ERROR(error_msg);
      res_msg = error_msg;
      error_code = k4a_msgs::k4aCameraExposureServiceErrorCode::IMAGE_NOT_RECEIVED_FAILURE;
      final_exposure = 15625;
      return false;
    }

    bool autoTune = k4aUpdateExposure(exp);
    
    if(!autoTune)
    {
      std::string error_msg = "Unable to update exposure in k4aAutoTuneExposure";
      ROS_ERROR(error_msg);
      res_msg = error_msg;
      error_code = k4a_msgs::k4aCameraExposureServiceErrorCode::CAMERA_EXPOSURE_SET_FAILURE;
      final_exposure = 15625;
      return false;
    }
    
    // calculate average blue value
    else{
      for(int i=0; i<rows; i++)
      {
        for(int j=0; j<cols; j++)
        {
          total_blue += color_channels[0].at<uchar>(i,j);
        }
      }
    }
    // calculate average blue value
    int current_avg_blue_value = total_blue / pixel_count;
    // did we achieve appropriate blue at this exposure
    if(current_avg_blue_value >= target_blue_value)
    {
      std::string error_msg = ("Successfully calibrated exposure for blue value of [%d]", target_blue_value);
      ROS_ERROR(error_msg);
      res_msg = error_msg;
      error_code = k4a_msgs::k4aCameraExposureServiceErrorCode::SUCCESS;
      final_exposure = exp;
      break;
    }
  }
  return true;
}

bool k4aUpdateExposureCallback(azure_kinect_ros_driver::k4a_update_exposure::Request &req,
                               azure_kinect_ros_driver::k4a_update_exposure::Response &res)
{
  // prepare response
  res.message = "";

  ROS_INFO("Received K4A exposure update request: [%d]", req.new_exp);

  // check exposure limits
  uint32_t req_exposure = req.new_exp;
  uint32_t min_exposure = 488;
  uint32_t max_exposure = 1000000;

  if(req_exposure < min_exposure || req_exposure > max_exposure)
  {
    res.message += ("Requested exposure out of range [%d] - [%d]", min_exposure, max_exposure);
    res.k4aExposureServiceErrorCode = REQUESTED_CAMERA_EXPOSURE_OUT_OF_BOUNDS_FAILURE;
    return true;
  }
  
  bool tuningRes = k4aUpdateExposure(req.new_exp);
  
  ROS_INFO("Sending back response...");

  if(!tuningRes)
  {
    res.message += "Unable to update exposure_time, k4aUpdateExposure failed";
    res.k4aExposureServiceErrorCode = CAMERA_EXPOSURE_SET_FAILURE;
    return true;
  }
  else
  {
    res.message += "Exposure updated";
    res.k4aExposureServiceErrorCode = SUCCESS;
    return true;
  }
}

bool k4aAutoTuneExposureCallback(azure_kinect_ros_driver::k4a_auto_tune_exposure::Request &req,
                                 azure_kinect_ros_driver::k4a_auto_tune_exposure::Response &res)
{
  // prepare response
  res.message = "";
  int error_code, calibrated_exposure;
  std::string res_msg;

  ROS_INFO("Received K4A exposure auto tuning request for blue value: [%d]", req.target_blue_val);

  // check blue value limits
  uint32_t req_blue = req.target_blue_val;
  uint32_t min_blue = 0;
  uint32_t max_blue = 255;
  int final_exposure = 15625;
  int * final_exposure_ptr = &final_exposure;

  if(req_blue < min_blue || req_blue > max_blue)
  {
    res.message += ("Requested blue value out of range [%d] - [%d]", min_blue, max_blue);
    res.k4aExposureServiceErrorCode = REQUESTED_CAMERA_BLUE_VALUE_OUT_OF_BOUNDS_FAILURE;
    return true;
  }
  
  bool autoTuningRes = k4aAutoTuneExposure(req.target_blue_val, calibrated_exposure, error_code, res_msg);
  
  ROS_INFO("Sending back response...");

  if(!autoTuningRes)
  {
    res.message += "Unable to auto tune exposure_time, k4aAutoTuneExposure failed";
    res.k4aExposureServiceErrorCode = CAMERA_EXPOSURE_SET_FAILURE;
    return true;
  }
  else
  {
    res.message += "Exposure updated, target blue value achieved";
    res.k4aExposureServiceErrorCode = SUCCESS;
    return true;
  }
}

void p2Callback(const sensor_msgs::PointCloud2& msg)
{
  ROS_DEBUG("k4a_exposure_calibration_node subscribed to /points2");
}

void rgbRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("k4a_exposure_calibration_node subscribed to /rgb/raw/image");
  try
  {
    // convert ROS image message to OpenCV
    CvImagePtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    (
    std::lock_guard<std::mutex> lock(latest_k4a_image_mutex);
    *latest_k4a_image_ptr = CvImagePtr->image;
    )

    // check if conversion worked
    if(latest_k4a_image.empty())
    {
      ROS_ERROR("Failed to convert k4a rgb image to OpenCV mat");
    }
    else
    {
      ROS_DEBUG("Updated latest_k4a_image");
    }
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: [%s]", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "k4a_exposure_calibration");
  
  ROS_INFO("Initialized K4A Exposure Calibration Node");
  
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  ros::Subscriber subPC = nh.subscribe("/points2", 1, p2Callback);
  image_transport::Subscriber subRGBRaw = it.subscribe("/rgb/raw/image", 1, rgbRawImageCallback);
  ROS_INFO("K4A Exposure Calibration Node subscribed to /points2 and /rgb/raw/image");

  // Advertise services
  ros::ServiceServer update_exposure_service = nh.advertiseService("k4a_update_exposure", k4aUpdateExposureCallback);
  ros::ServiceServer auto_tune_exposure_service = nh.advertiseService("k4a_auto_tune_exposure", k4aAutoTuneExposureCallback);

  ROS_INFO("Spinning K4A Exposure Calibration Node");
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
