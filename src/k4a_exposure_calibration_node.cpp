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

// Associated headers
#include "azure_kinect_ros_driver/AzureKinectParamsConfig.h"
#include "azure_kinect_ros_driver/k4a_update_exposure.h"
#include "azure_kinect_ros_driver/k4a_auto_tune_exposure.h"

// make memory space to store latest image
cv::Mat latest_k4a_image;
cv::Mat* latest_k4a_image_ptr = &latest_k4a_image;
// do we need to fetch latest image? true = yes
// see sensor_manager.cpp lines 464/2018
bool fetch_latest_k4a_image = true;

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
  // after updating the exposure we will need an updated latest image
  fetch_latest_k4a_image = true;
  return true;
}

// auto tune exposure with given target blue value
bool k4aAutoTuneExposure(int target_blue_value)
{
  ROS_ERROR("Starting auto exposure tuning...");
  // exposure loop
  int total_blue = 0;
  for(int exp=488; exp<1000000; exp+=2)
  {
    // split OpenCV mat into three color channels
    cv::Mat color_channels[3];
    cv::split(*latest_k4a_image_ptr, color_channels);
    // reminders: default exposure is 15625, min 488 max 1000000
    bool autoTune = k4aUpdateExposure(exp);
    int rows = latest_k4a_image.rows;
    int cols = latest_k4a_image.cols;
    int pixel_count = rows * cols;
    if(!autoTune)
    {
      ROS_ERROR("Unable to update exposure in k4aAutoTuneExposure");
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
      ROS_ERROR("Successfully calibrated exposure for blue value of [%d]", target_blue_value);
      // after updating the exposure we will need an updated latest image
      fetch_latest_k4a_image = true;
      break;
    }
  }
  // after updating the exposure we will need an updated latest image
  fetch_latest_k4a_image = true;
  return true;
}

bool k4aUpdateExposureCallback(azure_kinect_ros_driver::k4a_update_exposure::Request &req,
                               azure_kinect_ros_driver::k4a_update_exposure::Response &res)
{
  // prepare response
  res.message = "";

  ROS_INFO("Received exposure tuning request: [%d]", req.new_exp);

  // check exposure limits
  uint32_t req_exposure = req.new_exp;
  uint32_t min_exposure = 488;
  uint32_t max_exposure = 1000000;

  if(req_exposure < min_exposure || req_exposure > max_exposure)
  {
    res.message += ("Requested exposure out of range [%d] - [%d]", min_exposure, max_exposure);
    res.success = false;
    return true;
  }
  
  bool tuningRes = k4aUpdateExposure(req.new_exp);
  
  ROS_INFO("Sending back response...");

  if(!tuningRes)
  {
    res.success = false;
    res.message += "Unable to update exposure_time, k4aUpdateExposure failed";
    return true;
  }
  else
  {
    res.success = true;
    res.updated_exp = req.new_exp;
    res.message += "Exposure updated";
    // after updating the exposure we will need an updated latest image
    fetch_latest_k4a_image = true;
    return true;
  }
}

// insert lil wayne noises
bool k4aAutoTuneExposureCallback(azure_kinect_ros_driver::k4a_auto_tune_exposure::Request &req,
                                 azure_kinect_ros_driver::k4a_auto_tune_exposure::Response &res)
{
  // prepare response
  res.message = "";

  ROS_INFO("Received exposure auto tuning request: [%d]", req.target_blue_val);

  // check exposure limits
  uint32_t req_blue = req.target_blue_val;
  uint32_t min_blue = 0;
  uint32_t max_blue = 255;

  if(req_blue < min_blue || req_blue > max_blue)
  {
    res.message += ("Requested blue out of range [%d] - [%d]", min_blue, max_blue);
    res.success = false;
    return true;
  }
  
  bool autoTuningRes = k4aAutoTuneExposure(req.target_blue_val);
  
  ROS_INFO("Sending back response...");

  if(!autoTuningRes)
  {
    res.success = false;
    res.message += "Unable to auto tune exposure_time, k4aAutoTuneExposure failed";
    return true;
  }
  else
  {
    res.success = true;
    res.message += "Exposure updated";
    // after updating the exposure we will need an updated latest image
    fetch_latest_k4a_image = true;
    return true;
  }
}

void p2Callback(const sensor_msgs::PointCloud2& msg)
{
  ROS_DEBUG("exposure_calibration subscribed to /points2");
}

void rgbRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("exposure_calibration subscribed to /rgb/raw/image");
  // do we need to update our image
  //if(fetch_latest_k4a_image)
  //{
    try
    {
      // convert ROS image message to OpenCV
      cv_bridge::CvImageConstPtr CvImagePtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      *latest_k4a_image_ptr = CvImagePtr->image;

      // check if conversion worked
      if(latest_k4a_image.empty())
      {
        ROS_ERROR("Failed to convert k4a rgb image to OpenCV mat");
      }
      else
      {
        ROS_ERROR("PRINTING IMAGE FOR FUNSIES");
        cv::imshow("Latest Image", latest_k4a_image);
        cv::waitKey(0);
        fetch_latest_k4a_image = false;
      }
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: [%s]", e.what());
    }
  //}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "k4a_exposure_calibration");
  
  ROS_INFO("Initialized Exposure Calibration");
  
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  ros::Subscriber subPC = nh.subscribe("/points2", 1, p2Callback);
  image_transport::Subscriber subRGBRaw = it.subscribe("/rgb/raw/image", 1, rgbRawImageCallback);
  ROS_INFO("Exposure Calibration subscribed to /points2 and /rgb/raw/image");

  // Advertise services
  ros::ServiceServer update_exposure_service = nh.advertiseService("k4a_update_exposure", k4aUpdateExposureCallback);
  ros::ServiceServer auto_tune_exposure_service = nh.advertiseService("k4a_auto_tune_exposure", k4aAutoTuneExposureCallback);
  
  ros::Rate r(0.2);
  r.sleep();

  ROS_INFO("Spinning Exposure Calibration Node");
  ros::spin();
  return 0;
}
