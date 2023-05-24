// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Associated headers
#include "azure_kinect_ros_driver/AzureKinectParamsConfig.h"
#include "azure_kinect_ros_driver/k4a_exposure_tuning.h"

// Library headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>

bool firstk4aImageForExposureTuning = false;

bool k4aExposureTuning(int reqExposure)
{
  ROS_INFO("Adjusting exposure_time");

  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::IntParameter int_param;
  dynamic_reconfigure::Config conf;

  int_param.name = "exposure_time";
  int_param.value = reqExposure;
  conf.ints.push_back(int_param);

  srv_req.config = conf;
  ros::service::call("/k4a_nodelet_manager/set_parameters", srv_req, srv_resp);

  return true;
}

bool autoExposureTuning(cv::Mat &cvMat, int target_blue_value)
{
  ROS_ERROR("Starting auto exposure tuning...");
  // split OpenCV mat into three color channels
  cv::Mat color_channels[3];
  cv::split(cvMat, color_channels);
  // reminders: default exposure is 15625, min 488 max 1000000
  int total_blue = 0;
  int pixel_count = cvMat.rows * cvMat.cols;
  // exposure loop
  for(int exp=488; exp<1000000; exp+=500)
  {
    bool autoTune = k4aExposureTuning(exp);
    if(!autoTune)
    {
      ROS_ERROR("Unable to update exposure in autoExposureTuning");
      return false;
    }
    // calculate average blue value
    else{
      for(int i=0; i<cvMat.rows; i++)
      {
        for(int j=0; j<cvMat.cols; j++)
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
      ROS_ERROR("Successfully calibrated to blue value of [%d]", target_blue_value);
      break;
    }
  }

}

bool rosk4aExposureTuningCallback(azure_kinect_ros_driver::k4a_exposure_tuning::Request &req,
                                  azure_kinect_ros_driver::k4a_exposure_tuning::Response &res)
{
  // prepare response
  res.message = "";

  ROS_INFO("Received exposure tuning request: [%d]", req.new_exp);
  ROS_INFO("Requesting exposure update to: [%d]", req.new_exp);

  // check exposure limits
  uint32_t req_exposure = req.new_exp;
  uint32_t min_exposure = 488;
  uint32_t max_exposure = 1000000;

  if(req_exposure < min_exposure || req_exposure > max_exposure)
  {
    res.message += "Requested exposure out of range (488-1,000,000)";
    res.success = false;
    return true;
  }
  
  bool tuningRes = k4aExposureTuning(req.new_exp);
  
  ROS_INFO("Sending back response...");

  if(!tuningRes)
  {
    res.success = false;
    res.message += "Unable to change exposure_time";
    return true;
  }
  else
  {
    res.success = true;
    res.updated_exp = req.new_exp;
    res.message += "Exposure updated";
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
  if(!firstk4aImageForExposureTuning)
  {
    try
    {
      // convert ROS image message to OpenCV
      cv_bridge::CvImageConstPtr CvImagePtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat cvImage = CvImagePtr->image;

      // check if conversion worked
      if(cvImage.empty())
      {
        ROS_ERROR("Failed to convert k4a image to OpenCV mat");
      }
      else
      {
        ROS_ERROR("PRINTING IMAGE FOR FUNSIES");
        cv::imshow("First Image", cvImage);
        cv::waitKey(0);
        firstk4aImageForExposureTuning = true;
        int blue_value_requested = 255;
        ROS_ERROR("Attempting autoTune with blue value of [%d]", blue_value_requested);
        bool autoTuneAttempt = autoExposureTuning(cvImage, blue_value_requested);
      }
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: [%s]", e.what());
    }
  }
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

  // Advertise calibrate_exposure service
  ros::ServiceServer service = nh.advertiseService("k4a_exposure_tuning", rosk4aExposureTuningCallback);
  
  ros::Rate r(0.2);
  r.sleep();

  ROS_INFO("Spinning Exposure Calibration Node");
  ros::spin();
  return 0;
}
