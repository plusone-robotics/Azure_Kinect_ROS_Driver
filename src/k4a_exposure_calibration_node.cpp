// Plus One Robotics
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

void p2Callback(const sensor_msgs::PointCloud2& msg)
{
  ROS_DEBUG("exposure_calibration hearing /points2");
}

void rgbRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("exposure_calibration hearing /rgb/raw/image");
  try
  {
    // convert ROS image message to OpenCV
    cv_bridge::CvImageConstPtr CvImagePtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat cvImage = CvImagePtr->image;

    ROS_ERROR("PRINTING IMAGE FOR FUNSIES");
    cv::imshow("RGB Raw Image", cvImage);
    cv::waitKey(1);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: [%s]", e.what());
  }
}

// convert k4a image to OpenCV mat
cv::Mat convertk4aToOpenCV(k4a_image_t k4aImage)
{
  k4a_image_format_t format = k4a_image_get_format

}

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "k4a_exposure_calibration");
  
  ROS_INFO("Initialized Exposure Calibration");
  
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  ros::Subscriber subPC = nh.subscribe("/points2", 100, p2Callback);
  image_transport::Subscriber subRGBRaw = nh.subscribe("/rgb/raw/image", 100, rgbRawImageCallback);
  ROS_INFO("Exposure Calibration subscribed to /points2 and /rgb/raw/image");

  // Advertise calibrate_exposure service
  ros::ServiceServer service = nh.advertiseService("k4a_exposure_tuning", rosk4aExposureTuningCallback);
  
  ros::Rate r(0.2);
  r.sleep();

  ROS_INFO("Spinning Exposure Calibration Node");
  ros::spin();
  return 0;
}
