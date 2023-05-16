#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "dynamic_reconfigure/server.h"
#include "azure_kinect_ros_driver/AzureKinectParamsConfig.h"

void p2Callback(const sensor_msgs::PointCloud2& msg)
{
  ROS_INFO("exposure_calibration hearing /points2");
}

void rgbRawCallback(const sensor_msgs::Image& msg)
{
  ROS_INFO("exposure_calibration hearing /rgb/raw/image");
}

void exposureCalibrationCallback(azure_kinect_ros_driver::AzureKinectParamsConfig &config, uint32_t level)
{
  int exposure_time = config.exposure_time;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "k4a_exposure_calibration");
  
  ROS_INFO("Initialized Exposure Calibration");
  
  ros::NodeHandle nh;
  ros::Subscriber subPC = nh.subscribe("/points2", 100, p2Callback);
  ros::Subscriber subRGBRaw = nh.subscribe("/rgb/raw/image", 100, rgbRawCallback);
  
  ROS_INFO("Exposure Calibration subscribed to /points2 and /rgb/raw/image");
  
  ROS_INFO("Setting up dynamic reconfigure server");

  dynamic_reconfigure::Server<azure_kinect_ros_driver::AzureKinectParamsConfig> exposureCalibrationServer;
  dynamic_reconfigure::Server<azure_kinect_ros_driver::AzureKinectParamsConfig>::CallbackType f;
  f = boost::bind(&exposureCalibrationCallback, _1, _2);
  exposureCalibrationServer.setCallback(f);

  ROS_INFO("Looping through exposures");
  while(ros::ok)
  {
    for(int k4aExposureIncrement = 488; k4aExposureIncrement <= 1,000,000; k4aExposureIncrement += 1000)
    {
      ROS_ERROR("UPDATING EXPOSURE TO: [%d]", k4aExposureIncrement);
      nh.setParam("exposure_time", k4aExposureIncrement);
      ros::Duration(0.1).sleep();
    }
  }

  ROS_INFO("Spinning Exposure Calibration Node");
  ros::spin();
  return 0;
}
