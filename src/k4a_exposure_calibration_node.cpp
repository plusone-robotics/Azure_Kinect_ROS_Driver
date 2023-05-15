#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "dynamic_reconfigure/server.h"
#include "Azure_Kinect_ROS_Driver/AzureKinectParams.h"

void p2Callback(const sensor_msgs::PointCloud2& msg)
{
  ROS_INFO("exposure_calibration hearing /points2");
}

void rgbRawCallback(const sensor_msgs::Image& msg)
{
  ROS_INFO("exposure_calibration hearing /rgb/raw/image");
}

void calibrateExposure(azure_kinect_ros_driver::AzureKinectParamsConfig &config, uint32_t level)
{
  ROS_ERROR("Reconfigure request: %d",
            config.exposure_time);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "k4a_exposure_calibration");
  
  ROS_INFO("Initialized Exposure Calibration");
  
  ros::NodeHandle nh;
  ros::Subscriber subPC = nh.subscribe("/points2", 100, p2Callback);
  ros::Subscriber subRGBRaw = nh.subscribe("/rgb/raw/image", 100, rgbRawCallback);
  
  ROS_INFO("Exposure Calibration subscribed to /points2 and /rgb/raw/image");
  
  dynamic_reconfigure::Server<azure_kinect_ros_driver::AzureKinectParamsConfig> server;
  dynamic_reconfigure::Server<azure_kinect_ros_driver::AzureKinectParamsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning Exposure Calibration Node");
  ros::spin();
  return 0;
}
