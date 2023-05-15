#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

void p2Callback(const sensor_msgs::PointCloud2& msg)
{
  ROS_ERROR("calibration_listener (/points2) heard: [%s]", msg->data.c_str());
}

void rgbRawCallback(const sensor_msgs::Image& msg)
{
  ROS_ERROR("calibration_listener (/rgb/raw/image) heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_listener");
  
  ROS_ERROR("initialized calibration listener");
  
  ros::NodeHandle nh;
  ros::Subscriber subPC = nh.subscribe("/points2", 100, p2Callback);
  ros::Subscriber subRGBRaw = nh.subscribe("/rgb/raw/image", 100, rgbRawCallback);
  
  ROS_ERROR("should have subscribed by now");
  
  ros::spin();
  return 0;
}