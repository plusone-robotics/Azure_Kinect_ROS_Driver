#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void p2Callback(const sensor_msgs::PointCloud2& msg)
{
  // ROS_INFO("points2_listener heard: [%s]", msg->data.c_str());
  ROS_ERROR("we hearin things");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points2_listener");
  
  ROS_ERROR("testing");
  
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/points2", 100, p2Callback);
  
  ROS_ERROR("should have subscribed by now");
  
  ros::spin();
  return 0;
}