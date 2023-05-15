#include "ros/ros.h"
#include "std_msgs/String.h"

void p2Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("points2_listener heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points2_listener");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud2>("points2", 1, p2Callback);

  ros::spin();
  return 0;
}