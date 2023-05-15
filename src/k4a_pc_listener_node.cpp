#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void p2Callback(const sensor_msgs::PointCloud2& msg)
{
  // ROS_INFO("points2_listener heard: [%s]", msg->data.c_str());
  ROS_INFO("we hearin things");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points2_listener");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("points2", 100, p2Callback);

  ros::spin();
  return 0;
}