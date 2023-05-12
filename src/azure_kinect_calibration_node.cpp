// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// System headers
//
#include <sstream>

// Library headers
//
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <k4a/k4a.h>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_device.h"

void azureCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("azure_kinect_calibration heard: [%s]", msg->data.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "azure_kinect_calibration");

  ros::NodeHandle nh;
  ros::Subscriber subP2 = nh.subscribe("points2", 1000, azureCallback);
  ros::Subscriber subRGBRaw = nh.subscribe("/rgb/raw/image", 1000, azureCallback);

  ros::spin();

  return 0;
}