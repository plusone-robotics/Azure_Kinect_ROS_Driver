// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// System headers
//
#include <sstream>

// Library headers
//
#include <ros/ros.h>
#include <k4a/k4a.h>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_device.h"
#include "azure_kinect_ros_driver/k4a_exposure_calibration_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "k4a_bridge");

  // Setup the K4A device
  std::shared_ptr<K4AROSDevice> device(new K4AROSDevice);

  k4a_result_t result = device->startCameras();

  if (result != K4A_RESULT_SUCCEEDED)
  {
    ROS_ERROR_STREAM("Failed to start cameras");
    return -1;
  }

  ROS_INFO("K4A Started");

  if (result == K4A_RESULT_SUCCEEDED)
  {
    // Set up POR node
    ros::NodeHandle nh;
    K4AExposureCalibration por_calibrator(new K4AExposureCalibration(nh));
    ROS_INFO("Initialized K4A Exposure Calibration Node");
    
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    ROS_INFO("ROS Exit Started");
  }

  device.reset();

  ROS_INFO("ROS Exit");

  ros::shutdown();

  ROS_INFO("ROS Shutdown complete");

  return 0;
}