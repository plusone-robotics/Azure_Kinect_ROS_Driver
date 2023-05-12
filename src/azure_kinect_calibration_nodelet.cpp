// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/azure_kinect_calibration_nodelet.h"

// System headers
//

// Library headers
//
#include <pluginlib/class_list_macros.h>

// Project headers
//

PLUGINLIB_EXPORT_CLASS(Azure_Kinect_ROS_Driver::AzureKinectCalibrationNodelet, nodelet::Nodelet)

namespace Azure_Kinect_ROS_Driver
{
AzureKinectCalibrationNodelet::AzureKinectCalibrationNodelet() : Nodelet(), camera_started_verification(bool)
{
}

AzureKinectCalibrationNodelet::~AzureKinectCalibrationNodelet()
{
}

void AzureKinectCalibrationNodelet::onInit()
{
  NODELET_INFO("Azure Kinect Calibration Nodelet Start");

  camera_started_verification = ros::param::get("camera_start_verification", true);

  if (!camera_started_verification)
  {
    throw nodelet::Exception("Could not start Azure Kinect Calibration");
  }

  NODELET_INFO("Azure Kinect Calibration started");
}
}  // namespace Azure_Kinect_ROS_Driver