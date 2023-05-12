// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef AZURE_KINECT_CALIBRATION_NODELET_H
#define AZURE_KINECT_CALIBRATION_NODELET_H

// System headers
//

// Library headers
//
#include <ros/ros.h>
#include <nodelet/nodelet.h>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_device.h"

namespace Azure_Kinect_ROS_Driver
{
class AzureKinectCalibrationNodelet : public nodelet::Nodelet
{
public:
  AzureKinectCalibrationNodelet();
  ~AzureKinectCalibrationNodelet();

  virtual void onInit();

private:
  std::bool camera_started_verification;
};
}  // namespace Azure_Kinect_ROS_Driver

#endif  // AZURE_KINECT_CALIBRATION_NODELET_H