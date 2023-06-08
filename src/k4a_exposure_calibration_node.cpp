// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Associated headers
#include "azure_kinect_ros_driver/k4a_exposure_calibration.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "k4a_exposure_calibration");
  ros::NodeHandle nh;
  
  K4AExposureCalibration k4a_Exposure_Calibration(nh);
  ROS_INFO("Initialized K4A Exposure Calibration Node");

  ROS_INFO("Spinning K4A Exposure Calibration Node");
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
