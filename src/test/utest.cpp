// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Library headers
#include <gtest/gtest.h>

// Project headers
#include "azure_kinect_ros_driver/k4a_exposure_calibration_node.h"

TEST(ExposureCalibrationTest, CameraExposureUpdateCheckTest)
{
  ROS_ERROR("SANITY CHECK: CompareExposureTest");
  
  K4AExposureCalibration test_node_chExp;
  
  ROS_ERROR("SANITY CHECK: CompareExposureTest, test_node created");

  int test_k4aExposureServiceErrorCode_chExp;
  std::string test_message_chExp = "";

  // test accurately changed appropriate exposure value 1000
  ROS_ERROR("SANITY CHECK: CompareExposureTest, about to call k4aCompare for okExp");
  bool chExp = test_node_chExp.k4aCameraExposureUpdateCheck(1000, 1000, test_k4aExposureServiceErrorCode, test_message);
  ROS_ERROR("SANITY CHECK: CompareExposureTest, called k4aCompareExposure for okExp");
  ASSERT_TRUE(chExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_chExp == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_chExp, "Exposure update successful");

  K4AExposureCalibration test_node_unchExp;

  int test_k4aExposureServiceErrorCode_unchExp;
  std::string test_message_unchExp = "";

  // test unchanged appropriate exposure value (requested 1000, updated 15625 [default])
  ROS_ERROR("SANITY CHECK: CompareExposureTest, about to call k4aCompare for unchExp");
  bool unchExp = test_node_unchExp.k4aCameraExposureUpdateCheck(1000, 15625, test_k4aExposureServiceErrorCode, test_message);
  ROS_ERROR("SANITY CHECK: CompareExposureTest, called k4aCompareExposure for unchExp");
  ASSERT_FALSE(unchExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_unchExp == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::CAMERA_EXPOSURE_SET_FAILURE);
  ASSERT_EQ(test_message_unchExp, "Failed to update exposure");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "k4a_exposure_calibration_tests");
  
  return RUN_ALL_TESTS();
}