// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Library headers
#include <gtest/gtest.h>

// Project headers
#include "azure_kinect_ros_driver/k4a_exposure_calibration.h"

TEST(ExposureCalibrationTest, CameraExposureUpdateCheckTest)
{
  K4AExposureCalibration test_node;

  int8_t test_k4aExposureServiceErrorCode_chExp;
  std::string test_message_chExp = "";

  // test accurately changed appropriate exposure value 1000
  const uint32_t req_1000 = 1000;
  bool chExp = test_node.k4aCameraExposureUpdateCheck(req_1000, 1000, test_k4aExposureServiceErrorCode_chExp, test_message_chExp);
  ASSERT_TRUE(chExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_chExp == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_chExp, "Exposure update successful");

  // test unchanged appropriate exposure value (requested 1000, updated 15625 [default])
  int8_t test_k4aExposureServiceErrorCode_unchExp;
  std::string test_message_unchExp = "";

  bool unchExp = test_node.k4aCameraExposureUpdateCheck(req_1000, 15625, test_k4aExposureServiceErrorCode_unchExp, test_message_unchExp);
  ASSERT_FALSE(unchExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_unchExp == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::CAMERA_PARAM_SET_FAILURE);
  ASSERT_EQ(test_message_unchExp, "Failed to update exposure");
}

TEST(ExposureCalibrationTest, CameraExposureBoundsCheckTest)
{
  K4AExposureCalibration test_node;

  int8_t test_k4aExposureServiceErrorCode_inExp;
  std::string test_message_inExp = "";

  // test in bounds exposure value 1000
  const uint32_t req_1000 = 1000;
  bool inExp = test_node.k4aCameraExposureBoundsCheck(req_1000, test_k4aExposureServiceErrorCode_inExp, test_message_inExp);
  ASSERT_TRUE(inExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_inExp == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_inExp, "");

  // test out of bounds exposure value 450
  const uint32_t req_450 = 450;
  int8_t test_k4aExposureServiceErrorCode_outLowExp;
  std::string test_message_outLowExp = "";

  bool outLowExp = test_node.k4aCameraExposureBoundsCheck(req_450, test_k4aExposureServiceErrorCode_outLowExp, test_message_outLowExp);
  ASSERT_FALSE(outLowExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_outLowExp == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_PARAM_OUT_OF_BOUNDS_FAILURE);
  ASSERT_EQ(test_message_outLowExp, "Requested exposure out of range");

  // test out of bounds exposure value 1000005
  const uint32_t req_1000005 = 1000005;
  int8_t test_k4aExposureServiceErrorCode_outHighExp;
  std::string test_message_outHighExp = "";

  bool outHighExp = test_node.k4aCameraExposureBoundsCheck(req_1000005, test_k4aExposureServiceErrorCode_outHighExp, test_message_outHighExp);
  ASSERT_FALSE(outHighExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_outHighExp == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_PARAM_OUT_OF_BOUNDS_FAILURE);
  ASSERT_EQ(test_message_outHighExp, "Requested exposure out of range");
}

TEST(ExposureCalibrationTest, CameraWhiteBalanceUpdateCheckTest)
{
  K4AExposureCalibration test_node;

  int8_t test_k4aExposureServiceErrorCode_chWB;
  std::string test_message_chWB = "";

  // test accurately changed appropriate white balance value 3000
  const uint16_t req_3000 = 3000;
  bool chWB = test_node.k4aCameraWhiteBalanceUpdateCheck(req_3000, 3000, test_k4aExposureServiceErrorCode_chWB, test_message_chWB);
  ASSERT_TRUE(chWB);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_chWB == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_chExp, "White balance update successful");

  // test unchanged appropriate white balance value (requested 3000, updated 4500 [default])
  int8_t test_k4aExposureServiceErrorCode_unchWB;
  std::string test_message_unchWB = "";

  bool unchWB = test_node.k4aCameraWhiteBalanceUpdateCheck(req_3000, 4500, test_k4aExposureServiceErrorCode_unchWB, test_message_unchWB);
  ASSERT_FALSE(unchWB);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_unchWB == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::CAMERA_PARAM_SET_FAILURE);
  ASSERT_EQ(test_message_unchExp, "Failed to update white balance");
}

TEST(ExposureCalibrationTest, CameraWhiteBalanceBoundsCheckTest)
{
  K4AExposureCalibration test_node;

  int8_t test_k4aExposureServiceErrorCode_inWB;
  std::string test_message_inWB = "";

  // test in bounds white balance value 3000
  const uint16_t req_3000 = 3000;
  bool inWB = test_node.k4aCameraExposureBoundsCheck(req_3000, test_k4aExposureServiceErrorCode_inWB, test_message_inWB);
  ASSERT_TRUE(inWB);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_inWB == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_inWB, "");

  // test out of bounds white balance value 2000
  const uint16_t req_2000 = 2000;
  int8_t test_k4aExposureServiceErrorCode_outLowWB;
  std::string test_message_outLowWB = "";

  bool outLowWB = test_node.k4aCameraExposureBoundsCheck(req_2000, test_k4aExposureServiceErrorCode_outLowWB, test_message_outLowWB);
  ASSERT_FALSE(outLowWB);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_outLowWB == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_PARAM_OUT_OF_BOUNDS_FAILURE);
  ASSERT_EQ(test_message_outLowWB, "Requested white balance out of range");

  // test out of bounds exposure value 13000
  const uint16_t req_13000 = 13000;
  int8_t test_k4aExposureServiceErrorCode_outHighWB;
  std::string test_message_outHighWB = "";

  bool outHighWB = test_node.k4aCameraExposureBoundsCheck(req_13000, test_k4aExposureServiceErrorCode_outHighWB, test_message_outHighWB);
  ASSERT_FALSE(outHighWB);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_outHighWB == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_PARAM_OUT_OF_BOUNDS_FAILURE);
  ASSERT_EQ(test_message_outHighWB, "Requested white balance out of range");
}

TEST(ExposureCalibrationTest, TargetBlueCheckTest)
{
  K4AExposureCalibration test_node;
  
  // test blue target met
  const uint8_t req_target_blue = 100;
  int8_t test_k4aExposureServiceErrorCode_bMet;
  std::string test_message_bMet = "";
  bool chBlueMet = test_node.k4aTargetBlueCheck(req_target_blue, 100, test_k4aExposureServiceErrorCode_bMet, test_message_bMet);
  ASSERT_TRUE(chBlueMet);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_bMet == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_bMet, "Successfully calibrated exposure for target blue value");

  // test blue target exceeded
  int8_t test_k4aExposureServiceErrorCode_bExc;
  std::string test_message_bExc = "";
  bool chBlueExc = test_node.k4aTargetBlueCheck(req_target_blue, 200, test_k4aExposureServiceErrorCode_bExc, test_message_bExc);
  ASSERT_TRUE(chBlueExc);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_bExc == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_bExc, "Successfully calibrated exposure for target blue value");

  // test blue target not met
  int8_t test_k4aExposureServiceErrorCode_bNot;
  std::string test_message_bNot = "";
  bool chBlueNot = test_node.k4aTargetBlueCheck(req_target_blue, 0, test_k4aExposureServiceErrorCode_bNot, test_message_bNot);
  ASSERT_FALSE(chBlueNot);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_bNot == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_BLUE_VALUE_NOT_MET);
  ASSERT_EQ(test_message_bNot, "");
}

TEST(ExposureCalibrationTest, ImagePopulatedCheckTest)
{
  K4AExposureCalibration test_node;

  // test populated cv::Mat
  int8_t test_k4aExposureServiceErrorCode_imagePop;
  std::string test_message_imagePop = "";
  cv::Mat popImage(480, 640, CV_8UC3, cv::Scalar(50, 0, 0));
  bool imagePop = test_node.k4aImagePopulatedCheck(popImage, test_k4aExposureServiceErrorCode_imagePop, test_message_imagePop);
  ASSERT_TRUE(imagePop);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_imagePop == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_imagePop, "OpenCV mat is populated");

  // test empty cv::Mat
  int8_t test_k4aExposureServiceErrorCode_imageEmpty;
  std::string test_message_imageEmpty = "";
  cv::Mat emptyImage;
  bool imageEmpty = test_node.k4aImagePopulatedCheck(emptyImage, test_k4aExposureServiceErrorCode_imageEmpty, test_message_imageEmpty);
  ASSERT_FALSE(imageEmpty);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_imageEmpty == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::IMAGE_NOT_RECEIVED_FAILURE);
  ASSERT_EQ(test_message_imageEmpty, "OpenCV mat is empty");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "k4a_exposure_calibration_tests");
  
  return RUN_ALL_TESTS();
}
