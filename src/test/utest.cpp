// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Library headers
#include <gtest/gtest.h>

// Project headers
#include "azure_kinect_ros_driver/k4a_por_calibration.h"

TEST(PORCalibrationTest, KinectStandardizeExposureTest)
{
  const uint32_t EXPOSURE_OPTIONS[12] = { 488, 977, 1953, 3906, 7813, 15625, 31250, 62500, 125000, 250000, 500000, 1000000 };

  K4APORCalibration test_node;
  // _# refers to what index this value should evaluate to
  const uint32_t zero = 488;
  const uint32_t zero_0 = 732;
  const uint32_t zero_1 = 733;
  const uint32_t one = 977;
  const uint32_t one_1 = 1464;
  const uint32_t one_2 = 1465;
  const uint32_t two = 1953;
  const uint32_t two_2 = 2929;
  const uint32_t two_3 = 2930;
  const uint32_t three = 3906;
  const uint32_t three_3 = 5859;
  const uint32_t three_4 = 5860;
  const uint32_t four = 7813;
  const uint32_t four_4 = 11718;
  const uint32_t four_5 = 11719;
  const uint32_t five = 15625;
  const uint32_t five_5 = 23437;
  const uint32_t five_6 = 23438;
  const uint32_t six = 31250;
  const uint32_t six_6 = 46874;
  const uint32_t six_7 = 46875;
  const uint32_t seven = 62500;
  const uint32_t seven_7 = 93749;
  const uint32_t seven_8 = 93750;
  const uint32_t eight = 125000;
  const uint32_t eight_8 = 187499;
  const uint32_t eight_9 = 187500;
  const uint32_t nine = 250000;
  const uint32_t nine_9 = 374999;
  const uint32_t nine_10 = 375000;
  const uint32_t ten = 500000;
  const uint32_t ten_10 = 749999;
  const uint32_t ten_11 = 750000;
  const uint32_t eleven = 1000000;

  const uint32_t result;

  // 488
  result = test_node.k4aStandardizeExposure(zero);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[0]);
  result = test_node.k4aStandardizeExposure(zero_0);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[0]);

  // 977
  result = test_node.k4aStandardizeExposure(zero_1);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[1]);
  result = test_node.k4aStandardizeExposure(one);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[1]);
  result = test_node.k4aStandardizeExposure(one_1);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[1]);

  // 1953
  result = test_node.k4aStandardizeExposure(one_2);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[2]);
  result = test_node.k4aStandardizeExposure(two);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[2]);
  result = test_node.k4aStandardizeExposure(two_2);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[2]);

  // 3906
  result = test_node.k4aStandardizeExposure(two_3);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[3]);
  result = test_node.k4aStandardizeExposure(three);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[3]);
  result = test_node.k4aStandardizeExposure(three_3);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[3]);

  // 7813
  result = test_node.k4aStandardizeExposure(three_4);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[4]);
  result = test_node.k4aStandardizeExposure(four);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[4]);
  result = test_node.k4aStandardizeExposure(four_4);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[4]);
  
  // 15625
  result = test_node.k4aStandardizeExposure(four_5);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[5]);
  result = test_node.k4aStandardizeExposure(five);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[5]);
  result = test_node.k4aStandardizeExposure(five_5);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[5]);
  
  // 31250
  result = test_node.k4aStandardizeExposure(five_6);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[6]);
  result = test_node.k4aStandardizeExposure(six);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[6]);
  result = test_node.k4aStandardizeExposure(six_6);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[6]);

  // 62500
  result = test_node.k4aStandardizeExposure(six_7);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[7]);
  result = test_node.k4aStandardizeExposure(seven);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[7]);
  result = test_node.k4aStandardizeExposure(seven_7);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[7]);
  
  // 125000
  result = test_node.k4aStandardizeExposure(seven_8);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[8]);
  result = test_node.k4aStandardizeExposure(eight);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[8]);
  result = test_node.k4aStandardizeExposure(eight_8);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[8]);

  // 250000
  result = test_node.k4aStandardizeExposure(eight_9);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[9]);
  result = test_node.k4aStandardizeExposure(nine);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[9]);
  result = test_node.k4aStandardizeExposure(nine_9);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[9]);
  
  // 500000
  result = test_node.k4aStandardizeExposure(nine_10);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[10]);
  result = test_node.k4aStandardizeExposure(ten);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[10]);
  result = test_node.k4aStandardizeExposure(ten_10);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[10]);
    
  // 1000000
  result = test_node.k4aStandardizeExposure(ten_11);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[11]);
  result = test_node.k4aStandardizeExposure(eleven);
  ASSERT_EQ(result, EXPOSURE_OPTIONS[11]);
}

TEST(PORCalibrationTest, CameraExposureUpdateCheckTest)
{
  K4APORCalibration test_node;

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

TEST(PORCalibrationTest, CameraExposureBoundsCheckTest)
{
  K4APORCalibration test_node;

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

TEST(PORCalibrationTest, CameraWhiteBalanceUpdateCheckTest)
{
  K4APORCalibration test_node;

  int8_t test_k4aExposureServiceErrorCode_chWB;
  std::string test_message_chWB = "";

  // test accurately changed appropriate white balance value 3000
  const uint16_t req_3000 = 3000;
  bool chWB = test_node.k4aCameraWhiteBalanceUpdateCheck(req_3000, 3000, test_k4aExposureServiceErrorCode_chWB, test_message_chWB);
  ASSERT_TRUE(chWB);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_chWB == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_chWB, "White balance update successful");

  // test unchanged appropriate white balance value (requested 3000, updated 4500 [default])
  int8_t test_k4aExposureServiceErrorCode_unchWB;
  std::string test_message_unchWB = "";

  bool unchWB = test_node.k4aCameraWhiteBalanceUpdateCheck(req_3000, 4500, test_k4aExposureServiceErrorCode_unchWB, test_message_unchWB);
  ASSERT_FALSE(unchWB);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_unchWB == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::CAMERA_PARAM_SET_FAILURE);
  ASSERT_EQ(test_message_unchWB, "Failed to update white balance");
}

TEST(PORCalibrationTest, CameraWhiteBalanceBoundsCheckTest)
{
  K4APORCalibration test_node;

  int8_t test_k4aExposureServiceErrorCode_inWB;
  std::string test_message_inWB = "";

  // test in bounds white balance value 3000
  const uint16_t req_3000 = 3000;
  bool inWB = test_node.k4aCameraWhiteBalanceBoundsCheck(req_3000, test_k4aExposureServiceErrorCode_inWB, test_message_inWB);
  ASSERT_TRUE(inWB);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_inWB == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_inWB, "");

  // test out of bounds white balance value 2000
  const uint16_t req_2000 = 2000;
  int8_t test_k4aExposureServiceErrorCode_outLowWB;
  std::string test_message_outLowWB = "";

  bool outLowWB = test_node.k4aCameraWhiteBalanceBoundsCheck(req_2000, test_k4aExposureServiceErrorCode_outLowWB, test_message_outLowWB);
  ASSERT_FALSE(outLowWB);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_outLowWB == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_PARAM_OUT_OF_BOUNDS_FAILURE);
  ASSERT_EQ(test_message_outLowWB, "Requested white balance out of range");

  // test out of bounds exposure value 13000
  const uint16_t req_13000 = 13000;
  int8_t test_k4aExposureServiceErrorCode_outHighWB;
  std::string test_message_outHighWB = "";

  bool outHighWB = test_node.k4aCameraWhiteBalanceBoundsCheck(req_13000, test_k4aExposureServiceErrorCode_outHighWB, test_message_outHighWB);
  ASSERT_FALSE(outHighWB);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_outHighWB == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_PARAM_OUT_OF_BOUNDS_FAILURE);
  ASSERT_EQ(test_message_outHighWB, "Requested white balance out of range");
}

TEST(PORCalibrationTest, TargetBlueCheckTest)
{
  K4APORCalibration test_node;
  
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

TEST(PORCalibrationTest, ImagePopulatedCheckTest)
{
  K4APORCalibration test_node;

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
  ros::init(argc, argv, "k4a_por_calibration_tests");
  
  return RUN_ALL_TESTS();
}
