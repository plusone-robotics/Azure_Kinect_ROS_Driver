// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Library headers
#include <gtest/gtest.h>
#include <dynamic_reconfigure/Reconfigure.h>

// Project headers
#include "azure_kinect_ros_driver/k4a_exposure_calibration_node.h"

void publishk4aFakeImageData(ros::NodeHandle& pub_nh)
{
  // Create a fake RGB image (BGR8 format)
  cv::Mat k4aFakeImage;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(50, 0, 0));
  k4aFakeImage = image.clone();

  // Convert the image to ROS sensor_msgs/Image
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = ros::Time::now();
  cvImage.encoding = "bgr8";
  cvImage.image = image;
  sensor_msgs::ImagePtr msg = cvImage.toImageMsg();

  // Publish the fake image data
  image_transport::ImageTransport it(pub_nh);
  image_transport::Publisher pub = it.advertise("/rgb/raw/image", 1);
  pub.publish(msg);
}

// Mock implementation of dynamic reconfigure callback for k4aUpdateExposure
bool k4aTestMockReconfigure(dynamic_reconfigure::Reconfigure::Request& req,
                            dynamic_reconfigure::Reconfigure::Response& res)
{
  dynamic_reconfigure::IntParameter updated_exposure_param;
  updated_exposure_param.name = "exposure_time";
  for(const auto& param : req.config.ints)
  {
    if(param.name == "exposure_time")
    {
      updated_exposure_param.value = param.value;
      break;
    }
  }
  res.config.ints.push_back(updated_exposure_param);

  return true;
}

TEST(ExposureCalibrationTest, CameraExposureUpdateCheckTest)
{
  ros::NodeHandle nh;
  K4AExposureCalibration test_node(nh);

  int test_k4aExposureServiceErrorCode_chExp;
  std::string test_message_chExp = "";

  // test accurately changed appropriate exposure value 1000
  bool chExp = test_node.k4aCameraExposureUpdateCheck(1000, 1000, test_k4aExposureServiceErrorCode_chExp, test_message_chExp);
  ASSERT_TRUE(chExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_chExp == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_chExp, "Exposure update successful");

  // test unchanged appropriate exposure value (requested 1000, updated 15625 [default])
  int test_k4aExposureServiceErrorCode_unchExp;
  std::string test_message_unchExp = "";

  bool unchExp = test_node.k4aCameraExposureUpdateCheck(1000, 15625, test_k4aExposureServiceErrorCode_unchExp, test_message_unchExp);
  ASSERT_FALSE(unchExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_unchExp == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::CAMERA_EXPOSURE_SET_FAILURE);
  ASSERT_EQ(test_message_unchExp, "Failed to update exposure");
}

TEST(ExposureCalibrationTest, CameraExposureBoundsCheckTest)
{
  ros::NodeHandle nh;
  K4AExposureCalibration test_node(nh);

  int test_k4aExposureServiceErrorCode_inExp;
  std::string test_message_inExp = "";

  // test in bounds exposure value 1000
  bool inExp = test_node.k4aCameraExposureBoundsCheck(1000, test_k4aExposureServiceErrorCode_inExp, test_message_inExp);
  ASSERT_TRUE(inExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_inExp == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_inExp, "");

  // test out of bounds exposure value 450
  int test_k4aExposureServiceErrorCode_outLowExp;
  std::string test_message_outLowExp = "";

  bool outLowExp = test_node.k4aCameraExposureBoundsCheck(450, test_k4aExposureServiceErrorCode_outLowExp, test_message_outLowExp);
  ASSERT_FALSE(outLowExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_outLowExp == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_EXPOSURE_OUT_OF_BOUNDS_FAILURE);
  ASSERT_EQ(test_message_outLowExp, "Requested exposure out of range");

  // test out of bounds exposure value 1000005
  int test_k4aExposureServiceErrorCode_outHighExp;
  std::string test_message_outHighExp = "";

  bool outHighExp = test_node.k4aCameraExposureBoundsCheck(1000005, test_k4aExposureServiceErrorCode_outHighExp, test_message_outHighExp);
  ASSERT_FALSE(outHighExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_outHighExp == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_EXPOSURE_OUT_OF_BOUNDS_FAILURE);
  ASSERT_EQ(test_message_outHighExp, "Requested exposure out of range");
}

TEST(ExposureCalibrationTest, TargetBlueCheckTest)
{
  ros::NodeHandle nh;
  K4AExposureCalibration test_node(nh);
  
  // test blue target met
  int test_k4aExposureServiceErrorCode_bMet;
  std::string test_message_bMet = "";
  bool chBlueMet = test_node.k4aTargetBlueCheck(100, 100, test_k4aExposureServiceErrorCode_bMet, test_message_bMet);
  ASSERT_TRUE(chBlueMet);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_bMet == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_bMet, "Successfully calibrated exposure for target blue value");

  // test blue target exceeded
  int test_k4aExposureServiceErrorCode_bExc;
  std::string test_message_bExc = "";
  bool chBlueExc = test_node.k4aTargetBlueCheck(100, 200, test_k4aExposureServiceErrorCode_bExc, test_message_bExc);
  ASSERT_TRUE(chBlueExc);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_bExc == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_bExc, "Successfully calibrated exposure for target blue value");

  // test blue target not met
  int test_k4aExposureServiceErrorCode_bNot;
  std::string test_message_bNot = "";
  bool chBlueNot = test_node.k4aTargetBlueCheck(100, 0, test_k4aExposureServiceErrorCode_bNot, test_message_bNot);
  ASSERT_FALSE(chBlueNot);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_bNot == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_BLUE_VALUE_NOT_MET);
  ASSERT_EQ(test_message_bNot, "");
}

TEST(ExposureCalibrationTest, BlueBoundsCheckTest)
{
  ros::NodeHandle nh;
  K4AExposureCalibration test_node(nh);

  int test_k4aExposureServiceErrorCode_inBlue;
  std::string test_message_inBlue = "";

  // test in bounds blue value 100
  bool inBlue = test_node.k4aBlueBoundsCheck(100, test_k4aExposureServiceErrorCode_inBlue, test_message_inBlue);
  ASSERT_TRUE(inBlue);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_inBlue == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_inBlue, "");

  // test out of bounds blue value -1
  int test_k4aExposureServiceErrorCode_outLowBlue;
  std::string test_message_outLowBlue = "";

  bool outLowBlue = test_node.k4aBlueBoundsCheck(-1, test_k4aExposureServiceErrorCode_outLowBlue, test_message_outLowBlue);
  ASSERT_FALSE(outLowBlue);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_outLowBlue == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_BLUE_VALUE_OUT_OF_BOUNDS_FAILURE);
  ASSERT_EQ(test_message_outLowBlue, "Requested target blue value out of range");

  // test out of bounds blue value 260
  int test_k4aExposureServiceErrorCode_outHighBlue;
  std::string test_message_outHighBlue = "";

  bool outHighBlue = test_node.k4aBlueBoundsCheck(260, test_k4aExposureServiceErrorCode_outHighBlue, test_message_outHighBlue);
  ASSERT_FALSE(outHighBlue);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_outHighBlue == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_BLUE_VALUE_OUT_OF_BOUNDS_FAILURE);
  ASSERT_EQ(test_message_outHighBlue, "Requested target blue value out of range");
}

TEST(ExposureCalibrationTest, ImagePopulatedCheckTest)
{
  ros::NodeHandle nh;
  K4AExposureCalibration test_node(nh);

  // test populated cv::Mat
  int test_k4aExposureServiceErrorCode_imagePop;
  std::string test_message_imagePop = "";
  cv::Mat popImage(480, 640, CV_8UC3, cv::Scalar(50, 0, 0));
  bool imagePop = test_node.k4aImagePopulatedCheck(popImage, test_k4aExposureServiceErrorCode_imagePop, test_message_imagePop);
  ASSERT_TRUE(imagePop);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_imagePop == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_imagePop, "OpenCV mat is populated");

  // test empty cv::Mat
  int test_k4aExposureServiceErrorCode_imageEmpty;
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