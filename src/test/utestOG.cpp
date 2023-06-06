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
  ROS_ERROR("SANITY CHECK: publishk4aFakeImageData");
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
  ROS_ERROR("SANITY CHECK: k4aTestMockReconfigure");
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

void publishk4aBadImageData(ros::NodeHandle& pub_nh)
{
  // mimic failure to convert k4a image to OpenCV mat (empty mat)
  cv::Mat k4aFakeImage;
  cv::Mat image;
  k4aFakeImage = image.clone();

  // Convert the empty image to ROS sensor_msgs/Image
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

// Test case for k4aUpdateExposure in range
TEST(ExposureCalibrationTest, UpdateExposureInRangeTest)
{
  // Create a ROS node publisher handle
  ros::NodeHandle pub_nh;

  // Create a ROS node handle for testing class
  ros::NodeHandle test_nh;
  K4AExposureCalibration test_node(test_nh);

  // Publish fake image data
  publishk4aFakeImageData(pub_nh);

  // Call the k4aUpdateExposure function with a fake exposure value
  int error_code;
  std::string res_msg;
  bool result = test_node.k4aUpdateExposure(1000, error_code, res_msg);

  // Check the result
  EXPECT_TRUE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
}

// Test case for k4aUpdateExposure out of range (less than)
TEST(ExposureCalibrationTest, UpdateExposureOutOfRangeLTTest)
{
  // Create a ROS node publisher handle
  ros::NodeHandle pub_nh;
  
  // Create a ROS node handle for testing class
  ros::NodeHandle test_nh;
  K4AExposureCalibration test_node(test_nh);

  // Publish fake image data
  publishk4aFakeImageData(pub_nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aUpdateExposure function with a fake exposure value
  int error_code;
  std::string res_msg;
  bool result = test_node.k4aUpdateExposure(100, error_code, res_msg);

  // Check the result
  EXPECT_FALSE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_EXPOSURE_OUT_OF_BOUNDS_FAILURE);
  k4aFakeImage.release();
}

// Test case for k4aUpdateExposure out of range (greater than)
TEST(ExposureCalibrationTest, UpdateExposureOutOfRangeGTTest)
{
  // Create a ROS node publisher handle
  ros::NodeHandle pub_nh;
  
    // Create a ROS node handle for testing class
  ros::NodeHandle test_nh;
  K4AExposureCalibration test_node(test_nh);

  // Publish fake image data
  publishk4aFakeImageData(pub_nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aUpdateExposure function with a fake exposure value
  int error_code;
  std::string res_msg;
  bool result = test_node.k4aUpdateExposure(1000005, error_code, res_msg);

  // Check the result
  EXPECT_FALSE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_EXPOSURE_OUT_OF_BOUNDS_FAILURE);
  k4aFakeImage.release();
}

// Test case for k4aAutoTuneExposure
TEST(ExposureCalibrationTest, AutoTuneExposureTest)
{
  // Create a ROS node publisher handle
  ros::NodeHandle pub_nh;
  
  // Create a ROS node handle for testing class
  ros::NodeHandle test_nh;
  K4AExposureCalibration test_node(test_nh);

  // Publish fake image data
  publishk4aFakeImageData(pub_nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aAutoTuneExposure function with a fake target blue value
  int final_exposure, error_code;
  std::string res_msg;
  bool result = test_node.k4aAutoTuneExposure(150, final_exposure, error_code, res_msg);

  // Check the result
  EXPECT_TRUE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  EXPECT_GE(final_exposure, 488); // Assuming minimum exposure value
  EXPECT_LE(final_exposure, 1000000); // Assuming maximum exposure value
  k4aFakeImage.release();
}

// Test case for k4aAutoTuneExposure with empty image
TEST(ExposureCalibrationTest, AutoTuneExposureEmptyImageTest)
{
  // Create a ROS node publisher handle
  ros::NodeHandle pub_nh;
  
  // Create a ROS node handle for testing class
  ros::NodeHandle test_nh;
  K4AExposureCalibration test_node(test_nh);

  // Publish fake image data
  publishk4aBadImageData(pub_nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aAutoTuneExposure function with a fake target blue value
  int final_exposure, error_code;
  std::string res_msg;
  bool result = test_node.k4aAutoTuneExposure(150, final_exposure, error_code, res_msg);

  // Check the result
  EXPECT_FALSE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::IMAGE_NOT_RECEIVED_FAILURE);
  EXPECT_EQ(final_exposure, 0); // failure, never set
  k4aFakeImage.release();
}

// Test case for k4aAutoTuneExposure out of range (less than)
TEST(ExposureCalibrationTest, AutoTuneExposureLTTest)
{
  // Create a ROS node publisher handle
  ros::NodeHandle pub_nh;
  
  // Create a ROS node handle for testing class
  ros::NodeHandle test_nh;
  K4AExposureCalibration test_node(test_nh);

  // Publish fake image data
  publishk4aFakeImageData(pub_nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aAutoTuneExposure function with a fake target blue value
  int final_exposure, error_code;
  std::string res_msg;
  bool result = test_node.k4aAutoTuneExposure(-1, final_exposure, error_code, res_msg);

  // Check the result
  EXPECT_FALSE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_BLUE_VALUE_OUT_OF_BOUNDS_FAILURE);
  EXPECT_EQ(final_exposure, 0); // failure, never set
  k4aFakeImage.release();
}

// Test case for k4aAutoTuneExposure out of range (greater than)
TEST(ExposureCalibrationTest, AutoTuneExposureGTTest)
{
  // Create a ROS node publisher handle
  ros::NodeHandle pub_nh;
  
  // Create a ROS node handle for testing class
  ros::NodeHandle test_nh;
  K4AExposureCalibration test_node(test_nh);

  // Publish fake image data
  publishk4aFakeImageData(pub_nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aAutoTuneExposure function with a fake target blue value
  int final_exposure, error_code;
  std::string res_msg;
  bool result = test_node.k4aAutoTuneExposure(256, final_exposure, error_code, res_msg);

  // Check the result
  EXPECT_FALSE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_BLUE_VALUE_OUT_OF_BOUNDS_FAILURE);
  EXPECT_EQ(final_exposure, 0); // failure, never set
  k4aFakeImage.release();
}

TEST(ExposureCalibrationTest, azure_kinect_ros_driver_framework)
{
  ASSERT_TRUE(true);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "k4a_exposure_calibration_tests");
  
  return RUN_ALL_TESTS();
}