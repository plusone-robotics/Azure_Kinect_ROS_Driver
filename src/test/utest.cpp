// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "azure_kinect_ros_driver/k4a_exposure_calibration_node.h"

// fake image
cv::Mat k4aFakeImage;

void publishk4aFakeImageData(ros::NodeHandle& nh)
{
  // Create a fake RGB image (BGR8 format)
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(50, 0, 0));
  k4aFakeImage = image.clone();

  // Convert the image to ROS sensor_msgs/Image
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = ros::Time::now();
  cvImage.encoding = "bgr8";
  cvImage.image = image;
  sensor_msgs::ImagePtr msg = cvImage.toImageMsg();

  // Publish the fake image data
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/rgb/raw/image", 1);
  pub.publish(msg);
}

void publishk4aBadImageData(ros::NodeHandle& nh)
{
  // mimic failure to convert k4a image to OpenCV mat
  cv::Mat image;
  k4aFakeImage = image.clone();

  // Convert the empty image to ROS sensor_msgs/Image
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = ros::Time::now();
  cvImage.encoding = "bgr8";
  cvImage.image = image;
  sensor_msgs::ImagePtr msg = cvImage.toImageMsg();

  // Publish the fake image data
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/rgb/raw/image", 1);
  pub.publish(msg);
}

// Test case for k4aUpdateExposure in range
TEST(ExposureCalibrationTest, UpdateExposureInRangeTest)
{
  // Create a ROS node handle
  ros::NodeHandle nh;

  // Publish fake image data
  publishk4aFakeImageData(nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aUpdateExposure function with a fake exposure value
  int error_code;
  std::string res_msg;
  bool result = k4aUpdateExposure(1000, error_code, res_msg);

  // Check the result
  EXPECT_TRUE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
}

// Test case for k4aUpdateExposure out of range (less than)
TEST(ExposureCalibrationTest, UpdateExposureOutOfRangeLTTest)
{
  // Create a ROS node handle
  ros::NodeHandle nh;

  // Publish fake image data
  publishk4aFakeImageData(nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aUpdateExposure function with a fake exposure value
  int error_code;
  std::string res_msg;
  bool result = k4aUpdateExposure(100, error_code, res_msg);

  // Check the result
  EXPECT_FALSE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_EXPOSURE_OUT_OF_BOUNDS_FAILURE);
}

// Test case for k4aUpdateExposure out of range (greater than)
TEST(ExposureCalibrationTest, UpdateExposureOutOfRangeGTTest)
{
  // Create a ROS node handle
  ros::NodeHandle nh;

  // Publish fake image data
  publishk4aFakeImageData(nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aUpdateExposure function with a fake exposure value
  int error_code;
  std::string res_msg;
  bool result = k4aUpdateExposure(1000005, error_code, res_msg);

  // Check the result
  EXPECT_FALSE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_EXPOSURE_OUT_OF_BOUNDS_FAILURE);
}

// Test case for k4aAutoTuneExposure
TEST(ExposureCalibrationTest, AutoTuneExposureTest)
{
  // Create a ROS node handle
  ros::NodeHandle nh;

  // Publish fake image data
  publishk4aFakeImageData(nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aAutoTuneExposure function with a fake target blue value
  int final_exposure, error_code;
  std::string res_msg;
  bool result = k4aAutoTuneExposure(150, final_exposure, error_code, res_msg);

  // Check the result
  EXPECT_TRUE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  EXPECT_GE(final_exposure, 488); // Assuming minimum exposure value
  EXPECT_LE(final_exposure, 1000000); // Assuming maximum exposure value
}

// Test case for k4aAutoTuneExposure with empty image
TEST(ExposureCalibrationTest, AutoTuneExposureEmptyImageTest)
{
  // Create a ROS node handle
  ros::NodeHandle nh;

  // Publish fake image data
  publishk4aBadImageData(nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aAutoTuneExposure function with a fake target blue value
  int final_exposure, error_code;
  std::string res_msg;
  bool result = k4aAutoTuneExposure(150, final_exposure, error_code, res_msg);

  // Check the result
  EXPECT_FALSE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::IMAGE_NOT_RECEIVED_FAILURE);
  EXPECT_EQ(final_exposure, 0); // failure, never set
}

// Test case for k4aAutoTuneExposure out of range (less than)
TEST(ExposureCalibrationTest, AutoTuneExposureLTTest)
{
  // Create a ROS node handle
  ros::NodeHandle nh;

  // Publish fake image data
  publishk4aFakeImageData(nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aAutoTuneExposure function with a fake target blue value
  int final_exposure, error_code;
  std::string res_msg;
  bool result = k4aAutoTuneExposure(-1, final_exposure, error_code, res_msg);

  // Check the result
  EXPECT_FALSE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_BLUE_VALUE_OUT_OF_BOUNDS_FAILURE);
  EXPECT_EQ(final_exposure, 0); // failure, never set
}

// Test case for k4aAutoTuneExposure out of range (greater than)
TEST(ExposureCalibrationTest, AutoTuneExposureGTTest)
{
  // Create a ROS node handle
  ros::NodeHandle nh;

  // Publish fake image data
  publishk4aFakeImageData(nh);

  // Wait for the image callback to update the latest_k4a_image
  ros::Duration(1.0).sleep();

  // Call the k4aAutoTuneExposure function with a fake target blue value
  int final_exposure, error_code;
  std::string res_msg;
  bool result = k4aAutoTuneExposure(256, final_exposure, error_code, res_msg);

  // Check the result
  EXPECT_FALSE(result);
  EXPECT_EQ(error_code, azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_BLUE_VALUE_OUT_OF_BOUNDS_FAILURE);
  EXPECT_EQ(final_exposure, 0); // failure, never set
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