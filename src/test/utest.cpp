// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Library headers
#include <gtest/gtest.h>

// Project headers
#include "azure_kinect_ros_driver/k4a_exposure_calibration_node.h"

TEST(ExposureCalibrationTest, UpdateExposureTest)
{
  // make fake image
  cv::Mat k4aFakeImage;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(50, 0, 0));
  k4aFakeImage = image.clone();

  // Convert the image to ROS sensor_msgs/Image
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = ros::Time::now();
  cvImage.encoding = "bgr8";
  cvImage.image = image;
  sensor_msgs::ImagePtr msg = cvImage.toImageMsg();

  // make fake req/res
  azure_kinect_ros_driver::k4a_update_exposure::Request test_req;
  azure_kinect_ros_driver::k4a_update_exposure::Response test_res;

  // test appropriate exposure value
  test_req.new_exp = 1000;
  bool okExp = azure_kinect_ros_driver::k4a_exposure_calibration_node::(test_req, test_res);
  ASSERT_TRUE(okExp);
  ASSERT_TRUE(test_res.k4aExposureServiceErrorCode == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_TRUE(test_res.message == "Updated exposure");
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