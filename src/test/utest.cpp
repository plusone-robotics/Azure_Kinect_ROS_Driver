// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Library headers
#include <gtest/gtest.h>

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

TEST(ExposureCalibrationTest, UpdateExposureTest)
{
  // appease the ros gods
  ros::Time::init();

  // Create ROS node publisher handle
  ros::NodeHandle pub_nh;
  publishk4aFakeImageData(pub_nh);

  // make test node
  K4AExposureCalibration test_node;

  // test appropriate exposure value 1000
  int test_k4aExposureServiceErrorCode;
  std::string test_message = "";

  bool okExp = test_node.k4aUpdateExposure(1000, test_k4aExposureServiceErrorCode, test_message);
  ASSERT_TRUE(okExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message, "Updated exposure");
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