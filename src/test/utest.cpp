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

TEST(ExposureCalibrationTest, CompareExposureTest)
{
  ROS_ERROR("SANITY CHECK: CompareExposureTest");
  
  K4AExposureCalibration test_node_eqExp;
  
  ROS_ERROR("SANITY CHECK: CompareExposureTest, test_node created");

  // test appropriate exposure value 1000
  int test_k4aExposureServiceErrorCode;
  std::string test_message = "";

  ROS_ERROR("SANITY CHECK: CompareExposureTest, about to call k4aCompare");
  bool eqExp = test_node_eqExp.k4aCompareExposure(test_k4aExposureServiceErrorCode, test_message, 1000, 1000);
  ROS_ERROR("SANITY CHECK: CompareExposureTest, called k4aCompareExposure");
  ASSERT_TRUE(eqExp);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message, "Exposure update successful");
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