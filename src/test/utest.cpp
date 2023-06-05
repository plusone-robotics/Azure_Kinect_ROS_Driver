// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Library headers
#include <gtest/gtest.h>
#include <dynamic_reconfigure/Reconfigure.h>

// Project headers
#include "azure_kinect_ros_driver/k4a_exposure_calibration_node.h"
#include "azure_kinect_ros_driver/k4a_ros_bridge_nodelet.h"

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

TEST(ExposureCalibrationTest, UpdateExposureTest)
{
  // appease the ros gods
  ros::Time::init();

  // Create mock service/publisher
  ros::NodeHandle mock_pub_nh;

  // Set up mock dynamic reconfigure service
  ros::ServiceServer mock_service = mock_pub_nh.advertiseService("/k4a_nodelet_manager/set_parameters", k4aTestMockReconfigure);

  // Publish fake image data
  //publishk4aFakeImageData(mock_pub_nh);

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