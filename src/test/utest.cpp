// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Library headers
#include <gtest/gtest.h>
// #include <dynamic_reconfigure/Reconfigure.h>

// Project headers
#include "azure_kinect_ros_driver/k4a_exposure_calibration_node.h"

// void publishk4aFakeImageData(ros::NodeHandle& pub_nh)
// {
//   ROS_ERROR("SANITY CHECK: publishk4aFakeImageData");
//   // Create a fake RGB image (BGR8 format)
//   cv::Mat k4aFakeImage;
//   cv::Mat image(480, 640, CV_8UC3, cv::Scalar(50, 0, 0));
//   k4aFakeImage = image.clone();

//   // Convert the image to ROS sensor_msgs/Image
//   cv_bridge::CvImage cvImage;
//   cvImage.header.stamp = ros::Time::now();
//   cvImage.encoding = "bgr8";
//   cvImage.image = image;
//   sensor_msgs::ImagePtr msg = cvImage.toImageMsg();

//   // Publish the fake image data
//   image_transport::ImageTransport it(pub_nh);
//   image_transport::Publisher pub = it.advertise("/rgb/raw/image", 1);
//   pub.publish(msg);
// }

// // Mock implementation of dynamic reconfigure callback for k4aUpdateExposure
// bool k4aTestMockReconfigure(dynamic_reconfigure::Reconfigure::Request& req,
//                             dynamic_reconfigure::Reconfigure::Response& res)
// {
//   ROS_ERROR("SANITY CHECK: k4aTestMockReconfigure");
//   dynamic_reconfigure::IntParameter updated_exposure_param;
//   updated_exposure_param.name = "exposure_time";
//   for(const auto& param : req.config.ints)
//   {
//     if(param.name == "exposure_time")
//     {
//       updated_exposure_param.value = param.value;
//       break;
//     }
//   }
//   res.config.ints.push_back(updated_exposure_param);

//   return true;
// }

TEST(ExposureCalibrationTest, CameraExposureUpdateCheckTest)
{
  K4AExposureCalibration test_node;

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

TEST(ExposureCalibrationTest, TargetBlueCheckTest)
{
  K4AExposureCalibration test_node;
  
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
  // nothing is assigned, nothing is updated
  ASSERT_FALSE(chBlueNot);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_bNot == 0);
  ASSERT_EQ(test_message_bNot, "");
}

TEST(ExposureCalibrationTest, ImageReceivedCheckTest)
{
  K4AExposureCalibration test_node;
  
  // image receivec
  int test_k4aExposureServiceErrorCode_rec;
  std::string test_message_rec = "";
  bool chRec = test_node.k4aImageReceivedCheck(test_k4aExposureServiceErrorCode_rec, test_k4aExposureServiceErrorCode_rec);
  ASSERT_TRUE(chRec);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_rec == azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS);
  ASSERT_EQ(test_message_rec, "Successfully calibrated exposure for target blue value");

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
  // nothing is assigned, nothing is updated
  ASSERT_FALSE(chBlueNot);
  ASSERT_TRUE(test_k4aExposureServiceErrorCode_bNot == 0);
  ASSERT_EQ(test_message_bNot, "");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "k4a_exposure_calibration_tests");
  
  return RUN_ALL_TESTS();
}