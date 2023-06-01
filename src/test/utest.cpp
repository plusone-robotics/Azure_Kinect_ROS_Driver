#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(TestSuite, azure_kinect_ros_driver_framework)
{
  ASSERT_TRUE(true);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "k4a_exposure_calibration_tests");
  ros::Time::init();
  
  return RUN_ALL_TESTS();
}