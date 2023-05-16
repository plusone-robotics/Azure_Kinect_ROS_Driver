#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "dynamic_reconfigure/server.h"
#include "azure_kinect_ros_driver/AzureKinectParamsConfig.h"

void p2Callback(const sensor_msgs::PointCloud2& msg)
{
  ROS_INFO("exposure_calibration hearing /points2");
}

void rgbRawCallback(const sensor_msgs::Image& msg)
{
  ROS_INFO("exposure_calibration hearing /rgb/raw/image");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "k4a_exposure_calibration");
  
  ROS_INFO("Initialized Exposure Calibration");
  
  ros::NodeHandle nh;
  ros::Subscriber subPC = nh.subscribe("/points2", 100, p2Callback);
  ros::Subscriber subRGBRaw = nh.subscribe("/rgb/raw/image", 100, rgbRawCallback);
  
  ROS_INFO("Exposure Calibration subscribed to /points2 and /rgb/raw/image");

  ros::Duration(5.0).sleep();

  ROS_INFO("Looping through exposures");

  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::IntParameter int_param;
  dynamic_reconfigure::Config conf;

  int_param.name = "exposure_time";
  int_param.value = 488;
  conf.ints.push_back(int_param);

  srv_req.config = conf;
  ros::service::call("/k4a_nodelet_manager/set_parameters", srv_req, srv_resp);
  
  ROS_INFO("Spinning Exposure Calibration Node");
  ros::spin();
  return 0;
}
