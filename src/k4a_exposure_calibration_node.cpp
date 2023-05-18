#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "dynamic_reconfigure/server.h"
#include "azure_kinect_ros_driver/AzureKinectParamsConfig.h"
// #include "azure_kinect_ros_driver/update_exp_cal.h"

void p2Callback(const sensor_msgs::PointCloud2& msg)
{
  ROS_ERROR("exposure_calibration hearing /points2");
}

void rgbRawCallback(const sensor_msgs::Image& msg)
{
  ROS_ERROR("exposure_calibration hearing /rgb/raw/image");
}

bool k4aExposureCalibration(azure_kinect_ros_driver::update_exp_cal::Request &req,
                            azure_kinect_ros_driver::update_exp_cal::Response & res)
{
  res.updated_cal_exp = req.new_cal_exp;
  ROS_ERROR("HEY WE GOT A REQUEST [%d]", req.new_cal_exp);
  ROS_ERROR("UPDATIN THE EXPOSURE TO [%d] yeye", res.updated_cal_exp);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "k4a_exposure_calibration");
  
  ROS_ERROR("Initialized Exposure Calibration");
  
  ros::NodeHandle nh;
  ros::Subscriber subPC = nh.subscribe("/points2", 100, p2Callback);
  ros::Subscriber subRGBRaw = nh.subscribe("/rgb/raw/image", 100, rgbRawCallback);
  
  ROS_ERROR("Exposure Calibration subscribed to /points2 and /rgb/raw/image");

  // Advertise calibrate_exposure service
  ros::ServiceServer service = nh.advertiseService("tune_exposure", k4aExposureCalibration);

  ROS_ERROR("Adjusting exposure_time");

  // dynamic_reconfigure::ReconfigureRequest srv_req;
  // dynamic_reconfigure::ReconfigureResponse srv_resp;
  // dynamic_reconfigure::IntParameter int_param;
  // dynamic_reconfigure::Config conf;

  // int_param.name = "exposure_time";
  // int_param.value = 31337;
  // conf.ints.push_back(int_param);

  // srv_req.config = conf;
  // ros::service::call("/k4a_nodelet_manager/set_parameters", srv_req, srv_resp);
  
  ROS_ERROR("Spinning Exposure Calibration Node");
  ros::spin();
  return 0;
}

bool k4aExposureCalibration(azure_kinect_ros_driver::update_exp_cal::Request &req,
                            azure_kinect_ros_driver::update_exp_cal::Response &res)
{
  res.updated_cal_exp = req.new_cal_exp;
  ROS_ERROR("k4a_exposure_calibration_node received request that exposure be updated to: [%d]", (uint32)req.new_cal_exp);
  ROS_ERROR("k4a_sending back response: [%d]", (unit32)res.updated_cal_exp);
  return true;
}
