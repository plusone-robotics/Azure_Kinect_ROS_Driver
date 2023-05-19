#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "dynamic_reconfigure/server.h"
#include "azure_kinect_ros_driver/AzureKinectParamsConfig.h"
#include "azure_kinect_ros_driver/k4a_exposure_tuning.h"

void p2Callback(const sensor_msgs::PointCloud2& msg)
{
  ROS_DEBUG("exposure_calibration hearing /points2");
}

void rgbRawCallback(const sensor_msgs::Image& msg)
{
  ROS_DEBUG("exposure_calibration hearing /rgb/raw/image");
}

bool k4aExposureTuning(int reqExposure)
{
  ROS_INFO("Adjusting exposure_time");

  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::IntParameter int_param;
  dynamic_reconfigure::Config conf;

  int_param.name = "exposure_time";
  int_param.value = reqExposure;
  conf.ints.push_back(int_param);

  srv_req.config = conf;
  ros::service::call("/k4a_nodelet_manager/set_parameters", srv_req, srv_resp);

  return true;
}

bool rosk4aExposureTuningCallback(azure_kinect_ros_driver::k4a_exposure_tuning::Request &req,
                                  azure_kinect_ros_driver::k4a_exposure_tuning::Response &res)
{
  // prepare response
  res.message = "";

  ROS_INFO("Received exposure tuning request: [%d]", req.new_exp);
  ROS_INFO("Requesting exposure update to: [%d]", req.new_exp);

  // check exposure limits
  uint32_t req_exposure = req.new_exp;
  uint32_t min_exposure = 488;
  uint32_t max_exposure = 1000000;

  if(req_exposure < min_exposure || req_exposure > max_exposure)
  {
    res.message += "Requested exposure out of range (488-1,000,000)";
    res.success = false;
    return true;
  }
  
  bool tuningRes = k4aExposureTuning(req.new_exp);
  
  ROS_INFO("Sending back response...");

  if(!tuningRes)
  {
    res.success = false;
    res.message += "Unable to change exposure_time";
    return true;
  }
  else
  {
    res.success = true;
    res.updated_exp = req.new_exp;
    res.message += "Exposure updated";
    return true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "k4a_exposure_calibration");
  
  ROS_INFO("Initialized Exposure Calibration");
  
  ros::NodeHandle nh;
  ros::Subscriber subPC = nh.subscribe("/points2", 100, p2Callback);
  ros::Subscriber subRGBRaw = nh.subscribe("/rgb/raw/image", 100, rgbRawCallback);
  
  ROS_INFO("Exposure Calibration subscribed to /points2 and /rgb/raw/image");

  // Advertise calibrate_exposure service
  ros::ServiceServer service = nh.advertiseService("k4a_exposure_tuning", rosk4aExposureTuningCallback);
  
  ros::Rate r(0.2);
  r.sleep();

  ROS_INFO("Spinning Exposure Calibration Node");
  ros::spin();
  return 0;
}
