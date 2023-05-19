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

bool k4aExposureTuning(uint_32 reqExposure)
{
  ROS_ERROR("Adjusting exposure_time");
  if(reqExposure < 488 || reqExposure > 1,000,000)
  {
    ROS_ERROR("k4a exposure limited to 488 =< exposure <= 1,000,000");
    return false;
  }
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

void rosk4aExposureTuningCallback(azure_kinect_ros_driver::k4a_exposure_tuning::Request &req,
                                  azure_kinect_ros_driver::k4a_exposure_tuning::Response &res)
{
  // prepare response
  ROS_ERROR("HEY WE GOT A REQUEST [%d]", req.new_exp);
  res.success = true;
  res.message = "";

  ROS_ERROR("ATTEMPTIN UPDATIN THE EXPOSURE TO [%d] yeye", res.updated_exp);
  bool res = k4aExposureTuning(req.new_exp);
  
  if(!res)
  {
    res.success = false;
    res.message += "\nUnable to change exposure_time"
  }
  else
  {
    res.updated_exp = req.new_exp;
    res.message += "Exposure updated";
  }

  ROS_ERROR("Sending back response...");

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
  ros::ServiceServer service = nh.advertiseService("k4a_exposure_tuning", rosk4aExposureTuningCallback);
  
  ROS_ERROR("Spinning Exposure Calibration Node");
  ros::spin();
  return 0;
}
