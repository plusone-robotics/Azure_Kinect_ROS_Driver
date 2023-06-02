// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Associated headers
#include "azure_kinect_ros_driver/k4a_exposure_calibration_node.h"

K4AExposureCalibration::K4AExposureCalibration(ros::NodeHandle& nh)
{
  init(nh);
}

void K4AExposureCalibration::init(ros::NodeHandle& nh)
{
  nh_ = nh;
  image_transport::ImageTransport it(nh_);

  latest_k4a_image_ptr = &latest_k4a_image;
  subPC = nh_.subscribe("/points2", 1, &K4AExposureCalibration::p2Callback, this);
  subRGBRaw = it.subscribe("/rgb/raw/image", 1, &K4AExposureCalibration::rgbRawImageCallback, this);

  // advertise services
  ros::ServiceServer update_exposure_service = nh_.advertiseService("k4a_update_exposure", &K4AExposureCalibration::k4aUpdateExposureCallback, this);
  ros::ServiceServer auto_tune_exposure_service = nh_.advertiseService("k4a_auto_tune_exposure", &K4AExposureCalibration::k4aAutoTuneExposureCallback, this);

  ROS_INFO("Spinning K4A Exposure Calibration Node");
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
}

K4AExposureCalibration::~K4AExposureCalibration()
{
}

// call k4a_nodelet_manager/set_parameters to update exposure value
bool K4AExposureCalibration::k4aUpdateExposure(int req_exposure, int& error_code, std::string& res_msg)
{
  ROS_INFO("Updating exposure_time to: [%d]", req_exposure);

  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::IntParameter int_param;
  dynamic_reconfigure::Config req_conf;

  int_param.name = "exposure_time";
  int_param.value = req_exposure;
  req_conf.ints.push_back(int_param);
  srv_req.config = req_conf;
  ros::service::call("/k4a_nodelet_manager/set_parameters", srv_req, srv_resp);
  // check to make sure parameter was actually set
  int updated_exposure = -1;
  for(const auto& param : srv_resp.config.ints)
  {
    if(param.name == "exposure_time")
    {
      updated_exposure = param.value;
      break;
    }
  }
  if(updated_exposure != req_exposure || updated_exposure == -1)
  {
    std::string error_msg = "Failed to update exposure in k4aUpdateExposure";
    ROS_ERROR("Failed to update exposure in k4aUpdateExposure");
    res_msg = error_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::CAMERA_EXPOSURE_SET_FAILURE;
    return false;
  }
  else
  {
    std::string success_msg = "Updated exposure";
    ROS_INFO("Updated exposure to: [%d]", updated_exposure);
    res_msg = success_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS;
    return true;
  }
}

// auto tune exposure with given target blue value
bool K4AExposureCalibration::k4aAutoTuneExposure(int target_blue_value, int& final_exposure, int& error_code, std::string& res_msg)
{
  ROS_INFO("Starting K4A auto exposure tuning...");

  // exposure loop
  int total_blue = 0;
  for(int exp=488; exp<1000000; exp+=500)
  {
    // split OpenCV mat into three color channels
    cv::Mat color_channels[3];
    std::lock_guard<std::mutex> lock(latest_k4a_image_mutex);
    cv::split(*latest_k4a_image_ptr, color_channels);

    int rows = latest_k4a_image.rows;
    int cols = latest_k4a_image.cols;
    int pixel_count = rows * cols;
    if(rows <= 0 || cols <= 0)
    {
      std::string error_msg = "Failed to retrieve latest image in k4aAutoTuneExposure";
      ROS_ERROR("Failed to retrieve latest image in k4aAutoTuneExposure");
      res_msg = error_msg;
      error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::IMAGE_NOT_RECEIVED_FAILURE;
      return false;
    }

    bool autoTune = k4aUpdateExposure(exp, error_code, res_msg);
    
    if(!autoTune)
    {
      // k4aUpdateExposure handles updating response
      ROS_ERROR("Unable to update exposure in k4aAutoTuneExposure");
      return false;
    }
    
    // calculate average blue value
    else{
      for(int i=0; i<rows; i++)
      {
        for(int j=0; j<cols; j++)
        {
          total_blue += color_channels[0].at<uchar>(i,j);
        }
      }
    }
    // calculate average blue value
    int current_avg_blue_value = total_blue / pixel_count;
    // did we achieve appropriate blue at this exposure?
    if(current_avg_blue_value >= target_blue_value)
    {
      std::string error_msg = "Successfully calibrated exposure for target blue value";
      ROS_INFO("Successfully calibrated exposure for target blue value of [%d]", target_blue_value);
      res_msg = error_msg;
      error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS;
      final_exposure = exp;
      break;
    }
  }
  return true;
}

bool K4AExposureCalibration::k4aUpdateExposureCallback(azure_kinect_ros_driver::k4a_update_exposure::Request &req,
                               azure_kinect_ros_driver::k4a_update_exposure::Response &res)
{
  // prepare response
  res.message = "";
  int error_code;

  ROS_INFO("Received K4A exposure update request: [%d]", req.new_exp);

  // check exposure limits
  uint32_t req_exposure = req.new_exp;
  uint32_t min_exposure = 488;
  uint32_t max_exposure = 1000000;

  if(req_exposure < min_exposure || req_exposure > max_exposure)
  {
    std::string error_msg = "Requested exposure out of range";
    ROS_ERROR("Requested exposure out of range [%d] - [%d]", min_exposure, max_exposure);
    res.message = error_msg;
    res.k4aExposureServiceErrorCode = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_EXPOSURE_OUT_OF_BOUNDS_FAILURE;
    return true;
  }
  
  bool tuningRes = k4aUpdateExposure(req.new_exp, error_code, res.message);
  
  ROS_INFO("Sending back response...");

  // k4aUpdateExposure handles updating response
  if(!tuningRes)
  {
    ROS_ERROR("Unable to update exposure_time, k4aUpdateExposure failed");
    res.k4aExposureServiceErrorCode = error_code;
    return true;
  }
  else
  {
    res.k4aExposureServiceErrorCode = error_code;
    return true;
  }
}

bool K4AExposureCalibration::k4aAutoTuneExposureCallback(azure_kinect_ros_driver::k4a_auto_tune_exposure::Request &req,
                                                         azure_kinect_ros_driver::k4a_auto_tune_exposure::Response &res)
{
  // prepare response
  res.message = "";
  int error_code, calibrated_exposure;
  std::string res_msg;

  ROS_INFO("Received K4A exposure auto tuning request for blue value: [%d]", req.target_blue_val);

  // check blue value limits
  uint32_t req_blue = req.target_blue_val;
  uint32_t min_blue = 0;
  uint32_t max_blue = 255;

  if(req_blue < min_blue || req_blue > max_blue)
  {
    std::string error_msg = "Requested blue value out of range";
    ROS_ERROR("Requested blue value out of range [%d] - [%d]", min_blue, max_blue);
    res.message = error_msg;
    res.k4aExposureServiceErrorCode = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_BLUE_VALUE_OUT_OF_BOUNDS_FAILURE;
    return true;
  }
  
  bool autoTuningRes = k4aAutoTuneExposure(req.target_blue_val, calibrated_exposure, error_code, res_msg);
  
  ROS_INFO("Sending back response...");

  // k4aAutoTuneExposure and k4aUpdateExposure handle updating response
  if(!autoTuningRes)
  {
    ROS_ERROR("k4aAutoTuneExposure failed in callback");
    res.message = res_msg;
    res.k4aExposureServiceErrorCode = error_code;
    return true;
  }
  else
  {
    ROS_INFO("Successfully updated exposure to [%d] for target blue value of [%d]", calibrated_exposure, req.target_blue_val);
    res.message = res_msg;
    res.k4aExposureServiceErrorCode = error_code;
    res.calibrated_exposure = calibrated_exposure;
    return true;
  }
}

void K4AExposureCalibration::p2Callback(const sensor_msgs::PointCloud2& msg)
{
  ROS_DEBUG("k4a_exposure_calibration_node subscribed to /points2");
}

void K4AExposureCalibration::rgbRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("k4a_exposure_calibration_node subscribed to /rgb/raw/image");
  try
  {
    // convert ROS image message to OpenCV
    k4aCvImagePtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    std::lock_guard<std::mutex> lock(latest_k4a_image_mutex);
    *latest_k4a_image_ptr = k4aCvImagePtr->image;

    // check if conversion worked
    if(latest_k4a_image.empty())
    {
      ROS_ERROR("Failed to convert k4a rgb image to OpenCV mat");
    }
    else
    {
      ROS_DEBUG("Updated latest_k4a_image");
    }
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: [%s]", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "k4a_exposure_calibration");
  ros::NodeHandle nh;
  
  K4AExposureCalibration k4a_Exposure_Calibration(nh);
  ROS_INFO("Initialized K4A Exposure Calibration Node");

  return 0;
}