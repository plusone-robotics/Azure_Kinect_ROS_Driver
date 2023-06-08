// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Associated headers
#include "azure_kinect_ros_driver/k4a_exposure_calibration_node.h"

K4AExposureCalibration::K4AExposureCalibration()
{
}

K4AExposureCalibration::K4AExposureCalibration(ros::NodeHandle& nh)
{
  nh_ = nh;
  image_transport::ImageTransport it(nh_);

  subRGBRaw = it.subscribe("/rgb/raw/image", 1, &K4AExposureCalibration::rgbRawImageCallback, this);

  update_exposure_service = nh_.advertiseService("k4a_update_exposure", &K4AExposureCalibration::k4aUpdateExposureCallback, this);
  auto_tune_exposure_service = nh_.advertiseService("k4a_auto_tune_exposure", &K4AExposureCalibration::k4aAutoTuneExposureCallback, this);
}

K4AExposureCalibration::~K4AExposureCalibration()
{
}

bool K4AExposureCalibration::k4aCameraExposureUpdateCheck(int requested_exposure, int updated_exposure, int& error_code, std::string& res_msg)
{
  if(updated_exposure != requested_exposure)
  {
    std::string error_msg = "Failed to update exposure";
    res_msg = error_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::CAMERA_EXPOSURE_SET_FAILURE;
    return false;
  }
  else
  {
    std::string success_msg = "Exposure update successful";
    ROS_INFO("Updated exposure to: [%d]", updated_exposure);
    res_msg = success_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS;
    return true;
  }
}

bool K4AExposureCalibration::k4aCameraExposureBoundsCheck(int requested_exposure, int& error_code, std::string& res_msg)
{
  if(requested_exposure < MIN_EXPOSURE || requested_exposure > MAX_EXPOSURE)
  {
    std::string error_msg = "Requested exposure out of range";
    ROS_ERROR("Requested exposure out of range [%d] - [%d]", MIN_EXPOSURE, MAX_EXPOSURE);
    res_msg = error_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_EXPOSURE_OUT_OF_BOUNDS_FAILURE;
    return false;
  }
  else
  {
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS;
    return true;
  }
}

bool K4AExposureCalibration::k4aTargetBlueCheck(int target_blue_value, int current_avg_blue_value, int& error_code, std::string& res_msg)
{
  if(current_avg_blue_value >= target_blue_value)
  {
    std::string error_msg = "Successfully calibrated exposure for target blue value";
    ROS_INFO("Successfully calibrated exposure for target blue value of [%d]", target_blue_value);
    res_msg = error_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS;
    return true;
  }
  else
  {
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_BLUE_VALUE_NOT_MET;
    return false;
  }
}

bool K4AExposureCalibration::k4aBlueBoundsCheck(int target_blue_value, int& error_code, std::string& res_msg)
{
  if(target_blue_value < MIN_BLUE || target_blue_value > MAX_BLUE)
  {
    std::string error_msg = "Requested target blue value out of range";
    ROS_ERROR("Requested target blue value out of range [%d] - [%d]", MIN_BLUE, MAX_BLUE);
    res_msg = error_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_CAMERA_BLUE_VALUE_OUT_OF_BOUNDS_FAILURE;
    return false;
  }
  else
  {
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS;
    return true;
  }
}

bool K4AExposureCalibration::k4aImagePopulatedCheck(cv::Mat& mat, int& error_code, std::string& res_msg)
{
  if(mat.empty())
  {
    std::string error_msg = "OpenCV mat is empty";
    res_msg = error_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::IMAGE_NOT_RECEIVED_FAILURE;
    return false;
  }
  else
  {
    std::string error_msg = "OpenCV mat is populated";
    res_msg = error_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS;
    return true;
  }
}

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
  int updated_exposure = -1;
  for(const auto& param : srv_resp.config.ints)
  {
    if(param.name == "exposure_time")
    {
      updated_exposure = param.value;
      break;
    }
  }
  return k4aCameraExposureUpdateCheck(req_exposure, updated_exposure, error_code, res_msg);
}

bool K4AExposureCalibration::k4aAutoTuneExposure(int target_blue_value, int& final_exposure, int& error_code, std::string& res_msg)
{
  ROS_INFO("Starting K4A auto exposure tuning...");

  int total_blue = 0;
  for(int exp=MIN_EXPOSURE; exp<MAX_EXPOSURE; exp+=EXPOSURE_INC)
  {
    bool channelsPop = k4aImagePopulatedCheck(*latest_k4a_image_ptr, error_code, res_msg);
    if(!channelsPop)
    {
      return false;
    }
    else
    {
      cv::Mat color_channels[3];
      std::lock_guard<std::mutex> lock(latest_k4a_image_mutex);
      cv::split(*latest_k4a_image_ptr, color_channels);
      int rows = latest_k4a_image.rows;
      int cols = latest_k4a_image.cols;
      int pixel_count = rows * cols;

      bool updateExposure = k4aUpdateExposure(exp, error_code, res_msg);
      
      if(!updateExposure)
      {
        return false;
      }
      else{
        for(int i=0; i<rows; i++)
        {
          for(int j=0; j<cols; j++)
          {
            total_blue += color_channels[0].at<uchar>(i,j);
          }
        }
      }
      int current_avg_blue_value = total_blue / pixel_count;
      bool targetBlueCheck = k4aTargetBlueCheck(target_blue_value, current_avg_blue_value, error_code, res_msg);
      if(targetBlueCheck)
      {
        final_exposure = exp;
        break;
      }
    }
  } 
  ROS_INFO("Successfully updated exposure to [%d] for target blue value of [%d]", final_exposure, target_blue_value);
  return true;
}

bool K4AExposureCalibration::k4aUpdateExposureCallback(azure_kinect_ros_driver::k4a_update_exposure::Request &req,
                                                       azure_kinect_ros_driver::k4a_update_exposure::Response &res)
{
  res.message = "";

  ROS_INFO("Received K4A exposure update request: [%d]", req.new_exp);

  int error_code;
  bool expBoundCheck = k4aCameraExposureBoundsCheck(req.new_exp, error_code, res.message);
  if(expBoundCheck)
  {
    bool tuningRes = k4aUpdateExposure(req.new_exp, error_code, res.message);
    res.k4aExposureServiceErrorCode = error_code;
    if(!tuningRes)
    {
      ROS_ERROR("Unable to update exposure_time, k4aUpdateExposure failed");
      return false;
    }
    else
    {
      ROS_INFO("Exposure updated to: [%d]", req.new_exp);
      return true;
    }
  }
  else
  {
    ROS_ERROR("Requested exposure out of bounds");
    res.k4aExposureServiceErrorCode = error_code;
    return false;
  }
}

bool K4AExposureCalibration::k4aAutoTuneExposureCallback(azure_kinect_ros_driver::k4a_auto_tune_exposure::Request &req,
                                                         azure_kinect_ros_driver::k4a_auto_tune_exposure::Response &res)
{
  res.message = "";
  int error_code, calibrated_exposure;
  std::string res_msg;

  ROS_INFO("Received K4A exposure auto tuning request for blue value: [%d]", req.target_blue_val);

  bool blueBound = k4aBlueBoundsCheck(req.target_blue_val, error_code, res_msg);
  res.k4aExposureServiceErrorCode = error_code;
  res.message = res_msg;

  if(!blueBound)
  {
    return false;
  }
  else
  {
    bool autoTuningRes = k4aAutoTuneExposure(req.target_blue_val, calibrated_exposure, error_code, res_msg);
    res.k4aExposureServiceErrorCode = error_code;
    res.message = res_msg;
    res.calibrated_exposure = calibrated_exposure;

    if(!autoTuningRes)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
}

void K4AExposureCalibration::rgbRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("k4a_exposure_calibration_node subscribed to /rgb/raw/image");
  try
  {
    k4aCvImagePtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    std::lock_guard<std::mutex> lock(latest_k4a_image_mutex);
    *latest_k4a_image_ptr = k4aCvImagePtr->image;

    if(k4aCvImagePtr->image.empty())
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
    ROS_ERROR("cv_bridge exception in K4AExposureCalibration::rgbRawImageCallback: [%s]", e.what());
  }
}
