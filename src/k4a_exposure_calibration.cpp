// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Associated headers
#include "azure_kinect_ros_driver/k4a_exposure_calibration.h"

K4AExposureCalibration::K4AExposureCalibration()
{
}

K4AExposureCalibration::~K4AExposureCalibration()
{
}

K4AExposureCalibration::K4AExposureCalibration(ros::NodeHandle& nh)
{
  nh_ = nh;
  image_transport::ImageTransport it_(nh_);

  subRGBRaw_ = it_.subscribe("/rgb/raw/image", 1, &K4AExposureCalibration::rgbRawImageCallback, this);

  update_exposure_service_ = nh_.advertiseService("k4a_update_exposure", &K4AExposureCalibration::k4aUpdateExposureCallback, this);
  update_white_balance_service_ = nh_.advertiseService("k4a_update_white_balance", &K4AExposureCalibration::k4aUpdateWhiteBalanceCallback, this);
  auto_tune_exposure_service_ = nh_.advertiseService("k4a_auto_tune_exposure", &K4AExposureCalibration::k4aAutoTuneExposureCallback, this);
}

bool K4AExposureCalibration::k4aCameraExposureUpdateCheck(const uint32_t requested_exposure, uint32_t updated_exposure, int8_t& error_code, std::string& res_msg)
{
  if(updated_exposure != requested_exposure)
  {
    std::string error_msg = "Failed to update exposure";
    res_msg = error_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::CAMERA_PARAM_SET_FAILURE;
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

bool K4AExposureCalibration::k4aCameraExposureBoundsCheck(const uint32_t requested_exposure, int8_t& error_code, std::string& res_msg)
{
  if(requested_exposure < MIN_EXPOSURE_ || requested_exposure > MAX_EXPOSURE_)
  {
    std::string error_msg = "Requested exposure out of range";
    ROS_ERROR("Requested exposure out of range [%d] - [%d]", MIN_EXPOSURE_, MAX_EXPOSURE_);
    res_msg = error_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_PARAM_OUT_OF_BOUNDS_FAILURE;
    return false;
  }
  else
  {
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS;
    return true;
  }
}

bool K4AExposureCalibration::k4aCameraWhiteBalanceUpdateCheck(const uint16_t requested_white_balance, uint16_t updated_white_balance, int8_t& error_code, std::string& res_msg)
{
  if(updated_white_balance != requested_white_balance)
  {
    std::string error_msg = "Failed to update white balance";
    res_msg = error_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::CAMERA_PARAM_SET_FAILURE;
    return false;
  }
  else
  {
    std::string success_msg = "White balance update successful";
    ROS_INFO("Updated white balance to: [%d]", updated_white_balance);
    res_msg = success_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS;
    return true;
  }
}

bool K4AExposureCalibration::k4aCameraWhiteBalanceBoundsCheck(const uint16_t requested_white_balance, int8_t& error_code, std::string& res_msg)
{
  if(requested_white_balance < MIN_WHITE_BALANCE_ || requested_white_balance > MAX_WHITE_BALANCE_)
  {
    std::string error_msg = "Requested white balance out of range";
    ROS_ERROR("Requested white balance out of range [%d] - [%d]", MIN_WHITE_BALANCE_, MAX_WHITE_BALANCE_);
    res_msg = error_msg;
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::REQUESTED_PARAM_OUT_OF_BOUNDS_FAILURE;
    return false;
  }
  else
  {
    error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::SUCCESS;
    return true;
  }
}

bool K4AExposureCalibration::k4aTargetBlueCheck(const uint8_t target_blue_value, uint8_t current_avg_blue_value, int8_t& error_code, std::string& res_msg)
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

bool K4AExposureCalibration::k4aImagePopulatedCheck(cv::Mat& mat, int8_t& error_code, std::string& res_msg)
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

bool K4AExposureCalibration::k4aUpdateExposure(const uint32_t req_exposure, int8_t& error_code, std::string& res_msg)
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
  uint32_t updated_exposure = 0;
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

bool K4AExposureCalibration::k4aUpdateWhiteBalance(const uint16_t req_white_balance, int8_t& error_code, std::string& res_msg)
{
  ROS_INFO("Updating white_balance to: [%d]", req_white_balance);

  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::IntParameter int_param;
  dynamic_reconfigure::Config req_conf;

  int_param.name = "white_balance";
  int_param.value = req_white_balance;
  req_conf.ints.push_back(int_param);
  srv_req.config = req_conf;
  ros::service::call("/k4a_nodelet_manager/set_parameters", srv_req, srv_resp);
  uint32_t updated_white_balance = 0;
  for(const auto& param : srv_resp.config.ints)
  {
    if(param.name == "white_balance")
    {
      updated_white_balance = param.value;
      break;
    }
  }
  return k4aCameraWhiteBalanceUpdateCheck(req_white_balance, updated_white_balance, error_code, res_msg);
}

bool K4AExposureCalibration::k4aAutoTuneExposure(const uint8_t target_blue_value, uint32_t& final_exposure, int8_t& error_code, std::string& res_msg)
{
  ROS_INFO("Starting K4A auto exposure tuning...");

  uint32_t total_blue = 0;
  for(uint32_t exp=MIN_EXPOSURE_; exp<MAX_EXPOSURE_; exp+=EXPOSURE_INC_)
  {
    bool channelsPop = k4aImagePopulatedCheck(*latest_k4a_image_ptr_, error_code, res_msg);
    if(!channelsPop)
    {
      return false;
    }
    else
    {
      cv::Mat color_channels[3];
      std::lock_guard<std::mutex> lock(latest_k4a_image_mutex_);
      cv::split(*latest_k4a_image_ptr_, color_channels);
      uint32_t rows = latest_k4a_image_.rows;
      uint32_t cols = latest_k4a_image_.cols;
      uint32_t pixel_count = rows * cols;

      bool updateExposure = k4aUpdateExposure(exp, error_code, res_msg);
      
      if(!updateExposure)
      {
        return false;
      }
      else{
        for(uint32_t i=0; i<rows; i++)
        {
          for(uint32_t j=0; j<cols; j++)
          {
            total_blue += color_channels[0].at<uchar>(i,j);
          }
        }
      }
      uint8_t current_avg_blue_value = total_blue / pixel_count;
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

  int8_t error_code;
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

bool K4AExposureCalibration::k4aUpdateWhiteBalanceCallback(azure_kinect_ros_driver::k4a_update_white_balance::Request &req,
                                                           azure_kinect_ros_driver::k4a_update_white_balance::Response &res)
{
  res.message = "";

  ROS_INFO("Received K4A white balance update request: [%d]", req.new_wb);

  int8_t error_code;
  bool wbBoundCheck = k4aCameraWhiteBalanceBoundsCheck(req.new_wb, error_code, res.message);
  if(wbBoundCheck)
  {
    bool upWBRes = k4aUpdateWhiteBalance(req.new_wb, error_code, res.message);
    res.k4aExposureServiceErrorCode = error_code;
    if(!upWBRes)
    {
      ROS_ERROR("Unable to update white_balance, k4aUpdateWhiteBalance failed");
      return false;
    }
    else
    {
      ROS_INFO("White balance updated to: [%d]", req.new_wb);
      return true;
    }
  }
  else
  {
    ROS_ERROR("Requested white balance out of bounds");
    res.k4aExposureServiceErrorCode = error_code;
    return false;
  }
}

bool K4AExposureCalibration::k4aAutoTuneExposureCallback(azure_kinect_ros_driver::k4a_auto_tune_exposure::Request &req,
                                                         azure_kinect_ros_driver::k4a_auto_tune_exposure::Response &res)
{
  res.message = "";
  int8_t error_code;
  uint32_t calibrated_exposure;
  std::string res_msg;

  ROS_INFO("Received K4A exposure auto tuning request for blue value: [%d]", req.target_blue_val);

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

void K4AExposureCalibration::rgbRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("k4a_exposure_calibration_node subscribed to /rgb/raw/image");
  try
  {
    k4aCvImagePtr_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    std::lock_guard<std::mutex> lock(latest_k4a_image_mutex_);
    *latest_k4a_image_ptr_ = k4aCvImagePtr_->image;

    if(k4aCvImagePtr_->image.empty())
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
