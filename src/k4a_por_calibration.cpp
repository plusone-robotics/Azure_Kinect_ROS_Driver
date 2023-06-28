// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Library headers
#include <random>
#include <cmath>

// Associated headers
#include "azure_kinect_ros_driver/k4a_por_calibration.h"

K4APORCalibration::K4APORCalibration()
{
}

K4APORCalibration::~K4APORCalibration()
{
}

K4APORCalibration::K4APORCalibration(ros::NodeHandle& nh)
{
  nh_ = nh;
  image_transport::ImageTransport it_(nh_);

  subRGBRaw_ = it_.subscribe("/rgb/raw/image", 1, &K4APORCalibration::rgbRawImageCallback, this);

  update_exposure_service_ = nh_.advertiseService("k4a_update_exposure", &K4APORCalibration::k4aUpdateExposureCallback, this);
  update_white_balance_service_ = nh_.advertiseService("k4a_update_white_balance", &K4APORCalibration::k4aUpdateWhiteBalanceCallback, this);
  auto_tune_exposure_service_ = nh_.advertiseService("k4a_auto_tune_exposure", &K4APORCalibration::k4aAutoTuneExposureCallback, this);
  sgd_tune_service_ = nh_.advertiseService("k4a_sgd_tune", &K4APORCalibration::k4aSGDTuneCallback, this);
}

uint32_t K4APORCalibration::k4aStandardizeExposure(const uint32_t exposure)
{
  uint32_t kinect_exposure;
  for(int i = 0; i <= 10; i++)
  {
    if(EXPOSURES_[i] <= exposure && exposure <= EXPOSURES_[i+1])
    {
      uint32_t low_diff = exposure - EXPOSURES_[i];
      uint32_t high_diff = EXPOSURES_[i+1] - exposure;
      if(low_diff < high_diff)
      {
        kinect_exposure = EXPOSURES_[i];
        break;
      }
      else
      {
        kinect_exposure = EXPOSURES_[i+1];
        break;
      }
    }
  }
  return kinect_exposure;
}

bool K4APORCalibration::k4aCameraExposureUpdateCheck(const uint32_t requested_exposure, uint32_t updated_exposure, int8_t& error_code, std::string& res_msg)
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

bool K4APORCalibration::k4aCameraExposureBoundsCheck(const uint32_t requested_exposure, int8_t& error_code, std::string& res_msg)
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

bool K4APORCalibration::k4aCameraWhiteBalanceUpdateCheck(const uint16_t requested_white_balance, uint16_t updated_white_balance, int8_t& error_code, std::string& res_msg)
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

bool K4APORCalibration::k4aCameraWhiteBalanceBoundsCheck(const uint16_t requested_white_balance, int8_t& error_code, std::string& res_msg)
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

bool K4APORCalibration::k4aTargetBlueCheck(const uint8_t target_blue_value, uint8_t current_avg_blue_value, int8_t& error_code, std::string& res_msg)
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

bool K4APORCalibration::k4aImagePopulatedCheck(cv::Mat& mat, int8_t& error_code, std::string& res_msg)
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

bool K4APORCalibration::k4aUpdateExposure(const uint32_t req_exposure, int8_t& error_code, std::string& res_msg)
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

bool K4APORCalibration::k4aUpdateWhiteBalance(const uint16_t req_white_balance, int8_t& error_code, std::string& res_msg)
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

bool K4APORCalibration::k4aAutoTuneExposure(const uint8_t target_blue_value, uint32_t& final_exposure, int8_t& error_code, std::string& res_msg)
{
  ROS_INFO("Starting K4A auto exposure tuning...");

  uint32_t total_blue = 0;
  for(uint32_t exp_index=0; exp_index<11; exp_index++)
  {
    bool channelsPop = k4aImagePopulatedCheck(*latest_k4a_image_ptr_, error_code, res_msg);
    if(!channelsPop)
    {
      return false;
    }
    else
    {
      bool updateExposure = k4aUpdateExposure(EXPOSURES_[exp_index], error_code, res_msg);
      
      if(!updateExposure)
      {
        return false;
      }
      else{
        cv::Mat color_channels[3];
        std::lock_guard<std::mutex> lock(latest_k4a_image_mutex_);
        cv::split(*latest_k4a_image_ptr_, color_channels);
        uint32_t rows = latest_k4a_image_.rows;
        uint32_t cols = latest_k4a_image_.cols;
        uint32_t pixel_count = rows * cols;

        for(uint32_t i=0; i<rows; i++)
        {
          for(uint32_t j=0; j<cols; j++)
          {
            total_blue += color_channels[0].at<uchar>(i,j);
          }
        }
      
        uint8_t current_avg_blue_value = total_blue / pixel_count;
        bool targetBlueCheck = k4aTargetBlueCheck(target_blue_value, current_avg_blue_value, error_code, res_msg);
        if(targetBlueCheck)
        {
          final_exposure = EXPOSURES_[exp_index];
          break;
        }
      }
    }
  } 
  ROS_INFO("Successfully updated exposure to [%d] for target blue value of [%d]", final_exposure, target_blue_value);
  return true;
}

float K4APORCalibration::k4aRMSE(const float current, const float target, const int iteration, float &rmse)
{
  float diff = current - target;
  rmse = std::sqrt((1.0 / ((float)iteration + 1.0)) * (diff * diff));
  return rmse;
}

// // prediction function
// float K4APORCalibration::k4aDotProduct(const std::vector<float>& a,
//                                        const std::vector<float>& b)
// {
//   float product = 0.0;
//   for(size_t i=0; i<a.size(); i++)
//   {
//     product += a[i] * b[i];
//   }
//   return product;
// }

// // cost function
// float K4APORCalibration::k4aCostFunction(const std::vector<float>& theta,
//                                          const std::vector<float>& X,
//                                          const std::vector<float>& y)
// {
//   float cost = 0.0;
//   size_t lenX = X.size();
//   for(size_t i=0; i < lenX; i++)
//   {
//     float prediction = k4aDotProduct(theta, X[i]);
//     float diff = prediction - y[i];
//     cost += (diff * diff);
//   }
//   return (1.0/(2.0 * lenX)) * cost;
// }

// // function to calculate gradient at given theta
// std::vector<float> K4APORCalibration::k4aGradient(const std::vector<float>& theta,
//                                                   const std::vector<float>& X,
//                                                   const std::vector<float>& y)
// {
//   size_t lenX = X.size();
//   std::vector<float> gradTheta(theta.size(), 0.0);
//   for(size_t i = 0; i < lenX; i++)
//   {
//     float prediction = k4aDotProduct(theta, X[i]);
//     float diff = prediction - y[i];
//   }
//   for(size_t j=0; j < theta.size(); j++)
//   {
//     gradTheta[j] /= lenX;
//   }
//   return gradTheta;
// }

bool K4APORCalibration::k4aSGDTune(const float target_blue_value,
                                   const float target_green_value,
                                   const float target_red_value,
                                   const float target_white_value,
                                   uint32_t& final_exposure,
                                   uint16_t& final_white_balance,
                                   float final_blue_val,
                                   float final_green_val,
                                   float final_red_val,
                                   float final_white_val,
                                   int8_t& error_code,
                                   std::string& res_msg)
{
  ROS_INFO("Starting K4A SGD tuning...");

  // camera params to be modified
  float exposure_time_float = (float)DEFAULT_EXPOSURE_;
  float* const exposure_time_float_ptr = &exposure_time_float;
  float white_balance_float = (float)DEFAULT_WHITE_BALANCE_;
  float* const white_balance_float_ptr = &white_balance_float;
  uint32_t exposure_time_uint;
  uint16_t white_balance_uint;
  float blue_rmse;
  float* const blue_rmse_ptr = &blue_rmse;
  float green_rmse;
  float* const green_rmse_ptr = &green_rmse;
  float red_rmse;
  float* const red_rmse_ptr = &red_rmse;
  float white_rmse;
  float* const white_rmse_ptr = &white_rmse;

  // rng
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(-1.0, 1.0);

  // adjust camera params using SGD
  for(int i = 0; i < MAX_ITERATIONS_; i++)
  {
    ROS_INFO("Current iteration: [%d]", i);
    float total_blue = 0;
    float total_green = 0;
    float total_red = 0;
    float total_white = 0;
    
    bool channelsPop = k4aImagePopulatedCheck(*latest_k4a_image_ptr_, error_code, res_msg);
    if(!channelsPop)
    {
      return false;
    }
    else
    {
      cv::Mat color_channels[3];
      cv::Mat hls_channels[3];
      std::lock_guard<std::mutex> lock(latest_k4a_image_mutex_);
      cv::split(*latest_k4a_image_ptr_, color_channels);
      cv::split(*latest_k4a_image_hls__ptr_, hls_channels);
      uint32_t rows = latest_k4a_image_.rows;
      uint32_t cols = latest_k4a_image_.cols;
      uint32_t pixel_count = rows * cols;

      // calculate mean values
      for(uint32_t r=0; r<rows; r++)
      {
        for(uint32_t c=0; c<cols; c++)
        {
          total_blue += color_channels[0].at<uchar>(r,c);
          total_green += color_channels[1].at<uchar>(r,c);
          total_red += color_channels[2].at<uchar>(r,c);
          total_white += hls_channels[1].at<uchar>(r,c);
        }
      }
      float blue_avg = total_blue / pixel_count;
      float green_avg = total_green / pixel_count;
      float red_avg = total_red / pixel_count;
      float white_avg = total_white / pixel_count;

      ROS_INFO("Current blue_avg: [%f]", blue_avg);
      ROS_INFO("Current green_avg: [%f]", green_avg);
      ROS_INFO("Current red_avg: [%f]", red_avg);
      ROS_INFO("Current white_avg: [%f]", white_avg);

      // update rmse
      blue_rmse = k4aRMSE(blue_avg, target_blue_value, i, *blue_rmse_ptr);
      green_rmse = k4aRMSE(green_avg, target_green_value, i, *green_rmse_ptr);
      red_rmse = k4aRMSE(red_avg, target_red_value, i, *red_rmse_ptr);
      white_rmse = k4aRMSE(white_avg, target_white_value, i, *white_rmse_ptr);

      ROS_INFO("Current blue RMSE: [%f]", blue_rmse);
      ROS_INFO("Current green RMSE: [%f]", green_rmse);
      ROS_INFO("Current red RMSE: [%f]", red_rmse);
      ROS_INFO("Current white RMSE: [%f]", white_rmse);

      // update camera params
      *exposure_time_float_ptr -= LEARNING_RATE_ * white_rmse * 100;
      if(*exposure_time_float_ptr < MIN_EXPOSURE_)
      {
        exposure_time_uint = MIN_EXPOSURE_;
      }
      else if(*exposure_time_float_ptr > MAX_EXPOSURE_)
      {
        exposure_time_uint = MAX_EXPOSURE_;
      }
      else
      {
         exposure_time_uint = k4aStandardizeExposure((uint32_t)*exposure_time_float_ptr);
      }

      *white_balance_float_ptr -= LEARNING_RATE_ * (blue_rmse + green_rmse + red_rmse) * 10;
      if(*white_balance_float_ptr < MIN_WHITE_BALANCE_)
      {
        white_balance_uint = MIN_WHITE_BALANCE_;
      }
      else if(*white_balance_float_ptr > MAX_WHITE_BALANCE_)
      {
        white_balance_uint = MAX_WHITE_BALANCE_;
      }
      else
      {
        white_balance_uint = (uint16_t)*white_balance_float_ptr;
      }
      bool update_exp = k4aUpdateExposure(exposure_time_uint, error_code, res_msg);
      bool update_wb = k4aUpdateWhiteBalance(white_balance_uint, error_code, res_msg);
      if(!update_exp || !update_wb)
      {
        return false;
      }
      else
      {
        final_exposure = exposure_time_uint;
        final_white_balance = white_balance_uint;
        final_blue_val = blue_avg;
        final_green_val = green_avg;
        final_red_val = red_avg;
        final_white_val = white_avg;
      }
    }
  }
  ROS_INFO("Successfully updated exposure to [%d] and white balance to [%d]", final_exposure, final_white_balance);
  res_msg = "SGD tuning complete";
  return true;
}

bool K4APORCalibration::k4aUpdateExposureCallback(azure_kinect_ros_driver::k4a_update_exposure::Request &req,
                                                  azure_kinect_ros_driver::k4a_update_exposure::Response &res)
{
  res.message = "";

  ROS_INFO("Received K4A exposure update request: [%d]", req.new_exp);

  int8_t error_code;
  bool expBoundCheck = k4aCameraExposureBoundsCheck(req.new_exp, error_code, res.message);
  if(expBoundCheck)
  {
    uint32_t kinect_exp = k4aStandardizeExposure(req.new_exp);
    bool tuningRes = k4aUpdateExposure(kinect_exp, error_code, res.message);
    res.k4aExposureServiceErrorCode = error_code;
    if(!tuningRes)
    {
      ROS_ERROR("Unable to update exposure_time, k4aUpdateExposure failed");
      return false;
    }
    else
    {
      ROS_INFO("Exposure updated to: [%d]", kinect_exp);
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

bool K4APORCalibration::k4aUpdateWhiteBalanceCallback(azure_kinect_ros_driver::k4a_update_white_balance::Request &req,
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

bool K4APORCalibration::k4aAutoTuneExposureCallback(azure_kinect_ros_driver::k4a_auto_tune_exposure::Request &req,
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

bool K4APORCalibration::k4aSGDTuneCallback(azure_kinect_ros_driver::k4a_sgd_tune::Request &req,
                                           azure_kinect_ros_driver::k4a_sgd_tune::Response &res)
{
  res.message = "";
  int8_t error_code;
  uint32_t calibrated_exposure;
  uint16_t calibrated_white_balance;
  float final_blue;
  float final_green;
  float final_red;
  float final_white;
  std::string res_msg;

  ROS_INFO("Received K4A sgd auto tuning request for:");
  ROS_INFO("blue value: [%f]", req.target_blue_val);
  ROS_INFO("green value: [%f]", req.target_green_val);
  ROS_INFO("red value: [%f]", req.target_red_val);
  ROS_INFO("white value: [%f]", req.target_white_val);

  bool sgdTuningRes = k4aSGDTune(req.target_blue_val,
                                 req.target_green_val,
                                 req.target_red_val,
                                 req.target_white_val,
                                 calibrated_exposure,
                                 calibrated_white_balance,
                                 final_blue,
                                 final_green,
                                 final_red,
                                 final_white,
                                 error_code,
                                 res_msg);
  res.k4aExposureServiceErrorCode = error_code;
  res.message = res_msg;
  res.calibrated_exposure = calibrated_exposure;
  res.calibrated_white_balance = calibrated_white_balance;
  res.final_blue_val = final_blue;
  res.final_green_val = final_green;
  res.final_red_val = final_red;
  res.final_white_val = final_white;

  if(!sgdTuningRes)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void K4APORCalibration::rgbRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
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
      cv::cvtColor(*latest_k4a_image_ptr_, *latest_k4a_image_hls__ptr_, cv::COLOR_BGR2HLS);
      ROS_DEBUG("Updated latest_k4a_image");
    }
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception in K4APORCalibration::rgbRawImageCallback: [%s]", e.what());
  }
}
