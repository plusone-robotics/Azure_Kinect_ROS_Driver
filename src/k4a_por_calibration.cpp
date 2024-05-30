// PlusOne Robotics
// Author: Shannon Stoehr
// email:  shannon.stoehr@plusonerobotics.com

// Library headers
#include <cmath>
#include <random>
#include <numeric>

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

bool K4APORCalibration::k4aAutoTuneExposure(const uint8_t target_blue_value, uint32_t& final_exposure, uint8_t& final_blue_value, int8_t& error_code, std::string& res_msg)
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
          final_blue_value = current_avg_blue_value;
          break;
        }
      }
    }
  } 
  ROS_INFO("Successfully updated exposure to [%d] for target blue value of [%d] (actual: [%d])", final_exposure, target_blue_value, final_blue_value);
  return true;
}

float K4APORCalibration::k4aCalculateMean(const cv::Mat& img)
{
  float sum = 0.0;
  uint32_t rows = img.rows;
  uint32_t cols = img.cols;
  uint32_t pixel_count = rows * cols;

  for(uint32_t r=0; r<rows; r++)
  {
    for(uint32_t c=0; c<cols; c++)
    {
      sum += img.at<uchar>(r,c);
    }
  }
  return sum / pixel_count;
}

float K4APORCalibration::k4aCalculateStdDev(const cv::Mat& img)
{
  if(img.empty())
  {
    ROS_ERROR("Error: Input data for k4aCalculateStdDev is empty.");
    return -1.0;
  }
  else
  {
    float mean = k4aCalculateMean(img);
    float sumSquaredDiff = 0.0;

      uint32_t rows = img.rows;
      uint32_t cols = img.cols;
      uint32_t pixel_count = rows * cols;

      for(uint32_t r=0; r<rows; r++)
      {
        for(uint32_t c=0; c<cols; c++)
        {
          double diff = img.at<uchar>(r,c) - mean;
          sumSquaredDiff += diff * diff;
        }
      }

    float meanSquaredDiff = sumSquaredDiff / pixel_count;
    float standardDev = std::sqrt(meanSquaredDiff);
    return standardDev;
  }
}

float K4APORCalibration::k4aRMSE(const float current, const float target)
{
  float se, rmse;
  float diff = current - target;
  se = diff * diff;
  rmse = std::sqrt(se);
  return rmse;
}

bool K4APORCalibration::k4aSGDTune(const float target_blue_value,
                                   const float target_green_value,
                                   const float target_red_value,
                                   const float target_white_value,
                                   const float target_std_dev_blue,
                                   const float target_std_dev_green,
                                   const float target_std_dev_red,
                                   const float target_std_dev_white,
                                   uint32_t& final_exposure,
                                   uint16_t& final_white_balance,
                                   int8_t& error_code,
                                   std::string& res_msg)
{
  ROS_INFO("Starting K4A SGD tuning...");

  // initialize exposure setting randomly
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dis(0, 11);

  int start_exposure_index = dis(gen);
  int* exposure_index_ptr = &start_exposure_index;
  ROS_INFO("Starting exposure: [%d]", EXPOSURES_[start_exposure_index]);
  bool start_exposure = k4aUpdateExposure(EXPOSURES_[start_exposure_index], error_code, res_msg);

  bool tuned = false;

  if(!start_exposure)
  {
    return false;
  }
  else
  {
    // adjust camera params using SGD
    while(!tuned)
    {
      bool channelsPop = k4aImagePopulatedCheck(*latest_k4a_image_ptr_, error_code, res_msg);
      if(!channelsPop)
      {
        return false;
      }
      else
      {
        // calculate current state
        cv::Mat color_channels[3];
        cv::Mat hls_channels[3];
        latest_k4a_image_mutex_.lock();
        cv::split(*latest_k4a_image_ptr_, color_channels);
        cv::split(*latest_k4a_image_hls__ptr_, hls_channels);
        latest_k4a_image_mutex_.unlock();

        // calculate current state means
        float blue_mean = k4aCalculateMean(color_channels[0]);
        float green_mean = k4aCalculateMean(color_channels[1]);
        float red_mean = k4aCalculateMean(color_channels[2]);
        float white_mean = k4aCalculateMean(hls_channels[1]);

        // calculate current state standard deviations
        float blue_std = k4aCalculateStdDev(color_channels[0]);
        float green_std = k4aCalculateStdDev(color_channels[1]);
        float red_std = k4aCalculateStdDev(color_channels[2]);
        float white_std = k4aCalculateStdDev(hls_channels[1]);

        // update current state rmse
        float blue_rmse = k4aRMSE(blue_mean, target_blue_value);
        float green_rmse = k4aRMSE(green_mean, target_green_value);
        float red_rmse = k4aRMSE(red_mean, target_red_value);
        float mean_bgr_error = (blue_rmse + green_rmse + red_rmse) / 3;
        float white_rmse = k4aRMSE(white_mean, target_white_value);
        
        // randomly pick around current state, determine step direction
        // check decreased state
        int dec_index = (start_exposure_index - 1) % 12;
        bool dec_exposure = k4aUpdateExposure(EXPOSURES_[dec_index], error_code, res_msg);
        if(!dec_exposure)
        {
          return false;
        }
        else
        {
          // calculate decreased state
          cv::Mat color_channels_dec[3];
          cv::Mat hls_channels_dec[3];
          latest_k4a_image_mutex_.lock();
          cv::split(*latest_k4a_image_ptr_, color_channels_dec);
          cv::split(*latest_k4a_image_hls__ptr_, hls_channels_dec);
          latest_k4a_image_mutex_.unlock();

          // calculate decreased state means
          float blue_mean_dec = k4aCalculateMean(color_channels_dec[0]);
          float green_mean_dec = k4aCalculateMean(color_channels_dec[1]);
          float red_mean_dec = k4aCalculateMean(color_channels_dec[2]);
          float white_mean_dec = k4aCalculateMean(hls_channels_dec[1]);

          // calculate decreased state standard deviations
          float blue_std_dec = k4aCalculateStdDev(color_channels_dec[0]);
          float green_std_dec = k4aCalculateStdDev(color_channels_dec[1]);
          float red_std_dec = k4aCalculateStdDev(color_channels_dec[2]);
          float white_std_dec = k4aCalculateStdDev(hls_channels_dec[1]);

          // update decreased state rmse
          float blue_rmse_dec = k4aRMSE(blue_mean_dec, target_blue_value);
          float green_rmse_dec = k4aRMSE(green_mean_dec, target_green_value);
          float red_rmse_dec = k4aRMSE(red_mean_dec, target_red_value);
          float mean_bgr_error_dec = (blue_rmse_dec + green_rmse_dec + red_rmse_dec) / 3;
          float white_rmse_dec = k4aRMSE(white_mean_dec, target_white_value);

          if(mean_bgr_error_dec > mean_bgr_error)
          {
            // check increased state
            int inc_index = (start_exposure_index + 1) % 12;
            bool inc_exposure = k4aUpdateExposure(EXPOSURES_[inc_index], error_code, res_msg);
            if(!inc_exposure)
            {
              return false;
            }
            else
            {
              // calculate increased state
              cv::Mat color_channels_inc[3];
              cv::Mat hls_channels_inc[3];
              latest_k4a_image_mutex_.lock();
              cv::split(*latest_k4a_image_ptr_, color_channels_inc);
              cv::split(*latest_k4a_image_hls__ptr_, hls_channels_inc);
              latest_k4a_image_mutex_.unlock();

              // calculate increased state means
              float blue_mean_inc = k4aCalculateMean(color_channels_inc[0]);
              float green_mean_inc = k4aCalculateMean(color_channels_inc[1]);
              float red_mean_inc = k4aCalculateMean(color_channels_inc[2]);
              float white_mean_inc = k4aCalculateMean(hls_channels_inc[1]);

              // // calculate increased state standard deviations
              float blue_std_inc = k4aCalculateStdDev(color_channels_inc[0]);
              float green_std_inc = k4aCalculateStdDev(color_channels_inc[1]);
              float red_std_inc = k4aCalculateStdDev(color_channels_inc[2]);
              float white_std_inc = k4aCalculateStdDev(hls_channels_inc[1]);

              // update increased state rmse
              float blue_rmse_inc = k4aRMSE(blue_mean_inc, target_blue_value);
              float green_rmse_inc = k4aRMSE(green_mean_inc, target_green_value);
              float red_rmse_inc = k4aRMSE(red_mean_inc, target_red_value);
              float mean_bgr_error_inc = (blue_rmse_inc + green_rmse_inc + red_rmse_inc) / 3;
              float white_rmse_inc = k4aRMSE(white_mean_inc, target_white_value);

              if(mean_bgr_error_inc > mean_bgr_error)
              {
                final_exposure = EXPOSURES_[start_exposure_index];
                final_white_balance = 4500;
                ROS_INFO("Successfully updated exposure to [%d] and white balance to [%d]", final_exposure, final_white_balance);
                ROS_INFO("Mean BGR Error: [%f]", mean_bgr_error);
                res_msg = "SGD tuning complete";
                tuned = true;
                return true;
              }
              else
              {
                // move to increased state, repeat
                *exposure_index_ptr = inc_index;
                continue;
              }
            }
          }
          else
          {
            // move to decreased state, repeat
            *exposure_index_ptr = dec_index;
            continue;
          }
        }
      }
    }
  }
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
  uint8_t calibrated_blue_value;
  std::string res_msg;

  ROS_INFO("Received K4A exposure auto tuning request for blue value: [%d]", req.target_blue_val);

  bool autoTuningRes = k4aAutoTuneExposure(req.target_blue_val, calibrated_exposure, calibrated_blue_value, error_code, res_msg);
  res.k4aExposureServiceErrorCode = error_code;
  res.message = res_msg;
  res.calibrated_exposure = calibrated_exposure;
  res.calibrated_blue_val = calibrated_blue_value;

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
                                 req.std_dev_blue,
                                 req.std_dev_green,
                                 req.std_dev_red,
                                 req.std_dev_white,
                                 calibrated_exposure,
                                 calibrated_white_balance,
                                 error_code,
                                 res_msg);
  res.k4aExposureServiceErrorCode = error_code;
  res.message = res_msg;
  res.calibrated_exposure = calibrated_exposure;
  res.calibrated_white_balance = calibrated_white_balance;

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
