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

  subPC = nh_.subscribe("/points2", 1, &K4AExposureCalibration::p2Callback, this);
  subRGBRaw = it.subscribe("/rgb/raw/image", 1, &K4AExposureCalibration::rgbRawImageCallback, this);

  // advertise services
  update_exposure_service = nh_.advertiseService("k4a_update_exposure", &K4AExposureCalibration::k4aUpdateExposureCallback, this);
  auto_tune_exposure_service = nh_.advertiseService("k4a_auto_tune_exposure", &K4AExposureCalibration::k4aAutoTuneExposureCallback, this);
}

K4AExposureCalibration::~K4AExposureCalibration()
{
}

// check if dynamic_reconfigure response has updated exposure
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

// check if target_blue_value has been achieved
bool K4AExposureCalibration::k4aCameraExposureBoundsCheck(int requested_exposure, int& error_code, std::string& res_msg)
{
  int min_exposure = 488;
  int max_exposure = 1000000;
  if(requested_exposure < min_exposure || requested_exposure > max_exposure)
  {
    std::string error_msg = "Requested exposure out of range";
    ROS_ERROR("Requested exposure out of range [%d] - [%d]", min_exposure, max_exposure);
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

// check if target_blue_value has been achieved
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

// check if target_blue_value has been achieved
bool K4AExposureCalibration::k4aBlueBoundsCheck(int target_blue_value, int& error_code, std::string& res_msg)
{
  int min_blue = 0;
  int max_blue = 255;
  if(target_blue_value < min_blue || target_blue_value > max_blue)
  {
    std::string error_msg = "Requested target blue value out of range";
    ROS_ERROR("Requested target blue value out of range [%d] - [%d]", min_blue, max_blue);
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

// // did the node receive an image at all?
// bool K4AExposureCalibration::k4aImagePopulatedCheck(int& error_code, std::string& res_msg)
// {
//   std::string error_msg = "Failed to retrieve latest image in k4aAutoTuneExposure";
//   res_msg = error_msg;
//   error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::IMAGE_NOT_RECEIVED_FAILURE;
//   return false;
// }

// // do we have the latest image?
// bool K4AExposureCalibration::k4aLatestImageReceivedCheck(int& error_code, std::string& res_msg)
// {
//   std::string error_msg = "Failed to retrieve latest image in k4aAutoTuneExposure";
//   res_msg = error_msg;
//   error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::IMAGE_NOT_RECEIVED_FAILURE;
//   return false;
// }

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
  return k4aCameraExposureUpdateCheck(req_exposure, updated_exposure, error_code, res_msg);
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
      res_msg = error_msg;
      error_code = azure_kinect_ros_driver::k4aCameraExposureServiceErrorCode::IMAGE_NOT_RECEIVED_FAILURE;
      return false;
    }

    bool updateExposure = k4aUpdateExposure(exp, error_code, res_msg);
    
    if(!updateExposure)
    {
      // k4aUpdateExposure handles updating response
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
    bool targetBlueCheck = k4aTargetBlueCheck(target_blue_value, current_avg_blue_value, error_code, res_msg);
    if(targetBlueCheck)
    {
      final_exposure = exp;
      break;
    }
  }
  ROS_INFO("Successfully updated exposure to [%d] for target blue value of [%d]", final_exposure, target_blue_value);
  return true;
}

bool K4AExposureCalibration::k4aUpdateExposureCallback(azure_kinect_ros_driver::k4a_update_exposure::Request &req,
                                                       azure_kinect_ros_driver::k4a_update_exposure::Response &res)
{
  // prepare response
  res.message = "";

  ROS_INFO("Received K4A exposure update request: [%d]", req.new_exp);

  // check exposure limits
  int error_code;
  bool expBoundCheck = k4aCameraExposureBoundsCheck(req.new_exp, error_code, res.message);
  if(expBoundCheck)
  {
    bool tuningRes = k4aUpdateExposure(req.new_exp, error_code, res.message);
    res.k4aExposureServiceErrorCode = error_code;
    // k4aUpdateExposure handles updating response
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
  // prepare response
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

    // k4aAutoTuneExposure and k4aUpdateExposure handle updating response
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "k4a_exposure_calibration");
  ros::NodeHandle nh;
  
  K4AExposureCalibration k4a_Exposure_Calibration(nh);
  ROS_INFO("Initialized K4A Exposure Calibration Node");

  ROS_INFO("Spinning K4A Exposure Calibration Node");
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}