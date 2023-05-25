uint32 new_exp_cal

# Overall successful exposure calibration
int8 SUCCESS = 1

# Failure to call service 'camera/driver/set_parameters'
int8 CAMERA_EXPOSURE_SET_FAILURE = -1

# Failure to receive Image from callback
int8 IMAGE_NOT_RECEIVED_FAILURE = -2

# Target [x, y] is out-of-bounds Failure
int8 TARGET_X_Y_OUT_OF_BOUNDS_FAILURE = -3

# Auto-update Config Failure
int8 AUTO_UPDATE_CONFIG_FAILURE = -4
