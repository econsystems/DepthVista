#ifndef DEPTHVISTA_ENUMS_H
#define DEPTHVISTA_ENUMS_H

/**
 * @brief The data modes that determine the frame output from the device.
 */
typedef enum {
	Depth_IR_Mode = 0,
	Depth_Mode = 2,
	IR_Mode = 3,
	Depth_IR_RGB_VGA_Mode = 4,
	Depth_IR_RGB_HD_Mode = 5,
	RGB_VGA_Mode = 6,
	RGB_HD_Mode = 7,
	RGB_Full_HD_Mode = 8,
	RGB_1200p_Mode = 9,
}DataMode;

typedef enum{
	Low = 0,
	Critical = 1,
	High = 2,
	Disable = 3
}LogLevel;

typedef enum{
	Console = 1,
	LogFile = 2,
	BothConsoleAndLogFile = 3,
}Logging;

typedef enum {
	DualCameraMode = 0,
	RGBCameraMode = 1,
	TOFCameraMode = 2,
}CameraMode;
/**
 * @brief Depth range setting.\n
 *        These set estimated ranges. Detection distances may be greater than what is listed for the given setting. \n
 *        Precision and minimum distance for depth detection varies with longer ranges.
 */
typedef enum {
    NearRange = 0,   	//!< Range0.2- 1.2m
	FarRange = 1,		//!< Range1 - 6m
}DepthRange;


/**
 * @brief Specifies the type of image frame.
 */
typedef enum {
	IRPreviewFrame,				//!< IR frame with 12 bits per pixel multiplied with IR Gain.
    DepthColorMap,              //!< Colormap applied for raw depth frame
	RGBFrame,					//!< RGB Frame with 16bit per pixel.
    DepthRawFrame,				//!< Depth raw frame with 16 bits per pixel.
}FrameType;

/**
 * @brief Return status codes for all APIs.\n
 * 		  Ok = 1 means the API successfully completed its operation.\n
 * 		  All other codes indicate a device, parameter, or API usage error.
 */
typedef enum {
	Ok = 1,						//!< The function completed successfully.
	NotInitialized = -1,		//!< The APIs are not initialized
	NotDeInitialized = -2,		//!< The APIs are not deinitialized
	InvalidFrameType = -3,		//!< The input frame type is invalid.
	NoStreaming = -4,			//!< The camera is not streaming.
	AlreadyStreaming = -5,		//!< The camera is already streaming.
	InvalidNode = -6,			//!< The device Node is invalid.
	CameraNotOpened = -7,		//!< The camera has not been opened.
	InvalidDeviceIndex = -8,	//!< The input device index is invalid.
	NoDeviceConnected = -9,		//!< There is no depth camera connected or the camera has not been connected correctly.
	NoPropertyValueGet = -10,	//!< Cannot get the value for the specified property.
	NoPropertyValueSet = -11,	//!< Cannot set the value for the specified property.
	SysCallFail = -12,			//!< System call function failed.
	InvalidValue = -13,	 		//!< One or more of the parameter values provided are invalid.
	HidWriteFail = -14,			//!< Cannot write the buffer to the hid handle.
	HidReadFail = -15,			//!< Cannot read the buffer from the hid handle.
	UVCOpenFail = -16,
	TimeoutError = -17,			//!< Read on file descriptor timedout.
    InvalidBuffer = -18,		//!< Frame from the camera is invalid.
    RGB_DCalibNotFound = -19,   //!< RGB-D Calibration Data not found in the device
    FrameSeparationFailed = -20, //!< Frame separation logic failed
    RegisterCallBackFailed = -21,  //!< Registering Call back to SDK failed
    CameraAlreadyOpen = -22,  //!< Device is already open>
    InvalidDataMode = -23,   //<! Current Datamode doesn't support the request
    Failed = -24,
	Others = -255,				//!< An unknown error occurred.
}Result;


typedef enum {
	Center = 0,					//!< To get the average depth and ir value at center region of the image.
    CustomPtr = 1,			//!< To get the average depth and ir value at region around the mouse pointer.
}AvgRegion;

typedef enum {
    TOF_UVC_CID_BRIGHTNESS,
    TOF_UVC_CID_CONTRAST,
    TOF_UVC_CID_SATURATION,
    TOF_UVC_CID_WB_AUTO,
    TOF_UVC_CID_GAMMA,
    TOF_UVC_CID_GAIN,
    TOF_UVC_CID_PWR_LINE_FREQ,
	TOF_UVC_CID_WB_TEMP,
    TOF_UVC_CID_SHARPNESS,
    TOF_UVC_CID_EXPOSURE_AUTO,
    TOF_UVC_CID_EXPSOURE_ABS,
}UVCPropID;



typedef enum {
  Not_Calibrated = -1,
	Only_VGA = 0,
	Only_HD = 1,
	Both_VGA_HD = 2
}CalibValidFlag;




#endif	/* DEPTHVISTA_ENUMS_H */
