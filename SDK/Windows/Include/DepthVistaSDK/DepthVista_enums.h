#ifndef DEPTHVISTA_ENUMS_H
#define DEPTHVISTA_ENUMS_H

/**
 * @brief The data modes that determine the frame output from the device.
 */
typedef enum {
	ModeUnknown = 255,
	Depth_IR_Mode = 0, 			
	Raw_Mode = 254,			
	Depth_Mode = 2,
	IR_Mode = 3,
	Depth_IR_RGB_VGA_Mode = 4,
	Depth_IR_RGB_HD_Mode = 5,
	RGB_VGA_Mode = 6,
	RGB_HD_Mode = 7,
	RGB_Full_HD_Mode = 8,
	RGB_Original_Mode = 9,
}DataMode;

typedef enum {
	Dual_Camera_Mode = 0,
	RGB_Camera_Mode = 1,
	TOF_Camera_Mode = 2,
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
	// Raw Mode frames
	RawIRFrame,					//!< Raw IR frame with 16 bits per pixel.
	// Depth Mode frames
	DepthPreviewFrame,			//!< Depth frame with 8 bits per pixel.
	IRPreviewFrame,				//!< IR frame with 12 bits per pixel multiplied with IR Gain.
	DepthColorMap,
	RGBFrame,					//!< RGB Frame with 16bit per pixel.
	// Raw frames.
	DepthIrRgbRawFrame,			//!< Raw Depth+IR+RGB frame with 40 bits per pixel.
	DepthIrRawFrame,			//!< Depth+IR frame with 12 bits per pixel.
	DepthRawFrame,				//!< Depth frame with 12 bits per pixel.
	IRRawFrame,					//!< Raw IR frame with 12 bits per pixel.
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
    InvalidHidHandle = -19,
    RGB_DCalibNotFound = -20,   //!< RGB-D Calibration Data not found in the device
    FrameSeparationFailed = -21, //!< Frame separation logic failed
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
	PIX_FMT_UYVY,
	PIX_FMT_Y16,
	PIX_FMT_RGB,
}PixFormat;

#endif	/* DEPTHVISTA_ENUMS_H */
