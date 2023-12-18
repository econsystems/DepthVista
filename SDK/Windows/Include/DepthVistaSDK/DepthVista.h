#ifndef DEPTHVISTA_H
#define DEPTHVISTA_H

#ifdef _WIN32
#include <windows.h>
#endif

#include "DepthVista_types.h"
#include "DepthVista_enums.h"
#include "DepthVista_Export.h"

/**
* @brief 		Initializes the API on the device. This function must be invoked before any other DepthVista APIs.
* @return		::Ok if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result Initialize();
/**
* @brief 		DeInitialize the API on the device and clears all resources allocated by the API. After invoking this function, no other DepthVista APIs can be invoked.
* @return		::Ok	if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result DeInitialize();
/**
* @brief 		Returns the number of camera devices currently connected.
* @param[out]	gDeviceCount	Pointer to a 32-bit integer variable in which to return the device count.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDeviceCount(uint32_t *gDeviceCount);
/**
* @brief 		Returns the info lists of the deviceCount camera devices.
* @param[in] 	deviceCount		the number of camera devices.
* @param[out]	gDevicesList	Pointer to a buffer in which to store the deviceCount devices infos.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDeviceListInfo(uint32_t deviceCount, DeviceInfo* gDevicesList);
/**
* @brief 		Returns the info of the deviceIndex camera device.
* @param[in] 	deviceIndex	The index of the device to open. Device indices range from 0 to device count - 1.
* @param[out]	gDevice	Pointer to a buffer in which to store the device info.
* @return 		::Ok	if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDeviceInfo(uint32_t deviceIndex,DeviceInfo* gDevice);
/**
* @brief 		Opens the device specified by deviceIndex. The device must be subsequently closed using CloseDevice().
* @param[in] 	deviceIndex	The index of the device to open. Device indices range from 0 to device count - 1.
* @return: 		::Ok	if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result OpenDevice(uint32_t deviceIndex);
/**
* @brief 		Closes the device that was opened using OpenDevice.
* @return: 		::Ok	if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result CloseDevice();
/**
* @brief 		Return if the device is opened or not.
* @return: 		::Ok	if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result IsOpened();
/**
* @brief 		Retrieves the image frame from the device that was opened using OpenDevice(). This API must be invoked before retrieving frame data using GetToFFrame().
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetNextFrame();
/**
* @brief 		Returns the image data of ToFFrame structure for the current frame specified by FrameType from the device that was opened using OpenDevice().
Before invoking this API, invoke GetNextFrame() to capture one image frame from the device.
* @param[in] 	frameType		The image frame type.
* @param[out]	Frame		Pointer to a buffer in which to store the returned image data.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetToFFrame(FrameType frameType, ToFFrame* Frame);
/**
* @brief  		Gets the number of frames obtained from the device per second
* @return 		The number of frames obtained from the stream
*/
DEPTHVISTA_EXPORT int GetFramesPerSecond();
/**
* @brief  		Sets the stream mode for the device specified by deviceHandle.
* @param[in]	sDataMode		The output stream mode. See ::DataMode for more information.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetDataMode( DataMode sDataMode);
/**
* @brief  		Returns the data mode for the device specified by deviceHandle.
* @param[out]	gDataMode		The output data mode. See ::DataMode for more information.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDataMode( DataMode* gDataMode);
/**
* @brief  		Sets the output Depth range mode for the device specified by deviceHandle.
* @param[in]	sDepthRange		The output data mode. See ::DepthRange for more information.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetDepthRange( uint16_t sDepthRange);
/**
* @brief  		Returns the output depth range from the device specified by deviceHandle.
* @param[Out]	gDepthRange		The output data mode. See ::DepthRange for more information.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDepthRange( uint16_t *gDepthRange);
/**
* @brief 		Sets the device Coring value on a device.
* @param[in] 	sTOFCore 		The Coring value to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetTOFCoring( uint16_t sTOFCore);
/**
* @brief 		Returns the device's Coring value.
* @param[in] 	gTOFCore 		Pointer to a 16-bit unsigned integer variable in which to store the Coring value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetTOFCoring( uint16_t* gTOFCore);
/**
* @brief 		Sets the device IR Gain on a device.
* @param[in] 	sTOFIRGain 		The IR Gain value to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetTOFIRGain( uint16_t sTOFIRGain);
/**
* @brief 		Returns the device's IR Gain value.
* @param[in] 	gTOFIRGain 		Pointer to a 16-bit unsigned integer variable in which to store the IR Gain value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetTOFIRGain( uint16_t* gTOFIRGain);
/**
* @brief 		Returns the the device's UVC control value of control specified by gControlID.
* @param[in] 	gControlID			The UVC control id.
* @param[out]	gControlValue 	Pointer to a 32-bit signed integer variable in which to store the UVC control value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetUVCControl( int32_t gControlID, UVCProp* gControlValue);
/**
* @brief 		Sets the the device's UVC control value of control specified by gControlID.
* @param[in] 	gControlID			The UVC control id.
* @param[in] 	gControlValue 	The UVC control value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetUVCControl( int32_t sControlID, int32_t sControlValue);
/**
* @brief 		Sets the IMU Configuration mode of the IMU in the device.
* @param[in] 	lIMUConfig			IMUCONFIG_TypeDef structure containing Config details.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetIMUConfig(IMUCONFIG_TypeDef lIMUConfig);
/**
* @brief 		Gets the IMU Configuration mode of the IMU in the device.
* @param[out] 	lIMUConfig			pointer to IMUCONFIG_TypeDef structure.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetIMUConfig(IMUCONFIG_TypeDef* lIMUConfig);
/**
* @brief 		Sets the IMU Value update in the device.
* @param[out] 	lIMUInput			pointer to IMUDATAINPUT_TypeDef structure.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result ControlIMUCapture(IMUDATAINPUT_TypeDef* lIMUInput);
/**
* @brief 		Gets the IMU axis data from the device.
* @param[out] 	lIMUAxes	pointer to IMUDATAOUTPUT_TypeDef structure.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
#ifdef  __linux__
DEPTHVISTA_EXPORT Result GetIMUValue(IMUDATAOUTPUT_TypeDef *lIMUAxes);
#elif _WIN32
/**
* @brief 		Gets the IMU axis data from the device.
* @param[out] 	lIMUAxes	pointer to IMUDATAOUTPUT_TypeDef structure.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetIMUValue(IMUDATAOUTPUT_TypeDef* lIMUAxes);
#endif
/**
* @brief 		Returns the Anti Flicker detection Mode from the device.
* @param[out] 	gAFDetect			Pointer to a 8-bit unsigned integer variable in which to store the Anti Flicker detection Mode.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetAntiFlickerDetection( uint8_t* gAFDetect);
/**
* @brief 		Sets the Anti Flicker detection Mode on the device.
* @param[in] 	gAFDetect			The Anti Flicker detection Mode to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetAntiFlickerDetection( uint8_t sAFDetect);
/**
* @brief 		Returns the Scene Mode from the device .
* @param[out]	gSceneMode			Pointer to a 8-bit unsigned integer array in which to store the Scene Mode.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetSceneMode( uint8_t* gSceneMode);
/**
* @brief 		Sets the Scene Mode to the device.
* @param[in] 	sSceneMode		The Scene Mode to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetSceneMode( uint8_t sSceneMode);
/**
* @brief 		Returns the Special Effect from the device.
* @param[out] 	gSplEffect		Pointer to a 8-bit unsigned integer array in which to store the Special Effect.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetSpecialEffect( uint8_t* gSplEffect);
/**
* @brief 		Sets the Special Effect to the device.
* @param[in] 	sSplEffect		the Special Effect to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetSpecialEffect( uint8_t sSplEffect);
/**
* @brief 		Returns the Denoise value from the device.
* @param[out]	gDenoise		Pointer to a 8-bit unsigned integer array in which to store the Denoise value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDenoise( uint8_t* gDenoise);
/**
* @brief 		Sets the Denoise value to the device.
* @param[in] 	sSplEffect		the Denoise value to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetDenoise( uint8_t sDenoise);
/**
* @brief 		Returns the ROI Mode and Window size from the device.
* @param[out]	gROIMode		Pointer to a 8-bit unsigned integer array in which to store the ROI Mode.
* @param[out]	gWinSize		Pointer to a 8-bit unsigned integer array in which to store the Window size.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetAutoExposureROI( uint8_t* gROIMode,uint8_t* gWinSize);
/**
* @brief 		Sets the ROI Mode and Window size to the device.
* @param[in] 	sROIMode		The ROI Mode to set.
* @param[in] 	sWidth			The width of the frame.
* @param[in] 	sHeight			The height of the frame.
* @param[in] 	sXCor			The X co-ordinate on which to set ROI.
* @param[in] 	sYCor			The Y co-ordinate on which to set ROI.
* @param[in] 	sWinSize.		The ROI window size.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetAutoExposureROI( uint8_t sROIMode,uint32_t sWidth, uint32_t sHeight, uint32_t sXCor, uint32_t sYCor, uint8_t sWinSize);
/**
* @brief 		Returns the Orientation value from the device.
* @param[out]	gOrientation	Pointer to a 8-bit unsigned integer array in which to store the Orientation value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetOrientation( uint8_t* gOrientation);
/**
* @brief 		Sets the Orientation value to the device.
* @param[in] 	sOrientation	The Orientation value to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetOrientation( uint8_t sOrientation);
/**
* @brief 		Returns the Face Detection option to the device.
* @param[out] 	gFacedet	Pointer to a 8-bit unsigned integer array in which to store the Face Detection option.
* @param[out] 	gStatusStruct Pointer to a 8-bit unsigned integer array in which to store the Status Structure.
* @param[out] 	gOverlayRect	Pointer to a 8-bit unsigned integer array in which to store the Overlay Rectangle option.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetFaceDetection( uint8_t* gFacedet, uint8_t* gStatusStruct, uint8_t* gOverlayRect);
/**
* @brief 		Sets the Face Detection option to the device.
* @param[in] 	sFacedet	The Face Detection option to set.
* @param[in] 	sStatusStruct
* @param[in] 	sOverlayRect	The Face Detection Overlay Rectangle option to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetFaceDetection( uint8_t sFacedet, uint8_t sStatusStruct, uint8_t sOverlayRect);
/**
* @brief 		Returns the Smile Detection option to the device.
* @param[out] 	gSmiledet	Pointer to a 8-bit unsigned integer array in which to store the Smile Detection option.
* @param[out] 	gStatusStruct Pointer to a 8-bit unsigned integer array in which to store the Status Structure.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetSmileDetection( uint8_t* gSmiledet, uint8_t* gStatusStruct);
/**
* @brief 		Sets the Smile Detection option to the device.
* @param[in] 	gSmiledet	The Smile Detection option to set.
* @param[in] 	sStatusStruct
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetSmileDetection( uint8_t sSmiledet, uint8_t sStatusStruct);

/**
* @brief 		Returns the IMU embedded option from the device.
* @param[out] 	gIMUData	Pointer to a 8-bit unsigned integer array in which to store the IMU embedded option.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetIMUEmbeddedData(uint8_t*  gIMUData);

/**
* @brief 		Returns the IMU embedded option to the device.
* @param[out] 	sIMUData	IMU embedded option to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetIMUEmbeddedData(uint8_t  sIMUData);

/**
* @brief 		Returns the Exposure Compensation value from the device.
* @param[out]	gExpoComp	Pointer to a 32-bit unsigned integer array in which to store the Exposure Compensation value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetExposureCompensation( uint32_t* gExpoComp);
/**
* @brief 		Sets the Exposure Compensation value to the device.
* @param[in] 	sExpoComp	The Exposure Compensation value to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetExposureCompensation( uint32_t sExpoComp);
/**
* @brief 		Returns the Exposure Compensation value from the device.
* @param[out]	gExpoComp	Pointer to a 8-bit unsigned integer array in which to store the Exposure Compensation value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetFrameRateCtrl( uint8_t* gFrameRateCtrl);
/**
* @brief 		Sets the FrameRate control value to the device.
* @param[in] 	sExpoComp	The FrameRate control value to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetFrameRateCtrl( uint8_t sFrameRateCtrl);
/**
* @brief 		Sets All the HID control to default values on the device.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetDefault();
/**
* @brief 		Returns the Unique ID from the device.
* @param[out]	gUniqueID	Pointer to a 32-bit unsigned integer array in which to store the Unique ID.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetUniqueID( uint64_t* gUniqueID);
/**
* @brief  		Sets the average Depth and IR calculation area specified by region.
* @param[in] 	region	The region where you want to calculate the average depth and IR.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetAvgRegion(AvgRegion region);
/**
* @brief  		Sets the Median Filter Flag
* @param[in] 	value to be set
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetFilterType(int ctrlID, bool selected);
/**
* @brief  		Sets the average Depth and IR calculation area specified by pos.
* @param[in] 	pos	The x and y co-ordinates of mouse pointer where you want to calculate the average depth and IR.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetDepthPos(DepthPtr pos);
/**
* @brief  		Sets the cursor color to the white or black.
* @param[in] 	color The color value which needs to be set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetCursorColor(int color);
/**
* @brief  		Enables or Disables Undistortion.
* @param[in] 	undistort - The value which determine whether it enables or disables.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetUnDistortion(int undistort);

/**
* @brief  		Enables or Disables rgbDMapping.
* @param[in] 	rgbDMapping - The value which determine whether it enables or disables.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetRGBDMapping(int rgbDMapping);
/**
* @brief  		Enables or Disables planarization.
* @param[in] 	planarize The value which determine whether it enables or disables.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetPlanarization(int planarize);
/**
* @brief 		Returns the average depth & IR and sigma depth and IR.
* @param[out]	gAvgDepth	Pointer to a 32-bit unsigned integer array in which to store the Average depth value.
* @param[out]	gStdDepth	Pointer to a 32-bit unsigned integer array in which to store the Sigma depth value.
* @param[out]	gAvgIR	Pointer to a 32-bit unsigned integer array in which to store the Average IR value.
* @param[out]	gStdIR	Pointer to a 32-bit unsigned integer array in which to store the Sigma IR value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDepthIRValues(int *gAvgDepth,int *gStdDepth,int *gAvgIR,int *gStdIR);
/**
* @brief  		Register a callback function specified by cb when frame is received from the camera.
* @param[in] 	Function pointer of the callback function.
*/
DEPTHVISTA_EXPORT void RegisterFrameCallback(std::function<void(int)> cb);
/**
* @brief  		Register a callback function specified by cb when notification is received from handle.
* @param[in] 	Function pointer of the callback function.
*/
DEPTHVISTA_EXPORT void RegisterNotificationCallback(std::function<void(int)> cb);
/**
* @brief  		Updates the ROI for which the avg depth is being calculated
* @param[in] 	avg_x	width of ROI.
* @param[in] 	avg_y	height of ROI.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result UpdateAvgXandY(int avg_x, int avg_y);
/**
* @brief  		Updates the color map properties
* @param[in] 	min			Range min.
* @param[in] 	max			Range max.
* @param[in] 	colorMap	colorMap.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result UpdateColorMap(int min, int max, int colormap);
/**
* @brief  		Sets Average IR Display
* @param[in] 	avgIRDisplay Zero to unset, non-zero to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetAvgIRDisplay(uint8_t avgIRDisplay);
/**
* @brief 		produces a error message on standard error describing the last error
		encountered during a call to the tof library function.
* @param[in] 	message			if the argument is passed, it is printed along with error description seperated by a colon and a blank
*/
DEPTHVISTA_EXPORT void pError(std::string message = "");
/**
* @brief  		Reads the firmware version from the device
* @param[out] 	gMajorVersion			Major Firmware Version
* @param[out] 	gMinorVersion1			Minor Firmware Version 1
* @param[out] 	gMinorVersion2			Minor Firmware Version 2
* @param[out] 	gMinorVersion3			Minor Firmware Version 3
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result ReadFirmwareVersion(uint8_t* gMajorVersion, uint8_t* gMinorVersion1, uint16_t* gMinorVersion2, uint16_t* gMinorVersion3);
/**
* @brief  		Requests the device to read intrinsic calibration data of the depth camera
* @param[out] 	lDepthIntFileLength			Length of the valid calibration Data
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result CalibReadReqDepthInstrinsic(int* lDepthIntFileLength);
/**
* @brief  		Reads the intrinsic calibration data of the depth camera
* @param[out] 	lDepthIntFileLength			Length of the valid calibration Data read
* @param[out] 	DepthIntrinsicBuffer		intrinsic calibration Data of depth camera
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result CalibReadDepthInstrinsic(int* lDepthIntFileLength, unsigned char* DepthIntrinsicBuffer);
/**
* @brief  		Requests the device to read intrinsic calibration data of the RGB camera
* @param[out] 	lRGBIntFileLength			Length of the valid calibration Data
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result CalibReadReqRGBInstrinsic(int* lRGBIntFileLength);
/**
* @brief  		Reads the intrinsic calibration data of the RGB camera
* @param[out] 	lRGBIntFileLength			Length of the valid calibration Data read
* @param[out] 	RGBIntrinsicBuffer		intrinsic calibration Data of RGB camera
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result CalibReadRGBInstrinsic(int* lRGBIntFileLength, unsigned char* RGBIntrinsicBuffer);
/**
* @brief  		Requests the device to read Extrinsic calibration data of the device
* @param[out] 	lExtFileLength			Length of the valid calibration Data
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result CalibReadReqExtrinsic(int* lExtFileLength);
/**
* @brief  		Reads the Extrinsic calibration data of the Device
* @param[out] 	lExtFileLength			Length of the valid calibration Data read
* @param[out] 	ExtrinsicBuffer		Extrinsic calibration data of the Device
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result CalibReadExtrinsic(int* lExtFileLength, unsigned char* ExtrinsicBuffer);

#endif	/* DEPTHVISTA_H */
