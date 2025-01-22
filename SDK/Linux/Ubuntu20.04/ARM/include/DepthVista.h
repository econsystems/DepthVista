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
* @param[out]	deviceCount	Pointer to a 32-bit integer variable in which to return the device count.
* @return 		::Ok	if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDeviceCount(uint32_t *deviceCount);
/**
* @brief 		Returns the info of the deviceIndex camera device.
* @param[in] 	deviceIndex     The index of the device to open. Device indices range from 0 to device count - 1.
* @param[out]	device         Pointer to a buffer in which to store the device info.
* @return 		::Ok	if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDeviceInfo(uint32_t deviceIndex,DeviceInfo* device);
/**
* @brief 		Returns the info lists of the deviceCount camera devices.
* @param[in] 	deviceCount	the number of camera devices.
* @param[out]	devicesList	Pointer to DeviceInfo structure which retrieves an array of device information of all the number of devices mentioned in deviceCount.
* @return 		::Ok	if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDeviceListInfo(uint32_t deviceCount, DeviceInfo* devicesList);
/**
* @brief 		Opens the device specified by info. The device must be subsequently closed using CloseDevice().
* @param[in] 	devInfo	The DeviceInfo of the device to open.
* @param[out]	devHandle	Pointer to DeviceHandle structure which retrieves unique handle for device corresponding to devInfo.
* @return: 		::Ok	if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result OpenDevice(DeviceInfo devInfo, DeviceHandle *devHandle);//change to device info
/**
* @brief 		Closes the device that was opened using OpenDevice.
* @param[in] 	devHandle	The DeviceHandle of the device to close.
* @return: 		::Ok	if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result CloseDevice(DeviceHandle devHandle);
/**
* @brief 		Return if the device is opened or not.
* @param[in] 	devHandle	The DeviceHandle of the device whose open status has to be checked.
* @return: 		::Ok	if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result IsOpened(DeviceHandle devHandle);
/**
* @brief 		Retrieves the image frame from the device that was opened using OpenDevice(). This API must be invoked before retrieving frame data using GetToFFrame().
* @param[in] 	devHandle	The DeviceHandle of the device from which frames are being requested.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetNextFrame(DeviceHandle devHandle);
/**
* @brief 		Returns the image data of ToFFrame structure for the current frame specified by FrameType from the device that was opened using OpenDevice().
Before invoking this API, invoke GetNextFrame() to capture one image frame from the device.
* @param[in] 	devHandle	The DeviceHandle of the device from which frames are being requested.
* @param[in] 	frameType	The image frame type.
* @param[out]	Frame		Pointer to a buffer in which to store the retrieved image data.
* @return 		::Ok	if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetToFFrame(DeviceHandle devHandle, FrameType frameType, ToFFrame* Frame);
/**
* @brief 		Returns the image data of Frames structure for the current frame from the device that was opened using OpenDevice().
Before invoking this API, invoke GetNextFrame() to capture one image frame from the device.
* @param[in] 	devHandle	The DeviceHandle of the device from which frames are being requested.
* @param[out]	frames		Pointer to a buffer in which to store the retrieved image data.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetFrames(DeviceHandle devHandle, Frames* frames);
/**
* @brief  		Gets the number of frames obtained from the device per second
* @param[in] 	devHandle	The DeviceHandle of the device from which Frames per second is requested.
* @return 		The number of frames obtained from the stream per second.
*/
DEPTHVISTA_EXPORT int GetFramesPerSecond(DeviceHandle devHandle);
/**
* @brief  		Sets the stream mode for the device specified by deviceHandle.
* @param[in] 	devHandle	The DeviceHandle of the device whose DataMode has to be set.
* @param[in]	dataMode	The output stream mode. See ::DataMode for more information.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetDataMode(DeviceHandle devHandle, DataMode dataMode);
/**
* @brief  		Returns the data mode for the device specified by deviceHandle.
* @param[in] 	devHandle	The DeviceHandle of the device whose DataMode have to be requested.
* @param[out]	dataMode	The output data mode. See ::DataMode for more information.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDataMode(DeviceHandle devHandle, DataMode* dataMode);
/**
* @brief  		Sets the output Depth range mode for the device specified by deviceHandle.
* @param[in] 	devHandle	The DeviceHandle of the device whose DepthRange has to be set.
* @param[in]	depthRange	The output depth range. See ::DepthRange for more information.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetDepthRange(DeviceHandle devHandle, uint16_t depthRange);
/**
* @brief  		Returns the output depth range from the device specified by deviceHandle.
* @param[in] 	devHandle	The DeviceHandle of the device whose DepthRange have to be requested.
* @param[Out]	depthRange	The output depth range. See ::DepthRange for more information.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDepthRange(DeviceHandle devHandle, uint16_t *depthRange);
/**
* @brief 		Sets the device Coring value on a device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Coring value has to be set.
* @param[in] 	TOFCore 	The Coring value to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetTOFCoring(DeviceHandle devHandle, uint16_t TOFCore);
/**
* @brief 		Returns the device's Coring value.
* @param[in] 	devHandle	The DeviceHandle of the device whose Coring value have to be requested.
* @param[in] 	TOFCore 	Pointer to a 16-bit unsigned integer variable in which to store the Coring value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetTOFCoring(DeviceHandle devHandle, uint16_t* TOFCore);
/**
* @brief 		Sets the device IR Gain on a device.
* @param[in] 	devHandle	The DeviceHandle of the device whose IR Gain value has to be set.
* @param[in] 	TOFIRGain 	The IR Gain value to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetTOFIRGain(DeviceHandle devHandle, uint16_t TOFIRGain);
/**
* @brief 		Returns the device's IR Gain value.
* @param[in] 	devHandle	The DeviceHandle of the device whose IR Gain value have to be requested.
* @param[in] 	TOFIRGain 	Pointer to a 16-bit unsigned integer variable in which to store the IR Gain value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetTOFIRGain(DeviceHandle devHandle, uint16_t* TOFIRGain);
/**
* @brief 		Returns the the device's UVC control value of control specified by gControlID.
* @param[in] 	devHandle	The DeviceHandle of the device whose UVC control value have to be requested.
* @param[in] 	controlID	The UVC control id.
* @param[out]	controlValue 	Pointer to a 32-bit signed integer variable in which to store the UVC control value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetUVCControl(DeviceHandle devHandle, int32_t controlID, UVCProp* controlValue);
/**
* @brief 		Sets the the device's UVC control value of control specified by gControlID.
* @param[in] 	devHandle	The DeviceHandle of the device whose UVC control value has to be set.
* @param[in] 	controlID	The UVC control id.
* @param[in] 	controlValue 	The UVC control value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetUVCControl(DeviceHandle devHandle, int32_t controlID, int32_t controlValue);
/**
* @brief 		Sets the IMU Configuration mode of the IMU in the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose IMU Configuration has to be set.
* @param[in] 	IMUConfig	IMUCONFIG_TypeDef structure containing Config details.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetIMUConfig(DeviceHandle devHandle, IMUCONFIG_TypeDef IMUConfig);
/**
* @brief 		Gets the IMU Configuration mode of the IMU in the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose IMU Configuration have to be requested.
* @param[out] 	IMUConfig	Pointer to IMUCONFIG_TypeDef structure.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetIMUConfig(DeviceHandle devHandle, IMUCONFIG_TypeDef* IMUConfig);
/**
* @brief 		Sets the IMU Value update in the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose IMU configuration has to be controlled.
* @param[out] 	IMUInput	Pointer to IMUDATAINPUT_TypeDef structure.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result ControlIMUCapture(DeviceHandle devHandle, IMUDATAINPUT_TypeDef* IMUInput);
/**
* @brief 		Gets the IMU axis data from the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose IMU values have to be requested.
* @param[out] 	IMUAxes	pointer to IMUDATAOUTPUT_TypeDef structure.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetIMUValue(DeviceHandle devHandle, IMUDATAOUTPUT_TypeDef *IMUAxes);

/**
* @brief 		Returns the Anti Flicker detection Mode from the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Anti-flicker detection mode have to be requested.
* @param[out] 	AFDetect	Pointer to a 8-bit unsigned integer variable in which to store the Anti Flicker detection Mode.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetAntiFlickerDetection(DeviceHandle devHandle, uint8_t* AFDetect);
/**
* @brief 		Sets the Anti Flicker detection Mode on the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Anti-flicker detection mode has to be set.
* @param[in] 	AFDetect	The Anti Flicker detection Mode to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetAntiFlickerDetection(DeviceHandle devHandle, uint8_t AFDetect);
/**
* @brief 		Returns the Scene Mode from the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Scene mode have to be requested.
* @param[out]	sceneMode	Pointer to a 8-bit unsigned integer array in which to store the Scene Mode.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetSceneMode(DeviceHandle devHandle, uint8_t* sceneMode);
/**
* @brief 		Sets the Scene Mode to the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Scene mode has to be set.
* @param[in] 	sceneMode	The Scene Mode to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetSceneMode(DeviceHandle devHandle, uint8_t sceneMode);
/**
* @brief 		Returns the Special Effect from the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Special effects value have to be requested.
* @param[out] 	splEffect	Pointer to a 8-bit unsigned integer array in which to store the Special Effect.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetSpecialEffect(DeviceHandle devHandle, uint8_t* splEffect);
/**
* @brief 		Sets the Special Effect to the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Special effects value has to be set.
* @param[in] 	splEffect	The Special Effect to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetSpecialEffect(DeviceHandle devHandle, uint8_t splEffect);
/**
* @brief 		Returns the Denoise value from the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Denoise value have to be requested.
* @param[out]	denoise	Pointer to a 8-bit unsigned integer array in which to store the Denoise value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDenoise(DeviceHandle devHandle, uint8_t* denoise);
/**
* @brief 		Sets the Denoise value to the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Denoise value has to be set.
* @param[in] 	denoise	The Denoise value to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetDenoise(DeviceHandle devHandle, uint8_t denoise);
/**
* @brief 		Returns the ROI Mode and Window size from the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose ROI mode and window size have to be requested.
* @param[out]	ROIMode	Pointer to a 8-bit unsigned integer array in which to store the ROI Mode.
* @param[out]	winSize	Pointer to a 8-bit unsigned integer array in which to store the Window size.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetAutoExposureROI(DeviceHandle devHandle, uint8_t* ROIMode,uint8_t* winSize);
/**
* @brief 		Sets the ROI Mode and Window size to the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose ROI mode has to be set.
* @param[in] 	ROIMode	The ROI Mode to set.
* @param[in] 	Width		The width of the frame.
* @param[in] 	Height	The height of the frame.
* @param[in] 	XCor		The X co-ordinate on which to set ROI.
* @param[in] 	YCor		The Y co-ordinate on which to set ROI.
* @param[in] 	winSize.	The ROI window size.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetAutoExposureROI(DeviceHandle devHandle, uint8_t ROIMode,uint32_t Width, uint32_t Height, uint32_t XCor, uint32_t YCor, uint8_t winSize);
/**
* @brief 		Returns the Orientation value from the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Orientation value have to be requested.
* @param[out]	orientation	Pointer to a 8-bit unsigned integer array in which to store the Orientation value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetOrientation(DeviceHandle devHandle, uint8_t* orientation);
/**
* @brief 		Sets the Orientation value to the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Orientation value has to be set.
* @param[in] 	orientation	The Orientation value to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetOrientation(DeviceHandle devHandle, uint8_t orientation);
/**
* @brief 		Returns the Face Detection option to the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Face Detection option have to be requested.
* @param[out] 	facedet	Pointer to a 8-bit unsigned integer array in which to store the Face Detection option.
* @param[out] 	statusStruct	Pointer to a 8-bit unsigned integer array in which to store the Status Structure.
* @param[out] 	overlayRect	Pointer to a 8-bit unsigned integer array in which to store the Overlay Rectangle option.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetFaceDetection(DeviceHandle devHandle, uint8_t* facedet, uint8_t* statusStruct, uint8_t* overlayRect);
/**
* @brief 		Sets the Face Detection option to the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Face Detection option has to be set.
* @param[in] 	facedet	The Face Detection option to set.
* @param[in] 	statusStruct	The Face Detection status structure to set.
* @param[in] 	overlayRect	The Face Detection Overlay Rectangle option to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetFaceDetection(DeviceHandle devHandle, uint8_t facedet, uint8_t statusStruct, uint8_t overlayRect);
/**
* @brief 		Returns the Smile Detection option to the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Smile Detection option have to be requested.
* @param[out] 	smiledet	Pointer to a 8-bit unsigned integer array in which to store the Smile Detection option.
* @param[out] 	statusStruct	Pointer to a 8-bit unsigned integer array in which to store the Status Structure.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetSmileDetection(DeviceHandle devHandle, uint8_t* smiledet, uint8_t* statusStruct);
/**
* @brief 		Sets the Smile Detection option to the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Smile Detection option has to be set.
* @param[in] 	smiledet	The Smile Detection option to set.
* @param[in] 	statusStruct	The Smile Detection status structure to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetSmileDetection(DeviceHandle devHandle, uint8_t smiledet, uint8_t statusStruct);
/**
* @brief 		Returns the IMU embedded option from the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose IMU Embedded Data mode have to be requested.
* @param[out] 	IMUData	Pointer to a 8-bit unsigned integer array in which to store the IMU embedded option.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetIMUEmbeddedData(DeviceHandle devHandle, uint8_t*  IMUData);
/**
* @brief 		Returns the IMU embedded option to the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose IMU Embedded Data mode has to be set.
* @param[out] 	IMUData	IMU embedded option to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetIMUEmbeddedData(DeviceHandle devHandle, uint8_t  IMUData);
/**
* @brief 		Returns the Exposure Compensation value from the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Exposure Compensation value have to be requested.
* @param[out]	sxpoComp	Pointer to a 32-bit unsigned integer array in which to store the Exposure Compensation value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetExposureCompensation(DeviceHandle devHandle, uint32_t* expoComp);
/**
* @brief 		Sets the Exposure Compensation value to the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Exposure Compensation value has to be set.
* @param[in] 	sExpoComp	The Exposure Compensation value to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetExposureCompensation(DeviceHandle devHandle, uint32_t expoComp);
/**
* @brief 		Returns the Exposure Compensation value from the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose FrameRate control value have to be requested.
* @param[out]	gExpoComp	Pointer to a 8-bit unsigned integer array in which to store the Exposure Compensation value.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetFrameRateCtrl(DeviceHandle devHandle, uint8_t* frameRateCtrl);
/**
* @brief 		Sets the FrameRate control value to the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose FrameRate control value has to be set.
* @param[in] 	expoComp	The FrameRate control value to set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetFrameRateCtrl(DeviceHandle devHandle, uint8_t frameRateCtrl);
/**
* @brief 		Sets All the HID control to default values on the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Default values has to be set.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result SetDefault(DeviceHandle devHandle);
/**
* @brief 		Returns the Unique ID from the device.
* @param[in] 	devHandle	The DeviceHandle of the device whose Unique ID have to be requested.
* @param[out]	uniqueID	Pointer to a 64-bit unsigned integer array in which to store the Unique ID.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetUniqueID(DeviceHandle devHandle, uint64_t* uniqueID);
/**
* @brief  		Sets the average Depth and IR calculation area specified by region.
* @param[in] 	devHandle	The DeviceHandle of the device whose AvgRegion has to be set.
* @param[in] 	region	The region where you want to calculate the average depth and IR.
* @return 		::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetAvgRegion(DeviceHandle devHandle, AvgRegion region);
/**
* @brief  	Enables or disables the Filter specified by ctrlID.
* @param[in] 	devHandle	The DeviceHandle of the device in which the Filter specified by ctrlID is to be enabled or deisbaled.
* @param[in] 	ctrlID		flag ctrl ID
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetFilterType(DeviceHandle devHandle, int ctrlID, bool selected);
/**
* @brief  	Sets the average Depth and IR calculation area specified by pos.
* @param[in] 	devHandle	The DeviceHandle of the device in which the average Depth and IR calculation is to be set.
* @param[in] 	pos		The x and y co-ordinates of mouse pointer where you want to calculate the average depth and IR.
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetDepthPos(DeviceHandle devHandle, DepthPtr pos);
/**
* @brief  	Sets the cursor color in Depth Color Map to the white or black.
* @param[in] 	devHandle	The DeviceHandle of the device in which the cursor color in Depth Color Map is to be set.
* @param[in] 	color 		The color value which needs to be set.
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetCursorColor(DeviceHandle devHandle, int color);
/**
* @brief  	Enables or Disables Undistortion.
* @param[in] 	devHandle	The DeviceHandle of the device in which the Undistortion is to be enabled or disabled.
* @param[in] 	undistort 	The value which determine whether it enables or disables.
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetUnDistortion(DeviceHandle devHandle, int undistort);
/**
* @brief  	Enables or Disables rgbDMapping.
* @param[in] 	devHandle	The DeviceHandle of the device in which the RGB-D Mapping is to be enabled or disabled.
* @param[in] 	rgbDMapping 	The value which determine whether it enables or disables.
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetRGBDMapping(DeviceHandle devHandle, int rgbDMapping);
/**
* @brief  	Enables or Disables planarization.
* @param[in] 	devHandle	The DeviceHandle of the device in which the planarization filter is to be enabled or disabled.
* @param[in] 	planarize 	The value which determine whether it enables or disables.
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetPlanarization(DeviceHandle devHandle, int planarize);
/**
* @brief  	Enables or Disables flying pixel filter.
* @param[in] 	devHandle	The DeviceHandle of the device in which the flying pixel filter is to be enabled or disabled.
* @param[in] 	state		The value which determine whether it enables or disables.
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetFlyingPixelFilter(DeviceHandle devHandle, int state);
/**
* @brief 	Gets the average depth & IR and sigma depth and IR.
* @param[in] 	devHandle	The DeviceHandle of the device from which the average depth & IR and sigma depth and IR is to be obatined.
* @param[out]	AvgDepth	Pointer to a 32-bit unsigned integer array in which to store the Average depth value.
* @param[out]	StdDepth	Pointer to a 32-bit unsigned integer array in which to store the Sigma depth value.
* @param[out]	AvgIR		Pointer to a 32-bit unsigned integer array in which to store the Average IR value.
* @param[out]	StdIR		Pointer to a 32-bit unsigned integer array in which to store the Sigma IR value.
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*/
DEPTHVISTA_EXPORT Result GetDepthIRValues(DeviceHandle devHandle, int *avgDepth,int *stdDepth,int *avgIR,int* stdIR);
/**
* @brief  	Register a callback function specified by cb when frame is received from the camera.
* @param[in] 	devHandle	The DeviceHandle of the device in which the callback function for frames received is to be registered.
* @param[in] 	cb		Function pointer of the callback function.
*/
DEPTHVISTA_EXPORT void RegisterFrameCallback(DeviceHandle devHandle, std::function<void(int, int)> cb);
/**
* @brief  	Register a callback function specified by cb when notification is received from handle.
* @param[in] 	devHandle	The DeviceHandle of the device in which the callback function for notification is to be registered.
* @param[in] 	cb		Function pointer of the callback function.
*/
DEPTHVISTA_EXPORT void RegisterNotificationCallback(DeviceHandle devHandle, std::function<void(int, int)> cb);
/**
* @brief  	Updates the ROI for which the avg depth is being calculated
* @param[in] 	devHandle	The DeviceHandle of the device whose ROI for which the avg depth is being calculated is to be updated.
* @param[in] 	avg_x		width of ROI.
* @param[in] 	avg_y		height of ROI.
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result UpdateAvgXandY(DeviceHandle devHandle, int avg_x, int avg_y);
/**
* @brief  	Updates the color map properties
* @param[in] 	devHandle	The DeviceHandle of the device whose colormap properties is to be updated.
* @param[in] 	min		Range min.
* @param[in] 	max		Range max.
* @param[in] 	colorMap	colorMap.
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result UpdateColorMap(DeviceHandle devHandle, int min, int max, int colormap);
/**
* @brief  	Sets Average IR Display
* @param[in] 	devHandle	The DeviceHandle of the device whose Average IR Display is to be set.
* @param[in] 	avgIRDisplay	Zero to unset, non-zero to set.
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetAvgIRDisplay(DeviceHandle devHandle, uint8_t avgIRDisplay);
/**
* @brief 	produces a error message on standard error describing the last error
		encountered during a call to the tof library function.
* @param[in] 	message			if the argument is passed, it is printed along with error description seperated by a colon and a blank
*/
DEPTHVISTA_EXPORT void pError(std::string message = "");
/**
* @brief  	Reads the firmware version from the device
* @param[in] 	devHandle		The DeviceHandle of the device whose firmware version is to be read.
* @param[out] 	gMajorVersion		Major Firmware Version
* @param[out] 	gMinorVersion1		Minor Firmware Version 1
* @param[out] 	gMinorVersion2		Minor Firmware Version 2
* @param[out] 	gMinorVersion3		Minor Firmware Version 3
* @return 	::Ok			if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result ReadFirmwareVersion(DeviceHandle devHandle, uint8_t* majorVersion, uint8_t* minorVersion1, uint16_t* minorVersion2, uint16_t* minorVersion3);

/**
* @brief  	Requests the device to read intrinsic calibration data of the depth camera
* @param[in] 	devHandle		The DeviceHandle of the device in which intrinsic calibration Data Read for Depth camera is to be requested.
* @param[out] 	lDepthIntFileLength	Length of the valid calibration Data
* @return 	::Ok			if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result CalibReadReqDepthIntrinsic(DeviceHandle devHandle, int* depthIntFileLength);
/**
* @brief  	Reads the intrinsic calibration data of the depth camera
* @param[in] 	devHandle		The DeviceHandle of the device whose intrinsic calibration Data of Depth camera is to be read.
* @param[out] 	lDepthIntFileLength	Length of the valid calibration Data read
* @param[out] 	DepthIntrinsicBuffer	intrinsic calibration Data of depth camera
* @return 	::Ok			if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result CalibReadDepthIntrinsic(DeviceHandle devHandle, int* depthIntFileLength, unsigned char* DepthIntrinsicBuffer);
/**
* @brief  		Requests the device to read Extrinsic calibration data of the device
* @param[in] 	devHandle		The DeviceHandle of the device in which extrinsic calibration Data Read is to be requested.
* @param[out] 	lExtFileLength		Length of the valid calibration Data
* @return 	::Ok			if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result CalibReadReqExtrinsic(DeviceHandle devHandle, int* extFileLength);
/**
* @brief  	Reads the Extrinsic calibration data of the Device
* @param[in] 	devHandle		The DeviceHandle of the device whose extrinsic calibration Data is to be read.
* @param[out] 	lExtFileLength		Length of the valid calibration Data read
* @param[out] 	ExtrinsicBuffer		Extrinsic calibration data of the Device
* @return 	::Ok			if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result CalibReadExtrinsic(DeviceHandle devHandle, int* extFileLength, unsigned char* ExtrinsicBuffer);

/**
* @brief  	Requests the device to read intrinsic calibration data of the RGB camera
* @param[in] 	devHandle		The DeviceHandle of the device in which intrinsic calibration Data Read for RGB camera is to be requested.
* @param[out] 	lRGBIntFileLength	Length of the valid calibration Data
* @return 	::Ok			if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result CalibReadReqRGBIntrinsic(DeviceHandle devHandle, int* RGBIntFileLength);
/**
* @brief  	Reads the intrinsic calibration data of the RGB camera
* @param[in] 	devHandle		The DeviceHandle of the device whose intrinsic calibration Data of RGB camera is to be read.
* @param[out] 	lRGBIntFileLength	Length of the valid calibration Data read
* @param[out] 	RGBIntrinsicBuffer	intrinsic calibration Data of RGB camera
* @return 	::Ok			if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result CalibReadRGBIntrinsic(DeviceHandle devHandle, int* RGBIntFileLength, unsigned char* RGBIntrinsicBuffer);

/**
* @brief  	Reads the SDK version
* @param[out] 	gMajorVersion		Major SDK Version
* @param[out] 	gMinorVersion1		Minor SDK Version 1
* @param[out] 	gMinorVersion2		Minor SDK Version 2
* @return 	::Ok			if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result GetSDKVersion(uint8_t* majorVersion, uint8_t* minorVersion1, uint16_t* minorVersion2);
/**
* @brief  	Sets the logging state to SDK
* @param[in] 	logState	State of Log, any value from LoggingState enum
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetLogLevel(LogLevel logState);
/**
* @brief  	Sets the logging location to SDK
* @param[in] 	logLocation	Location of Log, any value from LoggingLocation enum
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result SetLogging(Logging log);

/**
* @brief  	Reads the Calibration Data from the device
* @param[in] 	devHandle	The DeviceHandle of the device whose Calibration data is to be obtained.
* @param[out] 	calibparams	Pointer to CalibrationParams structure in which the Calibration data will be filled
* @return 	::Ok		if the function succeeded, or one of the error values defined by ::Result.
*
*/
DEPTHVISTA_EXPORT Result GetDeviceCalibrationParams(DeviceHandle devHandle, CalibrationParams* calibparams);

DEPTHVISTA_EXPORT void PrintLog(uint16_t logPriority, std::string message);

#endif	/* DEPTHVISTA_H */
