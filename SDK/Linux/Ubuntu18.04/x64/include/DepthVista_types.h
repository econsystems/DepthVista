#ifndef DEPTHVISTA_TYPES_H
#define DEPTHVISTA_TYPES_H

#define DB_HIGH			0
#define DB_CRITICAL		0
#define DB_LOW			1

#include <string>
#include <functional>

/**
 * @brief Specifies the information about the device.
 */
typedef struct
{
    char deviceName[50];
    char vid[5];
    char pid[5];
    char devicePath[500];
    char serialNo[50];
}DeviceInfo;

typedef struct
{
	int id;
	int min;
	int max;
	int cur;
	int step;
	int default_val;
}UVCProp;
/**
 * @brief Specifies the X and Y coordinates of the mousePtr.
 */
typedef struct
{
	int X;
	int Y;
}MousePtr;

typedef struct
{
	int X;
	int Y;
	int depth;
	int IR;
}Avgstrct;

typedef struct
{
	unsigned char* frame_data;
	uint16_t width;
	uint16_t height;
	uint8_t pixel_format;
	uint32_t total_size;
}ToFFrame;

typedef struct {
	uint8_t IMU_MODE;
	uint8_t ACC_AXIS_CONFIG;
	uint8_t ACC_SENSITIVITY_CONFIG;
	uint8_t GYRO_AXIS_CONFIG;
	uint8_t GYRO_SENSITIVITY_CONFIG;
	uint8_t IMU_ODR_CONFIG;
} IMUCONFIG_TypeDef;

typedef struct {
	uint16_t imu_value_id;
	double accX;
	double accY;
	double accZ;
	double gyroX;
	double gyroY;
	double gyroZ;
} IMUDATAOUTPUT_TypeDef;

typedef struct {
	uint8_t imu_update_mode;
	uint16_t imu_num_of_values;
} IMUDATAINPUT_TypeDef;




#endif	/* DEPTHVISTA_TYPES_H */
