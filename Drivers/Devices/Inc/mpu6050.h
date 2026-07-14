/*
 * mpu6050.h
 *
 *  Created on: Jun 24, 2026
 *      Author: vietht-hl
 */

#ifndef DEVICES_INC_MPU6050_H_
#define DEVICES_INC_MPU6050_H_

#include "stdint.h"
#include "math.h"
#include "stm32f4xx_hal.h"

#define MPU6050_STILLNESS_SMPL_CNT200 200
#define MPU6050_STILLNESS_TIMEOUT5000 5000
#define MPU6050_STILLNESS_SAMPLE_DELAY_MS 2U

#define MPU6050_TEMP_SCALE 340.0f
#define MPU6050_TEMP_OFFSET 36.53f

#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_SMPRT_DIV 0x19
#define MPU6050_REG_ACCEL_XOUTH 0x3B
#define MPU6050_REG_ACCEL_XOUTL 0x3C
#define MPU6050_REG_ACCEL_YOUTH 0x3D
#define MPU6050_REG_ACCEL_YOUTL 0x3E
#define MPU6050_REG_ACCEL_ZOUTH 0x3F
#define MPU6050_REG_ACCEL_ZOUTL 0x40
#define MPU6050_REG_TEMP_OUTH 0x41
#define MPU6050_REG_TEMP_OUTL 0x42
#define MPU6050_REG_GYRO_XOUTH 0x43
#define MPU6050_REG_GYRO_XOUTL 0x44
#define MPU6050_REG_GYRO_YOUTH 0x45
#define MPU6050_REG_GYRO_YOUTL 0x46
#define MPU6050_REG_GYRO_ZOUTH 0x47
#define MPU6050_REG_GYRO_ZOUTL 0x48




#define MPU6050_WHO_AM_I_VALUE 0x68


#define MPU6050_PWR1_CLKSEL_MASK    0x07
#define MPU6050_PWR1_CLKSEL_POS	0
#define MPU6050_CLKSRC_INTERNAL 0
#define MPU6050_CLKSRC_PLL_X 1
#define MPU6050_CLKSRC_PLL_Y 2
#define MPU6050_CLKSRC_PLL_Z 3
#define MPU6050_CLKSRC_PLL_EX_32 4
#define MPU6050_CLKSRC_PLL_EX_19 5
#define MPU6050_CLKSRC_STOPCLK 7

#define MPU6050_CONFIG_DLPF_CFG_MASK     0x07
#define MPU6050_CONFIG_DLPF_CFG_POS      0
#define MPU6050_DLPF_CFG_0       0x00
#define MPU6050_DLPF_CFG_1       0x01
#define MPU6050_DLPF_CFG_2       0x02
#define MPU6050_DLPF_CFG_3       0x03
#define MPU6050_DLPF_CFG_4       0x04
#define MPU6050_DLPF_CFG_5       0x05
#define MPU6050_DLPF_CFG_6       0x06


#define MPU6050_GYRO_CONFIG_FS_SEL_MASK 0x03
#define MPU6050_GYRO_CONFIG_FS_SEL_POS 3U
#define MPU6050_GYRO_CONFIG_FS_250DPS 0
#define MPU6050_GYRO_CONFIG_FS_500DPS 1
#define MPU6050_GYRO_CONFIG_FS_1000DPS 2
#define MPU6050_GYRO_CONFIG_FS_2000DPS 3
#define MPU6050_GYRO_CONFIG_FS_SEN0 131.0f
#define MPU6050_GYRO_CONFIG_FS_SEN1 65.5f
#define MPU6050_GYRO_CONFIG_FS_SEN2 32.8f
#define MPU6050_GYRO_CONFIG_FS_SEN3 16.4f
#define MPU6050_GYRO_STILLNESS_THRES0 MPU6050_GYRO_CONFIG_FS_SEN0
#define MPU6050_GYRO_STILLNESS_THRES1 MPU6050_GYRO_CONFIG_FS_SEN1
#define MPU6050_GYRO_STILLNESS_THRES2 MPU6050_GYRO_CONFIG_FS_SEN2
#define MPU6050_GYRO_STILLNESS_THRES3 MPU6050_GYRO_CONFIG_FS_SEN3


#define MPU6050_ACCEL_CONFIG_AFS_SEL_MASK 0x03
#define MPU6050_ACCEL_CONFIG_AFS_SEL_POS 3U
#define MPU6050_ACCEL_CONFIG_AFS_2G 0
#define MPU6050_ACCEL_CONFIG_AFS_4G 1
#define MPU6050_ACCEL_CONFIG_AFS_8G 2
#define MPU6050_ACCEL_CONFIG_AFS_16G 3
#define MPU6050_ACCEL_CONFIG_AFS_SEN0 16384.0f
#define MPU6050_ACCEL_CONFIG_AFS_SEN1 8192.0f
#define MPU6050_ACCEL_CONFIG_AFS_SEN2 4096.0f
#define MPU6050_ACCEL_CONFIG_AFS_SEN3 2048.0f
#define MPU6050_ACCEL_STILLNESS_THRES_COEFFICIENT 1
#define MPU6050_ACCEL_STILLNESS_AXIS_COEFFICIENT 0.05f
#define MPU6050_ACCEL_STILLNESS_ALLOWED_GAP_COEFFICIENT 0.1f




typedef struct {
	I2C_HandleTypeDef* hi2c;
	uint8_t address;
	float accel_scale;
	float temp_scale;
	float gyro_scale;
	float accel_offset_x;
	float accel_offset_y;
	float accel_offset_z;
	float temp_offset;
	float gyro_offset_x;
	float gyro_offset_y;
	float gyro_offset_z;
} MPU6050_Handle_t;

typedef struct {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} MPU6050_RawData_t;

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float temp_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} MPU6050_Data_t;


typedef struct {
	float gyro_threshold;
	float accel_axis_threshold;
	float accel_threshold;
	float accel_allowed_gap;
	uint32_t sample_count;
	uint32_t sample_delay_ms;
	uint32_t timeout_ms;
} MPU6050_StillnessConfig_t;

typedef enum {
	MPU6050_OK = 0,
	MPU6050_ERROR,
	MPU6050_I2C_ERROR,
	MPU6050_I2C_BUSY,
	MPU6050_I2C_TIMEOUT,
	MPU6050_ERR_TIMEOUT,
	MPU6050_ERR_NULL,
	MPU6050_ERR_BAD_DEVICE_ID,
	MPU6050_ERR_MOVING,
	MPU6050_INVALID_CONFIG
} MPU6050_Status_t;

MPU6050_Status_t MPU6050_Init(
		MPU6050_Handle_t* mpu,
		I2C_HandleTypeDef* hi2c ,
		uint8_t address,
		uint8_t clksrc,
		uint8_t dlpf_config,
		uint8_t fs_sel_config,
		uint8_t accel_sel_config,
		uint8_t sample_rate_value);
MPU6050_Status_t MPU6050_CheckDeviceID(MPU6050_Handle_t* mpu);
MPU6050_Status_t MPU6050_WakeUpChip(MPU6050_Handle_t* mpu);
MPU6050_Status_t MPU6050_SetClockSource(MPU6050_Handle_t* mpu, uint8_t clksrc);
MPU6050_Status_t MPU6050_SetDLPF(MPU6050_Handle_t* mpu, uint8_t dlpf_config);
MPU6050_Status_t MPU6050_SetGyroConfig(MPU6050_Handle_t* mpu, uint8_t fs_sel_config);
MPU6050_Status_t MPU6050_SetAccelConfig(MPU6050_Handle_t* mpu, uint8_t accel_sel_config);
MPU6050_Status_t MPU6050_SetSampleRate(MPU6050_Handle_t* mpu, uint8_t sample_rate_div);
MPU6050_Status_t MPU6050_ReadRawData(MPU6050_Handle_t* mpu, MPU6050_RawData_t* raw);
MPU6050_Status_t MPU6050_ReadRawAccel(MPU6050_Handle_t* mpu, MPU6050_RawData_t* raw);
MPU6050_Status_t MPU6050_ReadRawGyro(MPU6050_Handle_t* mpu, MPU6050_RawData_t* raw);
MPU6050_Status_t MPU6050_ReadRawTemp(MPU6050_Handle_t* mpu, MPU6050_RawData_t* raw);
MPU6050_Status_t MPU6050_ReadScaledData(MPU6050_Handle_t* mpu, MPU6050_Data_t* scaled);
MPU6050_Status_t MPU6050_ConvertRawToScaled(MPU6050_Handle_t* mpu, const MPU6050_RawData_t* raw, MPU6050_Data_t* scaled);
MPU6050_Status_t MPU6050_CalibrateGyroOffset(MPU6050_Handle_t* mpu, MPU6050_RawData_t* raw, MPU6050_StillnessConfig_t* still);
MPU6050_Status_t MPU6050_CalibrateAccelOffset(
		MPU6050_Handle_t* mpu,
		MPU6050_RawData_t* raw,
		MPU6050_StillnessConfig_t* still,
		float accel_ref_x_g,
		float accel_ref_y_g,
		float accel_ref_z_g);
MPU6050_Status_t MPU6050_ResetOffsets(MPU6050_Handle_t* mpu);
MPU6050_Status_t MPU6050_SetStillnessConfig(MPU6050_Handle_t* mpu, MPU6050_StillnessConfig_t* still);
const char* MPU6050_ConvertStatusToString(MPU6050_Status_t status);


#endif /* DEVICES_INC_MPU6050_H_ */
