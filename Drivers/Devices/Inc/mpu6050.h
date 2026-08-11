#ifndef DEVICES_INC_MPU6050_H_
#define DEVICES_INC_MPU6050_H_

#include <stdint.h>

#include "FreeRTOS.h"
#include "device_IO.h"
#include "vector_utils.h"

/* -------------------------------------------------------------------------- */
/* Calibration                                                                    */
/* -------------------------------------------------------------------------- */

#define MPU6050_ACCEL_OFFSET_X_G    0.0f
#define MPU6050_ACCEL_OFFSET_Y_G    0.0ff
#define MPU6050_ACCEL_OFFSET_Z_G    0.0f

#define MPU6050_GYRO_OFFSET_X_DPS   0.0f
#define MPU6050_GYRO_OFFSET_Y_DPS   0.0f
#define MPU6050_GYRO_OFFSET_Z_DPS   0.0f


/* -------------------------------------------------------------------------- */
/* Address                                                                    */
/* -------------------------------------------------------------------------- */

#define MPU6050_I2C_ADDRESS_AD0_LOW 0x68U
#define MPU6050_I2C_ADDRESS_AD0_HIGH 0x69U

/* -------------------------------------------------------------------------- */
/* General                                                                    */
/* -------------------------------------------------------------------------- */

#define MPU6050_I2C_TIMEOUT_MS 100U
#define MPU6050_RAW_DATA_LENGTH 14U

/* -------------------------------------------------------------------------- */
/* Clock source                                                               */
/* -------------------------------------------------------------------------- */

#define MPU6050_CLKSRC_INTERNAL 0U
#define MPU6050_CLKSRC_PLL_X 1U
#define MPU6050_CLKSRC_PLL_Y 2U
#define MPU6050_CLKSRC_PLL_Z 3U
#define MPU6050_CLKSRC_PLL_EX_32 4U
#define MPU6050_CLKSRC_PLL_EX_19 5U
#define MPU6050_CLKSRC_STOPCLK 7U

/* -------------------------------------------------------------------------- */
/* DLPF                                                                       */
/* -------------------------------------------------------------------------- */

#define MPU6050_DLPF_CFG_0 0U
#define MPU6050_DLPF_CFG_1 1U
#define MPU6050_DLPF_CFG_2 2U
#define MPU6050_DLPF_CFG_3 3U
#define MPU6050_DLPF_CFG_4 4U
#define MPU6050_DLPF_CFG_5 5U
#define MPU6050_DLPF_CFG_6 6U

/* -------------------------------------------------------------------------- */
/* Gyroscope range                                                            */
/* -------------------------------------------------------------------------- */

#define MPU6050_GYRO_CONFIG_FS_250DPS 0U
#define MPU6050_GYRO_CONFIG_FS_500DPS 1U
#define MPU6050_GYRO_CONFIG_FS_1000DPS 2U
#define MPU6050_GYRO_CONFIG_FS_2000DPS 3U

/* -------------------------------------------------------------------------- */
/* Accelerometer range                                                        */
/* -------------------------------------------------------------------------- */

#define MPU6050_ACCEL_CONFIG_AFS_2G 0U
#define MPU6050_ACCEL_CONFIG_AFS_4G 1U
#define MPU6050_ACCEL_CONFIG_AFS_8G 2U
#define MPU6050_ACCEL_CONFIG_AFS_16G 3U

/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

typedef enum {
  MPU6050_OK = 0,

  MPU6050_I2C_ERROR,
  MPU6050_I2C_BUSY,
  MPU6050_I2C_TIMEOUT,

  MPU6050_ERR_NULL,
  MPU6050_ERR_BAD_DEVICE_ID,
  MPU6050_ERR_MOVING,
  MPU6050_ERR_TIMEOUT,
  MPU6050_ERR_UNINITIALIZED,
  MPU6050_INVALID_CONFIG
} MPU6050_Status_t;

/* -------------------------------------------------------------------------- */
/* Data                                                                       */
/* -------------------------------------------------------------------------- */

typedef struct {
  Vector3i16_t accel;
  int16_t temp;
  Vector3i16_t gyro;
} MPU6050_RawData_t;

typedef struct {
  Vector3f_t accel_g;
  float temp_c;
  Vector3f_t gyro_dps;
  TickType_t timestamp_tick;
} MPU6050_Data_t;

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

typedef struct {
  uint8_t address;

  uint8_t clksrc;
  uint8_t dlpf_config;

  uint8_t fs_sel_config;
  uint8_t accel_sel_config;

  uint8_t sample_rate_value;
} MPU6050_Config_t;

typedef struct {
  uint8_t int_level;
  uint8_t int_open;
  uint8_t latch_int_en;
  uint8_t int_rd_clear;
} MPU6050_InterruptConfig_t;

/* -------------------------------------------------------------------------- */
/* Calibration                                                                */
/* -------------------------------------------------------------------------- */

typedef struct {
  Vector3f_t accel_offset_g;
  Vector3f_t gyro_offset_dps;
} MPU6050_Calibration_t;

typedef struct {
  float gyro_threshold;
  float accel_axis_threshold;
  float accel_threshold;
  float accel_allowed_gap;

  uint32_t sample_count;
  uint32_t sample_delay_ms;
  uint32_t timeout_ms;
} MPU6050_StillnessConfig_t;

/* -------------------------------------------------------------------------- */
/* Handle                                                                     */
/* -------------------------------------------------------------------------- */

typedef struct {

  const DeviceIO_t *io;

  MPU6050_Config_t config;

  float accel_scale;
  float gyro_scale;

  float temp_scale;
  float temp_offset;

  Vector3f_t accel_offset_g;
  Vector3f_t gyro_offset_dps;

  uint8_t read_it_buffer[MPU6050_RAW_DATA_LENGTH];

  uint8_t initialized;

} MPU6050_Handle_t;

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *mpu, const DeviceIO_t *io,
                              const MPU6050_Config_t *config);

/* -------------------------------------------------------------------------- */
/* Blocking read                                                              */
/* -------------------------------------------------------------------------- */

MPU6050_Status_t MPU6050_ReadRawData(MPU6050_Handle_t *mpu,
                                     MPU6050_RawData_t *raw);

MPU6050_Status_t MPU6050_ReadRawAccel(MPU6050_Handle_t *mpu,
                                      MPU6050_RawData_t *raw);

MPU6050_Status_t MPU6050_ReadRawGyro(MPU6050_Handle_t *mpu,
                                     MPU6050_RawData_t *raw);

MPU6050_Status_t MPU6050_ReadRawTemp(MPU6050_Handle_t *mpu,
                                     MPU6050_RawData_t *raw);

MPU6050_Status_t MPU6050_ReadScaledData(MPU6050_Handle_t *mpu,
                                        MPU6050_Data_t *data);

/* -------------------------------------------------------------------------- */
/* Non-blocking read                                                          */
/* -------------------------------------------------------------------------- */

MPU6050_Status_t MPU6050_StartReadRawDataIT(MPU6050_Handle_t *mpu);
MPU6050_Status_t MPU6050_GetRawDataIT(MPU6050_Handle_t *mpu,
                                      MPU6050_RawData_t *raw);

/* -------------------------------------------------------------------------- */
/* Conversion                                                                 */
/* -------------------------------------------------------------------------- */

MPU6050_Status_t MPU6050_ConvertRawToPhysical(const MPU6050_Handle_t *mpu,
                                              const MPU6050_RawData_t *raw,
                                              MPU6050_Data_t *physical);

MPU6050_Status_t MPU6050_ApplyCalibration(const MPU6050_Handle_t *mpu,
                                          const MPU6050_Data_t *physical,
                                          MPU6050_Data_t *calibrated);

/* -------------------------------------------------------------------------- */
/* Calibration                                                                */
/* -------------------------------------------------------------------------- */

MPU6050_Status_t MPU6050_ResetCalibration(MPU6050_Handle_t *mpu);

MPU6050_Status_t
MPU6050_SetCalibration(MPU6050_Handle_t *mpu,
                       const MPU6050_Calibration_t *calibration);

MPU6050_Status_t MPU6050_GetCalibration(const MPU6050_Handle_t *mpu,
                                        MPU6050_Calibration_t *calibration);

MPU6050_Status_t
MPU6050_SetStillnessConfig(const MPU6050_Handle_t *mpu,
                           MPU6050_StillnessConfig_t *stillness);

MPU6050_Status_t
MPU6050_CalibrateGyroOffset(MPU6050_Handle_t *mpu, MPU6050_RawData_t *raw,
                            const MPU6050_StillnessConfig_t *stillness);

MPU6050_Status_t
MPU6050_CalibrateAccelOffset(MPU6050_Handle_t *mpu, MPU6050_RawData_t *raw,
                             const MPU6050_StillnessConfig_t *stillness,
                             const Vector3f_t *accel_reference_g);

/* -------------------------------------------------------------------------- */
/* Interrupt configuration                                                    */
/* -------------------------------------------------------------------------- */

MPU6050_Status_t
MPU6050_ConfigureInterrupt(MPU6050_Handle_t *mpu,
                           const MPU6050_InterruptConfig_t *config);

MPU6050_Status_t MPU6050_SetDataReadyInterrupt(MPU6050_Handle_t *mpu,
                                               uint8_t enable);

MPU6050_Status_t MPU6050_EnableBypass(MPU6050_Handle_t *mpu);

#endif /* DEVICES_INC_MPU6050_H_ */
