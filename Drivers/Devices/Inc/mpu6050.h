/*
 * mpu6050.h
 *
 *  Created on: Jun 24, 2026
 *      Author: vietht-hl
 */

#ifndef DEVICES_INC_MPU6050_H_
#define DEVICES_INC_MPU6050_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "vector_utils.h"

#define MPU6050_STILLNESS_SMPL_CNT200          200U
#define MPU6050_STILLNESS_TIMEOUT5000          5000U
#define MPU6050_STILLNESS_SAMPLE_DELAY_MS      2U

#define MPU6050_TEMP_SCALE                     340.0f
#define MPU6050_TEMP_OFFSET                    36.53f

#define MPU6050_I2C_ADDRESS_AD0_LOW            0x68U
#define MPU6050_I2C_ADDRESS_AD0_HIGH           0x69U
#define MPU6050_I2C_TIMEOUT_MS                 100U
#define MPU6050_RAW_DATA_LENGTH                14U

#define MPU6050_REG_SMPRT_DIV                  0x19U
#define MPU6050_REG_CONFIG                     0x1AU
#define MPU6050_REG_GYRO_CONFIG                0x1BU
#define MPU6050_REG_ACCEL_CONFIG               0x1CU
#define MPU6050_REG_INT_PIN_CFG                0x37U
#define MPU6050_REG_INT_ENABLE                 0x38U
#define MPU6050_REG_INT_STATUS                 0x3AU
#define MPU6050_REG_ACCEL_XOUTH                0x3BU
#define MPU6050_REG_ACCEL_XOUTL                0x3CU
#define MPU6050_REG_ACCEL_YOUTH                0x3DU
#define MPU6050_REG_ACCEL_YOUTL                0x3EU
#define MPU6050_REG_ACCEL_ZOUTH                0x3FU
#define MPU6050_REG_ACCEL_ZOUTL                0x40U
#define MPU6050_REG_TEMP_OUTH                  0x41U
#define MPU6050_REG_TEMP_OUTL                  0x42U
#define MPU6050_REG_GYRO_XOUTH                 0x43U
#define MPU6050_REG_GYRO_XOUTL                 0x44U
#define MPU6050_REG_GYRO_YOUTH                 0x45U
#define MPU6050_REG_GYRO_YOUTL                 0x46U
#define MPU6050_REG_GYRO_ZOUTH                 0x47U
#define MPU6050_REG_GYRO_ZOUTL                 0x48U
#define MPU6050_REG_USER_CTRL                  0x6AU
#define MPU6050_REG_PWR_MGMT_1                 0x6BU
#define MPU6050_REG_WHO_AM_I                   0x75U


#define MPU6050_WHO_AM_I_VALUE                 0x68U

#define MPU6050_PWR1_CLKSEL_MASK               0x07U
#define MPU6050_PWR1_CLKSEL_POS                0U
#define MPU6050_CLKSRC_INTERNAL                0U
#define MPU6050_CLKSRC_PLL_X                   1U
#define MPU6050_CLKSRC_PLL_Y                   2U
#define MPU6050_CLKSRC_PLL_Z                   3U
#define MPU6050_CLKSRC_PLL_EX_32               4U
#define MPU6050_CLKSRC_PLL_EX_19               5U
#define MPU6050_CLKSRC_STOPCLK                 7U

#define MPU6050_CONFIG_DLPF_CFG_MASK           0x07U
#define MPU6050_CONFIG_DLPF_CFG_POS            0U
#define MPU6050_DLPF_CFG_0                     0x00U
#define MPU6050_DLPF_CFG_1                     0x01U
#define MPU6050_DLPF_CFG_2                     0x02U
#define MPU6050_DLPF_CFG_3                     0x03U
#define MPU6050_DLPF_CFG_4                     0x04U
#define MPU6050_DLPF_CFG_5                     0x05U
#define MPU6050_DLPF_CFG_6                     0x06U

#define MPU6050_GYRO_CONFIG_FS_SEL_MASK        0x03U
#define MPU6050_GYRO_CONFIG_FS_SEL_POS         3U
#define MPU6050_GYRO_CONFIG_FS_250DPS          0U
#define MPU6050_GYRO_CONFIG_FS_500DPS          1U
#define MPU6050_GYRO_CONFIG_FS_1000DPS         2U
#define MPU6050_GYRO_CONFIG_FS_2000DPS         3U
#define MPU6050_GYRO_CONFIG_FS_SEN0            131.0f
#define MPU6050_GYRO_CONFIG_FS_SEN1            65.5f
#define MPU6050_GYRO_CONFIG_FS_SEN2            32.8f
#define MPU6050_GYRO_CONFIG_FS_SEN3            16.4f

#define MPU6050_ACCEL_CONFIG_AFS_SEL_MASK      0x03U
#define MPU6050_ACCEL_CONFIG_AFS_SEL_POS       3U
#define MPU6050_ACCEL_CONFIG_AFS_2G            0U
#define MPU6050_ACCEL_CONFIG_AFS_4G            1U
#define MPU6050_ACCEL_CONFIG_AFS_8G            2U
#define MPU6050_ACCEL_CONFIG_AFS_16G           3U
#define MPU6050_ACCEL_CONFIG_AFS_SEN0          16384.0f
#define MPU6050_ACCEL_CONFIG_AFS_SEN1          8192.0f
#define MPU6050_ACCEL_CONFIG_AFS_SEN2          4096.0f
#define MPU6050_ACCEL_CONFIG_AFS_SEN3          2048.0f

#define MPU6050_INT_ENABLE_DATA_RDY_EN         0U

#define MPU6050_INT_PIN_CFG_I2C_BYPASS_EN   1U
#define MPU6050_INT_PIN_CFG_FSYNC_INT_EN    2U
#define MPU6050_INT_PIN_CFG_FSYNC_INT_LEVEL 3U
#define MPU6050_INT_PIN_CFG_INT_RD_CLEAR    4U
#define MPU6050_INT_PIN_CFG_LATCH_INT_EN    5U
#define MPU6050_INT_PIN_CFG_INT_OPEN           6U
#define MPU6050_INT_PIN_CFG_INT_LEVEL          7U


#define MPU6050_GYRO_STILLNESS_THRESHOLD_DPS   1.0f
#define MPU6050_ACCEL_STILLNESS_THRES_COEFFICIENT 1.0f
#define MPU6050_ACCEL_STILLNESS_AXIS_COEFFICIENT  0.05f
#define MPU6050_ACCEL_STILLNESS_ALLOWED_GAP_COEFFICIENT 0.1f

typedef struct {
    Vector3i16_t accel;
    int16_t temp;
    Vector3i16_t gyro;
} MPU6050_RawData_t;

typedef struct {
    Vector3f_t accel_g;
    float temp_c;
    Vector3f_t gyro_dps;
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
    MPU6050_INVALID_STATE,
    MPU6050_I2C_ERROR,
    MPU6050_I2C_BUSY,
    MPU6050_I2C_TIMEOUT,
    MPU6050_ERR_TIMEOUT,
    MPU6050_ERR_NULL,
    MPU6050_ERR_BAD_DEVICE_ID,
    MPU6050_ERR_MOVING,
	MPU6050_ERR_UNINITIALIZED,
    MPU6050_NOT_READY_TO_READ,
    MPU6050_INVALID_CONFIG
} MPU6050_Status_t;

typedef enum {
    MPU6050_READ_IT_IDLE = 0,
    MPU6050_READ_IT_BUSY,
    MPU6050_READ_IT_COMPLETE,
    MPU6050_READ_IT_ERROR,
    MPU6050_READ_IT_ABORTING
} MPU6050_ReadITState_t;

typedef struct {
    Vector3f_t accel_offset_g;
    Vector3f_t gyro_offset_dps;
} MPU6050_Calibration_t;

typedef struct {
    uint8_t address;
    uint8_t clksrc;
    uint8_t dlpf_config;
    uint8_t fs_sel_config;
    uint8_t accel_sel_config;
    uint8_t sample_rate_value;
} MPU6050_Config_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;

    float accel_scale;
    float temp_scale;
    float gyro_scale;

    Vector3f_t accel_offset_g;
    float temp_offset;
    Vector3f_t gyro_offset_dps;

    MPU6050_Config_t config;

    uint8_t read_it_buffer[MPU6050_RAW_DATA_LENGTH];
    volatile MPU6050_ReadITState_t read_it_state;
    volatile MPU6050_Status_t read_it_result;
    uint8_t initialized;
} MPU6050_Handle_t;

/* ==================== Debug/Status API ==================== */

MPU6050_Status_t MPU6050_Init(
        MPU6050_Handle_t *mpu,
        I2C_HandleTypeDef *hi2c,
        const MPU6050_Config_t *config);

void MPU6050_NotiStatus(
        MPU6050_Status_t status,
        const char *message);



/* ==================== Blocking API ==================== */

MPU6050_Status_t MPU6050_ReadRawData(
        MPU6050_Handle_t *mpu,
        MPU6050_RawData_t *raw);

MPU6050_Status_t MPU6050_ReadRawAccel(
        MPU6050_Handle_t *mpu,
        MPU6050_RawData_t *raw);

MPU6050_Status_t MPU6050_ReadRawGyro(
        MPU6050_Handle_t *mpu,
        MPU6050_RawData_t *raw);

MPU6050_Status_t MPU6050_ReadRawTemp(
        MPU6050_Handle_t *mpu,
        MPU6050_RawData_t *raw);

MPU6050_Status_t MPU6050_ReadScaledData(
        MPU6050_Handle_t *mpu,
        MPU6050_Data_t *scaled);


/* ==================== Calibration API ==================== */


MPU6050_Status_t MPU6050_ConvertRawToPhysical(
        const MPU6050_Handle_t *mpu,
        const MPU6050_RawData_t *raw,
        MPU6050_Data_t *physical);

MPU6050_Status_t MPU6050_ApplyCalibration(
        const MPU6050_Handle_t *mpu,
        const MPU6050_Data_t *physical,
        MPU6050_Data_t *calibrated);

MPU6050_Status_t MPU6050_ResetCalibration(
        MPU6050_Handle_t *mpu);

MPU6050_Status_t MPU6050_SetCalibration(
        MPU6050_Handle_t *mpu,
        const MPU6050_Calibration_t *calibration);

MPU6050_Status_t MPU6050_GetCalibration(
        const MPU6050_Handle_t *mpu,
        MPU6050_Calibration_t *calibration);

MPU6050_Status_t MPU6050_SetStillnessConfig(
        const MPU6050_Handle_t *mpu,
        MPU6050_StillnessConfig_t *still);

MPU6050_Status_t MPU6050_CalibrateGyroOffset(
        MPU6050_Handle_t *mpu,
        MPU6050_RawData_t *raw,
        const MPU6050_StillnessConfig_t *still);

MPU6050_Status_t MPU6050_CalibrateAccelOffset(
        MPU6050_Handle_t *mpu,
        MPU6050_RawData_t *raw,
        const MPU6050_StillnessConfig_t *still,
        const Vector3f_t *accel_ref_g);

/* ==================== Bypass API ==================== */

MPU6050_Status_t MPU6050_EnableBypass(
        MPU6050_Handle_t *mpu);

/* ==================== Interrupt API ==================== */

typedef struct {
    uint8_t int_level;
    uint8_t int_open;
    uint8_t latch_int_en;
    uint8_t int_rd_clear;
} MPU6050_InterruptConfig_t;

MPU6050_Status_t MPU6050_SetDataReadyInterrupt(
        MPU6050_Handle_t *mpu,
        uint8_t enable);

MPU6050_Status_t MPU6050_ConfigureInterrupt(
        MPU6050_Handle_t *mpu,
        const MPU6050_InterruptConfig_t *config);

MPU6050_Status_t MPU6050_StartReadRawDataIT(
        MPU6050_Handle_t *mpu);

MPU6050_Status_t MPU6050_OnI2CMemRxComplete(
        MPU6050_Handle_t *mpu,
        I2C_HandleTypeDef *hi2c);

MPU6050_Status_t MPU6050_OnI2CError(
        MPU6050_Handle_t *mpu,
        I2C_HandleTypeDef *hi2c);

MPU6050_Status_t MPU6050_GetRawDataIT(
        MPU6050_Handle_t *mpu,
        MPU6050_RawData_t *raw);

MPU6050_ReadITState_t MPU6050_GetReadStateIT(
        const MPU6050_Handle_t *mpu);

MPU6050_Status_t MPU6050_AbortReadIT(
        MPU6050_Handle_t *mpu);

MPU6050_Status_t MPU6050_OnI2CAbortComplete(
        MPU6050_Handle_t *mpu,
        I2C_HandleTypeDef *hi2c);

#endif /* DEVICES_INC_MPU6050_H_ */
