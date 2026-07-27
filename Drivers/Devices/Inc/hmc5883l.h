/*
 * hmc5883l.h
 *
 *  Created on: Jul 15, 2026
 *      Author: vietht-hl
 */

#ifndef DEVICES_INC_HMC5883L_H_
#define DEVICES_INC_HMC5883L_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "vector_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== Device identification ==================== */

#define HMC5883L_ID                         "H43"
#define HMC5883L_ID_LENGTH                  3U
#define HMC5883L_OVERFLOW_VALUE             ((int16_t)-4096)
#define HMC5883L_RAW_DATA_LENGTH            6U
#define HMC5883L_I2C_TIMEOUT_MS             100U

/* ==================== Calibration constants ==================== */

#define HMC5883L_CAL_MIN_SAMPLES            100U
#define HMC5883L_CAL_MIN_AXIS_RANGE_G       0.01f


/* ==================== Register map ==================== */

#define HMC5883L_REG_CONF_A                 0x00U
#define HMC5883L_REG_CONF_B                 0x01U
#define HMC5883L_REG_MODE                   0x02U
#define HMC5883L_REG_X_MSB                  0x03U
#define HMC5883L_REG_X_LSB                  0x04U
#define HMC5883L_REG_Z_MSB                  0x05U
#define HMC5883L_REG_Z_LSB                  0x06U
#define HMC5883L_REG_Y_MSB                  0x07U
#define HMC5883L_REG_Y_LSB                  0x08U
#define HMC5883L_REG_STATUS                 0x09U
#define HMC5883L_REG_ID_A                   0x0AU
#define HMC5883L_REG_ID_B                   0x0BU
#define HMC5883L_REG_ID_C                   0x0CU

/* ==================== Configuration register A ==================== */

#define HMC5883L_CONF_A_CRA7                7U
#define HMC5883L_CONF_A_MA0                 5U
#define HMC5883L_CONF_A_DO0                 2U
#define HMC5883L_CONF_A_MS0                 0U

#define HMC5883L_CONF_A_CRA7_MASK           0x01U
#define HMC5883L_CONF_A_SAMPLE_AVG_MASK     0x03U
#define HMC5883L_CONF_A_OUTPUT_RATE_MASK    0x07U
#define HMC5883L_CONF_A_MEAS_MODE_MASK      0x03U

#define HMC5883L_MEAS_MODE_NORMAL           0x00U
#define HMC5883L_MEAS_MODE_POS_BIAS         0x01U
#define HMC5883L_MEAS_MODE_NEG_BIAS         0x02U

#define HMC5883L_OUTPUT_RATE_0_75            0x00U
#define HMC5883L_OUTPUT_RATE_1_5             0x01U
#define HMC5883L_OUTPUT_RATE_3               0x02U
#define HMC5883L_OUTPUT_RATE_7_5             0x03U
#define HMC5883L_OUTPUT_RATE_15              0x04U
#define HMC5883L_OUTPUT_RATE_30              0x05U
#define HMC5883L_OUTPUT_RATE_75              0x06U

#define HMC5883L_SAMPLE_AVG_1                0x00U
#define HMC5883L_SAMPLE_AVG_2                0x01U
#define HMC5883L_SAMPLE_AVG_4                0x02U
#define HMC5883L_SAMPLE_AVG_8                0x03U

/* ==================== Configuration register B ==================== */

#define HMC5883L_CONF_B_CRB5                 5U
#define HMC5883L_CONF_B_DEVICE_GAIN_MASK     0x07U

#define HMC5883L_DEVICE_GAIN_0_88            0U
#define HMC5883L_DEVICE_GAIN_1_3             1U
#define HMC5883L_DEVICE_GAIN_1_9             2U
#define HMC5883L_DEVICE_GAIN_2_5             3U
#define HMC5883L_DEVICE_GAIN_4_0             4U
#define HMC5883L_DEVICE_GAIN_4_7             5U
#define HMC5883L_DEVICE_GAIN_5_6             6U
#define HMC5883L_DEVICE_GAIN_8_1             7U

#define HMC5883L_LSB_PER_G_0_88              1370.0f
#define HMC5883L_LSB_PER_G_1_3               1090.0f
#define HMC5883L_LSB_PER_G_1_9               820.0f
#define HMC5883L_LSB_PER_G_2_5               660.0f
#define HMC5883L_LSB_PER_G_4_0               440.0f
#define HMC5883L_LSB_PER_G_4_7               390.0f
#define HMC5883L_LSB_PER_G_5_6               330.0f
#define HMC5883L_LSB_PER_G_8_1               230.0f

/* ==================== Mode register ==================== */

#define HMC5883L_MODE_CONTINUOUS_MEAS         0U
#define HMC5883L_MODE_SINGLE_MEAS             1U
#define HMC5883L_MODE_IDLE_MODE1              2U
#define HMC5883L_MODE_IDLE_MODE2              3U

#define HMC5883L_MODE_MD0                     0U
#define HMC5883L_MODE_MASK                    0x03U

/* ==================== Status register ==================== */

#define HMC5883L_STATUS_RDY                   0U

/* ==================== Public data types ==================== */

typedef struct {
	uint8_t address;
    uint8_t meas_mode;
    uint8_t output_rate;
    uint8_t sample_avg;
    uint8_t device_gain;
    uint8_t mode;
} HMC5883L_Config_t;

typedef enum {
    HMC5883L_OK = 0,
	HMC5883L_INVALID_STATE,
    HMC5883L_ERR_NULL,
    HMC5883L_I2C_TIMEOUT,
    HMC5883L_I2C_ERROR,
    HMC5883L_I2C_BUSY,
    HMC5883L_INVALID_CONFIG,
    HMC5883L_BAD_DEVICE_ID,
    HMC5883L_NOT_READY_TO_READ,
    HMC5883L_DATA_TIMEOUT,
    HMC5883L_DATA_OVERFLOW,
    HMC5883L_INVALID_CALIBRATION,
    HMC5883L_NOT_ENOUGH_SAMPLES
} HMC5883L_Status_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} HMC5883L_RawData_t;

typedef struct {
    float x_g;
    float y_g;
    float z_g;
} HMC5883L_Data_t;

typedef struct {
    Vector3f_t sensor_scale;
    Vector3f_t hard_iron_bias_g;
    float soft_iron_matrix[3][3];
} HMC5883L_Calibration_t;

typedef struct {
    Vector3f_t min_g;
    Vector3f_t max_g;
    uint32_t accepted_samples;
    uint32_t rejected_samples;
} HMC5883L_CalibrationSession_t;


typedef enum {
	HMC5883L_READ_IT_IDLE = 0,
	HMC5883L_READ_IT_BUSY,
	HMC5883L_READ_IT_COMPLETE,
	HMC5883L_READ_IT_ERROR,
	HMC5883L_READ_IT_ABORTING
} HMC5883L_ReadITState_t;

typedef struct {
    I2C_HandleTypeDef* hi2c;
    float lsb_per_gauss;
    HMC5883L_Config_t config;
    HMC5883L_Calibration_t calibration;

    uint8_t read_it_buffer[HMC5883L_RAW_DATA_LENGTH];
    volatile HMC5883L_ReadITState_t read_it_state;
    volatile HMC5883L_Status_t read_it_result;

} HMC5883L_Handle_t;

/* ==================== Debug/status API ==================== */

void HMC5883L_NotiStatus(
        HMC5883L_Status_t status,
        const char* str);

HMC5883L_Status_t HMC5883L_CheckDeviceID(
        HMC5883L_Handle_t* hmc);

/* ==================== Configuration API ==================== */

HMC5883L_Status_t HMC5883L_SetConfigurationA(
        HMC5883L_Handle_t* hmc,
        uint8_t meas_mode,
        uint8_t output_rate,
        uint8_t sample_avg);

HMC5883L_Status_t HMC5883L_SetConfigurationB(
        HMC5883L_Handle_t* hmc,
        uint8_t device_gain);

HMC5883L_Status_t HMC5883L_SetMode(
        HMC5883L_Handle_t* hmc,
        uint8_t mode);

HMC5883L_Status_t HMC5883L_Init(
        HMC5883L_Handle_t* hmc,
        I2C_HandleTypeDef* hi2c,
        const HMC5883L_Config_t* config);


/* ==================== Status/read API ==================== */

HMC5883L_Status_t HMC5883L_IsDataReady(
        HMC5883L_Handle_t* hmc,
        uint8_t* ready);


HMC5883L_Status_t HMC5883L_ReadRawData(
        HMC5883L_Handle_t* hmc,
        HMC5883L_RawData_t* raw);

HMC5883L_Status_t HMC5883L_ReadRawDataWait(
        HMC5883L_Handle_t* hmc,
        HMC5883L_RawData_t* raw,
        uint32_t timeout_ms);

HMC5883L_Status_t HMC5883L_ConvertRawToScaled(
        const HMC5883L_Handle_t* hmc,
        const HMC5883L_RawData_t* raw,
        HMC5883L_Data_t* scaled);

/* ==================== Calibration API ==================== */

HMC5883L_Status_t HMC5883L_CalibrationSetDefault(
        HMC5883L_Calibration_t* calibration);

HMC5883L_Status_t HMC5883L_CalibrationBegin(
        HMC5883L_CalibrationSession_t* session);

HMC5883L_Status_t HMC5883L_CalibrationAddSample(
        HMC5883L_CalibrationSession_t* session,
        const HMC5883L_Data_t* sample);

HMC5883L_Status_t HMC5883L_CalibrationFinish(
        HMC5883L_Handle_t* hmc,
        const HMC5883L_CalibrationSession_t* session);

HMC5883L_Status_t HMC5883L_ApplyCalibration(
        const HMC5883L_Handle_t* hmc,
        const HMC5883L_Data_t* input,
        HMC5883L_Data_t* output);

/* ==================== Interrupt API ==================== */


HMC5883L_Status_t HMC5883L_StartReadAfterDRDYIT(
        HMC5883L_Handle_t *hmc);

HMC5883L_Status_t HMC5883L_OnI2CMemRxComplete(
        HMC5883L_Handle_t *hmc,
        I2C_HandleTypeDef *hi2c);

HMC5883L_Status_t HMC5883L_OnI2CError(
        HMC5883L_Handle_t *hmc,
        I2C_HandleTypeDef *hi2c);

HMC5883L_Status_t HMC5883L_GetRawDataIT(
        HMC5883L_Handle_t *hmc,
        HMC5883L_RawData_t *raw);

HMC5883L_ReadITState_t HMC5883L_GetReadStateIT(
        const HMC5883L_Handle_t *hmc);
HMC5883L_Status_t HMC5883L_AbortReadIT(
        HMC5883L_Handle_t* hmc);
HMC5883L_Status_t HMC5883L_OnI2CAbortComplete(
        HMC5883L_Handle_t* hmc,
        I2C_HandleTypeDef* hi2c);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_INC_HMC5883L_H_ */
