/*
 * bmp180.h
 *
 *  Created on: Jul 11, 2026
 *      Author: vietht-hl
 */

#ifndef DEVICES_INC_BMP180_H_
#define DEVICES_INC_BMP180_H_

#include "stm32f4xx_hal.h"
#include "byte_utils.h"

#define BMP180_REG_CHIP_ID                 0xD0U
#define BMP180_REG_CTRL_MEAS               0xF4U
#define BMP180_REG_MSB                     0xF6U
#define BMP180_REG_LSB                     0xF7U
#define BMP180_REG_XLSB                    0xF8U
#define BMP180_REG_AC1                     0xAAU
#define BMP180_REG_AC2                     0xACU
#define BMP180_REG_AC3                     0xAEU
#define BMP180_REG_AC4                     0xB0U
#define BMP180_REG_AC5                     0xB2U
#define BMP180_REG_AC6                     0xB4U
#define BMP180_REG_B1                      0xB6U
#define BMP180_REG_B2                      0xB8U
#define BMP180_REG_MB                      0xBAU
#define BMP180_REG_MC                      0xBCU
#define BMP180_REG_MD                      0xBEU

#define BMP180_CHIP_ID_VALUE               0x55U
#define BMP180_CMD_TEMPERATURE             0x2EU
#define BMP180_CMD_PRESSURE                0x34U

#define BMP180_PRESSURE_OSS0               0U
#define BMP180_PRESSURE_OSS1               1U
#define BMP180_PRESSURE_OSS2               2U
#define BMP180_PRESSURE_OSS3               3U

#define BMP180_TEMPERATURE_DELAY_MS        5U
#define BMP180_OSS0_DELAY_MS               5U
#define BMP180_OSS1_DELAY_MS               8U
#define BMP180_OSS2_DELAY_MS              14U
#define BMP180_OSS3_DELAY_MS              26U


#define BMP180_HAL_I2C_TIMEOUT_MS         10U


#define BMP180_I2C_BUSY_TIMEOUT_MS        20U

#define BMP180_STANDARD_SEA_LEVEL_PRESSURE_PA  101325.0f
#define BMP180_ALTITUDE_EXPONENT               (1.0f / 5.255f)

#define BMP180_WINDOW_SIZE                4U

#define BMP180_FILTER_ALPHA_LOW           0.03f
#define BMP180_FILTER_ALPHA_MEDIUM        0.18f
#define BMP180_FILTER_ALPHA_HIGH          0.45f

#define BMP180_FILTER_THRESHOLD_LOW_PA    0.5f
#define BMP180_FILTER_THRESHOLD_HIGH_PA   3.0f

#define BMP180_STARTUP_SAMPLE_COUNT       64U
#define BMP180_STARTUP_TRIM_COUNT          8U
#define BMP180_STARTUP_SAMPLE_INTERVAL_MS 100U

typedef enum {
    BMP180_OK = 0,
    BMP180_IN_PROGRESS,
    BMP180_I2C_BUSY,
    BMP180_I2C_ERROR,
    BMP180_I2C_TIMEOUT,
    BMP180_ERR_NULL,
    BMP180_INVALID_CONFIG,
    BMP180_ERR_BAD_DEVICE_ID,
    BMP180_ERR_UNINITIALIZED,
    BMP180_ERR_INVALID_STATE
} BMP180_Status_t;

typedef enum {
    BMP180_STATE_IDLE = 0,
    BMP180_STATE_WAIT_TEMPERATURE,
    BMP180_STATE_START_PRESSURE,
    BMP180_STATE_WAIT_PRESSURE
} BMP180_State_t;

typedef enum {
    BMP180_STARTUP_IDLE = 0,
    BMP180_STARTUP_RUNNING,
    BMP180_STARTUP_COMPLETE,
    BMP180_STARTUP_ERROR
} BMP180_StartupState_t;

typedef struct {
    float pressure_pa[BMP180_WINDOW_SIZE];
    uint8_t write_index;
    uint8_t count;
    float sum;
} BMP180_PressureWindow_t;

typedef struct {
    float alpha_low;
    float alpha_medium;
    float alpha_high;
    float threshold_low_pa;
    float threshold_high_pa;
    float last_pressure_pa;
    float active_alpha;
    uint8_t first_data;
} BMP180_Filter_t;

typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} BMP180_Calibration_t;

typedef struct {
    uint16_t temperature;
    uint32_t pressure;
} BMP180_RawData_t;

typedef struct {
    float temperature_c;
    uint32_t pressure_pa;
} BMP180_Data_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    uint8_t pending_oss;
    uint8_t initialized;

    BMP180_State_t state;
    uint32_t ready_at_ms;

    uint8_t i2c_busy_tracking;
    uint32_t i2c_busy_started_ms;

    float startup_pressure_pa;
    BMP180_StartupState_t startup_state;
    uint8_t startup_sample_count;
    uint32_t next_startup_sample_ms;
    uint32_t startup_samples_pa[BMP180_STARTUP_SAMPLE_COUNT];
} BMP180_Handle_t;

BMP180_Status_t BMP180_Init(
        BMP180_Handle_t *bmp,
        I2C_HandleTypeDef *hi2c,
        uint8_t address);

BMP180_Status_t BMP180_DeInit(BMP180_Handle_t *bmp);
BMP180_Status_t BMP180_CheckDeviceID(BMP180_Handle_t *bmp);
BMP180_Status_t BMP180_ReadCalibrationOffsets(
        BMP180_Handle_t *bmp,
        BMP180_Calibration_t *offsets);

/* Low-level APIs, useful for tests. Do not call them during an active measurement. */
BMP180_Status_t BMP180_SendControlCmd(
        BMP180_Handle_t *bmp,
        uint8_t cmd_val,
        uint8_t oss_value);
BMP180_Status_t BMP180_ReadRawTemperature(
        BMP180_Handle_t *bmp,
        BMP180_RawData_t *raw);
BMP180_Status_t BMP180_ReadRawPressure(
        BMP180_Handle_t *bmp,
        BMP180_RawData_t *raw);

BMP180_Status_t BMP180_ConvertRawTempToScaled(
        BMP180_Handle_t *bmp,
        const BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets);
BMP180_Status_t BMP180_ConvertRawPressToScaled(
        BMP180_Handle_t *bmp,
        const BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets);
BMP180_Status_t BMP180_ConvertRawToScaled(
        BMP180_Handle_t *bmp,
        const BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets);

BMP180_Status_t BMP180_StartMeasurement(
        BMP180_Handle_t *bmp,
        uint8_t oss,
        uint32_t now_ms);

BMP180_Status_t BMP180_ProcessMeasurement(
        BMP180_Handle_t *bmp,
        BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets,
        BMP180_PressureWindow_t *window,
        uint32_t now_ms);

BMP180_Status_t BMP180_AbortMeasurement(BMP180_Handle_t *bmp);


BMP180_Status_t BMP180_StartStartupCalibration(
        BMP180_Handle_t *bmp,
        uint32_t now_ms);

BMP180_Status_t BMP180_ProcessStartupCalibration(
        BMP180_Handle_t *bmp,
        BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets,
        BMP180_PressureWindow_t *window,
        uint32_t now_ms);

BMP180_Status_t BMP180_CalculateAltitude(
        uint32_t pressure_pa,
        float *altitude_m);
BMP180_Status_t BMP180_CalculateRelativeAltitude(
        float current_pressure_pa,
        float startup_pressure_pa,
        float *relative_altitude_m);

void BMP180_WindowInit(BMP180_PressureWindow_t *window);
void BMP180_WindowPush(
        BMP180_PressureWindow_t *window,
        float pressure_pa);
float BMP180_WindowGetAvg(const BMP180_PressureWindow_t *window);

void BMP180_FilterInit(BMP180_Filter_t *filter);
float BMP180_EMAFilter(
        BMP180_Filter_t *filter,
        float current_pressure_pa);

#endif /* DEVICES_INC_BMP180_H_ */
