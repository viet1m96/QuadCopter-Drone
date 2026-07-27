/*
 * bmp180.c
 *
 *  Created on: Jul 11, 2026
 *      Author: vietht-hl
 */

#include "bmp180.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static uint8_t bmp180_time_reached(uint32_t now_ms, uint32_t target_ms)
{
    return ((int32_t)(now_ms - target_ms) >= 0) ? 1U : 0U;
}

static uint32_t bmp180_pressure_delay_ms(uint8_t oss)
{
    switch (oss) {
    case BMP180_PRESSURE_OSS0:
        return BMP180_OSS0_DELAY_MS;
    case BMP180_PRESSURE_OSS1:
        return BMP180_OSS1_DELAY_MS;
    case BMP180_PRESSURE_OSS2:
        return BMP180_OSS2_DELAY_MS;
    case BMP180_PRESSURE_OSS3:
        return BMP180_OSS3_DELAY_MS;
    default:
        return 0U;
    }
}

static BMP180_Status_t bmp180_check_initialized(const BMP180_Handle_t *bmp)
{
    if (bmp == NULL || bmp->hi2c == NULL) {
        return BMP180_ERR_NULL;
    }

    if (bmp->initialized == 0U) {
        return BMP180_ERR_UNINITIALIZED;
    }

    return BMP180_OK;
}

static BMP180_Status_t bmp180_map_hal_status(HAL_StatusTypeDef hal_status)
{
    switch (hal_status) {
    case HAL_OK:
        return BMP180_OK;
    case HAL_BUSY:
        return BMP180_I2C_BUSY;
    case HAL_TIMEOUT:
        return BMP180_I2C_TIMEOUT;
    case HAL_ERROR:
    default:
        return BMP180_I2C_ERROR;
    }
}

static BMP180_Status_t bmp180_track_i2c_status(
        BMP180_Handle_t *bmp,
        BMP180_Status_t status,
        uint32_t now_ms)
{
    if (bmp == NULL) {
        return BMP180_ERR_NULL;
    }

    if (status != BMP180_I2C_BUSY) {
        bmp->i2c_busy_tracking = 0U;
        bmp->i2c_busy_started_ms = 0U;
        return status;
    }

    if (bmp->i2c_busy_tracking == 0U) {
        bmp->i2c_busy_tracking = 1U;
        bmp->i2c_busy_started_ms = now_ms;
        return BMP180_I2C_BUSY;
    }

    if ((uint32_t)(now_ms - bmp->i2c_busy_started_ms) >=
        BMP180_I2C_BUSY_TIMEOUT_MS) {
        bmp->i2c_busy_tracking = 0U;
        bmp->i2c_busy_started_ms = 0U;
        return BMP180_I2C_TIMEOUT;
    }

    return BMP180_I2C_BUSY;
}

static void bmp180_reset_measurement(BMP180_Handle_t *bmp)
{
    if (bmp == NULL) {
        return;
    }

    bmp->state = BMP180_STATE_IDLE;
    bmp->ready_at_ms = 0U;
    bmp->i2c_busy_tracking = 0U;
    bmp->i2c_busy_started_ms = 0U;
}

static BMP180_Status_t bmp180_write_reg(
        BMP180_Handle_t *bmp,
        uint8_t reg,
        uint8_t value)
{
    if (bmp == NULL || bmp->hi2c == NULL) {
        return BMP180_ERR_NULL;
    }

    uint16_t dev_address = (uint16_t)((uint16_t)bmp->address << 1U);

    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Write(
            bmp->hi2c,
            dev_address,
            reg,
            I2C_MEMADD_SIZE_8BIT,
            &value,
            1U,
            BMP180_HAL_I2C_TIMEOUT_MS);

    return bmp180_map_hal_status(hal_status);
}

static BMP180_Status_t bmp180_read_reg(
        BMP180_Handle_t *bmp,
        uint8_t reg,
        uint8_t *result)
{
    if (bmp == NULL || bmp->hi2c == NULL || result == NULL) {
        return BMP180_ERR_NULL;
    }

    uint16_t dev_address = (uint16_t)((uint16_t)bmp->address << 1U);

    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read(
            bmp->hi2c,
            dev_address,
            reg,
            I2C_MEMADD_SIZE_8BIT,
            result,
            1U,
            BMP180_HAL_I2C_TIMEOUT_MS);

    return bmp180_map_hal_status(hal_status);
}

static BMP180_Status_t bmp180_read_regs(
        BMP180_Handle_t *bmp,
        uint8_t start_reg,
        uint8_t len,
        uint8_t *result)
{
    if (bmp == NULL || bmp->hi2c == NULL || result == NULL) {
        return BMP180_ERR_NULL;
    }

    if (len == 0U) {
        return BMP180_INVALID_CONFIG;
    }

    uint16_t dev_address = (uint16_t)((uint16_t)bmp->address << 1U);

    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read(
            bmp->hi2c,
            dev_address,
            start_reg,
            I2C_MEMADD_SIZE_8BIT,
            result,
            len,
            BMP180_HAL_I2C_TIMEOUT_MS);

    return bmp180_map_hal_status(hal_status);
}

BMP180_Status_t BMP180_Init(
        BMP180_Handle_t *bmp,
        I2C_HandleTypeDef *hi2c,
        uint8_t address)
{
    if (bmp == NULL || hi2c == NULL) {
        return BMP180_ERR_NULL;
    }

    if (address > 0x7FU) {
        return BMP180_INVALID_CONFIG;
    }

    (void)memset(bmp, 0, sizeof(*bmp));

    bmp->hi2c = hi2c;
    bmp->address = address;
    bmp->pending_oss = BMP180_PRESSURE_OSS0;
    bmp->state = BMP180_STATE_IDLE;
    bmp->startup_state = BMP180_STARTUP_IDLE;

    BMP180_Status_t status = BMP180_CheckDeviceID(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    bmp->initialized = 1U;
    return BMP180_OK;
}

BMP180_Status_t BMP180_DeInit(BMP180_Handle_t *bmp)
{
    if (bmp == NULL) {
        return BMP180_ERR_NULL;
    }

    (void)memset(bmp, 0, sizeof(*bmp));
    return BMP180_OK;
}

BMP180_Status_t BMP180_CheckDeviceID(BMP180_Handle_t *bmp)
{
    if (bmp == NULL || bmp->hi2c == NULL) {
        return BMP180_ERR_NULL;
    }

    uint8_t result = 0U;
    BMP180_Status_t status = bmp180_read_reg(
            bmp,
            BMP180_REG_CHIP_ID,
            &result);

    if (status != BMP180_OK) {
        return status;
    }

    if (result != BMP180_CHIP_ID_VALUE) {
        return BMP180_ERR_BAD_DEVICE_ID;
    }

    return BMP180_OK;
}

BMP180_Status_t BMP180_ReadCalibrationOffsets(
        BMP180_Handle_t *bmp,
        BMP180_Calibration_t *offsets)
{
    BMP180_Status_t status = bmp180_check_initialized(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    if (offsets == NULL) {
        return BMP180_ERR_NULL;
    }

    uint8_t data[22];
    status = bmp180_read_regs(
            bmp,
            BMP180_REG_AC1,
            (uint8_t)sizeof(data),
            data);

    if (status != BMP180_OK) {
        return status;
    }

    offsets->ac1 = byte_utils_i16_from_be(data[0], data[1]);
    offsets->ac2 = byte_utils_i16_from_be(data[2], data[3]);
    offsets->ac3 = byte_utils_i16_from_be(data[4], data[5]);
    offsets->ac4 = byte_utils_u16_from_be(data[6], data[7]);
    offsets->ac5 = byte_utils_u16_from_be(data[8], data[9]);
    offsets->ac6 = byte_utils_u16_from_be(data[10], data[11]);
    offsets->b1 = byte_utils_i16_from_be(data[12], data[13]);
    offsets->b2 = byte_utils_i16_from_be(data[14], data[15]);
    offsets->mb = byte_utils_i16_from_be(data[16], data[17]);
    offsets->mc = byte_utils_i16_from_be(data[18], data[19]);
    offsets->md = byte_utils_i16_from_be(data[20], data[21]);

    return BMP180_OK;
}

BMP180_Status_t BMP180_SendControlCmd(
        BMP180_Handle_t *bmp,
        uint8_t cmd_val,
        uint8_t oss_value)
{
    BMP180_Status_t status = bmp180_check_initialized(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    switch (cmd_val) {
    case BMP180_CMD_PRESSURE: {
        if (oss_value > BMP180_PRESSURE_OSS3) {
            return BMP180_INVALID_CONFIG;
        }

        uint8_t cmd = (uint8_t)(cmd_val | (uint8_t)(oss_value << 6U));
        status = bmp180_write_reg(bmp, BMP180_REG_CTRL_MEAS, cmd);

        if (status == BMP180_OK) {
            bmp->pending_oss = oss_value;
        }

        return status;
    }

    case BMP180_CMD_TEMPERATURE:
        return bmp180_write_reg(
                bmp,
                BMP180_REG_CTRL_MEAS,
                BMP180_CMD_TEMPERATURE);

    default:
        return BMP180_INVALID_CONFIG;
    }
}

BMP180_Status_t BMP180_ReadRawTemperature(
        BMP180_Handle_t *bmp,
        BMP180_RawData_t *raw)
{
    BMP180_Status_t status = bmp180_check_initialized(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    if (raw == NULL) {
        return BMP180_ERR_NULL;
    }

    uint8_t result[2];
    status = bmp180_read_regs(bmp, BMP180_REG_MSB, 2U, result);
    if (status != BMP180_OK) {
        return status;
    }

    raw->temperature =
            ((uint16_t)result[0] << 8U) |
            (uint16_t)result[1];

    return BMP180_OK;
}

BMP180_Status_t BMP180_ReadRawPressure(
        BMP180_Handle_t *bmp,
        BMP180_RawData_t *raw)
{
    BMP180_Status_t status = bmp180_check_initialized(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    if (raw == NULL) {
        return BMP180_ERR_NULL;
    }

    if (bmp->pending_oss > BMP180_PRESSURE_OSS3) {
        return BMP180_INVALID_CONFIG;
    }

    uint8_t result[3];
    status = bmp180_read_regs(bmp, BMP180_REG_MSB, 3U, result);
    if (status != BMP180_OK) {
        return status;
    }

    uint32_t raw24 =
            ((uint32_t)result[0] << 16U) |
            ((uint32_t)result[1] << 8U) |
            (uint32_t)result[2];

    raw->pressure = raw24 >> (8U - bmp->pending_oss);
    return BMP180_OK;
}

static BMP180_Status_t bmp180_calculate_b5(
        const BMP180_RawData_t *raw,
        const BMP180_Calibration_t *offsets,
        int32_t *b5,
        int32_t *temperature_01c)
{
    if (raw == NULL || offsets == NULL ||
        b5 == NULL || temperature_01c == NULL) {
        return BMP180_ERR_NULL;
    }

    int32_t x1 = (int32_t)(
            (((int64_t)raw->temperature - (int64_t)offsets->ac6) *
             (int64_t)offsets->ac5) >> 15);

    int32_t denominator = x1 + (int32_t)offsets->md;
    if (denominator == 0) {
        return BMP180_INVALID_CONFIG;
    }

    int32_t x2 = (int32_t)(
            ((int64_t)offsets->mc * 2048LL) /
            (int64_t)denominator);

    *b5 = x1 + x2;
    *temperature_01c = (*b5 + 8) >> 4;

    return BMP180_OK;
}

BMP180_Status_t BMP180_ConvertRawTempToScaled(
        BMP180_Handle_t *bmp,
        const BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets)
{
    BMP180_Status_t status = bmp180_check_initialized(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    if (raw == NULL || scaled == NULL || offsets == NULL) {
        return BMP180_ERR_NULL;
    }

    int32_t b5;
    int32_t temperature_01c;

    status = bmp180_calculate_b5(
            raw,
            offsets,
            &b5,
            &temperature_01c);

    if (status != BMP180_OK) {
        return status;
    }

    scaled->temperature_c = (float)temperature_01c / 10.0f;
    return BMP180_OK;
}

static BMP180_Status_t bmp180_calculate_pressure_pa(
        const BMP180_Handle_t *bmp,
        const BMP180_RawData_t *raw,
        const BMP180_Calibration_t *offsets,
        int32_t b5,
        uint32_t *pressure_pa)
{
    if (bmp == NULL || raw == NULL || offsets == NULL || pressure_pa == NULL) {
        return BMP180_ERR_NULL;
    }

    if (bmp->pending_oss > BMP180_PRESSURE_OSS3) {
        return BMP180_INVALID_CONFIG;
    }

    uint8_t oss = bmp->pending_oss;
    int32_t b6 = b5 - 4000;

    int32_t b6_squared_div_4096 = (int32_t)(
            ((int64_t)b6 * (int64_t)b6) >> 12);

    int32_t x1 = (int32_t)(
            ((int64_t)offsets->b2 *
             (int64_t)b6_squared_div_4096) >> 11);

    int32_t x2 = (int32_t)(
            ((int64_t)offsets->ac2 * (int64_t)b6) >> 11);

    int32_t x3 = x1 + x2;

    int32_t b3 = (int32_t)(
            ((((int64_t)offsets->ac1 * 4LL + (int64_t)x3) *
              (1LL << oss)) + 2LL) >> 2);

    x1 = (int32_t)(
            ((int64_t)offsets->ac3 * (int64_t)b6) >> 13);

    x2 = (int32_t)(
            ((int64_t)offsets->b1 *
             (int64_t)b6_squared_div_4096) >> 16);

    x3 = (x1 + x2 + 2) >> 2;

    int32_t b4_factor = x3 + 32768;
    if (b4_factor <= 0) {
        return BMP180_INVALID_CONFIG;
    }

    uint32_t b4 = (uint32_t)(
            ((uint64_t)offsets->ac4 *
             (uint64_t)(uint32_t)b4_factor) >> 15);

    if (b4 == 0U) {
        return BMP180_INVALID_CONFIG;
    }

    int64_t up_minus_b3 =
            (int64_t)raw->pressure - (int64_t)b3;

    if (up_minus_b3 <= 0) {
        return BMP180_INVALID_CONFIG;
    }

    uint64_t b7 =
            (uint64_t)up_minus_b3 *
            (uint64_t)(50000U >> oss);

    uint32_t p;
    if (b7 < 0x80000000ULL) {
        p = (uint32_t)((b7 * 2ULL) / b4);
    } else {
        p = (uint32_t)((b7 / b4) * 2ULL);
    }

    int32_t p_div_256 = (int32_t)(p >> 8U);

    x1 = (int32_t)(
            (((int64_t)p_div_256 * (int64_t)p_div_256) *
             3038LL) >> 16);

    x2 = (int32_t)(
            (-(int64_t)7357 * (int64_t)p) >> 16);

    int64_t final_pressure =
            (int64_t)p +
            (int64_t)((x1 + x2 + 3791) >> 4);

    if (final_pressure <= 0) {
        return BMP180_INVALID_CONFIG;
    }

    *pressure_pa = (uint32_t)final_pressure;
    return BMP180_OK;
}

BMP180_Status_t BMP180_ConvertRawPressToScaled(
        BMP180_Handle_t *bmp,
        const BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets)
{
    BMP180_Status_t status = bmp180_check_initialized(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    if (raw == NULL || scaled == NULL || offsets == NULL) {
        return BMP180_ERR_NULL;
    }

    int32_t b5;
    int32_t temperature_01c;

    status = bmp180_calculate_b5(
            raw,
            offsets,
            &b5,
            &temperature_01c);

    if (status != BMP180_OK) {
        return status;
    }

    return bmp180_calculate_pressure_pa(
            bmp,
            raw,
            offsets,
            b5,
            &scaled->pressure_pa);
}

BMP180_Status_t BMP180_ConvertRawToScaled(
        BMP180_Handle_t *bmp,
        const BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets)
{
    BMP180_Status_t status = bmp180_check_initialized(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    if (raw == NULL || scaled == NULL || offsets == NULL) {
        return BMP180_ERR_NULL;
    }

    int32_t b5;
    int32_t temperature_01c;

    status = bmp180_calculate_b5(
            raw,
            offsets,
            &b5,
            &temperature_01c);

    if (status != BMP180_OK) {
        return status;
    }

    scaled->temperature_c = (float)temperature_01c / 10.0f;

    return bmp180_calculate_pressure_pa(
            bmp,
            raw,
            offsets,
            b5,
            &scaled->pressure_pa);
}

BMP180_Status_t BMP180_StartMeasurement(
        BMP180_Handle_t *bmp,
        uint8_t oss,
        uint32_t now_ms)
{
    BMP180_Status_t status = bmp180_check_initialized(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    if (oss > BMP180_PRESSURE_OSS3) {
        return BMP180_INVALID_CONFIG;
    }

    if (bmp->state != BMP180_STATE_IDLE) {
        return BMP180_ERR_INVALID_STATE;
    }

    bmp->pending_oss = oss;

    status = BMP180_SendControlCmd(
            bmp,
            BMP180_CMD_TEMPERATURE,
            0U);

    status = bmp180_track_i2c_status(bmp, status, now_ms);
    if (status != BMP180_OK) {
        return status;
    }

    bmp->state = BMP180_STATE_WAIT_TEMPERATURE;
    bmp->ready_at_ms = now_ms + BMP180_TEMPERATURE_DELAY_MS;

    return BMP180_OK;
}

BMP180_Status_t BMP180_ProcessMeasurement(
        BMP180_Handle_t *bmp,
        BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets,
        BMP180_PressureWindow_t *window,
        uint32_t now_ms)
{
    BMP180_Status_t status = bmp180_check_initialized(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    if (raw == NULL || scaled == NULL || offsets == NULL) {
        return BMP180_ERR_NULL;
    }

    switch (bmp->state) {
    case BMP180_STATE_IDLE:
        return BMP180_ERR_INVALID_STATE;

    case BMP180_STATE_WAIT_TEMPERATURE:
        if (bmp180_time_reached(now_ms, bmp->ready_at_ms) == 0U) {
            return BMP180_IN_PROGRESS;
        }

        status = BMP180_ReadRawTemperature(bmp, raw);
        status = bmp180_track_i2c_status(bmp, status, now_ms);

        if (status == BMP180_I2C_BUSY) {
            return status;
        }

        if (status != BMP180_OK) {
            bmp180_reset_measurement(bmp);
            return status;
        }

        bmp->state = BMP180_STATE_START_PRESSURE;
        return BMP180_IN_PROGRESS;

    case BMP180_STATE_START_PRESSURE:
        status = BMP180_SendControlCmd(
                bmp,
                BMP180_CMD_PRESSURE,
                bmp->pending_oss);

        status = bmp180_track_i2c_status(bmp, status, now_ms);

        if (status == BMP180_I2C_BUSY) {
            return status;
        }

        if (status != BMP180_OK) {
            bmp180_reset_measurement(bmp);
            return status;
        }

        bmp->state = BMP180_STATE_WAIT_PRESSURE;
        bmp->ready_at_ms =
                now_ms + bmp180_pressure_delay_ms(bmp->pending_oss);

        return BMP180_IN_PROGRESS;

    case BMP180_STATE_WAIT_PRESSURE:
        if (bmp180_time_reached(now_ms, bmp->ready_at_ms) == 0U) {
            return BMP180_IN_PROGRESS;
        }

        status = BMP180_ReadRawPressure(bmp, raw);
        status = bmp180_track_i2c_status(bmp, status, now_ms);

        if (status == BMP180_I2C_BUSY) {
            return status;
        }

        if (status != BMP180_OK) {
            bmp180_reset_measurement(bmp);
            return status;
        }

        status = BMP180_ConvertRawToScaled(
                bmp,
                raw,
                scaled,
                offsets);

        if (status != BMP180_OK) {
            bmp180_reset_measurement(bmp);
            return status;
        }

        if (window != NULL) {
            BMP180_WindowPush(window, (float)scaled->pressure_pa);
        }

        bmp180_reset_measurement(bmp);
        return BMP180_OK;

    default:
        bmp180_reset_measurement(bmp);
        return BMP180_ERR_INVALID_STATE;
    }
}

BMP180_Status_t BMP180_AbortMeasurement(BMP180_Handle_t *bmp)
{
    BMP180_Status_t status = bmp180_check_initialized(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    bmp180_reset_measurement(bmp);
    return BMP180_OK;
}

static int bmp180_compare_u32(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a;
    uint32_t y = *(const uint32_t *)b;

    if (x < y) {
        return -1;
    }

    if (x > y) {
        return 1;
    }

    return 0;
}

static BMP180_Status_t bmp180_finish_startup_calibration(
        BMP180_Handle_t *bmp)
{
    if (bmp == NULL) {
        return BMP180_ERR_NULL;
    }

    if (bmp->startup_sample_count != BMP180_STARTUP_SAMPLE_COUNT) {
        return BMP180_ERR_INVALID_STATE;
    }

    qsort(
            bmp->startup_samples_pa,
            BMP180_STARTUP_SAMPLE_COUNT,
            sizeof(bmp->startup_samples_pa[0]),
            bmp180_compare_u32);

    uint64_t sum = 0ULL;
    uint8_t first = BMP180_STARTUP_TRIM_COUNT;
    uint8_t last =
            (uint8_t)(BMP180_STARTUP_SAMPLE_COUNT -
                      BMP180_STARTUP_TRIM_COUNT);

    for (uint8_t i = first; i < last; ++i) {
        sum += bmp->startup_samples_pa[i];
    }

    uint8_t kept_samples = (uint8_t)(last - first);
    if (kept_samples == 0U) {
        bmp->startup_state = BMP180_STARTUP_ERROR;
        return BMP180_INVALID_CONFIG;
    }

    bmp->startup_pressure_pa =
            (float)sum / (float)kept_samples;
    bmp->startup_state = BMP180_STARTUP_COMPLETE;

    return BMP180_OK;
}

BMP180_Status_t BMP180_StartStartupCalibration(
        BMP180_Handle_t *bmp,
        uint32_t now_ms)
{
    BMP180_Status_t status = bmp180_check_initialized(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    if (bmp->state != BMP180_STATE_IDLE ||
        bmp->startup_state == BMP180_STARTUP_RUNNING) {
        return BMP180_ERR_INVALID_STATE;
    }

    bmp->startup_pressure_pa = 0.0f;
    bmp->startup_sample_count = 0U;
    bmp->next_startup_sample_ms = now_ms;
    bmp->startup_state = BMP180_STARTUP_RUNNING;

    (void)memset(
            bmp->startup_samples_pa,
            0,
            sizeof(bmp->startup_samples_pa));

    return BMP180_OK;
}

BMP180_Status_t BMP180_ProcessStartupCalibration(
        BMP180_Handle_t *bmp,
        BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets,
        BMP180_PressureWindow_t *window,
        uint32_t now_ms)
{
    BMP180_Status_t status = bmp180_check_initialized(bmp);
    if (status != BMP180_OK) {
        return status;
    }

    if (raw == NULL || scaled == NULL || offsets == NULL) {
        return BMP180_ERR_NULL;
    }

    if (bmp->startup_state != BMP180_STARTUP_RUNNING) {
        return BMP180_ERR_INVALID_STATE;
    }

    if (bmp->startup_sample_count >= BMP180_STARTUP_SAMPLE_COUNT) {
        return bmp180_finish_startup_calibration(bmp);
    }

    if (bmp->state == BMP180_STATE_IDLE) {
        if (bmp180_time_reached(
                now_ms,
                bmp->next_startup_sample_ms) == 0U) {
            return BMP180_IN_PROGRESS;
        }

        status = BMP180_StartMeasurement(
                bmp,
                BMP180_PRESSURE_OSS3,
                now_ms);

        if (status == BMP180_OK) {
            return BMP180_IN_PROGRESS;
        }

        if (status == BMP180_I2C_BUSY) {
            return status;
        }

        bmp->startup_state = BMP180_STARTUP_ERROR;
        return status;
    }

    status = BMP180_ProcessMeasurement(
            bmp,
            raw,
            scaled,
            offsets,
            window,
            now_ms);

    if (status == BMP180_IN_PROGRESS || status == BMP180_I2C_BUSY) {
        return status;
    }

    if (status != BMP180_OK) {
        bmp->startup_state = BMP180_STARTUP_ERROR;
        return status;
    }

    bmp->startup_samples_pa[bmp->startup_sample_count] =
            scaled->pressure_pa;
    bmp->startup_sample_count++;

    if (bmp->startup_sample_count >= BMP180_STARTUP_SAMPLE_COUNT) {
        return bmp180_finish_startup_calibration(bmp);
    }

    bmp->next_startup_sample_ms =
            now_ms + BMP180_STARTUP_SAMPLE_INTERVAL_MS;

    return BMP180_IN_PROGRESS;
}

BMP180_Status_t BMP180_CalculateAltitude(
        uint32_t pressure_pa,
        float *altitude_m)
{
    if (altitude_m == NULL) {
        return BMP180_ERR_NULL;
    }

    if (pressure_pa == 0U) {
        return BMP180_INVALID_CONFIG;
    }

    float pressure_ratio =
            (float)pressure_pa /
            BMP180_STANDARD_SEA_LEVEL_PRESSURE_PA;

    *altitude_m =
            44330.0f *
            (1.0f - powf(
                    pressure_ratio,
                    BMP180_ALTITUDE_EXPONENT));

    return BMP180_OK;
}

BMP180_Status_t BMP180_CalculateRelativeAltitude(
        float current_pressure_pa,
        float startup_pressure_pa,
        float *relative_altitude_m)
{
    if (relative_altitude_m == NULL) {
        return BMP180_ERR_NULL;
    }

    if (current_pressure_pa <= 0.0f || startup_pressure_pa <= 0.0f) {
        return BMP180_INVALID_CONFIG;
    }

    float pressure_ratio =
            current_pressure_pa /
            startup_pressure_pa;

    *relative_altitude_m =
            44330.0f *
            (1.0f - powf(
                    pressure_ratio,
                    BMP180_ALTITUDE_EXPONENT));

    return BMP180_OK;
}

void BMP180_WindowInit(BMP180_PressureWindow_t *window)
{
    if (window == NULL) {
        return;
    }

    window->count = 0U;
    window->write_index = 0U;
    window->sum = 0.0f;

    for (uint8_t i = 0U; i < BMP180_WINDOW_SIZE; ++i) {
        window->pressure_pa[i] = 0.0f;
    }
}

void BMP180_WindowPush(
        BMP180_PressureWindow_t *window,
        float pressure_pa)
{
    if (window == NULL || pressure_pa <= 0.0f) {
        return;
    }

    if (window->count == BMP180_WINDOW_SIZE) {
        window->sum -= window->pressure_pa[window->write_index];
    } else {
        window->count++;
    }

    window->pressure_pa[window->write_index] = pressure_pa;
    window->sum += pressure_pa;
    window->write_index++;

    if (window->write_index >= BMP180_WINDOW_SIZE) {
        window->write_index = 0U;
    }
}

float BMP180_WindowGetAvg(const BMP180_PressureWindow_t *window)
{
    if (window == NULL || window->count == 0U) {
        return 0.0f;
    }

    return window->sum / (float)window->count;
}

void BMP180_FilterInit(BMP180_Filter_t *filter)
{
    if (filter == NULL) {
        return;
    }

    filter->alpha_low = BMP180_FILTER_ALPHA_LOW;
    filter->alpha_medium = BMP180_FILTER_ALPHA_MEDIUM;
    filter->alpha_high = BMP180_FILTER_ALPHA_HIGH;
    filter->threshold_low_pa = BMP180_FILTER_THRESHOLD_LOW_PA;
    filter->threshold_high_pa = BMP180_FILTER_THRESHOLD_HIGH_PA;
    filter->last_pressure_pa = 0.0f;
    filter->active_alpha = filter->alpha_low;
    filter->first_data = 1U;
}

float BMP180_EMAFilter(
        BMP180_Filter_t *filter,
        float current_pressure_pa)
{
    if (filter == NULL) {
        return 0.0f;
    }

    if (current_pressure_pa <= 0.0f) {
        return filter->last_pressure_pa;
    }

    if (filter->first_data != 0U) {
        filter->last_pressure_pa = current_pressure_pa;
        filter->active_alpha = filter->alpha_low;
        filter->first_data = 0U;
        return current_pressure_pa;
    }

    float pressure_error = fabsf(
            current_pressure_pa -
            filter->last_pressure_pa);

    if (pressure_error < filter->threshold_low_pa) {
        filter->active_alpha = filter->alpha_low;
    } else if (pressure_error < filter->threshold_high_pa) {
        filter->active_alpha = filter->alpha_medium;
    } else {
        filter->active_alpha = filter->alpha_high;
    }

    filter->last_pressure_pa +=
            filter->active_alpha *
            (current_pressure_pa - filter->last_pressure_pa);

    return filter->last_pressure_pa;
}









;
