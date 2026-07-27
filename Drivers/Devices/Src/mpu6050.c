#include "mpu6050.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include "stdio.h"
#include "byte_utils.h"

typedef struct {
    Vector3f_t gyro_sum_raw;
    Vector3f_t accel_sum_raw;
} MPU6050_CalibWindow_t;

static MPU6050_Status_t mpu6050_from_hal_status(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) return MPU6050_OK;
    if (status == HAL_BUSY) return MPU6050_I2C_BUSY;
    if (status == HAL_TIMEOUT) return MPU6050_I2C_TIMEOUT;
    return MPU6050_I2C_ERROR;
}

static MPU6050_Status_t mpu6050_write_reg(
        MPU6050_Handle_t *mpu, uint8_t reg, uint8_t value)
{
    if (mpu == NULL || mpu->hi2c == NULL) return MPU6050_ERR_NULL;

    return mpu6050_from_hal_status(HAL_I2C_Mem_Write(
            mpu->hi2c,
            (uint16_t)(mpu->config.address << 1U),
            reg,
            I2C_MEMADD_SIZE_8BIT,
            &value,
            1U,
            MPU6050_I2C_TIMEOUT_MS));
}

static MPU6050_Status_t mpu6050_read_reg(
        MPU6050_Handle_t *mpu, uint8_t reg, uint8_t *result)
{
    if (mpu == NULL || mpu->hi2c == NULL || result == NULL) {
        return MPU6050_ERR_NULL;
    }

    return mpu6050_from_hal_status(HAL_I2C_Mem_Read(
            mpu->hi2c,
            (uint16_t)(mpu->config.address << 1U),
            reg,
            I2C_MEMADD_SIZE_8BIT,
            result,
            1U,
            MPU6050_I2C_TIMEOUT_MS));
}

static MPU6050_Status_t mpu6050_read_regs(
        MPU6050_Handle_t *mpu,
        uint8_t start_reg,
        uint8_t len,
        uint8_t *result)
{
    if (mpu == NULL || mpu->hi2c == NULL || result == NULL) {
        return MPU6050_ERR_NULL;
    }
    if (len == 0U) return MPU6050_INVALID_CONFIG;

    return mpu6050_from_hal_status(HAL_I2C_Mem_Read(
            mpu->hi2c,
            (uint16_t)(mpu->config.address << 1U),
            start_reg,
            I2C_MEMADD_SIZE_8BIT,
            result,
            len,
            MPU6050_I2C_TIMEOUT_MS));
}

static void mpu6050_parse_vector3_be(
        const uint8_t *bytes, Vector3i16_t *vector)
{
    vector->x = byte_utils_i16_from_be(bytes[0], bytes[1]);
    vector->y = byte_utils_i16_from_be(bytes[2], bytes[3]);
    vector->z = byte_utils_i16_from_be(bytes[4], bytes[5]);
}

static MPU6050_Status_t mpu6050_validate_config(
        const MPU6050_Config_t *config)
{
    if (config == NULL) return MPU6050_ERR_NULL;
    if (config->address > 0x7FU) return MPU6050_INVALID_CONFIG;
    if (config->clksrc == 6U || config->clksrc > MPU6050_CLKSRC_STOPCLK) {
        return MPU6050_INVALID_CONFIG;
    }
    if (config->dlpf_config > MPU6050_DLPF_CFG_6
            || config->fs_sel_config > MPU6050_GYRO_CONFIG_FS_2000DPS
            || config->accel_sel_config > MPU6050_ACCEL_CONFIG_AFS_16G) {
        return MPU6050_INVALID_CONFIG;
    }
    return MPU6050_OK;
}

static MPU6050_Status_t mpu6050_collect_still_window(
        MPU6050_Handle_t *mpu,
        MPU6050_RawData_t *raw,
        const MPU6050_StillnessConfig_t *still,
        MPU6050_CalibWindow_t *window)
{
    if (mpu == NULL || raw == NULL || still == NULL || window == NULL) {
        return MPU6050_ERR_NULL;
    }
    if (still->sample_count == 0U) return MPU6050_INVALID_CONFIG;

    Vector3i16_t gyro_min = {INT16_MAX, INT16_MAX, INT16_MAX};
    Vector3i16_t gyro_max = {INT16_MIN, INT16_MIN, INT16_MIN};
    Vector3i16_t accel_min = {INT16_MAX, INT16_MAX, INT16_MAX};
    Vector3i16_t accel_max = {INT16_MIN, INT16_MIN, INT16_MIN};

    window->gyro_sum_raw = Vector3f_Zero();
    window->accel_sum_raw = Vector3f_Zero();

    for (uint32_t i = 0U; i < still->sample_count; ++i) {
        MPU6050_Status_t status = MPU6050_ReadRawData(mpu, raw);
        if (status != MPU6050_OK) return status;

        Vector3i16_UpdateMinMax(&gyro_min, &gyro_max, raw->gyro);
        Vector3i16_UpdateMinMax(&accel_min, &accel_max, raw->accel);
        Vector3f_AccumulateInt16(&window->gyro_sum_raw, raw->gyro);
        Vector3f_AccumulateInt16(&window->accel_sum_raw, raw->accel);

        if (still->sample_delay_ms > 0U) HAL_Delay(still->sample_delay_ms);
    }

    if (Vector3i16_RangeExceeds(gyro_min, gyro_max, still->gyro_threshold)) {
        return MPU6050_ERR_MOVING;
    }
    if (Vector3i16_RangeExceeds(
            accel_min, accel_max, still->accel_axis_threshold)) {
        return MPU6050_ERR_MOVING;
    }

    Vector3f_t avg_accel_raw = Vector3f_DivideScalar(
            window->accel_sum_raw, (float)still->sample_count);
    float avg_accel_magnitude = Vector3f_Magnitude(avg_accel_raw);

    if (fabsf(avg_accel_magnitude - still->accel_threshold)
            > still->accel_allowed_gap) {
        return MPU6050_ERR_MOVING;
    }
    return MPU6050_OK;
}

static MPU6050_Status_t mpu6050_check_device_id(
        MPU6050_Handle_t *mpu)
{
    uint8_t who_am_i = 0U;
    MPU6050_Status_t status = mpu6050_read_reg(
            mpu, MPU6050_REG_WHO_AM_I, &who_am_i);
    if (status != MPU6050_OK) return status;

    return who_am_i == MPU6050_WHO_AM_I_VALUE
            ? MPU6050_OK
            : MPU6050_ERR_BAD_DEVICE_ID;
}

static MPU6050_Status_t mpu6050_wake_up_chip(
        MPU6050_Handle_t *mpu)
{
    return mpu6050_write_reg(
            mpu,
            MPU6050_REG_PWR_MGMT_1,
            0x00U);
}

static MPU6050_Status_t mpu6050_set_clock_source(
        MPU6050_Handle_t *mpu,
        uint8_t clksrc)
{
    if (clksrc == 6U || clksrc > MPU6050_CLKSRC_STOPCLK) {
        return MPU6050_INVALID_CONFIG;
    }

    uint8_t value = 0U;
    MPU6050_Status_t status = mpu6050_read_reg(
            mpu, MPU6050_REG_PWR_MGMT_1, &value);
    if (status != MPU6050_OK) return status;

    value &= (uint8_t)~(MPU6050_PWR1_CLKSEL_MASK
            << MPU6050_PWR1_CLKSEL_POS);
    value |= (uint8_t)((clksrc & MPU6050_PWR1_CLKSEL_MASK)
            << MPU6050_PWR1_CLKSEL_POS);

    status = mpu6050_write_reg(
            mpu,
            MPU6050_REG_PWR_MGMT_1,
            value);

    if (status == MPU6050_OK) {
        mpu->config.clksrc = clksrc;
    }

    return status;
}

static MPU6050_Status_t mpu6050_set_dlpf(
        MPU6050_Handle_t *mpu,
        uint8_t dlpf_config)
{
    if (dlpf_config > MPU6050_DLPF_CFG_6) {
        return MPU6050_INVALID_CONFIG;
    }

    uint8_t value = 0U;
    MPU6050_Status_t status = mpu6050_read_reg(
            mpu, MPU6050_REG_CONFIG, &value);
    if (status != MPU6050_OK) return status;

    value &= (uint8_t)~(MPU6050_CONFIG_DLPF_CFG_MASK
            << MPU6050_CONFIG_DLPF_CFG_POS);
    value |= (uint8_t)((dlpf_config & MPU6050_CONFIG_DLPF_CFG_MASK)
            << MPU6050_CONFIG_DLPF_CFG_POS);

    status = mpu6050_write_reg(
            mpu,
            MPU6050_REG_CONFIG,
            value);

    if (status == MPU6050_OK) {
        mpu->config.dlpf_config = dlpf_config;
    }

    return status;
}

static MPU6050_Status_t mpu6050_set_gyro_config(
        MPU6050_Handle_t *mpu,
        uint8_t fs_sel_config)
{
    if (fs_sel_config > MPU6050_GYRO_CONFIG_FS_2000DPS) {
        return MPU6050_INVALID_CONFIG;
    }

    uint8_t value = 0U;
    MPU6050_Status_t status = mpu6050_read_reg(
            mpu, MPU6050_REG_GYRO_CONFIG, &value);
    if (status != MPU6050_OK) return status;

    value &= (uint8_t)~(MPU6050_GYRO_CONFIG_FS_SEL_MASK
            << MPU6050_GYRO_CONFIG_FS_SEL_POS);
    value |= (uint8_t)((fs_sel_config & MPU6050_GYRO_CONFIG_FS_SEL_MASK)
            << MPU6050_GYRO_CONFIG_FS_SEL_POS);

    status = mpu6050_write_reg(
            mpu,
            MPU6050_REG_GYRO_CONFIG,
            value);
    if (status != MPU6050_OK) return status;

    switch (fs_sel_config) {
    case MPU6050_GYRO_CONFIG_FS_250DPS:
        mpu->gyro_scale = MPU6050_GYRO_CONFIG_FS_SEN0;
        break;

    case MPU6050_GYRO_CONFIG_FS_500DPS:
        mpu->gyro_scale = MPU6050_GYRO_CONFIG_FS_SEN1;
        break;

    case MPU6050_GYRO_CONFIG_FS_1000DPS:
        mpu->gyro_scale = MPU6050_GYRO_CONFIG_FS_SEN2;
        break;

    case MPU6050_GYRO_CONFIG_FS_2000DPS:
        mpu->gyro_scale = MPU6050_GYRO_CONFIG_FS_SEN3;
        break;

    default:
        return MPU6050_INVALID_CONFIG;
    }

    mpu->config.fs_sel_config = fs_sel_config;
    return MPU6050_OK;
}

static MPU6050_Status_t mpu6050_set_accel_config(
        MPU6050_Handle_t *mpu,
        uint8_t accel_sel_config)
{
    if (accel_sel_config > MPU6050_ACCEL_CONFIG_AFS_16G) {
        return MPU6050_INVALID_CONFIG;
    }

    uint8_t value = 0U;
    MPU6050_Status_t status = mpu6050_read_reg(
            mpu, MPU6050_REG_ACCEL_CONFIG, &value);
    if (status != MPU6050_OK) return status;

    value &= (uint8_t)~(MPU6050_ACCEL_CONFIG_AFS_SEL_MASK
            << MPU6050_ACCEL_CONFIG_AFS_SEL_POS);
    value |= (uint8_t)((accel_sel_config
            & MPU6050_ACCEL_CONFIG_AFS_SEL_MASK)
            << MPU6050_ACCEL_CONFIG_AFS_SEL_POS);

    status = mpu6050_write_reg(
            mpu,
            MPU6050_REG_ACCEL_CONFIG,
            value);
    if (status != MPU6050_OK) return status;

    switch (accel_sel_config) {
    case MPU6050_ACCEL_CONFIG_AFS_2G:
        mpu->accel_scale = MPU6050_ACCEL_CONFIG_AFS_SEN0;
        break;

    case MPU6050_ACCEL_CONFIG_AFS_4G:
        mpu->accel_scale = MPU6050_ACCEL_CONFIG_AFS_SEN1;
        break;

    case MPU6050_ACCEL_CONFIG_AFS_8G:
        mpu->accel_scale = MPU6050_ACCEL_CONFIG_AFS_SEN2;
        break;

    case MPU6050_ACCEL_CONFIG_AFS_16G:
        mpu->accel_scale = MPU6050_ACCEL_CONFIG_AFS_SEN3;
        break;

    default:
        return MPU6050_INVALID_CONFIG;
    }

    mpu->config.accel_sel_config = accel_sel_config;
    return MPU6050_OK;
}

static MPU6050_Status_t mpu6050_set_sample_rate(
        MPU6050_Handle_t *mpu,
        uint8_t sample_rate_div)
{
    MPU6050_Status_t status = mpu6050_write_reg(
            mpu,
            MPU6050_REG_SMPRT_DIV,
            sample_rate_div);

    if (status == MPU6050_OK) {
        mpu->config.sample_rate_value = sample_rate_div;
    }

    return status;
}

MPU6050_Status_t MPU6050_Init(
        MPU6050_Handle_t *mpu,
        I2C_HandleTypeDef *hi2c,
        const MPU6050_Config_t *config)
{
    if (mpu == NULL || hi2c == NULL || config == NULL) {
        return MPU6050_ERR_NULL;
    }

    mpu->initialized = 0U;

    MPU6050_Status_t status = mpu6050_validate_config(config);
    if (status != MPU6050_OK) return status;

    mpu->hi2c = hi2c;
    mpu->config = *config;

    mpu->gyro_scale = 0.0f;
    mpu->accel_scale = 0.0f;
    mpu->temp_scale = 0.0f;
    mpu->temp_offset = 0.0f;

    mpu->read_it_state = MPU6050_READ_IT_IDLE;
    mpu->read_it_result = MPU6050_OK;

    status = mpu6050_check_device_id(mpu);
    if (status != MPU6050_OK) return status;

    status = mpu6050_wake_up_chip(mpu);
    if (status != MPU6050_OK) return status;

    status = mpu6050_set_clock_source(
            mpu,
            config->clksrc);
    if (status != MPU6050_OK) return status;

    status = mpu6050_set_dlpf(
            mpu,
            config->dlpf_config);
    if (status != MPU6050_OK) return status;

    status = mpu6050_set_gyro_config(
            mpu,
            config->fs_sel_config);
    if (status != MPU6050_OK) return status;

    status = mpu6050_set_accel_config(
            mpu,
            config->accel_sel_config);
    if (status != MPU6050_OK) return status;

    status = mpu6050_set_sample_rate(
            mpu,
            config->sample_rate_value);
    if (status != MPU6050_OK) return status;

    mpu->temp_scale = MPU6050_TEMP_SCALE;
    mpu->temp_offset = MPU6050_TEMP_OFFSET;
    mpu->accel_offset_g = Vector3f_Zero();
    mpu->gyro_offset_dps = Vector3f_Zero();

    mpu->initialized = 1U;
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_CheckDeviceID(
        MPU6050_Handle_t *mpu)
{
    if (mpu == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    return mpu6050_check_device_id(mpu);
}

MPU6050_Status_t MPU6050_WakeUpChip(
        MPU6050_Handle_t *mpu)
{
    if (mpu == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    return mpu6050_wake_up_chip(mpu);
}

MPU6050_Status_t MPU6050_SetClockSource(
        MPU6050_Handle_t *mpu,
        uint8_t clksrc)
{
    if (mpu == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    return mpu6050_set_clock_source(mpu, clksrc);
}

MPU6050_Status_t MPU6050_SetDLPF(
        MPU6050_Handle_t *mpu,
        uint8_t dlpf_config)
{
    if (mpu == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    return mpu6050_set_dlpf(mpu, dlpf_config);
}

MPU6050_Status_t MPU6050_SetGyroConfig(
        MPU6050_Handle_t *mpu,
        uint8_t fs_sel_config)
{
    if (mpu == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    return mpu6050_set_gyro_config(mpu, fs_sel_config);
}

MPU6050_Status_t MPU6050_SetAccelConfig(
        MPU6050_Handle_t *mpu,
        uint8_t accel_sel_config)
{
    if (mpu == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    return mpu6050_set_accel_config(
            mpu,
            accel_sel_config);
}

MPU6050_Status_t MPU6050_SetSampleRate(
        MPU6050_Handle_t *mpu,
        uint8_t sample_rate_div)
{
    if (mpu == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    return mpu6050_set_sample_rate(
            mpu,
            sample_rate_div);
}

MPU6050_Status_t MPU6050_ReadRawData(
        MPU6050_Handle_t *mpu, MPU6050_RawData_t *raw)
{
    if (mpu == NULL || raw == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    uint8_t result[MPU6050_RAW_DATA_LENGTH];
    MPU6050_Status_t status = mpu6050_read_regs(
            mpu, MPU6050_REG_ACCEL_XOUTH, (uint8_t)sizeof(result), result);
    if (status != MPU6050_OK) return status;

    mpu6050_parse_vector3_be(&result[0], &raw->accel);
    raw->temp = byte_utils_i16_from_be(result[6], result[7]);
    mpu6050_parse_vector3_be(&result[8], &raw->gyro);
    return MPU6050_OK;
}


MPU6050_Status_t MPU6050_StartReadRawDataIT(MPU6050_Handle_t *mpu)
{
    if (mpu == NULL || mpu->hi2c == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    if (mpu->read_it_state != MPU6050_READ_IT_IDLE) {
        return MPU6050_I2C_BUSY;
    }

    mpu->read_it_state = MPU6050_READ_IT_BUSY;
    mpu->read_it_result = MPU6050_OK;

    MPU6050_Status_t status = mpu6050_from_hal_status(
            HAL_I2C_Mem_Read_IT(
                    mpu->hi2c,
                    (uint16_t)(mpu->config.address << 1U),
                    MPU6050_REG_ACCEL_XOUTH,
                    I2C_MEMADD_SIZE_8BIT,
                    mpu->read_it_buffer,
                    MPU6050_RAW_DATA_LENGTH));

    if (status != MPU6050_OK) {
        mpu->read_it_result = status;
        mpu->read_it_state = MPU6050_READ_IT_IDLE;
    }

    return status;
}

MPU6050_Status_t MPU6050_AbortReadIT(
        MPU6050_Handle_t *mpu)
{
    if (mpu == NULL || mpu->hi2c == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    if (mpu->read_it_state == MPU6050_READ_IT_ABORTING) {
        return MPU6050_I2C_BUSY;
    }

    if (mpu->read_it_state != MPU6050_READ_IT_BUSY) {
        return MPU6050_INVALID_STATE;
    }

    mpu->read_it_state = MPU6050_READ_IT_ABORTING;
    mpu->read_it_result = MPU6050_I2C_TIMEOUT;

    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Abort_IT(
            mpu->hi2c,
            (uint16_t)(mpu->config.address << 1U));

    if (hal_status != HAL_OK) {
        mpu->read_it_state = MPU6050_READ_IT_BUSY;
        mpu->read_it_result = MPU6050_OK;
        return mpu6050_from_hal_status(hal_status);
    }

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_OnI2CMemRxComplete(
        MPU6050_Handle_t *mpu,
        I2C_HandleTypeDef *hi2c)
{
    if (mpu == NULL || hi2c == NULL || mpu->hi2c == NULL) {
        return MPU6050_ERR_NULL;
    }
    if (mpu->initialized == 0U) {
        return MPU6050_ERR_UNINITIALIZED;
    }

    if (hi2c != mpu->hi2c ||
        (mpu->read_it_state != MPU6050_READ_IT_BUSY &&
         mpu->read_it_state != MPU6050_READ_IT_ABORTING))
    {
        return MPU6050_INVALID_STATE;
    }

    mpu->read_it_result = MPU6050_OK;
    mpu->read_it_state = MPU6050_READ_IT_COMPLETE;
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_OnI2CError(
        MPU6050_Handle_t *mpu,
        I2C_HandleTypeDef *hi2c)
{
    if (mpu == NULL || hi2c == NULL || mpu->hi2c == NULL) {
        return MPU6050_ERR_NULL;
    }
    if (mpu->initialized == 0U) {
        return MPU6050_ERR_UNINITIALIZED;
    }

    if (hi2c != mpu->hi2c ||
        (mpu->read_it_state != MPU6050_READ_IT_BUSY &&
         mpu->read_it_state != MPU6050_READ_IT_ABORTING))
    {
        return MPU6050_INVALID_STATE;
    }

    if (mpu->read_it_state == MPU6050_READ_IT_BUSY) {
        mpu->read_it_result = MPU6050_I2C_ERROR;
    }

    mpu->read_it_state = MPU6050_READ_IT_ERROR;
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_OnI2CAbortComplete(
        MPU6050_Handle_t *mpu,
        I2C_HandleTypeDef *hi2c)
{
    if (mpu == NULL || hi2c == NULL || mpu->hi2c == NULL) {
        return MPU6050_ERR_NULL;
    }
    if (mpu->initialized == 0U) {
        return MPU6050_ERR_UNINITIALIZED;
    }

    if (hi2c != mpu->hi2c ||
        mpu->read_it_state != MPU6050_READ_IT_ABORTING)
    {
        return MPU6050_INVALID_STATE;
    }

    mpu->read_it_result = MPU6050_I2C_TIMEOUT;
    mpu->read_it_state = MPU6050_READ_IT_ERROR;
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_GetRawDataIT(
        MPU6050_Handle_t *mpu,
        MPU6050_RawData_t *raw)
{
    if (mpu == NULL || raw == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    if (mpu->read_it_state == MPU6050_READ_IT_BUSY ||
        mpu->read_it_state == MPU6050_READ_IT_ABORTING)
    {
        return MPU6050_I2C_BUSY;
    }

    if (mpu->read_it_state == MPU6050_READ_IT_IDLE) {
        return MPU6050_NOT_READY_TO_READ;
    }

    if (mpu->read_it_state == MPU6050_READ_IT_ERROR) {
        MPU6050_Status_t status = mpu->read_it_result;
        mpu->read_it_state = MPU6050_READ_IT_IDLE;
        mpu->read_it_result = MPU6050_OK;
        return status;
    }

    if (mpu->read_it_state != MPU6050_READ_IT_COMPLETE) {
        return MPU6050_INVALID_STATE;
    }

    mpu6050_parse_vector3_be(&mpu->read_it_buffer[0], &raw->accel);
    raw->temp = byte_utils_i16_from_be(
            mpu->read_it_buffer[6],
            mpu->read_it_buffer[7]);
    mpu6050_parse_vector3_be(&mpu->read_it_buffer[8], &raw->gyro);

    mpu->read_it_state = MPU6050_READ_IT_IDLE;
    mpu->read_it_result = MPU6050_OK;
    return MPU6050_OK;
}

MPU6050_ReadITState_t MPU6050_GetReadStateIT(
        const MPU6050_Handle_t *mpu)
{
    if (mpu == NULL || mpu->initialized == 0U) {
        return MPU6050_READ_IT_ERROR;
    }

    return mpu->read_it_state;
}

MPU6050_Status_t MPU6050_ReadRawAccel(
        MPU6050_Handle_t *mpu, MPU6050_RawData_t *raw)
{
    if (mpu == NULL || raw == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    uint8_t result[6];
    MPU6050_Status_t status = mpu6050_read_regs(
            mpu, MPU6050_REG_ACCEL_XOUTH, (uint8_t)sizeof(result), result);
    if (status != MPU6050_OK) return status;

    mpu6050_parse_vector3_be(result, &raw->accel);
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadRawGyro(
        MPU6050_Handle_t *mpu, MPU6050_RawData_t *raw)
{
    if (mpu == NULL || raw == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    uint8_t result[6];
    MPU6050_Status_t status = mpu6050_read_regs(
            mpu, MPU6050_REG_GYRO_XOUTH, (uint8_t)sizeof(result), result);
    if (status != MPU6050_OK) return status;

    mpu6050_parse_vector3_be(result, &raw->gyro);
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadRawTemp(
        MPU6050_Handle_t *mpu, MPU6050_RawData_t *raw)
{
    if (mpu == NULL || raw == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    uint8_t result[2];
    MPU6050_Status_t status = mpu6050_read_regs(
            mpu, MPU6050_REG_TEMP_OUTH, (uint8_t)sizeof(result), result);
    if (status != MPU6050_OK) return status;

    raw->temp = byte_utils_i16_from_be(result[0], result[1]);
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadScaledData(
        MPU6050_Handle_t *mpu,
        MPU6050_Data_t *scaled)
{
    if (mpu == NULL || scaled == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    MPU6050_RawData_t raw;
    MPU6050_Status_t status = MPU6050_ReadRawData(mpu, &raw);
    if (status != MPU6050_OK) return status;

    return MPU6050_ConvertRawToPhysical(mpu, &raw, scaled);
}

MPU6050_Status_t MPU6050_ConvertRawToPhysical(
        const MPU6050_Handle_t *mpu,
        const MPU6050_RawData_t *raw,
        MPU6050_Data_t *physical)
{
    if (mpu == NULL || raw == NULL || physical == NULL) {
        return MPU6050_ERR_NULL;
    }
    if (mpu->initialized == 0U) {
        return MPU6050_ERR_UNINITIALIZED;
    }

    if (mpu->accel_scale <= 0.0f
            || mpu->gyro_scale <= 0.0f
            || mpu->temp_scale <= 0.0f) {
        return MPU6050_INVALID_CONFIG;
    }

    Vector3f_t accel_raw = Vector3f_FromInt16(raw->accel);
    Vector3f_t gyro_raw = Vector3f_FromInt16(raw->gyro);

    physical->accel_g = Vector3f_DivideScalar(
            accel_raw,
            mpu->accel_scale);

    physical->temp_c =
            (float)raw->temp / mpu->temp_scale
            + mpu->temp_offset;

    physical->gyro_dps = Vector3f_DivideScalar(
            gyro_raw,
            mpu->gyro_scale);

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ApplyCalibration(
        const MPU6050_Handle_t *mpu,
        const MPU6050_Data_t *physical,
        MPU6050_Data_t *calibrated)
{
    if (mpu == NULL || physical == NULL || calibrated == NULL) {
        return MPU6050_ERR_NULL;
    }
    if (mpu->initialized == 0U) {
        return MPU6050_ERR_UNINITIALIZED;
    }

    calibrated->accel_g = Vector3f_Subtract(
            physical->accel_g,
            mpu->accel_offset_g);

    calibrated->temp_c = physical->temp_c;

    calibrated->gyro_dps = Vector3f_Subtract(
            physical->gyro_dps,
            mpu->gyro_offset_dps);

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ResetCalibration(MPU6050_Handle_t *mpu)
{
    if (mpu == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    mpu->accel_offset_g = Vector3f_Zero();
    mpu->gyro_offset_dps = Vector3f_Zero();
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_SetCalibration(
        MPU6050_Handle_t *mpu,
        const MPU6050_Calibration_t *calibration)
{
    if (mpu == NULL || calibration == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    mpu->accel_offset_g = calibration->accel_offset_g;
    mpu->gyro_offset_dps = calibration->gyro_offset_dps;
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_GetCalibration(
        const MPU6050_Handle_t *mpu,
        MPU6050_Calibration_t *calibration)
{
    if (mpu == NULL || calibration == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    calibration->accel_offset_g = mpu->accel_offset_g;
    calibration->gyro_offset_dps = mpu->gyro_offset_dps;
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_SetStillnessConfig(
        const MPU6050_Handle_t *mpu,
        MPU6050_StillnessConfig_t *still)
{
    if (mpu == NULL || still == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;
    if (mpu->gyro_scale <= 0.0f || mpu->accel_scale <= 0.0f) {
        return MPU6050_INVALID_CONFIG;
    }

    still->sample_count = MPU6050_STILLNESS_SMPL_CNT200;
    still->sample_delay_ms = MPU6050_STILLNESS_SAMPLE_DELAY_MS;
    still->timeout_ms = MPU6050_STILLNESS_TIMEOUT5000;
    still->gyro_threshold = mpu->gyro_scale
            * MPU6050_GYRO_STILLNESS_THRESHOLD_DPS;
    still->accel_threshold = mpu->accel_scale
            * MPU6050_ACCEL_STILLNESS_THRES_COEFFICIENT;
    still->accel_axis_threshold = mpu->accel_scale
            * MPU6050_ACCEL_STILLNESS_AXIS_COEFFICIENT;
    still->accel_allowed_gap = mpu->accel_scale
            * MPU6050_ACCEL_STILLNESS_ALLOWED_GAP_COEFFICIENT;
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_CalibrateGyroOffset(
        MPU6050_Handle_t *mpu,
        MPU6050_RawData_t *raw,
        const MPU6050_StillnessConfig_t *still)
{
    if (mpu == NULL || raw == NULL || still == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;
    if (still->sample_count == 0U || mpu->gyro_scale <= 0.0f) {
        return MPU6050_INVALID_CONFIG;
    }

    MPU6050_CalibWindow_t window;
    uint32_t start_tick = HAL_GetTick();

    do {
        MPU6050_Status_t status = mpu6050_collect_still_window(
                mpu, raw, still, &window);

        if (status == MPU6050_OK) {
            Vector3f_t avg_gyro_raw = Vector3f_DivideScalar(
                    window.gyro_sum_raw, (float)still->sample_count);
            mpu->gyro_offset_dps = Vector3f_DivideScalar(
                    avg_gyro_raw, mpu->gyro_scale);
            return MPU6050_OK;
        }
        if (status != MPU6050_ERR_MOVING) return status;
        if (still->timeout_ms == 0U) return status;
    } while ((HAL_GetTick() - start_tick) < still->timeout_ms);

    return MPU6050_ERR_TIMEOUT;
}

MPU6050_Status_t MPU6050_CalibrateAccelOffset(
        MPU6050_Handle_t *mpu,
        MPU6050_RawData_t *raw,
        const MPU6050_StillnessConfig_t *still,
        const Vector3f_t *accel_ref_g)
{
    if (mpu == NULL || raw == NULL || still == NULL || accel_ref_g == NULL) {
        return MPU6050_ERR_NULL;
    }
    if (mpu->initialized == 0U) {
        return MPU6050_ERR_UNINITIALIZED;
    }
    if (still->sample_count == 0U || mpu->accel_scale <= 0.0f) {
        return MPU6050_INVALID_CONFIG;
    }

    MPU6050_CalibWindow_t window;
    uint32_t start_tick = HAL_GetTick();

    do {
        MPU6050_Status_t status = mpu6050_collect_still_window(
                mpu, raw, still, &window);

        if (status == MPU6050_OK) {
            Vector3f_t avg_accel_raw = Vector3f_DivideScalar(
                    window.accel_sum_raw, (float)still->sample_count);
            Vector3f_t avg_accel_g = Vector3f_DivideScalar(
                    avg_accel_raw, mpu->accel_scale);

            mpu->accel_offset_g = Vector3f_Subtract(
                    avg_accel_g,
                    *accel_ref_g);
            return MPU6050_OK;
        }
        if (status != MPU6050_ERR_MOVING) return status;
        if (still->timeout_ms == 0U) return status;
    } while ((HAL_GetTick() - start_tick) < still->timeout_ms);

    return MPU6050_ERR_TIMEOUT;
}

void MPU6050_NotiStatus(
        MPU6050_Status_t status,
        const char *message)
{
    if (message != NULL && message[0] != '\0') {
        printf("%s\r\n", message);
    }

    switch (status) {
    case MPU6050_OK:
        return;

    case MPU6050_INVALID_STATE:
        printf("MPU6050 is in invalid state, status=%d\r\n", status);
        break;

    case MPU6050_I2C_ERROR:
        printf("There is a problem with I2C communication, status=%d\r\n",
               status);
        break;

    case MPU6050_I2C_BUSY:
        printf("I2C is busy, status=%d\r\n", status);
        break;

    case MPU6050_I2C_TIMEOUT:
        printf("I2C communication timed out, status=%d\r\n", status);
        break;

    case MPU6050_ERR_TIMEOUT:
        printf("MPU6050 operation timed out, status=%d\r\n", status);
        break;

    case MPU6050_ERR_NULL:
        printf("A null pointer was passed to the MPU6050 driver, status=%d\r\n",
               status);
        break;

    case MPU6050_ERR_UNINITIALIZED:
        printf("MPU6050 has not been initialized, status=%d\r\n",
               status);
        break;

    case MPU6050_ERR_BAD_DEVICE_ID:
        printf("The MPU6050 device ID is incorrect, status=%d\r\n",
               status);
        break;

    case MPU6050_ERR_MOVING:
        printf("The MPU6050 is moving during calibration, status=%d\r\n",
               status);
        break;

    case MPU6050_NOT_READY_TO_READ:
        printf("MPU6050 interrupt data is not ready, status=%d\r\n",
               status);
        break;

    case MPU6050_INVALID_CONFIG:
        printf("The MPU6050 configuration is invalid, status=%d\r\n",
               status);
        break;

    default:
        printf("Unrecognized MPU6050 status=%d\r\n", status);
        break;
    }
}
MPU6050_Status_t MPU6050_EnableBypass(MPU6050_Handle_t *mpu)
{
    if (mpu == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;

    uint8_t value = 0U;
    MPU6050_Status_t status = mpu6050_read_reg(
            mpu, MPU6050_REG_USER_CTRL, &value);
    if (status != MPU6050_OK) return status;

    value &= (uint8_t)~(1U << 5U);
    status = mpu6050_write_reg(mpu, MPU6050_REG_USER_CTRL, value);
    if (status != MPU6050_OK) return status;

    status = mpu6050_read_reg(mpu, MPU6050_REG_INT_PIN_CFG, &value);
    if (status != MPU6050_OK) return status;

    value |= (uint8_t)(1U << MPU6050_INT_PIN_CFG_I2C_BYPASS_EN);
    return mpu6050_write_reg(mpu, MPU6050_REG_INT_PIN_CFG, value);
}


static uint8_t mpu6050_check_interrupt_config(
        const MPU6050_InterruptConfig_t *config)
{
    if (config == NULL) return 0U;

    return (uint8_t)(
            config->int_level <= 1U
         && config->int_open <= 1U
         && config->latch_int_en <= 1U
         && config->int_rd_clear <= 1U);
}

MPU6050_Status_t MPU6050_SetDataReadyInterrupt(
        MPU6050_Handle_t *mpu,
        uint8_t enable)
{
    if (mpu == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;
    if (enable > 1U) return MPU6050_INVALID_CONFIG;

    uint8_t value = 0U;
    MPU6050_Status_t status = mpu6050_read_reg(
            mpu,
            MPU6050_REG_INT_ENABLE,
            &value);
    if (status != MPU6050_OK) return status;

    if (enable != 0U) {
        value |= (uint8_t)(1U << MPU6050_INT_ENABLE_DATA_RDY_EN);
    } else {
        value &= (uint8_t)~(1U << MPU6050_INT_ENABLE_DATA_RDY_EN);
    }

    return mpu6050_write_reg(
            mpu,
            MPU6050_REG_INT_ENABLE,
            value);
}

MPU6050_Status_t MPU6050_ConfigureInterrupt(
        MPU6050_Handle_t *mpu,
        const MPU6050_InterruptConfig_t *config)
{
    if (mpu == NULL || config == NULL) return MPU6050_ERR_NULL;
    if (mpu->initialized == 0U) return MPU6050_ERR_UNINITIALIZED;
    if (mpu6050_check_interrupt_config(config) == 0U) {
        return MPU6050_INVALID_CONFIG;
    }

    uint8_t value = 0U;
    MPU6050_Status_t status = mpu6050_read_reg(
            mpu,
            MPU6050_REG_INT_PIN_CFG,
            &value);
    if (status != MPU6050_OK) return status;

    const uint8_t managed_mask =
            (uint8_t)((1U << MPU6050_INT_PIN_CFG_INT_LEVEL)
                    | (1U << MPU6050_INT_PIN_CFG_INT_OPEN)
                    | (1U << MPU6050_INT_PIN_CFG_LATCH_INT_EN)
                    | (1U << MPU6050_INT_PIN_CFG_INT_RD_CLEAR));

    value &= (uint8_t)~managed_mask;

    value |= (uint8_t)(
            (config->int_level
                    << MPU6050_INT_PIN_CFG_INT_LEVEL)
          | (config->int_open
                    << MPU6050_INT_PIN_CFG_INT_OPEN)
          | (config->latch_int_en
                    << MPU6050_INT_PIN_CFG_LATCH_INT_EN)
          | (config->int_rd_clear
                    << MPU6050_INT_PIN_CFG_INT_RD_CLEAR));

    return mpu6050_write_reg(
            mpu,
            MPU6050_REG_INT_PIN_CFG,
            value);
}
