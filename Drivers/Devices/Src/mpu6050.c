#include "mpu6050.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#include "byte_utils.h"

#define MPU6050_REG_SMPRT_DIV 0x19U
#define MPU6050_REG_CONFIG 0x1AU
#define MPU6050_REG_GYRO_CONFIG 0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU

#define MPU6050_REG_INT_PIN_CFG 0x37U
#define MPU6050_REG_INT_ENABLE 0x38U

#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_TEMP_OUT_H 0x41U
#define MPU6050_REG_GYRO_XOUT_H 0x43U

#define MPU6050_REG_USER_CTRL 0x6AU
#define MPU6050_REG_PWR_MGMT_1 0x6BU

#define MPU6050_REG_WHO_AM_I 0x75U

#define MPU6050_WHO_AM_I_VALUE 0x68U

#define MPU6050_PWR1_CLKSEL_MASK 0x07U

#define MPU6050_CONFIG_DLPF_CFG_MASK 0x07U

#define MPU6050_CONFIG_FS_SEL_MASK 0x03U
#define MPU6050_CONFIG_FS_SEL_POS 3U

#define MPU6050_CONFIG_AFS_SEL_MASK 0x03U
#define MPU6050_CONFIG_AFS_SEL_POS 3U

#define MPU6050_INT_DATA_RDY_BIT 0U

#define MPU6050_INT_BYPASS_BIT 1U
#define MPU6050_INT_RD_CLEAR_BIT 4U
#define MPU6050_INT_LATCH_BIT 5U
#define MPU6050_INT_OPEN_BIT 6U
#define MPU6050_INT_LEVEL_BIT 7U

#define MPU6050_USER_I2C_MST_EN_BIT 5U

#define MPU6050_GYRO_SENS_250DPS 131.0f
#define MPU6050_GYRO_SENS_500DPS 65.5f
#define MPU6050_GYRO_SENS_1000DPS 32.8f
#define MPU6050_GYRO_SENS_2000DPS 16.4f

#define MPU6050_ACCEL_SENS_2G 16384.0f
#define MPU6050_ACCEL_SENS_4G 8192.0f
#define MPU6050_ACCEL_SENS_8G 4096.0f
#define MPU6050_ACCEL_SENS_16G 2048.0f

#define MPU6050_TEMP_SCALE 340.0f
#define MPU6050_TEMP_OFFSET 36.53f

#define MPU6050_CALIB_SAMPLE_COUNT 200U
#define MPU6050_CALIB_SAMPLE_DELAY_MS 2U
#define MPU6050_CALIB_TIMEOUT_MS 5000U

#define MPU6050_GYRO_STILLNESS_DPS 1.0f
#define MPU6050_ACCEL_AXIS_STILLNESS_G 0.05f
#define MPU6050_ACCEL_MAGNITUDE_G 1.0f
#define MPU6050_ACCEL_MAGNITUDE_GAP 0.1f

typedef struct {
  float gyro_x;
  float gyro_y;
  float gyro_z;

  float accel_x;
  float accel_y;
  float accel_z;
} MPU6050_CalibrationAverage_t;

static MPU6050_Status_t mpu6050_convert_io_status(DeviceIO_Status_t io_status) {
  switch (io_status) {
  case DEVICE_IO_OK:
    return MPU6050_OK;

  case DEVICE_IO_BUSY:
    return MPU6050_I2C_BUSY;

  case DEVICE_IO_TIMEOUT:
    return MPU6050_I2C_TIMEOUT;

  case DEVICE_IO_INVALID_ARGUMENT:
    return MPU6050_INVALID_CONFIG;

  case DEVICE_IO_ERROR:
  default:
    return MPU6050_I2C_ERROR;
  }
}

static uint8_t mpu6050_io_is_valid(const DeviceIO_t *io) {
  if (io == NULL || io->context == NULL || io->ops == NULL) {
    return 0U;
  }

  if (io->ops->write_registers == NULL || io->ops->read_registers == NULL ||
      io->ops->read_registers_it == NULL || io->ops->delay_ms == NULL ||
      io->ops->get_tick_ms == NULL) {
    return 0U;
  }

  return 1U;
}

static MPU6050_Status_t mpu6050_check_initialized(const MPU6050_Handle_t *mpu) {
  if (mpu == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (mpu->initialized == 0U) {
    return MPU6050_ERR_UNINITIALIZED;
  }

  return MPU6050_OK;
}

static MPU6050_Status_t
mpu6050_validate_config(const MPU6050_Config_t *config) {
  if (config == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (config->address > 0x7FU) {
    return MPU6050_INVALID_CONFIG;
  }

  if (config->clksrc == 6U || config->clksrc > MPU6050_CLKSRC_STOPCLK) {
    return MPU6050_INVALID_CONFIG;
  }

  if (config->dlpf_config > MPU6050_DLPF_CFG_6) {
    return MPU6050_INVALID_CONFIG;
  }

  if (config->fs_sel_config > MPU6050_GYRO_CONFIG_FS_2000DPS) {
    return MPU6050_INVALID_CONFIG;
  }

  if (config->accel_sel_config > MPU6050_ACCEL_CONFIG_AFS_16G) {
    return MPU6050_INVALID_CONFIG;
  }

  return MPU6050_OK;
}

static MPU6050_Status_t mpu6050_write_register(MPU6050_Handle_t *mpu,
                                               uint8_t register_address,
                                               uint8_t value) {
  DeviceIO_Status_t io_status = mpu->io->ops->write_registers(
      mpu->io->context, mpu->config.address, register_address, &value, 1U,
      MPU6050_I2C_TIMEOUT_MS);

  return mpu6050_convert_io_status(io_status);
}

static MPU6050_Status_t mpu6050_read_registers(MPU6050_Handle_t *mpu,
                                               uint8_t start_register,
                                               uint8_t *data, uint16_t length) {
  DeviceIO_Status_t io_status = mpu->io->ops->read_registers(
      mpu->io->context, mpu->config.address, start_register, data, length,
      MPU6050_I2C_TIMEOUT_MS);

  return mpu6050_convert_io_status(io_status);
}

static MPU6050_Status_t mpu6050_read_register(MPU6050_Handle_t *mpu,
                                              uint8_t register_address,
                                              uint8_t *value) {
  return mpu6050_read_registers(mpu, register_address, value, 1U);
}

static MPU6050_Status_t mpu6050_update_bits(MPU6050_Handle_t *mpu,
                                            uint8_t register_address,
                                            uint8_t mask, uint8_t value) {
  uint8_t current = 0U;

  MPU6050_Status_t status =
      mpu6050_read_register(mpu, register_address, &current);

  if (status != MPU6050_OK) {
    return status;
  }

  current = (uint8_t)((current & (uint8_t)~mask) | (value & mask));

  return mpu6050_write_register(mpu, register_address, current);
}

static void mpu6050_parse_vector(const uint8_t *buffer, Vector3i16_t *vector) {
  vector->x = byte_utils_i16_from_be(buffer[0], buffer[1]);

  vector->y = byte_utils_i16_from_be(buffer[2], buffer[3]);

  vector->z = byte_utils_i16_from_be(buffer[4], buffer[5]);
}

static void mpu6050_parse_raw_buffer(const uint8_t *buffer,
                                     MPU6050_RawData_t *raw) {
  mpu6050_parse_vector(&buffer[0], &raw->accel);

  raw->temp = byte_utils_i16_from_be(buffer[6], buffer[7]);

  mpu6050_parse_vector(&buffer[8], &raw->gyro);
}

static MPU6050_Status_t mpu6050_set_clock_source(MPU6050_Handle_t *mpu,
                                                 uint8_t clock_source) {
  MPU6050_Status_t status = mpu6050_update_bits(
      mpu, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_MASK, clock_source);

  if (status == MPU6050_OK) {
    mpu->config.clksrc = clock_source;
  }

  return status;
}

static MPU6050_Status_t mpu6050_set_dlpf(MPU6050_Handle_t *mpu,
                                         uint8_t dlpf_config) {
  MPU6050_Status_t status = mpu6050_update_bits(
      mpu, MPU6050_REG_CONFIG, MPU6050_CONFIG_DLPF_CFG_MASK, dlpf_config);

  if (status == MPU6050_OK) {
    mpu->config.dlpf_config = dlpf_config;
  }

  return status;
}

static MPU6050_Status_t mpu6050_set_gyro_range(MPU6050_Handle_t *mpu,
                                               uint8_t fs_sel) {
  MPU6050_Status_t status = mpu6050_update_bits(
      mpu, MPU6050_REG_GYRO_CONFIG,
      (uint8_t)(MPU6050_CONFIG_FS_SEL_MASK << MPU6050_CONFIG_FS_SEL_POS),
      (uint8_t)(fs_sel << MPU6050_CONFIG_FS_SEL_POS));

  if (status != MPU6050_OK) {
    return status;
  }

  switch (fs_sel) {
  case MPU6050_GYRO_CONFIG_FS_250DPS:
    mpu->gyro_scale = MPU6050_GYRO_SENS_250DPS;
    break;

  case MPU6050_GYRO_CONFIG_FS_500DPS:
    mpu->gyro_scale = MPU6050_GYRO_SENS_500DPS;
    break;

  case MPU6050_GYRO_CONFIG_FS_1000DPS:
    mpu->gyro_scale = MPU6050_GYRO_SENS_1000DPS;
    break;

  case MPU6050_GYRO_CONFIG_FS_2000DPS:
    mpu->gyro_scale = MPU6050_GYRO_SENS_2000DPS;
    break;

  default:
    return MPU6050_INVALID_CONFIG;
  }

  mpu->config.fs_sel_config = fs_sel;

  return MPU6050_OK;
}

static MPU6050_Status_t mpu6050_set_accel_range(MPU6050_Handle_t *mpu,
                                                uint8_t afs_sel) {
  MPU6050_Status_t status = mpu6050_update_bits(
      mpu, MPU6050_REG_ACCEL_CONFIG,
      (uint8_t)(MPU6050_CONFIG_AFS_SEL_MASK << MPU6050_CONFIG_AFS_SEL_POS),
      (uint8_t)(afs_sel << MPU6050_CONFIG_AFS_SEL_POS));

  if (status != MPU6050_OK) {
    return status;
  }

  switch (afs_sel) {
  case MPU6050_ACCEL_CONFIG_AFS_2G:
    mpu->accel_scale = MPU6050_ACCEL_SENS_2G;
    break;

  case MPU6050_ACCEL_CONFIG_AFS_4G:
    mpu->accel_scale = MPU6050_ACCEL_SENS_4G;
    break;

  case MPU6050_ACCEL_CONFIG_AFS_8G:
    mpu->accel_scale = MPU6050_ACCEL_SENS_8G;
    break;

  case MPU6050_ACCEL_CONFIG_AFS_16G:
    mpu->accel_scale = MPU6050_ACCEL_SENS_16G;
    break;

  default:
    return MPU6050_INVALID_CONFIG;
  }

  mpu->config.accel_sel_config = afs_sel;

  return MPU6050_OK;
}

static MPU6050_Status_t
mpu6050_collect_still_samples(MPU6050_Handle_t *mpu, MPU6050_RawData_t *raw,
                              const MPU6050_StillnessConfig_t *config,
                              MPU6050_CalibrationAverage_t *average) {
  if (mpu == NULL || raw == NULL || config == NULL || average == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (config->sample_count == 0U) {
    return MPU6050_INVALID_CONFIG;
  }

  Vector3i16_t gyro_min = {.x = INT16_MAX, .y = INT16_MAX, .z = INT16_MAX};

  Vector3i16_t gyro_max = {.x = INT16_MIN, .y = INT16_MIN, .z = INT16_MIN};

  Vector3i16_t accel_min = {.x = INT16_MAX, .y = INT16_MAX, .z = INT16_MAX};

  Vector3i16_t accel_max = {.x = INT16_MIN, .y = INT16_MIN, .z = INT16_MIN};

  float gyro_sum_x = 0.0f;
  float gyro_sum_y = 0.0f;
  float gyro_sum_z = 0.0f;

  float accel_sum_x = 0.0f;
  float accel_sum_y = 0.0f;
  float accel_sum_z = 0.0f;

  for (uint32_t i = 0U; i < config->sample_count; ++i) {

    MPU6050_Status_t status = MPU6050_ReadRawData(mpu, raw);

    if (status != MPU6050_OK) {
      return status;
    }

    if (raw->gyro.x < gyro_min.x) {
      gyro_min.x = raw->gyro.x;
    }

    if (raw->gyro.y < gyro_min.y) {
      gyro_min.y = raw->gyro.y;
    }

    if (raw->gyro.z < gyro_min.z) {
      gyro_min.z = raw->gyro.z;
    }

    if (raw->gyro.x > gyro_max.x) {
      gyro_max.x = raw->gyro.x;
    }

    if (raw->gyro.y > gyro_max.y) {
      gyro_max.y = raw->gyro.y;
    }

    if (raw->gyro.z > gyro_max.z) {
      gyro_max.z = raw->gyro.z;
    }

    if (raw->accel.x < accel_min.x) {
      accel_min.x = raw->accel.x;
    }

    if (raw->accel.y < accel_min.y) {
      accel_min.y = raw->accel.y;
    }

    if (raw->accel.z < accel_min.z) {
      accel_min.z = raw->accel.z;
    }

    if (raw->accel.x > accel_max.x) {
      accel_max.x = raw->accel.x;
    }

    if (raw->accel.y > accel_max.y) {
      accel_max.y = raw->accel.y;
    }

    if (raw->accel.z > accel_max.z) {
      accel_max.z = raw->accel.z;
    }

    gyro_sum_x += (float)raw->gyro.x;
    gyro_sum_y += (float)raw->gyro.y;
    gyro_sum_z += (float)raw->gyro.z;

    accel_sum_x += (float)raw->accel.x;
    accel_sum_y += (float)raw->accel.y;
    accel_sum_z += (float)raw->accel.z;

    if (config->sample_delay_ms > 0U) {
      mpu->io->ops->delay_ms(mpu->io->context, config->sample_delay_ms);
    }
  }

  if ((float)(gyro_max.x - gyro_min.x) > config->gyro_threshold ||
      (float)(gyro_max.y - gyro_min.y) > config->gyro_threshold ||
      (float)(gyro_max.z - gyro_min.z) > config->gyro_threshold) {
    return MPU6050_ERR_MOVING;
  }

  if ((float)(accel_max.x - accel_min.x) > config->accel_axis_threshold ||
      (float)(accel_max.y - accel_min.y) > config->accel_axis_threshold ||
      (float)(accel_max.z - accel_min.z) > config->accel_axis_threshold) {
    return MPU6050_ERR_MOVING;
  }

  float count = (float)config->sample_count;

  average->gyro_x = gyro_sum_x / count;

  average->gyro_y = gyro_sum_y / count;

  average->gyro_z = gyro_sum_z / count;

  average->accel_x = accel_sum_x / count;

  average->accel_y = accel_sum_y / count;

  average->accel_z = accel_sum_z / count;

  float accel_magnitude = sqrtf(average->accel_x * average->accel_x +
                                average->accel_y * average->accel_y +
                                average->accel_z * average->accel_z);

  if (fabsf(accel_magnitude - config->accel_threshold) >
      config->accel_allowed_gap) {
    return MPU6050_ERR_MOVING;
  }

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *mpu, const DeviceIO_t *io,
                              const MPU6050_Config_t *config) {
  if (mpu == NULL || io == NULL || config == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (mpu6050_io_is_valid(io) == 0U) {
    return MPU6050_INVALID_CONFIG;
  }

  MPU6050_Status_t status = mpu6050_validate_config(config);

  if (status != MPU6050_OK) {
    return status;
  }

  mpu->initialized = 0U;

  mpu->io = io;
  mpu->config = *config;

  mpu->accel_scale = 0.0f;
  mpu->gyro_scale = 0.0f;

  mpu->temp_scale = MPU6050_TEMP_SCALE;

  mpu->temp_offset = MPU6050_TEMP_OFFSET;

  mpu->accel_offset_g = (Vector3f_t){.x = 0.0f, .y = 0.0f, .z = 0.0f};

  mpu->gyro_offset_dps = (Vector3f_t){.x = 0.0f, .y = 0.0f, .z = 0.0f};

  uint8_t who_am_i = 0U;

  status = mpu6050_read_register(mpu, MPU6050_REG_WHO_AM_I, &who_am_i);

  if (status != MPU6050_OK) {
    return status;
  }

  if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
    return MPU6050_ERR_BAD_DEVICE_ID;
  }

  status = mpu6050_write_register(mpu, MPU6050_REG_PWR_MGMT_1, 0U);

  if (status != MPU6050_OK) {
    return status;
  }

  status = mpu6050_set_clock_source(mpu, config->clksrc);

  if (status != MPU6050_OK) {
    return status;
  }

  status = mpu6050_set_dlpf(mpu, config->dlpf_config);

  if (status != MPU6050_OK) {
    return status;
  }

  status = mpu6050_set_gyro_range(mpu, config->fs_sel_config);

  if (status != MPU6050_OK) {
    return status;
  }

  status = mpu6050_set_accel_range(mpu, config->accel_sel_config);

  if (status != MPU6050_OK) {
    return status;
  }

  status = mpu6050_write_register(mpu, MPU6050_REG_SMPRT_DIV,
                                  config->sample_rate_value);

  if (status != MPU6050_OK) {
    return status;
  }

  mpu->initialized = 1U;

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadRawData(MPU6050_Handle_t *mpu,
                                     MPU6050_RawData_t *raw) {
  if (raw == NULL) {
    return MPU6050_ERR_NULL;
  }

  MPU6050_Status_t status = mpu6050_check_initialized(mpu);

  if (status != MPU6050_OK) {
    return status;
  }

  uint8_t buffer[MPU6050_RAW_DATA_LENGTH];

  status = mpu6050_read_registers(mpu, MPU6050_REG_ACCEL_XOUT_H, buffer,
                                  sizeof(buffer));

  if (status != MPU6050_OK) {
    return status;
  }

  mpu6050_parse_raw_buffer(buffer, raw);

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadRawAccel(MPU6050_Handle_t *mpu,
                                      MPU6050_RawData_t *raw) {
  if (raw == NULL) {
    return MPU6050_ERR_NULL;
  }

  MPU6050_Status_t status = mpu6050_check_initialized(mpu);

  if (status != MPU6050_OK) {
    return status;
  }

  uint8_t buffer[6];

  status = mpu6050_read_registers(mpu, MPU6050_REG_ACCEL_XOUT_H, buffer,
                                  sizeof(buffer));

  if (status == MPU6050_OK) {
    mpu6050_parse_vector(buffer, &raw->accel);
  }

  return status;
}

MPU6050_Status_t MPU6050_ReadRawGyro(MPU6050_Handle_t *mpu,
                                     MPU6050_RawData_t *raw) {
  if (raw == NULL) {
    return MPU6050_ERR_NULL;
  }

  MPU6050_Status_t status = mpu6050_check_initialized(mpu);

  if (status != MPU6050_OK) {
    return status;
  }

  uint8_t buffer[6];

  status = mpu6050_read_registers(mpu, MPU6050_REG_GYRO_XOUT_H, buffer,
                                  sizeof(buffer));

  if (status == MPU6050_OK) {
    mpu6050_parse_vector(buffer, &raw->gyro);
  }

  return status;
}

MPU6050_Status_t MPU6050_ReadRawTemp(MPU6050_Handle_t *mpu,
                                     MPU6050_RawData_t *raw) {
  if (raw == NULL) {
    return MPU6050_ERR_NULL;
  }

  MPU6050_Status_t status = mpu6050_check_initialized(mpu);

  if (status != MPU6050_OK) {
    return status;
  }

  uint8_t buffer[2];

  status = mpu6050_read_registers(mpu, MPU6050_REG_TEMP_OUT_H, buffer,
                                  sizeof(buffer));

  if (status == MPU6050_OK) {
    raw->temp = byte_utils_i16_from_be(buffer[0], buffer[1]);
  }

  return status;
}

MPU6050_Status_t MPU6050_ReadScaledData(MPU6050_Handle_t *mpu,
                                        MPU6050_Data_t *data) {
  if (data == NULL) {
    return MPU6050_ERR_NULL;
  }

  MPU6050_RawData_t raw;

  MPU6050_Status_t status = MPU6050_ReadRawData(mpu, &raw);

  if (status != MPU6050_OK) {
    return status;
  }

  return MPU6050_ConvertRawToPhysical(mpu, &raw, data);
}

MPU6050_Status_t MPU6050_StartReadRawDataIT(MPU6050_Handle_t *mpu) {
  MPU6050_Status_t status = mpu6050_check_initialized(mpu);

  if (status != MPU6050_OK) {
    return status;
  }

  DeviceIO_Status_t io_status = mpu->io->ops->read_registers_it(
      mpu->io->context, mpu->config.address, MPU6050_REG_ACCEL_XOUT_H,
      mpu->read_it_buffer, MPU6050_RAW_DATA_LENGTH);

  return mpu6050_convert_io_status(io_status);
}

MPU6050_Status_t MPU6050_GetRawDataIT(MPU6050_Handle_t *mpu,
                                      MPU6050_RawData_t *raw) {
  if (raw == NULL) {
    return MPU6050_ERR_NULL;
  }

  MPU6050_Status_t status = mpu6050_check_initialized(mpu);

  if (status != MPU6050_OK) {
    return status;
  }

  mpu6050_parse_raw_buffer(mpu->read_it_buffer, raw);

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ConvertRawToPhysical(const MPU6050_Handle_t *mpu,
                                              const MPU6050_RawData_t *raw,
                                              MPU6050_Data_t *physical) {
  if (mpu == NULL || raw == NULL || physical == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (mpu->initialized == 0U) {
    return MPU6050_ERR_UNINITIALIZED;
  }

  physical->accel_g.x = (float)raw->accel.x / mpu->accel_scale;

  physical->accel_g.y = (float)raw->accel.y / mpu->accel_scale;

  physical->accel_g.z = (float)raw->accel.z / mpu->accel_scale;

  physical->gyro_dps.x = (float)raw->gyro.x / mpu->gyro_scale;

  physical->gyro_dps.y = (float)raw->gyro.y / mpu->gyro_scale;

  physical->gyro_dps.z = (float)raw->gyro.z / mpu->gyro_scale;

  physical->temp_c = ((float)raw->temp / mpu->temp_scale) + mpu->temp_offset;

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ApplyCalibration(const MPU6050_Handle_t *mpu,
                                          const MPU6050_Data_t *physical,
                                          MPU6050_Data_t *calibrated) {
  if (mpu == NULL || physical == NULL || calibrated == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (mpu->initialized == 0U) {
    return MPU6050_ERR_UNINITIALIZED;
  }

  calibrated->accel_g.x = physical->accel_g.x - mpu->accel_offset_g.x;

  calibrated->accel_g.y = physical->accel_g.y - mpu->accel_offset_g.y;

  calibrated->accel_g.z = physical->accel_g.z - mpu->accel_offset_g.z;

  calibrated->gyro_dps.x = physical->gyro_dps.x - mpu->gyro_offset_dps.x;

  calibrated->gyro_dps.y = physical->gyro_dps.y - mpu->gyro_offset_dps.y;

  calibrated->gyro_dps.z = physical->gyro_dps.z - mpu->gyro_offset_dps.z;

  calibrated->temp_c = physical->temp_c;

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ResetCalibration(MPU6050_Handle_t *mpu) {
  MPU6050_Status_t status = mpu6050_check_initialized(mpu);

  if (status != MPU6050_OK) {
    return status;
  }

  mpu->accel_offset_g = (Vector3f_t){.x = 0.0f, .y = 0.0f, .z = 0.0f};

  mpu->gyro_offset_dps = (Vector3f_t){.x = 0.0f, .y = 0.0f, .z = 0.0f};

  return MPU6050_OK;
}

MPU6050_Status_t
MPU6050_SetCalibration(MPU6050_Handle_t *mpu,
                       const MPU6050_Calibration_t *calibration) {
  if (mpu == NULL || calibration == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (mpu->initialized == 0U) {
    return MPU6050_ERR_UNINITIALIZED;
  }

  mpu->accel_offset_g = calibration->accel_offset_g;

  mpu->gyro_offset_dps = calibration->gyro_offset_dps;

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_GetCalibration(const MPU6050_Handle_t *mpu,
                                        MPU6050_Calibration_t *calibration) {
  if (mpu == NULL || calibration == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (mpu->initialized == 0U) {
    return MPU6050_ERR_UNINITIALIZED;
  }

  calibration->accel_offset_g = mpu->accel_offset_g;

  calibration->gyro_offset_dps = mpu->gyro_offset_dps;

  return MPU6050_OK;
}

MPU6050_Status_t
MPU6050_SetStillnessConfig(const MPU6050_Handle_t *mpu,
                           MPU6050_StillnessConfig_t *stillness) {
  if (mpu == NULL || stillness == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (mpu->initialized == 0U) {
    return MPU6050_ERR_UNINITIALIZED;
  }

  stillness->sample_count = MPU6050_CALIB_SAMPLE_COUNT;

  stillness->sample_delay_ms = MPU6050_CALIB_SAMPLE_DELAY_MS;

  stillness->timeout_ms = MPU6050_CALIB_TIMEOUT_MS;

  stillness->gyro_threshold = MPU6050_GYRO_STILLNESS_DPS * mpu->gyro_scale;

  stillness->accel_axis_threshold =
      MPU6050_ACCEL_AXIS_STILLNESS_G * mpu->accel_scale;

  stillness->accel_threshold = MPU6050_ACCEL_MAGNITUDE_G * mpu->accel_scale;

  stillness->accel_allowed_gap = MPU6050_ACCEL_MAGNITUDE_GAP * mpu->accel_scale;

  return MPU6050_OK;
}

MPU6050_Status_t
MPU6050_CalibrateGyroOffset(MPU6050_Handle_t *mpu, MPU6050_RawData_t *raw,
                            const MPU6050_StillnessConfig_t *stillness) {
  if (mpu == NULL || raw == NULL || stillness == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (mpu->initialized == 0U) {
    return MPU6050_ERR_UNINITIALIZED;
  }

  MPU6050_CalibrationAverage_t average;

  uint32_t start_time = mpu->io->ops->get_tick_ms(mpu->io->context);

  for (;;) {

    MPU6050_Status_t status =
        mpu6050_collect_still_samples(mpu, raw, stillness, &average);

    if (status == MPU6050_OK) {

      mpu->gyro_offset_dps.x = average.gyro_x / mpu->gyro_scale;

      mpu->gyro_offset_dps.y = average.gyro_y / mpu->gyro_scale;

      mpu->gyro_offset_dps.z = average.gyro_z / mpu->gyro_scale;

      return MPU6050_OK;
    }

    if (status != MPU6050_ERR_MOVING) {
      return status;
    }

    uint32_t now = mpu->io->ops->get_tick_ms(mpu->io->context);

    if ((now - start_time) >= stillness->timeout_ms) {
      return MPU6050_ERR_TIMEOUT;
    }
  }
}

MPU6050_Status_t
MPU6050_CalibrateAccelOffset(MPU6050_Handle_t *mpu, MPU6050_RawData_t *raw,
                             const MPU6050_StillnessConfig_t *stillness,
                             const Vector3f_t *accel_reference_g) {
  if (mpu == NULL || raw == NULL || stillness == NULL ||
      accel_reference_g == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (mpu->initialized == 0U) {
    return MPU6050_ERR_UNINITIALIZED;
  }

  MPU6050_CalibrationAverage_t average;

  uint32_t start_time = mpu->io->ops->get_tick_ms(mpu->io->context);

  for (;;) {

    MPU6050_Status_t status =
        mpu6050_collect_still_samples(mpu, raw, stillness, &average);

    if (status == MPU6050_OK) {

      mpu->accel_offset_g.x =
          (average.accel_x / mpu->accel_scale) - accel_reference_g->x;

      mpu->accel_offset_g.y =
          (average.accel_y / mpu->accel_scale) - accel_reference_g->y;

      mpu->accel_offset_g.z =
          (average.accel_z / mpu->accel_scale) - accel_reference_g->z;

      return MPU6050_OK;
    }

    if (status != MPU6050_ERR_MOVING) {
      return status;
    }

    uint32_t now = mpu->io->ops->get_tick_ms(mpu->io->context);

    if ((now - start_time) >= stillness->timeout_ms) {
      return MPU6050_ERR_TIMEOUT;
    }
  }
}

MPU6050_Status_t
MPU6050_ConfigureInterrupt(MPU6050_Handle_t *mpu,
                           const MPU6050_InterruptConfig_t *config) {
  if (mpu == NULL || config == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (mpu->initialized == 0U) {
    return MPU6050_ERR_UNINITIALIZED;
  }

  if (config->int_level > 1U || config->int_open > 1U ||
      config->latch_int_en > 1U || config->int_rd_clear > 1U) {
    return MPU6050_INVALID_CONFIG;
  }

  uint8_t mask =
      (uint8_t)((1U << MPU6050_INT_LEVEL_BIT) | (1U << MPU6050_INT_OPEN_BIT) |
                (1U << MPU6050_INT_LATCH_BIT) |
                (1U << MPU6050_INT_RD_CLEAR_BIT));

  uint8_t value = (uint8_t)((config->int_level << MPU6050_INT_LEVEL_BIT) |
                            (config->int_open << MPU6050_INT_OPEN_BIT) |
                            (config->latch_int_en << MPU6050_INT_LATCH_BIT) |
                            (config->int_rd_clear << MPU6050_INT_RD_CLEAR_BIT));

  return mpu6050_update_bits(mpu, MPU6050_REG_INT_PIN_CFG, mask, value);
}

MPU6050_Status_t MPU6050_SetDataReadyInterrupt(MPU6050_Handle_t *mpu,
                                               uint8_t enable) {
  if (mpu == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (mpu->initialized == 0U) {
    return MPU6050_ERR_UNINITIALIZED;
  }

  if (enable > 1U) {
    return MPU6050_INVALID_CONFIG;
  }

  uint8_t mask = (uint8_t)(1U << MPU6050_INT_DATA_RDY_BIT);

  uint8_t value = enable != 0U ? mask : 0U;

  return mpu6050_update_bits(mpu, MPU6050_REG_INT_ENABLE, mask, value);
}

MPU6050_Status_t MPU6050_EnableBypass(MPU6050_Handle_t *mpu) {
  if (mpu == NULL) {
    return MPU6050_ERR_NULL;
  }

  if (mpu->initialized == 0U) {
    return MPU6050_ERR_UNINITIALIZED;
  }

  MPU6050_Status_t status =
      mpu6050_update_bits(mpu, MPU6050_REG_USER_CTRL,
                          (uint8_t)(1U << MPU6050_USER_I2C_MST_EN_BIT), 0U);

  if (status != MPU6050_OK) {
    return status;
  }

  return mpu6050_update_bits(mpu, MPU6050_REG_INT_PIN_CFG,
                             (uint8_t)(1U << MPU6050_INT_BYPASS_BIT),
                             (uint8_t)(1U << MPU6050_INT_BYPASS_BIT));
}
