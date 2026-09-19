#include "device_setup.h"

#include <stdio.h>

#include "control_common.h"
#include "hal_i2c_transport.h"
#include "peripherals.h"
#include "stm32f4xx_hal.h"

uint8_t IBUS_Setup(HAL_IBUS_Transport_t *transport) {
  if (transport == NULL) {
    return 0U;
  }

  const HAL_IBUS_TransportStatus_t status =
      HAL_IBUS_TransportInit(transport, &husart1);

  return status == HAL_IBUS_TRANSPORT_OK;
}

uint8_t RCInput_Setup(RCInput_Handle_t *rc_input) {
  RCInput_Config_t config = {0};

  config.throttle.channel_idx = THROTTLE_CHANNEL_IDX;
  config.throttle.min = MIN_THROTTLE;
  config.throttle.max = MAX_THROTTLE;
  config.throttle.reversed = 0U;

  config.roll.channel_idx = ROLL_CHANNEL_IDX;
  config.roll.min = MIN_THROTTLE;
  config.roll.center = CENTER_THROTTLE;
  config.roll.max = MAX_THROTTLE;
  config.roll.deadband = COMMON_DEADBAND;
  config.roll.reversed = 0U;

  config.pitch.channel_idx = PITCH_CHANNEL_IDX;
  config.pitch.min = MIN_THROTTLE;
  config.pitch.center = PITCH_CENTER;
  config.pitch.max = MAX_THROTTLE;
  config.pitch.deadband = PITCH_DEADBAND;
  config.pitch.reversed = 0U;

  config.yaw.channel_idx = YAW_CHANNEL_IDX;
  config.yaw.min = MIN_THROTTLE;
  config.yaw.center = YAW_CENTER;
  config.yaw.max = MAX_THROTTLE;
  config.yaw.deadband = YAW_DEADBAND;
  config.yaw.reversed = 0U;

  RCInput_Status_t status = RCInput_Init(rc_input, &config);

  if (status != RC_INPUT_OK) {
    return 0U;
  }

  return 1U;
}

uint8_t MotorPWM_Setup(MotorPWM_Handle_t *motor_pwm) {
  MotorPWM_Config_t config = {
      .channels = {TIM_CHANNEL_4, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3},
      .frame_period_us = (uint16_t)(htim3.Init.Period + 1U)};

  MotorPWM_Status_t status = MotorPWM_Init(motor_pwm, &htim3, &config);

  if (status != MOTOR_PWM_OK) {
    return 0U;
  }

  return 1U;
}

static MPU6050_Status_t MPU6050_RunCalibration(MPU6050_Handle_t *mpu) {
  if (mpu == NULL) {
    return MPU6050_ERR_NULL;
  }

  MPU6050_RawData_t raw;
  MPU6050_StillnessConfig_t stillness;
  MPU6050_Calibration_t calibration;

  const Vector3f_t accel_reference_g = {.x = 0.0f, .y = 0.0f, .z = 1.0f};

  printf("\r\n");
  printf("========================================\r\n");
  printf("        MPU6050 CALIBRATION\r\n");
  printf("========================================\r\n");
  printf("Place the drone on a flat surface.\r\n");
  printf("Keep it completely still.\r\n");
  printf("Z axis must point upward.\r\n");
  printf("\r\n");
  printf("Calibration starts in 3 seconds...\r\n");

  printf("starting calibration\r\n");

  HAL_Delay(1000U);
  printf("3...\r\n");

  HAL_Delay(1000U);
  printf("2...\r\n");

  HAL_Delay(1000U);
  printf("1...\r\n");

  HAL_Delay(1000U);

  MPU6050_Status_t status = MPU6050_SetStillnessConfig(mpu, &stillness);

  if (status != MPU6050_OK) {
    printf("Failed to configure calibration: %d\r\n", (int)status);

    return status;
  }

  printf("\r\nCalibrating gyroscope...\r\n");

  status = MPU6050_CalibrateGyroOffset(mpu, &raw, &stillness);

  if (status != MPU6050_OK) {
    printf("Gyroscope calibration failed: %d\r\n", (int)status);

    return status;
  }

  printf("Gyroscope calibration complete.\r\n");

  printf("\r\nCalibrating accelerometer...\r\n");

  status =
      MPU6050_CalibrateAccelOffset(mpu, &raw, &stillness, &accel_reference_g);

  if (status != MPU6050_OK) {
    printf("Accelerometer calibration failed: %d\r\n", (int)status);

    return status;
  }

  printf("Accelerometer calibration complete.\r\n");

  status = MPU6050_GetCalibration(mpu, &calibration);

  if (status != MPU6050_OK) {
    printf("Failed to get calibration data: %d\r\n", (int)status);

    return status;
  }

  printf("\r\n");
  printf("========================================\r\n");
  printf("       CALIBRATION COMPLETE\r\n");
  printf("========================================\r\n");
  printf("\r\n");

  printf("#define MPU6050_ACCEL_OFFSET_X_G    %.8ff\r\n",
         calibration.accel_offset_g.x);

  printf("#define MPU6050_ACCEL_OFFSET_Y_G    %.8ff\r\n",
         calibration.accel_offset_g.y);

  printf("#define MPU6050_ACCEL_OFFSET_Z_G    %.8ff\r\n",
         calibration.accel_offset_g.z);

  printf("\r\n");

  printf("#define MPU6050_GYRO_OFFSET_X_DPS   %.8ff\r\n",
         calibration.gyro_offset_dps.x);

  printf("#define MPU6050_GYRO_OFFSET_Y_DPS   %.8ff\r\n",
         calibration.gyro_offset_dps.y);

  printf("#define MPU6050_GYRO_OFFSET_Z_DPS   %.8ff\r\n",
         calibration.gyro_offset_dps.z);

  printf("\r\n");
  printf("Copy the definitions above into your calibration header.\r\n");
  printf("========================================\r\n");

  return MPU6050_OK;
}

uint8_t MPU6050_Setup(MPU6050_Handle_t *mpu, DeviceIO_t *device_io) {
  const MPU6050_Config_t mpu_config = {
      .address = MPU6050_I2C_ADDRESS_AD0_LOW,
      .clksrc = MPU6050_CLKSRC_PLL_X,
      .dlpf_config = MPU6050_DLPF_CFG_2,
      .fs_sel_config = MPU6050_GYRO_CONFIG_FS_500DPS,
      .accel_sel_config = MPU6050_ACCEL_CONFIG_AFS_4G,
      .sample_rate_value = 1U};

  if (HAL_I2C_DeviceIO_Init(&hi2c1, device_io) != DEVICE_IO_OK) {
    return 0U;
  }

  if (MPU6050_Init(mpu, device_io, &mpu_config) != MPU6050_OK) {
    return 0U;
  }

  printf("Warming up MPU6050...\r\n");
  HAL_Delay(15000U);

  if (MPU6050_RunCalibration(mpu) != MPU6050_OK) {
    return 0U;
  }

  return 1U;
}

static void MPU6050_DRDY_GPIO_Init(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpiob_config = {0};
  gpiob_config.Pin = GPIO_PIN_12;
  gpiob_config.Mode = GPIO_MODE_IT_RISING;
  gpiob_config.Pull = GPIO_NOPULL;
  gpiob_config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HAL_GPIO_Init(GPIOB, &gpiob_config);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

uint8_t MPU6050_EnableDataReadyInterrupt(MPU6050_Handle_t *mpu) {
  if (mpu == NULL) {
    return 0U;
  }

  const MPU6050_InterruptConfig_t mpu_interrupt_config = {
      .int_level = 0U, .int_open = 0U, .latch_int_en = 0U, .int_rd_clear = 0U};

  if (MPU6050_ConfigureInterrupt(mpu, &mpu_interrupt_config) != MPU6050_OK) {
    return 0U;
  }

  MPU6050_DRDY_GPIO_Init();

  if (MPU6050_SetDataReadyInterrupt(mpu, 1U) != MPU6050_OK) {
    return 0U;
  }

  return 1U;
}
