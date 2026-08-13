/*
 * init.c
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#include "setup.h"
#include "device_IO.h"
#include "hal_i2c_transport.h"
#include "main.h"
#include "stdio.h"

void SystemClockConfig(void) {
  RCC_OscInitTypeDef osc_config = {0};
  RCC_ClkInitTypeDef clk_config = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc_config.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc_config.HSEState = RCC_HSE_OFF;
  osc_config.LSEState = RCC_LSE_OFF;
  osc_config.HSIState = RCC_HSI_ON;
  osc_config.LSIState = RCC_LSI_OFF;
  osc_config.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

  osc_config.PLL.PLLState = RCC_PLL_ON;
  osc_config.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc_config.PLL.PLLM = 8U;
  osc_config.PLL.PLLN = 180U;
  osc_config.PLL.PLLP = RCC_PLLP_DIV2;
  osc_config.PLL.PLLQ = 8U;
  osc_config.PLL.PLLR = 2U;

  if (HAL_RCC_OscConfig(&osc_config) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }

  clk_config.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

  clk_config.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk_config.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_config.APB1CLKDivider = RCC_HCLK_DIV4;
  clk_config.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&clk_config, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

void USART2_UART_Init(void) {
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200U;
  husart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husart2.Init.WordLength = UART_WORDLENGTH_8B;
  husart2.Init.StopBits = UART_STOPBITS_1;
  husart2.Init.Parity = UART_PARITY_NONE;
  husart2.Init.Mode = UART_MODE_TX;
  husart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&husart2) != HAL_OK) {
    Error_Handler();
  }
}

void USART1_UART_Init(void) {
  husart1.Instance = USART1;
  husart1.Init.BaudRate = 115200U;
  husart1.Init.WordLength = UART_WORDLENGTH_8B;
  husart1.Init.StopBits = UART_STOPBITS_1;
  husart1.Init.Parity = UART_PARITY_NONE;
  husart1.Init.Mode = UART_MODE_RX;
  husart1.Init.OverSampling = UART_OVERSAMPLING_16;
  husart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  if (HAL_UART_Init(&husart1) != HAL_OK) {
    Error_Handler();
  }
}

void DMA_UART1_Init(void) {
  __HAL_RCC_DMA2_CLK_ENABLE();

  hdma2_usart1_rx.Instance = DMA2_Stream2;
  hdma2_usart1_rx.Init.Channel = DMA_CHANNEL_4;
  hdma2_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma2_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma2_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma2_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma2_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma2_usart1_rx.Init.Mode = DMA_NORMAL;
  hdma2_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma2_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma2_usart1_rx) != HAL_OK) {
    Error_Handler();
  }

  __HAL_LINKDMA(&husart1, hdmarx, hdma2_usart1_rx);

  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

void TIM3_Init(void) {
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = TIM3_PRESCALER;
  htim3.Init.Period = TIM3_PERIOD;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  TIM_OC_InitTypeDef pwm_config = {0};

  pwm_config.OCMode = TIM_OCMODE_PWM1;
  pwm_config.Pulse = 0U;
  pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm_config.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
}

void I2C1_Init() {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = I2C_CLOCK_SPEED_FM;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.OwnAddress2 = 0U;

  HAL_I2C_Init(&hi2c1);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 6U, 0U);

  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 6U, 0U);
}

uint8_t IBUS_Setup(HAL_IBUS_Transport_t *transport) {
  if (transport == NULL) {
    return 0U;
  }

  const HAL_IBUS_TransportStatus_t status =
      HAL_IBUS_TransportInit(transport, &husart1);

  return status == HAL_IBUS_TRANSPORT_OK;
}

uint8_t RCInput_Setup(RCInput_Handle_t *rc_inp) {
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
  config.pitch.center = CENTER_THROTTLE;
  config.pitch.max = MAX_THROTTLE;
  config.pitch.deadband = COMMON_DEADBAND;
  config.pitch.reversed = 0U;

  config.yaw.channel_idx = YAW_CHANNEL_IDX;
  config.yaw.min = MIN_THROTTLE;
  config.yaw.center = CENTER_THROTTLE;
  config.yaw.max = MAX_THROTTLE;
  config.yaw.deadband = COMMON_DEADBAND;
  config.yaw.reversed = 0U;

  RCInput_Status_t status = RCInput_Init(rc_inp, &config);

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

uint8_t ReceiverTask_Setup(ReceiverTask_Context_t *ctx,
                           RCInput_Handle_t *rc_inp,
                           HAL_IBUS_Transport_t *transport) {
  if (ctx == NULL || transport == NULL || rc_inp == NULL)
    return 0U;
  ctx->raw_frame_queue = xQueueCreate(1U, sizeof(IBUS_RawFrame_t));
  if (ctx->raw_frame_queue == NULL)
    return 0U;
  ctx->command_queue = xQueueCreate(1U, sizeof(RCInput_Command_t));
  if (ctx->command_queue == NULL)
    return 0U;

  ctx->transport = transport;
  ctx->rc_input = rc_inp;
  ctx->timeout_ticks = pdMS_TO_TICKS(IBUS_TIMEOUT_MS);

  BaseType_t status = ReceiverTask_Create(ctx);
    if (status != pdPASS)
      return 0U;
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

  const MPU6050_InterruptConfig_t mpu_interrupt_config = {
      .int_level = 0U, .int_open = 0U, .latch_int_en = 0U, .int_rd_clear = 0U};

  if (HAL_I2C_DeviceIO_Init(&hi2c1, device_io) != DEVICE_IO_OK) {
	  return 0U;
  }


  if (MPU6050_Init(mpu, device_io, &mpu_config) != MPU6050_OK) {
	  return 0U;
  }

  if(MPU6050_RunCalibration(mpu) != MPU6050_OK) {
	  return 0U;
  }
  if (MPU6050_ConfigureInterrupt(mpu, &mpu_interrupt_config))
    return 0U;
  if (MPU6050_SetDataReadyInterrupt(mpu, 1U) != MPU6050_OK)
    return 0U;
  MPU6050_DRDY_GPIO_Init();
  return 1U;
}

void MPU6050_DRDY_GPIO_Init() {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef gpiob_config;
  gpiob_config.Pin = GPIO_PIN_12;
  gpiob_config.Mode = GPIO_MODE_IT_RISING;
  gpiob_config.Pull = GPIO_NOPULL;
  gpiob_config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HAL_GPIO_Init(GPIOB, &gpiob_config);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6U, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

uint8_t SensorTask_Setup(SensorTask_Context_t *sensor_ctx,
                         MPU6050_Handle_t *mpu) {
  if (sensor_ctx == NULL || mpu == NULL)
    return 0U;
  sensor_ctx->data_queue_to_control = xQueueCreate(1U, sizeof(MPU6050_Data_t));
  if (sensor_ctx->data_queue_to_control == NULL)
    return 0U;
  sensor_ctx->imu = mpu;
  BaseType_t status = SensorTask_Create(sensor_ctx);
  if (status != pdPASS)
    return 0U;
  return 1U;
}

uint8_t ControlTask_Setup(ControlTask_Context_t *control_ctx,
                          ReceiverTask_Context_t *receiver_ctx,
                          SensorTask_Context_t *sensor_ctx,
                          MotorPWM_Handle_t *motor_pwm) {
  if (control_ctx == NULL || receiver_ctx == NULL || sensor_ctx == NULL ||
      motor_pwm == NULL)
    return 0U;

  const PID_Config_t rate_pid_config = {
      .Kp = 0.005f,
      .Ki = 0.0f,
      .Kd = 0.0f,

      .integral_limit = 0.05f,
      .output_limit = 0.15f,

      .derivative_cut_of_hz = 20.0f
  };
  const PID_Config_t angle_pid_config = {
      .Kp = 0.0f,
      .Ki = 0.0f,
      .Kd = 0.0f,
      .integral_limit = 0.0f,
      .output_limit = 200.0f,
      .derivative_cut_of_hz = 0.0f
  };

  const ESC_Config_t esc_config = {
      .stop_pulse_us = MIN_THROTTLE,
      .idle_pulse_us = MIN_THROTTLE,
      .max_pulse_us = MAX_THROTTLE};

  control_ctx->sensor_queue = sensor_ctx->data_queue_to_control;
  control_ctx->command_queue = receiver_ctx->command_queue;
  control_ctx->command_timeout_ticks = receiver_ctx->timeout_ticks;

  if (PID_Init(&control_ctx->rate_pid_roll, &rate_pid_config) != PID_OK)
    return 0U;

  if (PID_Init(&control_ctx->rate_pid_pitch, &rate_pid_config) != PID_OK)
    return 0U;

  if (PID_Init(&control_ctx->rate_pid_yaw, &rate_pid_config) != PID_OK)
    return 0U;

  if (PID_Init(&control_ctx->angle_pid_roll, &angle_pid_config) != PID_OK)
    return 0U;

  if (PID_Init(&control_ctx->angle_pid_pitch, &angle_pid_config) != PID_OK)
    return 0U;

  if (ESC_Init(&control_ctx->esc, motor_pwm, &esc_config) != ESC_OK)
    return 0U;

  if (ESC_Start(&control_ctx->esc) != ESC_OK)
    return 0U;

  if (ControlTask_Create(control_ctx) != pdPASS)
    return 0U;

  return 1U;
}
