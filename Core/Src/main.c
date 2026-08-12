#include "receiver_task.h"
#include "sensor_task.h"
#include "control_task.h"
#include "stm32f4xx.h"

#include "main.h"

#include "motor_mixer.h"
#include "motor_pwm.h"

#include "setup.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "callbacks_list.h"
#include "device_IO.h"
#include "mpu6050.h"
#include "queue.h"
#include "task.h"

static RCInput_Handle_t rc_inp;
static HAL_IBUS_Transport_t ibus_transport;
static DeviceIO_t device_io;
static MPU6050_Handle_t mpu;
static MotorPWM_Handle_t motor_pwm;

static ReceiverTask_Context_t receiver_ctx;
static SensorTask_Context_t sensor_ctx;
static ControlTask_Context_t control_ctx;

static void UserLED_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef gpioa_config = {0};

  gpioa_config.Pin = GPIO_PIN_5;
  gpioa_config.Mode = GPIO_MODE_OUTPUT_PP;
  gpioa_config.Pull = GPIO_NOPULL;
  gpioa_config.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &gpioa_config);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

int main(void) {
  HAL_Init();

  UserLED_Init();

  SystemClockConfig();

  USART2_UART_Init();
  USART1_UART_Init();
  DMA_UART1_Init();

  TIM3_Init();
  I2C1_Init();

  if (!RCInput_Setup(&rc_inp)) {
    return 0;
  }

  if (!IBUS_Setup(&ibus_transport)) {
    return 0;
  }

  if (!MPU6050_Setup(&mpu, &device_io)) {
    return 0;
  }

  if (!MotorPWM_Setup(&motor_pwm)) {
    return 0;
  }

  if (!ReceiverTask_Setup(&receiver_ctx, &rc_inp, &ibus_transport)) {
    return 0;
  }

  if (!SensorTask_Setup(&sensor_ctx, &mpu)) {
    return 0;
  }

  if (!ControlTask_Setup(&control_ctx, &receiver_ctx, &sensor_ctx, &motor_pwm)) {
    return 0;
  }

  Callbacks_Init(&receiver_ctx, &sensor_ctx);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  vTaskStartScheduler();

  for (;;) {
  }
}
