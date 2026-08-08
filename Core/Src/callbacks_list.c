#include "callbacks_list.h"
#include "peripherals.h"
#include "stm32f4xx_hal.h"
#include "string.h"

static ReceiverTask_Context_t *receiver_ctx;
static SensorTask_Context_t *sensor_ctx;

static void NotifyReceiverTaskFromISR(uint32_t events,
                                      BaseType_t *higher_priority_task_woken) {
  if (receiver_ctx == NULL || receiver_ctx->task_handle == NULL) {
    return;
  }

  (void)xTaskNotifyFromISR(receiver_ctx->task_handle, events, eSetBits,
                           higher_priority_task_woken);
}

static void NotifySensorTaskFromISR(uint32_t events,
                                    BaseType_t *higher_priority_task_woken) {
  if (sensor_ctx == NULL || sensor_ctx->task_handle == NULL) {
    return;
  }

  (void)xTaskNotifyFromISR(sensor_ctx->task_handle, events, eSetBits,
                           higher_priority_task_woken);
}

void Callbacks_Init(ReceiverTask_Context_t *rcv_ctx,
                    SensorTask_Context_t *sen_ctx) {
  receiver_ctx = rcv_ctx;
  sensor_ctx = sen_ctx;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
  if (receiver_ctx == NULL || receiver_ctx->transport == NULL ||
      receiver_ctx->transport->huart != huart) {
    return;
  }

  BaseType_t higher_priority_task_woken = pdFALSE;

  if (size == IBUS_FRAME_SIZE) {
    IBUS_RawFrame_t frame;

    memcpy(frame.bytes, receiver_ctx->transport->rx_buffer, IBUS_FRAME_SIZE);

    (void)xQueueOverwriteFromISR(receiver_ctx->raw_frame_queue, &frame,
                                 &higher_priority_task_woken);

    NotifyReceiverTaskFromISR(RECEIVER_EVENT_FRAME_READY,
                              &higher_priority_task_woken);
  }

  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (receiver_ctx == NULL || receiver_ctx->transport == NULL ||
      receiver_ctx->transport->huart != huart) {
    return;
  }

  BaseType_t higher_priority_task_woken = pdFALSE;

  NotifyReceiverTaskFromISR(RECEIVER_EVENT_UART_ERROR,
                            &higher_priority_task_woken);

  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {
  if (gpio_pin != GPIO_PIN_12) {
    return;
  }

  BaseType_t higher_priority_task_woken = pdFALSE;

  NotifySensorTaskFromISR(SENSOR_EVENT_MPU6050_DRDY,
                          &higher_priority_task_woken);

  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c != &hi2c1) {
    return;
  }

  BaseType_t higher_priority_task_woken = pdFALSE;

  NotifySensorTaskFromISR(SENSOR_EVENT_I2C1_RX_DONE,
                          &higher_priority_task_woken);

  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c != &hi2c1) {
    return;
  }

  BaseType_t higher_priority_task_woken = pdFALSE;

  NotifySensorTaskFromISR(SENSOR_EVENT_I2C1_ERROR, &higher_priority_task_woken);

  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c != &hi2c1) {
    return;
  }

  BaseType_t higher_priority_task_woken = pdFALSE;

  NotifySensorTaskFromISR(SENSOR_EVENT_I2C1_ABORT_DONE,
                          &higher_priority_task_woken);

  portYIELD_FROM_ISR(higher_priority_task_woken);
}
