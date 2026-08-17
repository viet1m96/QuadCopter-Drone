#include "callbacks_list.h"
#include "peripherals.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "string.h"

static ReceiverTask_Context_t *receiver_ctx;
static SensorTask_Context_t *sensor_ctx;
volatile uint32_t cnt = 1U;

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

  HAL_UART_RxEventTypeTypeDef event = HAL_UARTEx_GetRxEventType(huart);
  if (event == HAL_UART_RXEVENT_TC) {
    uint16_t length =
        IBUS_DMA_BUFFER_SIZE - receiver_ctx->transport->last_idle_pos;

    if (length != IBUS_FRAME_SIZE) {
      return;
    }

    IBUS_RawFrame_t frame;

    uint16_t read_pos = receiver_ctx->transport->last_idle_pos;

    for (uint16_t i = 0U; i < IBUS_FRAME_SIZE; i++) {
      frame.bytes[i] = receiver_ctx->transport->rx_buffer[read_pos++];
    }

    receiver_ctx->transport->last_idle_pos = 0U;

    if (frame.bytes[0] != 0x20U || frame.bytes[1] != 0x40U) {
      return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;

    (void)xQueueOverwriteFromISR(receiver_ctx->raw_frame_queue, &frame,
                                 &higher_priority_task_woken);

    NotifyReceiverTaskFromISR(RECEIVER_EVENT_FRAME_READY,
                              &higher_priority_task_woken);

    portYIELD_FROM_ISR(higher_priority_task_woken);

    return;
  }
  if (event != HAL_UART_RXEVENT_IDLE) {
    return;
  }

  uint16_t current_pos = (size == IBUS_DMA_BUFFER_SIZE) ? 0U : size;

  uint16_t last_idle_pos = receiver_ctx->transport->last_idle_pos;

  uint16_t length;

  if (current_pos >= last_idle_pos) {
    length = current_pos - last_idle_pos;
  } else {
    length = (IBUS_DMA_BUFFER_SIZE - last_idle_pos) + current_pos;
  }

  if (length != IBUS_FRAME_SIZE) {
    receiver_ctx->transport->last_idle_pos = current_pos;
    return;
  }

  IBUS_RawFrame_t frame;

  uint16_t read_pos = last_idle_pos;

  for (uint16_t i = 0U; i < IBUS_FRAME_SIZE; i++) {
    frame.bytes[i] = receiver_ctx->transport->rx_buffer[read_pos];

    read_pos++;

    if (read_pos >= IBUS_DMA_BUFFER_SIZE) {
      read_pos = 0U;
    }
  }

  receiver_ctx->transport->last_idle_pos = current_pos;

  if (frame.bytes[0] != 0x20U || frame.bytes[1] != 0x40U) {
    return;
  }

  BaseType_t higher_priority_task_woken = pdFALSE;

  (void)xQueueOverwriteFromISR(receiver_ctx->raw_frame_queue, &frame,
                               &higher_priority_task_woken);

  NotifyReceiverTaskFromISR(RECEIVER_EVENT_FRAME_READY,
                            &higher_priority_task_woken);

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
  if (gpio_pin == GPIO_PIN_12) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    sensor_ctx->imu_request.timestamp_ms = PrecisionTimer_GetUs();
    NotifySensorTaskFromISR(SENSOR_EVENT_MPU6050_DRDY,
                            &higher_priority_task_woken);

    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
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
  cnt++;
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
