#ifndef TASKS_INC_SENSOR_TASK_H_
#define TASKS_INC_SENSOR_TASK_H_

#include "FreeRTOS.h"
#include "mpu6050.h"
#include "queue.h"
#include "sensor_i2c_bus.h"
#include "task.h"

#define SENSOR_EVENT_I2C1_RX_DONE (1UL << 0)
#define SENSOR_EVENT_I2C1_ERROR (1UL << 1)
#define SENSOR_EVENT_I2C1_ABORT_DONE (1UL << 2)
#define SENSOR_EVENT_MPU6050_DRDY (1UL << 3)

typedef struct {
  uint8_t pending;
  TickType_t deadline_tick;
  TickType_t max_latency_ticks;
  uint32_t timestamp_ms;
} SensorRequest_t;

typedef struct {
  MPU6050_Handle_t *imu;
  SensorRequest_t imu_request;

  SensorBusManager_t i2c1_manager;

  QueueHandle_t data_queue_to_control;
  TaskHandle_t task_handle;
} SensorTask_Context_t;

BaseType_t SensorTask_Create(SensorTask_Context_t *sensor_ctx);

#endif
