#ifndef TASKS_INC_SENSOR_TASK_H_
#define TASKS_INC_SENSOR_TASK_H_

#include "FreeRTOS.h"
#include "mpu6050.h"
#include "queue.h"
#include "task.h"

#define SENSOR_EVENT_I2C1_RX_DONE (1UL << 0)
#define SENSOR_EVENT_I2C1_ERROR (1UL << 1)
#define SENSOR_EVENT_I2C1_ABORT_DONE (1UL << 2)
#define SENSOR_EVENT_MPU6050_DRDY (1UL << 3)

typedef enum {
  SENSOR_BUS_IDLE = 0,
  SENSOR_BUS_BUSY,
  SENSOR_BUS_ABORTING
} SensorBusState_t;

typedef enum { SENSOR_OWNER_NONE = 0, SENSOR_OWNER_MPU6050 } SensorOwner_t;

typedef struct {
  uint8_t pending;
  TickType_t deadline_tick;
  TickType_t max_latency_ticks;
} SensorRequest_t;

typedef struct {
  SensorBusState_t state;
  SensorOwner_t owner;

  DeviceIO_t *device_io;
  uint8_t active_device_address;

  TickType_t state_started_tick;
  TickType_t transaction_timeout_ticks;
  TickType_t abort_timeout_ticks;
} SensorBusManager_t;

typedef struct {
  MPU6050_Handle_t *imu;
  SensorRequest_t imu_request;

  SensorBusManager_t i2c1_manager;

  QueueHandle_t data_queue_to_control;
  TaskHandle_t task_handle;
} SensorTask_Context_t;

BaseType_t SensorTask_Create(SensorTask_Context_t *sensor_ctx);

#endif
