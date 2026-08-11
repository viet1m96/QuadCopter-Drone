#include "sensor_task.h"
#include "stdio.h"

#define MPU6050_REQUEST_MAX_LATENCY_MS 2U
#define I2C1_TRANSACTION_TIMEOUT_MS 10U
#define I2C1_ABORT_TIMEOUT_MS 10U

typedef struct {
  SensorOwner_t owner;
  SensorRequest_t *request;
} SensorRequestCandidate_t;

static uint8_t DeadlineIsEarlier(TickType_t first, TickType_t second) {
  return (int32_t)(first - second) < 0;
}

static SensorOwner_t SelectEarliestRequest(SensorRequestCandidate_t *candidates,
                                           uint32_t count) {
  SensorOwner_t selected_owner = SENSOR_OWNER_NONE;

  TickType_t selected_deadline = 0U;

  for (uint32_t i = 0U; i < count; ++i) {
    SensorRequest_t *request = candidates[i].request;

    if (request->pending == 0U) {
      continue;
    }

    if (selected_owner == SENSOR_OWNER_NONE) {
      selected_owner = candidates[i].owner;

      selected_deadline = request->deadline_tick;

      continue;
    }

    if (DeadlineIsEarlier(request->deadline_tick, selected_deadline)) {
      selected_owner = candidates[i].owner;

      selected_deadline = request->deadline_tick;
    }
  }

  return selected_owner;
}

static void RecoverI2C1(SensorTask_Context_t *context) {
	(void)context;
}

static void HandleMPU6050Data(SensorTask_Context_t *context) {
  MPU6050_RawData_t raw;
  MPU6050_Data_t physical;
  MPU6050_Data_t calibrated;

  if (MPU6050_GetRawDataIT(context->imu, &raw) != MPU6050_OK) {
    return;
  }

  if (MPU6050_ConvertRawToPhysical(context->imu, &raw, &physical) !=
      MPU6050_OK) {
    return;
  }

  if (MPU6050_ApplyCalibration(context->imu, &physical, &calibrated) !=
      MPU6050_OK) {
    return;
  }
  calibrated.timestamp_tick = xTaskGetTickCount();
  printf("accel_x %f\r\n", calibrated.accel_g.x);
  printf("accel_y %f\r\n", calibrated.accel_g.y);
  printf("accel_z %f\r\n", calibrated.accel_g.z);

  printf("gyro_x %f\r\n", calibrated.gyro_dps.x);
  printf("gyro_y %f\r\n", calibrated.gyro_dps.y);
  printf("gyro_z %f\r\n", calibrated.gyro_dps.z);
  printf("----------------------\r\n");
  (void)xQueueOverwrite(context->data_queue_to_control, &calibrated);
}

static void HandleI2C1Events(SensorTask_Context_t *context, uint32_t events) {
  SensorBusManager_t *bus = &context->i2c1_manager;

  if (bus->state == SENSOR_BUS_ABORTING) {
    if ((events & SENSOR_EVENT_I2C1_ABORT_DONE) != 0U) {
      bus->state = SENSOR_BUS_IDLE;
      bus->owner = SENSOR_OWNER_NONE;
      bus->active_device_address = 0U;
    }

    return;
  }

  if (bus->state != SENSOR_BUS_BUSY) {
    return;
  }

  if ((events & SENSOR_EVENT_I2C1_ERROR) != 0U) {
    bus->state = SENSOR_BUS_IDLE;
    bus->owner = SENSOR_OWNER_NONE;
    bus->active_device_address = 0U;

    return;
  }

  if ((events & SENSOR_EVENT_I2C1_RX_DONE) == 0U) {
    return;
  }

  switch (bus->owner) {
  case SENSOR_OWNER_MPU6050:
    HandleMPU6050Data(context);
    break;

  default:
    break;
  }

  bus->state = SENSOR_BUS_IDLE;
  bus->owner = SENSOR_OWNER_NONE;
  bus->active_device_address = 0U;
}

static void HandleSensorEvents(SensorTask_Context_t *context, uint32_t events,
                               TickType_t now) {
  if ((events & SENSOR_EVENT_MPU6050_DRDY) != 0U) {
    if (context->imu_request.pending == 0U) {
      context->imu_request.pending = 1U;

      context->imu_request.deadline_tick =
          now + context->imu_request.max_latency_ticks;
    }
  }
}

static void HandleEvents(SensorTask_Context_t *context, uint32_t events,
                         TickType_t now) {
  HandleI2C1Events(context, events);

  HandleSensorEvents(context, events, now);
}

static void CheckI2C1Timeout(SensorTask_Context_t *context, TickType_t now) {
  SensorBusManager_t *bus = &context->i2c1_manager;

  if (bus->state == SENSOR_BUS_IDLE) {
    return;
  }

  TickType_t elapsed = now - bus->state_started_tick;

  if (bus->state == SENSOR_BUS_BUSY) {
    if (elapsed < bus->transaction_timeout_ticks) {
      return;
    }

    DeviceIO_Status_t status = bus->device_io->ops->abort_it(
        bus->device_io->context, bus->active_device_address);

    if (status == DEVICE_IO_OK) {
      bus->state = SENSOR_BUS_ABORTING;
      bus->state_started_tick = now;
      return;
    }

    RecoverI2C1(context);

    bus->state = SENSOR_BUS_IDLE;
    bus->owner = SENSOR_OWNER_NONE;
    bus->active_device_address = 0U;

    return;
  }

  if (bus->state == SENSOR_BUS_ABORTING) {
    if (elapsed < bus->abort_timeout_ticks) {
      return;
    }

    RecoverI2C1(context);

    bus->state = SENSOR_BUS_IDLE;
    bus->owner = SENSOR_OWNER_NONE;
    bus->active_device_address = 0U;
  }
}

static void CheckBusTimeouts(SensorTask_Context_t *context, TickType_t now) {
  CheckI2C1Timeout(context, now);
}

static void ScheduleI2C1(SensorTask_Context_t *context, TickType_t now) {
  SensorBusManager_t *bus = &context->i2c1_manager;

  if (bus->state != SENSOR_BUS_IDLE) {
    return;
  }

  SensorRequestCandidate_t candidates[] = {
      {.owner = SENSOR_OWNER_MPU6050, .request = &context->imu_request}};

  SensorOwner_t owner = SelectEarliestRequest(
      candidates, sizeof(candidates) / sizeof(candidates[0]));

  if (owner == SENSOR_OWNER_NONE) {
    return;
  }

  switch (owner) {
  case SENSOR_OWNER_MPU6050:
    if (MPU6050_StartReadRawDataIT(context->imu) != MPU6050_OK) {
      return;
    }

    context->imu_request.pending = 0U;

    bus->owner = SENSOR_OWNER_MPU6050;
    bus->active_device_address = context->imu->config.address;

    break;

  default:
    return;
  }

  bus->state = SENSOR_BUS_BUSY;
  bus->state_started_tick = now;
}

static void ScheduleRequests(SensorTask_Context_t *context, TickType_t now) {
  ScheduleI2C1(context, now);
}

static TickType_t CalculateBusWaitTime(SensorBusManager_t *bus,
                                       TickType_t now) {
  TickType_t timeout;

  if (bus->state == SENSOR_BUS_BUSY) {
    timeout = bus->transaction_timeout_ticks;
  } else if (bus->state == SENSOR_BUS_ABORTING) {
    timeout = bus->abort_timeout_ticks;
  } else {
    return portMAX_DELAY;
  }

  TickType_t elapsed = now - bus->state_started_tick;

  if (elapsed >= timeout) {
    return 0U;
  }

  return timeout - elapsed;
}

static TickType_t CalculateWaitTime(SensorTask_Context_t *context,
                                    TickType_t now) {
  TickType_t wait_time = portMAX_DELAY;

  TickType_t i2c1_wait = CalculateBusWaitTime(&context->i2c1_manager, now);

  if (i2c1_wait < wait_time) {
    wait_time = i2c1_wait;
  }

  return wait_time;
}

static void SensorTask(void *argument) {
  SensorTask_Context_t *context = (SensorTask_Context_t *)argument;

  uint32_t events = 0U;

  for (;;) {
    TickType_t now = xTaskGetTickCount();

    HandleEvents(context, events, now);

    CheckBusTimeouts(context, now);

    ScheduleRequests(context, now);

    events = 0U;

    (void)xTaskNotifyWait(0U, UINT32_MAX, &events,
                          CalculateWaitTime(context, now));
  }
}

BaseType_t SensorTask_Create(SensorTask_Context_t *sensor_ctx) {
  if (sensor_ctx == NULL || sensor_ctx->imu == NULL ||
      sensor_ctx->imu->io == NULL || sensor_ctx->imu->io->ops == NULL ||
      sensor_ctx->imu->io->ops->abort_it == NULL ||
      sensor_ctx->data_queue_to_control == NULL) {
    return pdFALSE;
  }

  sensor_ctx->imu_request.pending = 0U;
  sensor_ctx->imu_request.deadline_tick = 0U;
  sensor_ctx->imu_request.max_latency_ticks =
      pdMS_TO_TICKS(MPU6050_REQUEST_MAX_LATENCY_MS);

  sensor_ctx->i2c1_manager.state = SENSOR_BUS_IDLE;

  sensor_ctx->i2c1_manager.owner = SENSOR_OWNER_NONE;

  sensor_ctx->i2c1_manager.device_io = (DeviceIO_t *)sensor_ctx->imu->io;

  sensor_ctx->i2c1_manager.active_device_address = 0U;

  sensor_ctx->i2c1_manager.state_started_tick = 0U;

  sensor_ctx->i2c1_manager.transaction_timeout_ticks =
      pdMS_TO_TICKS(I2C1_TRANSACTION_TIMEOUT_MS);

  sensor_ctx->i2c1_manager.abort_timeout_ticks =
      pdMS_TO_TICKS(I2C1_ABORT_TIMEOUT_MS);

  sensor_ctx->task_handle = NULL;

  return xTaskCreate(SensorTask, "Sensor", 256U, sensor_ctx, 4U,
                     &sensor_ctx->task_handle);
}
