#include "sensor_task.h"
#include "peripherals.h"
#include "sensor_i2c_bus.h"

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

static void HandleMPU6050Data(SensorTask_Context_t *context) {
  MPU6050_RawData_t raw;
  MPU6050_Data_t physical;
  MPU6050_Data_t calibrated;
  calibrated.timestamp_us = context->imu_request.timestamp_ms;
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

  (void)xQueueOverwrite(context->data_queue_to_control, &calibrated);
}

static void HandleI2C1Events(SensorTask_Context_t *context, uint32_t events) {
  SensorOwner_t completed_owner =
      SensorI2CBus_HandleEvents(&context->i2c1_manager, events);

  switch (completed_owner) {
  case SENSOR_OWNER_MPU6050:
    HandleMPU6050Data(context);
    break;

  default:
    break;
  }
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

static void CheckBusTimeouts(SensorTask_Context_t *context, TickType_t now) {
  SensorI2CBus_CheckTimeout(&context->i2c1_manager, now);
}

static void ScheduleI2C1(SensorTask_Context_t *context, TickType_t now) {
  SensorBusManager_t *bus = &context->i2c1_manager;

  if (SensorI2CBus_IsIdle(bus) == 0U) {
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

    SensorI2CBus_Start(bus, SENSOR_OWNER_MPU6050,
                       (DeviceIO_t *)context->imu->io,
                       context->imu->config.address, now);

    break;

  default:
    return;
  }
}

static void ScheduleRequests(SensorTask_Context_t *context, TickType_t now) {
  ScheduleI2C1(context, now);
}

static TickType_t CalculateWaitTime(SensorTask_Context_t *context,
                                    TickType_t now) {
  TickType_t wait_time = portMAX_DELAY;

  TickType_t i2c1_wait =
      SensorI2CBus_CalculateWaitTime(&context->i2c1_manager, now);

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

  SensorI2CBus_Init(&sensor_ctx->i2c1_manager,
                    pdMS_TO_TICKS(I2C1_TRANSACTION_TIMEOUT_MS),
                    pdMS_TO_TICKS(I2C1_ABORT_TIMEOUT_MS));

  sensor_ctx->task_handle = NULL;

  return xTaskCreate(SensorTask, "Sensor", 256U, sensor_ctx, 4U,
                     &sensor_ctx->task_handle);
}
