#ifndef TASKS_INC_TELEMETRY_TASK_H_
#define TASKS_INC_TELEMETRY_TASK_H_

#include "FreeRTOS.h"
#include "motor_common.h"
#include "queue.h"
#include "stdint.h"
#include "task.h"




typedef struct {
  uint32_t timestamp_us;

  float throttle_command;
  float motor_throttle[MOTOR_PWM_QUANTITY];

  float roll_command;
  float pitch_command;
  float yaw_command;

  float roll_deg;
  float pitch_deg;

  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;

  float accel_x_g;
  float accel_y_g;
  float accel_z_g;

  float roll_correction;
  float pitch_correction;
  float yaw_correction;

} TelemetrySample_t;


typedef struct {
  QueueHandle_t sample_queue;
} TelemetryTask_Context_t;

BaseType_t TelemetryTask_Create(TelemetryTask_Context_t *context);




#endif
