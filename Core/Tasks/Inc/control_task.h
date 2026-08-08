/*
 * control_task.h
 *
 *  Created on: Aug 8, 2026
 *      Author: vietht-hl
 */

#ifndef TASKS_INC_CONTROL_TASK_H_
#define TASKS_INC_CONTROL_TASK_H_

#include "FreeRTOS.h"
#include "esc_simonk.h"
#include "pid_controller.h"
#include "queue.h"
#include "rc_input.h"
#include "stdint.h"
#include "task.h"

typedef enum {
  FLIGHT_STATE_DISARMED = 0,
  FLIGHT_STATE_ARMED,
  FLIGHT_STATE_FAILSAFE
} FlightState_t;

typedef struct {
  FlightState_t flight_state;

  PID_Handle_t rate_pid_roll;
  PID_Handle_t rate_pid_pitch;
  PID_Handle_t rate_pid_yaw;

  PID_Handle_t angle_pid_roll;
  PID_Handle_t angle_pid_pitch;

  QueueHandle_t sensor_queue;
  QueueHandle_t command_queue;

  ESC_Handle_t esc;

  TickType_t command_timeout_ticks;
} ControlTask_Context_t;

BaseType_t ControlTask_Create(ControlTask_Context_t *control_ctx);

#endif /* TASKS_INC_CONTROL_TASK_H_ */
