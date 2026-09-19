#ifndef CORE_INC_APPLICATION_SETUP_H_
#define CORE_INC_APPLICATION_SETUP_H_

#include <stdint.h>

#include "control_task.h"
#include "hal_ibus_transport.h"
#include "motor_pwm.h"
#include "mpu6050.h"
#include "rc_input.h"
#include "receiver_task.h"
#include "sensor_task.h"
#include "telemetry_task.h"

uint8_t TelemetryTask_Setup(TelemetryTask_Context_t *context);

uint8_t ReceiverTask_Setup(ReceiverTask_Context_t *context,
                           RCInput_Handle_t *rc_input,
                           HAL_IBUS_Transport_t *transport);
uint8_t SensorTask_Setup(SensorTask_Context_t *context, MPU6050_Handle_t *mpu);
uint8_t ControlTask_Setup(ControlTask_Context_t *control_context,
                          ReceiverTask_Context_t *receiver_context,
                          SensorTask_Context_t *sensor_context,
                          TelemetryTask_Context_t *telemetry_context,
                          MotorPWM_Handle_t *motor_pwm);

#endif
