#include "application_setup.h"

#include "control_common.h"
#include "ibus.h"
#include "queue.h"

uint8_t TelemetryTask_Setup(TelemetryTask_Context_t *context) {
  if (context == NULL)
    return 0U;
  context->sample_queue = xQueueCreate(1U, sizeof(TelemetrySample_t));
  if (context->sample_queue == NULL)
    return 0U;
  BaseType_t status = TelemetryTask_Create(context);
  if (status == pdPASS) {
    return 1U;
  }
  return 0U;
}

uint8_t ReceiverTask_Setup(ReceiverTask_Context_t *context,
                           RCInput_Handle_t *rc_input,
                           HAL_IBUS_Transport_t *transport) {
  if (context == NULL || transport == NULL || rc_input == NULL) {
    return 0U;
  }

  context->raw_frame_queue = xQueueCreate(1U, sizeof(IBUS_RawFrame_t));
  if (context->raw_frame_queue == NULL) {
    return 0U;
  }

  context->command_queue = xQueueCreate(1U, sizeof(RCInput_Command_t));
  if (context->command_queue == NULL) {
    return 0U;
  }

  context->transport = transport;
  context->rc_input = rc_input;
  context->timeout_ticks = pdMS_TO_TICKS(IBUS_TIMEOUT_MS);

  BaseType_t status = ReceiverTask_Create(context);
  if (status != pdPASS) {
    return 0U;
  }

  return 1U;
}

uint8_t SensorTask_Setup(SensorTask_Context_t *context, MPU6050_Handle_t *mpu) {
  if (context == NULL || mpu == NULL) {
    return 0U;
  }

  context->data_queue_to_control = xQueueCreate(1U, sizeof(MPU6050_Data_t));
  if (context->data_queue_to_control == NULL) {
    return 0U;
  }

  context->imu = mpu;

  BaseType_t status = SensorTask_Create(context);
  if (status != pdPASS) {
    return 0U;
  }

  return 1U;
}

uint8_t ControlTask_Setup(ControlTask_Context_t *control_context,
                          ReceiverTask_Context_t *receiver_context,
                          SensorTask_Context_t *sensor_context,
                          TelemetryTask_Context_t *telemetry_context,
                          MotorPWM_Handle_t *motor_pwm) {
  if (control_context == NULL || receiver_context == NULL ||
      sensor_context == NULL || telemetry_context == NULL ||
      motor_pwm == NULL) {
    return 0U;
  }

  const PID_Config_t roll_pitch_rate_pid_config = {.Kp = 0.0025f,
                                                   .Ki = 0.0030f,
                                                   .Kd = 0.00002f,
                                                   .integral_limit = 0.03f,
                                                   .output_limit = 0.12f,
                                                   .derivative_cut_of_hz =
                                                       20.0f};

  const PID_Config_t yaw_rate_pid_config = {.Kp = 0.0012f,
                                            .Ki = 0.0005f,
                                            .Kd = 0.0f,
                                            .integral_limit = 0.02f,
                                            .output_limit = 0.08f,
                                            .derivative_cut_of_hz = 20.0f};

  const PID_Config_t angle_pid_config = {.Kp = 2.2f,
                                         .Ki = 0.0f,
                                         .Kd = 0.0f,
                                         .integral_limit = 0.0f,
                                         .output_limit = 60.0f,
                                         .derivative_cut_of_hz = 0.0f};

  const ESC_Config_t esc_config = {.stop_pulse_us = MIN_THROTTLE,
                                   .idle_pulse_us = ESC_IDLE_PULSE_US,
                                   .max_pulse_us = MAX_THROTTLE};

  control_context->sensor_queue = sensor_context->data_queue_to_control;
  control_context->command_queue = receiver_context->command_queue;
  control_context->telemetry.sample_queue = telemetry_context->sample_queue;
  control_context->command_timeout_ticks = receiver_context->timeout_ticks;

  if (PID_Init(&control_context->rate_pid_roll, &roll_pitch_rate_pid_config) !=
      PID_OK) {
    return 0U;
  }

  if (PID_Init(&control_context->rate_pid_pitch, &roll_pitch_rate_pid_config) !=
      PID_OK) {
    return 0U;
  }

  if (PID_Init(&control_context->rate_pid_yaw, &yaw_rate_pid_config) !=
      PID_OK) {
    return 0U;
  }

  if (PID_Init(&control_context->angle_pid_roll, &angle_pid_config) != PID_OK) {
    return 0U;
  }

  if (PID_Init(&control_context->angle_pid_pitch, &angle_pid_config) !=
      PID_OK) {
    return 0U;
  }

  if (ESC_Init(&control_context->esc, motor_pwm, &esc_config) != ESC_OK) {
    return 0U;
  }

  if (ESC_Start(&control_context->esc) != ESC_OK) {
    return 0U;
  }

  if (ControlTask_Create(control_context) != pdPASS) {
    return 0U;
  }

  return 1U;
}
