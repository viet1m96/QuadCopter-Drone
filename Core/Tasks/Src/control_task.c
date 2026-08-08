#include "control_task.h"
#include "control_common.h"
#include "math.h"
#include "motor_mixer.h"
#include "mpu6050.h"
#include "stddef.h"

#define CONTROL_ARM_MAX_THROTTLE 0.05f

#define CONTROL_MAX_ANGLE_ROLL 30.0f
#define CONTROL_MAX_ANGLE_PITCH 30.0f

#define CONTROL_MAX_ROLL_RATE_DPS 200.0f
#define CONTROL_MAX_PITCH_RATE_DPS 200.0f
#define CONTROL_MAX_YAW_RATE_DPS 150.0f

#define CONTROL_MAX_DT_S 0.01f
#define CONTROL_SENSOR_TIMEOUT_MS 10U

#define CONTROL_RAD_TO_DEG 57.2957795f

#define CONTROL_TASK_STACK_DEPTH (configMINIMAL_STACK_SIZE * 2U)
#define CONTROL_TASK_PRIORITY (tskIDLE_PRIORITY + 5U)

static void ResetRatePID(ControlTask_Context_t *context) {
  (void)PID_Reset(&context->rate_pid_roll);
  (void)PID_Reset(&context->rate_pid_pitch);
  (void)PID_Reset(&context->rate_pid_yaw);
}

static void ResetAnglePID(ControlTask_Context_t *context) {
  (void)PID_Reset(&context->angle_pid_roll);
  (void)PID_Reset(&context->angle_pid_pitch);
}

static void ResetControlPID(ControlTask_Context_t *context) {
  ResetRatePID(context);
  ResetAnglePID(context);
}

static void Control_EnterFailSafe(ControlTask_Context_t *context) {
  context->flight_state = FLIGHT_STATE_FAILSAFE;

  ResetControlPID(context);

  (void)ESC_WriteStop(&context->esc);
}

static void ProcessArmState(ControlTask_Context_t *context,
                            const RCInput_Command_t *command) {
  switch (context->flight_state) {

  case FLIGHT_STATE_DISARMED:

    if (command->arm_request == 0U) {
      return;
    }

    if (command->throttle > CONTROL_ARM_MAX_THROTTLE) {
      return;
    }

    ResetControlPID(context);

    context->flight_state = FLIGHT_STATE_ARMED;

    break;

  case FLIGHT_STATE_ARMED:

    if (command->arm_request == 0U) {
      ResetControlPID(context);

      context->flight_state = FLIGHT_STATE_DISARMED;
    }

    break;

  case FLIGHT_STATE_FAILSAFE:

    if (command->arm_request == 0U) {
      ResetControlPID(context);

      context->flight_state = FLIGHT_STATE_DISARMED;
    }

    break;

  default:

    Control_EnterFailSafe(context);

    break;
  }
}

static float ComplementaryFilter(float gyro_angle_deg, float accel_angle_deg,
                                 float dt_s) {
  const float tau = 0.1f;

  float alpha = tau / (tau + dt_s);

  return alpha * gyro_angle_deg + (1.0f - alpha) * accel_angle_deg;
}

static void InitializeAngle(const MPU6050_Data_t *imu_data, Angle_t *angle) {
  float accel_yz = imu_data->accel_g.y * imu_data->accel_g.y +
                   imu_data->accel_g.z * imu_data->accel_g.z;

  angle->roll =
      atan2f(imu_data->accel_g.y, imu_data->accel_g.z) * CONTROL_RAD_TO_DEG;

  angle->pitch =
      atan2f(-imu_data->accel_g.x, sqrtf(accel_yz)) * CONTROL_RAD_TO_DEG;
}

static void CalculateAngle(const MPU6050_Data_t *imu_data, Angle_t *angle,
                           float dt_s) {
  float roll_acc =
      atan2f(imu_data->accel_g.y, imu_data->accel_g.z) * CONTROL_RAD_TO_DEG;

  float accel_yz = imu_data->accel_g.y * imu_data->accel_g.y +
                   imu_data->accel_g.z * imu_data->accel_g.z;

  float pitch_acc =
      atan2f(-imu_data->accel_g.x, sqrtf(accel_yz)) * CONTROL_RAD_TO_DEG;

  float roll_gyro = angle->roll + imu_data->gyro_dps.x * dt_s;

  float pitch_gyro = angle->pitch + imu_data->gyro_dps.y * dt_s;

  angle->roll = ComplementaryFilter(roll_gyro, roll_acc, dt_s);

  angle->pitch = ComplementaryFilter(pitch_gyro, pitch_acc, dt_s);
}

static PID_Status_t
RunRatePID(ControlTask_Context_t *context, const MPU6050_Data_t *imu_data,
           float roll_rate_setpoint, float pitch_rate_setpoint,
           float yaw_rate_setpoint, float dt_s, AxisCorrection_t *correction) {
  PID_Status_t status;

  status = PID_Update(&context->rate_pid_roll, roll_rate_setpoint,
                      imu_data->gyro_dps.x, dt_s, &correction->roll);

  if (status != PID_OK) {
    return status;
  }

  status = PID_Update(&context->rate_pid_pitch, pitch_rate_setpoint,
                      imu_data->gyro_dps.y, dt_s, &correction->pitch);

  if (status != PID_OK) {
    return status;
  }

  status = PID_Update(&context->rate_pid_yaw, yaw_rate_setpoint,
                      imu_data->gyro_dps.z, dt_s, &correction->yaw);

  return status;
}

static void ApplyMotorOutput(ControlTask_Context_t *context, float throttle,
                             const AxisCorrection_t *correction) {
  MotorMixer_Output_t output = {0};

  MotorMixer_Status_t mixer_status =
      MotorMixer_Mix(throttle, correction, &output);

  if (mixer_status != MIX_OK) {
    Control_EnterFailSafe(context);
    return;
  }

  ESC_Status_t esc_status = ESC_SetThrottleAll(&context->esc, output.motor);

  if (esc_status != ESC_OK) {
    Control_EnterFailSafe(context);
  }
}

static void ProcessRateMode(ControlTask_Context_t *context,
                            const RCInput_Command_t *command,
                            const MPU6050_Data_t *imu_data, float dt_s) {
  float roll_rate_setpoint = command->roll * CONTROL_MAX_ROLL_RATE_DPS;

  float pitch_rate_setpoint = command->pitch * CONTROL_MAX_PITCH_RATE_DPS;

  float yaw_rate_setpoint = command->yaw * CONTROL_MAX_YAW_RATE_DPS;

  AxisCorrection_t correction = {0};

  PID_Status_t pid_status =
      RunRatePID(context, imu_data, roll_rate_setpoint, pitch_rate_setpoint,
                 yaw_rate_setpoint, dt_s, &correction);

  if (pid_status != PID_OK) {
    Control_EnterFailSafe(context);
    return;
  }

  ApplyMotorOutput(context, command->throttle, &correction);
}

static void ProcessAngleMode(ControlTask_Context_t *context,
                             const RCInput_Command_t *command,
                             const MPU6050_Data_t *imu_data, float dt_s) {
  float roll_angle_setpoint = command->roll * CONTROL_MAX_ANGLE_ROLL;

  float pitch_angle_setpoint = command->pitch * CONTROL_MAX_ANGLE_PITCH;

  float roll_rate_setpoint = 0.0f;
  float pitch_rate_setpoint = 0.0f;

  float yaw_rate_setpoint = command->yaw * CONTROL_MAX_YAW_RATE_DPS;

  PID_Status_t pid_status;

  pid_status = PID_Update(&context->angle_pid_roll, roll_angle_setpoint,
                          context->cur_angle.roll, dt_s, &roll_rate_setpoint);

  if (pid_status != PID_OK) {
    Control_EnterFailSafe(context);
    return;
  }

  pid_status = PID_Update(&context->angle_pid_pitch, pitch_angle_setpoint,
                          context->cur_angle.pitch, dt_s, &pitch_rate_setpoint);

  if (pid_status != PID_OK) {
    Control_EnterFailSafe(context);
    return;
  }

  AxisCorrection_t correction = {0};

  pid_status =
      RunRatePID(context, imu_data, roll_rate_setpoint, pitch_rate_setpoint,
                 yaw_rate_setpoint, dt_s, &correction);

  if (pid_status != PID_OK) {
    Control_EnterFailSafe(context);
    return;
  }

  ApplyMotorOutput(context, command->throttle, &correction);
}

static void ProcessMode(ControlTask_Context_t *context,
                        const RCInput_Command_t *command,
                        const MPU6050_Data_t *imu_data, float dt_s) {
  switch (command->mode) {

  case RC_MODE_RATE:

    ProcessRateMode(context, command, imu_data, dt_s);

    break;

  case RC_MODE_ANGLE:

    ProcessAngleMode(context, command, imu_data, dt_s);

    break;

  default:

    Control_EnterFailSafe(context);

    break;
  }
}

static void ControlTask(void *argument) {
  ControlTask_Context_t *context = (ControlTask_Context_t *)argument;

  MPU6050_Data_t imu_data = {0};

  RCInput_Command_t command = {0};
  RCInput_Command_t new_command = {0};

  TickType_t previous_imu_tick;

  int previous_mode;

  (void)ESC_WriteStop(&context->esc);

  if (xQueueReceive(context->command_queue, &command, portMAX_DELAY) !=
      pdPASS) {

    Control_EnterFailSafe(context);
    vTaskDelete(NULL);
  }

  previous_mode = (int)command.mode;

  if (xQueueReceive(context->sensor_queue, &imu_data, portMAX_DELAY) !=
      pdPASS) {

    Control_EnterFailSafe(context);
    vTaskDelete(NULL);
  }

  previous_imu_tick = imu_data.timestamp_tick;

  InitializeAngle(&imu_data, &context->cur_angle);

  for (;;) {

    if (xQueueReceive(context->sensor_queue, &imu_data,
                      pdMS_TO_TICKS(CONTROL_SENSOR_TIMEOUT_MS)) != pdPASS) {

      Control_EnterFailSafe(context);
      continue;
    }

    TickType_t elapsed_ticks = imu_data.timestamp_tick - previous_imu_tick;

    previous_imu_tick = imu_data.timestamp_tick;

    float dt_s = (float)elapsed_ticks / (float)configTICK_RATE_HZ;

    if ((dt_s <= 0.0f) || (dt_s > CONTROL_MAX_DT_S)) {

      Control_EnterFailSafe(context);
      continue;
    }

    CalculateAngle(&imu_data, &context->cur_angle, dt_s);

    if (xQueueReceive(context->command_queue, &new_command, 0U) == pdPASS) {

      command = new_command;
    }

    if ((int)command.mode != previous_mode) {
      ResetControlPID(context);

      previous_mode = (int)command.mode;
    }

    TickType_t now = xTaskGetTickCount();

    TickType_t command_age = now - command.timestamp_tick;

    if (command_age >= context->command_timeout_ticks) {

      Control_EnterFailSafe(context);
      continue;
    }

    if (command.failsafe_active != 0U) {
      Control_EnterFailSafe(context);
      continue;
    }

    ProcessArmState(context, &command);

    if (context->flight_state != FLIGHT_STATE_ARMED) {

      (void)ESC_WriteStop(&context->esc);

      continue;
    }

    ProcessMode(context, &command, &imu_data, dt_s);
  }
}

BaseType_t ControlTask_Create(ControlTask_Context_t *control_ctx) {
  if (control_ctx == NULL) {
    return pdFAIL;
  }

  if ((control_ctx->sensor_queue == NULL) ||
      (control_ctx->command_queue == NULL)) {

    return pdFAIL;
  }

  if (control_ctx->command_timeout_ticks == 0U) {
    return pdFAIL;
  }

  control_ctx->flight_state = FLIGHT_STATE_DISARMED;

  return xTaskCreate(ControlTask, "ControlTask", CONTROL_TASK_STACK_DEPTH,
                     control_ctx, CONTROL_TASK_PRIORITY, NULL);
}
