#ifndef DEVICES_INC_ESC_SIMONK_H_
#define DEVICES_INC_ESC_SIMONK_H_

#include "motor_pwm.h"
#include <stdint.h>

typedef enum {
  ESC_OK = 0,
  ESC_ERR_NULL,
  ESC_ERR_UNINITIALIZED,
  ESC_ERR_MOTOR_PWM,
  ESC_ERR_INVALID_THROTTLE,
  ESC_ERR_INVALID_CONFIG
} ESC_Status_t;

typedef struct {
  uint16_t stop_pulse_us;
  uint16_t idle_pulse_us;
  uint16_t max_pulse_us;
} ESC_Config_t;

typedef struct {
  MotorPWM_Handle_t *motor_pwm;
  ESC_Config_t config;
  uint8_t initialized;
} ESC_Handle_t;

ESC_Status_t ESC_Init(ESC_Handle_t *esc, MotorPWM_Handle_t *motor_pwm,
                      const ESC_Config_t *config);

ESC_Status_t ESC_Start(ESC_Handle_t *esc);

ESC_Status_t ESC_WriteStop(ESC_Handle_t *esc);

ESC_Status_t ESC_SetThrottleAll(ESC_Handle_t *esc,
                                const float throttles[MOTOR_PWM_QUANTITY]);

ESC_Status_t ESC_SetThrottleAllSame(ESC_Handle_t *esc, float throttle);

ESC_Status_t ESC_Stop(ESC_Handle_t *esc);

#endif
