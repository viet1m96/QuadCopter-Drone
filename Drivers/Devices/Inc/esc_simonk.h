/*
 * ESC.h
 *
 *  Created on: Jul 25, 2026
 *      Author: vietht-hl
 */

#ifndef DEVICES_INC_ESC_H_
#define DEVICES_INC_ESC_H_
#include "motor_pwm.h"

typedef enum {
	ESC_STATE_UNINITIALIZED = 0,
	ESC_STATE_STOPPED,
	ESC_STATE_DISARMED,
	ESC_STATE_ARMING,
	ESC_STATE_ARMED,
	ESC_STATE_FAILSAFE
} ESC_State_t;

typedef enum {
	ESC_OK = 0,
	ESC_ERR_NULL,
	ESC_ERR_UNINITIALIZED,
	ESC_ERR_INVALID_STATE,
	ESC_ERR_MOTOR_PWM,
	ESC_INVALID_CONFIG
} ESC_Status_t;

typedef struct {
	uint16_t stop_pulse_us;
	uint16_t idle_pulse_us;
	uint16_t max_pulse_us;

	uint32_t arming_time_ms;
	uint32_t command_timeout_ms;
} ESC_Config_t;

typedef enum {
	ESC_FAILSAFE_NONE = 0,
	ESC_FAILSAFE_COMMAND_TIMEOUT
} ESC_FailSafeReason_t;

typedef struct {
	MotorPWM_Handle_t* motor_pwm;
	ESC_Config_t config;
	volatile ESC_State_t state;
	float last_throttles[MOTOR_PWM_QUANTITY];

	uint32_t start_arming_time_ms;
	uint32_t last_command_ms;

	ESC_FailSafeReason_t failsafe_reason;
} ESC_Handle_t;

ESC_Status_t ESC_Start(ESC_Handle_t* esc);

ESC_Status_t ESC_BeginArming(
		ESC_Handle_t *esc,
		uint32_t now_ms);
ESC_Status_t ESC_Process(
		ESC_Handle_t *esc,
		uint32_t now_ms);
ESC_Status_t ESC_SetThrottleAll(
		ESC_Handle_t* esc,
		const float throttles[MOTOR_PWM_QUANTITY],
		uint32_t now_ms);
ESC_Status_t ESC_SetThrottleAllSame(
		ESC_Handle_t* esc,
		float throttle,
		uint32_t now_ms);

ESC_Status_t ESC_Disarm(ESC_Handle_t* esc);

ESC_Status_t ESC_TriggerFailSafe(
		ESC_Handle_t* esc,
		ESC_FailSafeReason_t reason,
		uint32_t now_ms);

ESC_Status_t ESC_ClearFailSafe(
		ESC_Handle_t* esc);

ESC_Status_t ESC_Stop(ESC_Handle_t* esc);

ESC_Status_t ESC_Init(
        ESC_Handle_t *esc,
        MotorPWM_Handle_t *motor_pwm,
        const ESC_Config_t *config);











#endif /* DEVICES_INC_ESC_H_ */
