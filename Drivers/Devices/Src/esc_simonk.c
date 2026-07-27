/*
 * esc_simonk.c
 *
 *  Created on: Jul 25, 2026
 *      Author: vietht-hl
 */

#include "esc_simonk.h"
#include "stddef.h"
#include "stdint.h"

ESC_Status_t ESC_Start(ESC_Handle_t* esc) {
	if(esc == NULL || esc->motor_pwm == NULL) return ESC_ERR_NULL;
	if(esc->state == ESC_STATE_UNINITIALIZED) return ESC_ERR_UNINITIALIZED;
	if(esc->state != ESC_STATE_STOPPED) return ESC_ERR_INVALID_STATE;

	MotorPWM_Status_t motor_status = MotorPWM_Start(esc->motor_pwm, esc->config.stop_pulse_us);
	if(motor_status != MOTOR_PWM_OK) {
		esc->output_safe = 0U;
		return ESC_ERR_MOTOR_PWM;
	}

	for (uint32_t i = 0U; i < MOTOR_PWM_QUANTITY; ++i) {
		esc->last_throttles[i] = 0.0f;
	}

	esc->output_safe = 1U;
	esc->state = ESC_STATE_DISARMED;
	return ESC_OK;
}

ESC_Status_t ESC_BeginArming(
        ESC_Handle_t *esc,
        uint32_t now_ms) {
	if(esc == NULL || esc->motor_pwm == NULL) return ESC_ERR_NULL;
	if(esc->state == ESC_STATE_UNINITIALIZED) return ESC_ERR_UNINITIALIZED;
	if(esc->state != ESC_STATE_DISARMED) return ESC_ERR_INVALID_STATE;
	MotorPWM_Status_t status = MotorPWM_WriteAllSameUs(esc->motor_pwm, esc->config.stop_pulse_us);
	if(status != MOTOR_PWM_OK) {
		esc->output_safe = 0U;
		return ESC_ERR_MOTOR_PWM;
	}
	esc->output_safe = 1U;
	esc->start_arming_time_ms = now_ms;
	esc->state = ESC_STATE_ARMING;
	return ESC_OK;
}

ESC_Status_t ESC_Process(
		ESC_Handle_t *esc,
		uint32_t now_ms) {
	if(esc == NULL) return ESC_ERR_NULL;
	if(esc->state == ESC_STATE_UNINITIALIZED) {
		return ESC_ERR_UNINITIALIZED;
	}
	if(esc->state == ESC_STATE_ARMING) {
		if((now_ms - esc->start_arming_time_ms) >= esc->config.arming_time_ms) {
			esc->state = ESC_STATE_ARMED;
			esc->last_command_ms = now_ms;
		}
	} else if(esc->state == ESC_STATE_ARMED) {
		if((now_ms - esc->last_command_ms) >= esc->config.command_timeout_ms) {
			return ESC_TriggerFailSafe(
			                    esc,
			                    ESC_FAILSAFE_COMMAND_TIMEOUT,
			                    now_ms);
		}
	}
	return ESC_OK;
}

static uint16_t esc_convert_throttle_to_pulse_us(ESC_Handle_t* esc, float throttle) {
	float pulse_f = esc->config.idle_pulse_us +
					throttle * (esc->config.max_pulse_us - esc->config.idle_pulse_us);
	return (uint16_t)(pulse_f + 0.5f);
}



ESC_Status_t ESC_SetThrottleAll(
		ESC_Handle_t* esc,
		const float throttles[MOTOR_PWM_QUANTITY],
		uint32_t now_ms) {
	if(esc == NULL ||
	   esc->motor_pwm == NULL ||
	   throttles == NULL) {
	   return ESC_ERR_NULL;
	}

	if(esc->state != ESC_STATE_ARMED) {
		return ESC_ERR_INVALID_STATE;
	}
	for(uint32_t i = 0; i < MOTOR_PWM_QUANTITY; i++) {
		if (!(throttles[i] >= 0.0f && throttles[i] <= 1.0f)) {
		    return ESC_INVALID_CONFIG;
		}
	}
	uint16_t pulse_us[MOTOR_PWM_QUANTITY];
	for(uint32_t i = 0; i < MOTOR_PWM_QUANTITY; i++) {
		pulse_us[i] = esc_convert_throttle_to_pulse_us(esc, throttles[i]);
 	}
	MotorPWM_Status_t status = MotorPWM_WriteAllUs(esc->motor_pwm, pulse_us);
	if(status != MOTOR_PWM_OK) return ESC_ERR_MOTOR_PWM;
	esc->output_safe = 0U;
	for(uint32_t i = 0; i < MOTOR_PWM_QUANTITY; i++) {
		esc->last_throttles[i] = throttles[i];
	}
	esc->last_command_ms = now_ms;
	return ESC_OK;
}

ESC_Status_t ESC_SetThrottleAllSame(
		ESC_Handle_t* esc,
		float throttle,
		uint32_t now_ms) {
	if(esc == NULL ||
	   esc->motor_pwm == NULL) {
	   return ESC_ERR_NULL;
	}

	if(esc->state != ESC_STATE_ARMED) {
		return ESC_ERR_INVALID_STATE;
	}
	if (!(throttle >= 0.0f && throttle <= 1.0f)) {
	    return ESC_INVALID_CONFIG;
	}
	uint16_t pulse_us = esc_convert_throttle_to_pulse_us(esc, throttle);

	MotorPWM_Status_t status = MotorPWM_WriteAllSameUs(esc->motor_pwm, pulse_us);
	if(status != MOTOR_PWM_OK) return ESC_ERR_MOTOR_PWM;
	esc->output_safe = 0U;
	for(uint32_t i = 0; i < MOTOR_PWM_QUANTITY; i++) {
		esc->last_throttles[i] = throttle;
	}
	esc->last_command_ms = now_ms;
	return ESC_OK;
}

ESC_Status_t ESC_Disarm(ESC_Handle_t* esc) {
	if(esc == NULL || esc->motor_pwm == NULL) return ESC_ERR_NULL;
	if (esc->state == ESC_STATE_UNINITIALIZED) {
		return ESC_ERR_UNINITIALIZED;
	}
	if (esc->state != ESC_STATE_ARMING &&
		esc->state != ESC_STATE_ARMED &&
		esc->state != ESC_STATE_DISARMED)
	{
		return ESC_ERR_INVALID_STATE;
	}
	esc->output_safe = 0U;
	MotorPWM_Status_t status = MotorPWM_WriteAllSameUs(esc->motor_pwm, esc->config.stop_pulse_us);
	if(status != MOTOR_PWM_OK) {
		return ESC_ERR_MOTOR_PWM;
	}
	esc->output_safe = 1U;
	for(uint32_t i = 0; i < MOTOR_PWM_QUANTITY; i++) {
		esc->last_throttles[i] = 0.0f;
	}
	esc->state = ESC_STATE_DISARMED;
	return ESC_OK;
}


ESC_Status_t ESC_TriggerFailSafe(
		ESC_Handle_t* esc,
		ESC_FailSafeReason_t reason,
		uint32_t now_ms) {
	if(esc == NULL || esc->motor_pwm == NULL) return ESC_ERR_NULL;
	if(esc->state == ESC_STATE_UNINITIALIZED) {
		return ESC_ERR_UNINITIALIZED;
	}
	if(esc->state != ESC_STATE_ARMED && esc->state != ESC_STATE_ARMING) {
		return ESC_ERR_INVALID_STATE;
	}
	esc->failsafe_reason = reason;
	esc->last_command_ms = now_ms;
	esc->state = ESC_STATE_FAILSAFE;
	esc->output_safe = 0U;
	MotorPWM_Status_t status = MotorPWM_WriteAllSameUs(esc->motor_pwm, esc->config.stop_pulse_us);
	if(status != MOTOR_PWM_OK) return ESC_ERR_MOTOR_PWM;
	for(uint32_t i = 0; i < MOTOR_PWM_QUANTITY; i++) {
		esc->last_throttles[i] = 0.0f;
	}
	esc->output_safe = 1U;
	return ESC_OK;
}

ESC_Status_t ESC_ClearFailSafe(
		ESC_Handle_t* esc) {
	if(esc == NULL || esc->motor_pwm == NULL) return ESC_ERR_NULL;
	if(esc->state != ESC_STATE_FAILSAFE) return ESC_ERR_INVALID_STATE;
	esc->output_safe = 0U;
	MotorPWM_Status_t status = MotorPWM_WriteAllSameUs(
	                    			esc->motor_pwm,
									esc->config.stop_pulse_us);

	if (status != MOTOR_PWM_OK) {
		return ESC_ERR_MOTOR_PWM;
	}

	for (uint32_t i = 0U; i < MOTOR_PWM_QUANTITY; ++i) {
		esc->last_throttles[i] = 0.0f;
	}
	esc->output_safe = 1U;
	esc->failsafe_reason = ESC_FAILSAFE_NONE;
	esc->last_command_ms = 0U;
	esc->state = ESC_STATE_DISARMED;
	return ESC_OK;
}
ESC_Status_t ESC_Stop(ESC_Handle_t* esc)
{
    if (esc == NULL || esc->motor_pwm == NULL) {
        return ESC_ERR_NULL;
    }

    if (esc->state != ESC_STATE_FAILSAFE &&
        esc->state != ESC_STATE_DISARMED)
    {
        return ESC_ERR_INVALID_STATE;
    }

    MotorPWM_Status_t status =
            MotorPWM_Stop(esc->motor_pwm);

    if (status != MOTOR_PWM_OK) {
        return ESC_ERR_MOTOR_PWM;
    }

    for (uint32_t i = 0U; i < MOTOR_PWM_QUANTITY; ++i) {
        esc->last_throttles[i] = 0.0f;
    }

    esc->output_safe = 1U;
    esc->state = ESC_STATE_STOPPED;

    return ESC_OK;
}


ESC_Status_t ESC_Init(
        ESC_Handle_t *esc,
        MotorPWM_Handle_t *motor_pwm,
        const ESC_Config_t *config)
{
    if (esc == NULL ||
        motor_pwm == NULL ||
        config == NULL)
    {
        return ESC_ERR_NULL;
    }

    if (motor_pwm->htim == NULL ||
        motor_pwm->state == MOTOR_PWM_STATE_UNINITIALIZED ||
		motor_pwm->state == MOTOR_PWM_STATE_STARTED ||
		motor_pwm->state == MOTOR_PWM_STATE_FAULT)
    {
        return ESC_ERR_MOTOR_PWM;
    }



    if (config->stop_pulse_us == 0U ||
        config->stop_pulse_us > config->idle_pulse_us ||
        config->idle_pulse_us >= config->max_pulse_us ||
        config->max_pulse_us >= motor_pwm->config.frame_period_us ||
        config->arming_time_ms == 0U ||
        config->command_timeout_ms == 0U)
    {
        return ESC_INVALID_CONFIG;
    }

    esc->motor_pwm = motor_pwm;
    esc->config = *config;

    esc->state = ESC_STATE_STOPPED;

    for (uint32_t i = 0U; i < MOTOR_PWM_QUANTITY; ++i)
    {
        esc->last_throttles[i] = 0.0f;
    }

    esc->start_arming_time_ms = 0U;
    esc->last_command_ms = 0U;
    esc->failsafe_reason = ESC_FAILSAFE_NONE;
    esc->output_safe = 0U;

    return ESC_OK;
}










