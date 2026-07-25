/*
 * motor_pwm.c
 *
 *  Created on: Jul 24, 2026
 *      Author: vietht-hl
 */

#include "motor_pwm.h"
#include "stddef.h"
#include "stdio.h"

void MotorPWM_NotiStatus(
		MotorPWM_Status_t status,
		const char* str) {
	if (str != NULL && str[0] != '\0') {
		printf("%s\r\n", str);
	}
	switch(status) {
	case MOTOR_PWM_OK:
		printf("MOTOR_PWM_OK\r\n");
		break;
	case MOTOR_PWM_ERR_NULL:
		printf("MOTOR_PWM_ERR_NULL\r\n");
		break;
	case MOTOR_PWM_ERR_NOT_INITIALIZED:
		printf("MOTOR_PWM_ERR_NOT_INITIALIZED\r\n");
		break;
	case MOTOR_PWM_ERR_NOT_STARTED:
		printf("MOTOR_PWM_ERR_NOT_STARTED\r\n");
		break;
	case MOTOR_PWM_ERR_ALREADY_STARTED:
		printf("MOTOR_PWM_ERR_ALREADY_STARTED\r\n");
		break;
	case MOTOR_PWM_ERR_HAL:
		printf("MOTOR_PWM_ERR_HAL\r\n");
		break;
	case MOTOR_PWM_INVALID_CONFIG:
		printf("MOTOR_PWM_INVALID_CONFIG\r\n");
		break;
	case MOTOR_PWM_ERR_INVALID_PULSE:
		printf("MOTOR_PWM_ERR_INVALID_PULSE\r\n");
		break;
	default:
		printf("Unknown status\r\n");
		return;
	}

}

MotorPWM_Status_t MotorPWM_Start(
		MotorPWM_Handle_t *motor_pwm,
		uint16_t initial_pulse_us) {
	if(motor_pwm == NULL || motor_pwm -> htim == NULL)
		return MOTOR_PWM_ERR_NULL;
	if(motor_pwm->initialized == 0U)
		return MOTOR_PWM_ERR_NOT_INITIALIZED;
	if(motor_pwm->started != 0U)
		return MOTOR_PWM_ERR_ALREADY_STARTED;
	if(initial_pulse_us >= motor_pwm->config.frame_period_us)
		return MOTOR_PWM_ERR_INVALID_PULSE;

	/*1 tick = 1 us*/
	uint32_t compare_ticks = initial_pulse_us;
	for(uint32_t i = 0; i < MOTOR_PWM_QUANTITY; i++) {
		__HAL_TIM_SET_COMPARE(
				motor_pwm->htim,
				motor_pwm->config.channels[i],
				compare_ticks);
	}

	__HAL_TIM_SET_COUNTER(motor_pwm->htim, 0U);
	if(HAL_TIM_GenerateEvent(motor_pwm->htim, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
		return MOTOR_PWM_ERR_HAL;
	}
	__HAL_TIM_CLEAR_FLAG(motor_pwm->htim, TIM_FLAG_UPDATE);

	for(uint32_t i = 0; i < MOTOR_PWM_QUANTITY; i++) {
		if(HAL_TIM_PWM_Start(motor_pwm -> htim, motor_pwm->config.channels[i]) != HAL_OK) {
			for(uint32_t j = 0; j < i; j++) {
				(void)(HAL_TIM_PWM_Stop(motor_pwm -> htim, motor_pwm -> config.channels[j]));
			}
			return MOTOR_PWM_ERR_HAL;
		}
	}

	for (uint32_t i = 0U; i < MOTOR_PWM_QUANTITY; ++i) {
		motor_pwm->last_pulses[i] = initial_pulse_us;
	}

	motor_pwm->started = 1U;

	return MOTOR_PWM_OK;

}


MotorPWM_Status_t MotorPWM_WriteAllUs(
        MotorPWM_Handle_t *motor_pwm,
        const uint16_t pulse_us[MOTOR_PWM_QUANTITY])
{
    if (motor_pwm == NULL ||
        motor_pwm->htim == NULL ||
        pulse_us == NULL)
    {
        return MOTOR_PWM_ERR_NULL;
    }

    if (motor_pwm->initialized == 0U)
    {
        return MOTOR_PWM_ERR_NOT_INITIALIZED;
    }

    if (motor_pwm->started == 0U)
    {
        return MOTOR_PWM_ERR_NOT_STARTED;
    }
    for (uint32_t i = 0U; i < MOTOR_PWM_QUANTITY; ++i)
    {
        if (pulse_us[i] >= motor_pwm->config.frame_period_us)
        {
            return MOTOR_PWM_ERR_INVALID_PULSE;
        }
    }

    for (uint32_t i = 0U; i < MOTOR_PWM_QUANTITY; ++i)
    {
        __HAL_TIM_SET_COMPARE(
                motor_pwm->htim,
                motor_pwm->config.channels[i],
                pulse_us[i]);

        motor_pwm->last_pulses[i] = pulse_us[i];
    }

    return MOTOR_PWM_OK;
}

MotorPWM_Status_t MotorPWM_WriteAllSameUs(
        MotorPWM_Handle_t *motor_pwm,
        uint16_t pulse_us)
{
    if (motor_pwm == NULL || motor_pwm->htim == NULL)
    {
        return MOTOR_PWM_ERR_NULL;
    }

    if (motor_pwm->initialized == 0U)
    {
        return MOTOR_PWM_ERR_NOT_INITIALIZED;
    }

    if (motor_pwm->started == 0U)
    {
        return MOTOR_PWM_ERR_NOT_STARTED;
    }

    if (pulse_us >= motor_pwm->config.frame_period_us)
    {
        return MOTOR_PWM_ERR_INVALID_PULSE;
    }

    for (uint32_t i = 0U; i < MOTOR_PWM_QUANTITY; ++i)
    {
        __HAL_TIM_SET_COMPARE(
                motor_pwm->htim,
                motor_pwm->config.channels[i],
                pulse_us);

        motor_pwm->last_pulses[i] = pulse_us;
    }

    return MOTOR_PWM_OK;
}


MotorPWM_Status_t MotorPWM_Stop(
        MotorPWM_Handle_t *motor_pwm)
{
    if (motor_pwm == NULL || motor_pwm->htim == NULL)
    {
        return MOTOR_PWM_ERR_NULL;
    }

    if (motor_pwm->initialized == 0U)
    {
        return MOTOR_PWM_ERR_NOT_INITIALIZED;
    }

    if (motor_pwm->started == 0U)
    {
        return MOTOR_PWM_ERR_NOT_STARTED;
    }

    MotorPWM_Status_t result = MOTOR_PWM_OK;

    for (uint32_t i = 0U; i < MOTOR_PWM_QUANTITY; ++i)
    {
        if (HAL_TIM_PWM_Stop(
                motor_pwm->htim,
                motor_pwm->config.channels[i]) != HAL_OK)
        {
            result = MOTOR_PWM_ERR_HAL;
        }
    }

    if (result == MOTOR_PWM_OK)
    {
        motor_pwm->started = 0U;
    }

    return result;
}


MotorPWM_Status_t MotorPWM_Init(
        MotorPWM_Handle_t *motor_pwm,
        TIM_HandleTypeDef *htim,
        const MotorPWM_Config_t *config)
{
    if (motor_pwm == NULL ||
        htim == NULL ||
        config == NULL)
    {
        return MOTOR_PWM_ERR_NULL;
    }

    if (htim->Instance == NULL ||
        config->frame_period_us == 0U)
    {
        return MOTOR_PWM_INVALID_CONFIG;
    }

    motor_pwm->htim = htim;
    motor_pwm->config = *config;

    for (uint32_t i = 0U; i < MOTOR_PWM_QUANTITY; ++i)
    {
        motor_pwm->last_pulses[i] = 0U;
    }

    motor_pwm->started = 0U;
    motor_pwm->initialized = 1U;

    return MOTOR_PWM_OK;
}
