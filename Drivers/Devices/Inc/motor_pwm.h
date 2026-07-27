/*
 * motor_pwm.h
 *
 *  Created on: Jul 24, 2026
 *      Author: vietht-hl
 */

#ifndef DEVICES_INC_MOTOR_PWM_H_
#define DEVICES_INC_MOTOR_PWM_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "motor_common.h"


typedef enum {
    MOTOR_PWM_OK = 0,
    MOTOR_PWM_ERR_NULL,
    MOTOR_PWM_ERR_NOT_INITIALIZED,
    MOTOR_PWM_ERR_NOT_STARTED,
    MOTOR_PWM_ERR_ALREADY_STARTED,
    MOTOR_PWM_ERR_FAULT,
    MOTOR_PWM_ERR_HAL,
    MOTOR_PWM_ERR_INVALID_PULSE,
    MOTOR_PWM_INVALID_CONFIG
} MotorPWM_Status_t;

typedef enum {
    MOTOR_PWM_STATE_UNINITIALIZED = 0,
    MOTOR_PWM_STATE_STOPPED,
    MOTOR_PWM_STATE_STARTED,
    MOTOR_PWM_STATE_FAULT
} MotorPWM_State_t;

typedef struct {
    uint32_t channels[MOTOR_PWM_QUANTITY];
    uint16_t frame_period_us;
} MotorPWM_Config_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    MotorPWM_Config_t config;
    uint32_t last_pulses[MOTOR_PWM_QUANTITY];
    MotorPWM_State_t state;
} MotorPWM_Handle_t;

void MotorPWM_NotiStatus(
        MotorPWM_Status_t status,
        const char *str);

MotorPWM_Status_t MotorPWM_Start(
        MotorPWM_Handle_t *motor_pwm,
        uint16_t initial_pulse_us);

MotorPWM_Status_t MotorPWM_WriteAllUs(
        MotorPWM_Handle_t *motor_pwm,
        const uint16_t pulse_us[MOTOR_PWM_QUANTITY]);

MotorPWM_Status_t MotorPWM_WriteAllSameUs(
        MotorPWM_Handle_t *motor_pwm,
        uint16_t pulse_us);

MotorPWM_Status_t MotorPWM_Stop(
        MotorPWM_Handle_t *motor_pwm);

MotorPWM_Status_t MotorPWM_Init(
        MotorPWM_Handle_t *motor_pwm,
        TIM_HandleTypeDef *htim,
        const MotorPWM_Config_t *config);

#endif /* DEVICES_INC_MOTOR_PWM_H_ */
