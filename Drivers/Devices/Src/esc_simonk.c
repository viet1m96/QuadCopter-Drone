#include "esc_simonk.h"

#include <stddef.h>

static uint8_t esc_is_initialized(
        const ESC_Handle_t *esc)
{
    return esc != NULL
            && esc->initialized != 0U
            && esc->motor_pwm != NULL;
}

static uint8_t esc_is_valid_throttle(
        float throttle)
{
    return throttle >= 0.0f
            && throttle <= 1.0f;
}

static uint16_t esc_throttle_to_pulse_us(
        const ESC_Handle_t *esc,
        float throttle)
{
    const float pulse =
            (float)esc->config.idle_pulse_us
            + throttle
            * (float)(
                    esc->config.max_pulse_us
                    - esc->config.idle_pulse_us);

    return (uint16_t)(pulse + 0.5f);
}

ESC_Status_t ESC_Init(
        ESC_Handle_t *esc,
        MotorPWM_Handle_t *motor_pwm,
        const ESC_Config_t *config)
{
    if (esc == NULL
            || motor_pwm == NULL
            || config == NULL) {
        return ESC_ERR_NULL;
    }

    if (motor_pwm->htim == NULL
            || motor_pwm->state
                    == MOTOR_PWM_STATE_UNINITIALIZED
            || motor_pwm->state
                    == MOTOR_PWM_STATE_FAULT) {
        return ESC_ERR_MOTOR_PWM;
    }

    if (config->stop_pulse_us == 0U
            || config->stop_pulse_us
                    > config->idle_pulse_us
            || config->idle_pulse_us
                    >= config->max_pulse_us
            || config->max_pulse_us
                    >= motor_pwm->config.frame_period_us) {
        return ESC_ERR_INVALID_CONFIG;
    }

    esc->motor_pwm = motor_pwm;
    esc->config = *config;
    esc->initialized = 1U;

    return ESC_OK;
}

ESC_Status_t ESC_Start(
        ESC_Handle_t *esc)
{
    if (esc == NULL) {
        return ESC_ERR_NULL;
    }

    if (!esc_is_initialized(esc)) {
        return ESC_ERR_UNINITIALIZED;
    }

    const MotorPWM_Status_t status =
            MotorPWM_Start(
                    esc->motor_pwm,
                    esc->config.stop_pulse_us);

    if (status != MOTOR_PWM_OK) {
        return ESC_ERR_MOTOR_PWM;
    }

    return ESC_OK;
}

ESC_Status_t ESC_WriteStop(
        ESC_Handle_t *esc)
{
    if (esc == NULL) {
        return ESC_ERR_NULL;
    }

    if (!esc_is_initialized(esc)) {
        return ESC_ERR_UNINITIALIZED;
    }

    const MotorPWM_Status_t status =
            MotorPWM_WriteAllSameUs(
                    esc->motor_pwm,
                    esc->config.stop_pulse_us);

    if (status != MOTOR_PWM_OK) {
        return ESC_ERR_MOTOR_PWM;
    }

    return ESC_OK;
}

ESC_Status_t ESC_SetThrottleAll(
        ESC_Handle_t *esc,
        const float throttles[MOTOR_PWM_QUANTITY])
{
    if (esc == NULL || throttles == NULL) {
        return ESC_ERR_NULL;
    }

    if (!esc_is_initialized(esc)) {
        return ESC_ERR_UNINITIALIZED;
    }

    uint16_t pulses[MOTOR_PWM_QUANTITY];

    for (uint32_t i = 0U;
            i < MOTOR_PWM_QUANTITY;
            ++i) {

        if (!esc_is_valid_throttle(throttles[i])) {
            return ESC_ERR_INVALID_THROTTLE;
        }

        pulses[i] =
                esc_throttle_to_pulse_us(
                        esc,
                        throttles[i]);
    }

    const MotorPWM_Status_t status =
            MotorPWM_WriteAllUs(
                    esc->motor_pwm,
                    pulses);

    if (status != MOTOR_PWM_OK) {
        return ESC_ERR_MOTOR_PWM;
    }

    return ESC_OK;
}

ESC_Status_t ESC_SetThrottleAllSame(
        ESC_Handle_t *esc,
        float throttle)
{
    if (esc == NULL) {
        return ESC_ERR_NULL;
    }

    if (!esc_is_initialized(esc)) {
        return ESC_ERR_UNINITIALIZED;
    }

    if (!esc_is_valid_throttle(throttle)) {
        return ESC_ERR_INVALID_THROTTLE;
    }

    const uint16_t pulse_us =
            esc_throttle_to_pulse_us(
                    esc,
                    throttle);

    const MotorPWM_Status_t status =
            MotorPWM_WriteAllSameUs(
                    esc->motor_pwm,
                    pulse_us);

    if (status != MOTOR_PWM_OK) {
        return ESC_ERR_MOTOR_PWM;
    }

    return ESC_OK;
}

ESC_Status_t ESC_Stop(
        ESC_Handle_t *esc)
{
    if (esc == NULL) {
        return ESC_ERR_NULL;
    }

    if (!esc_is_initialized(esc)) {
        return ESC_ERR_UNINITIALIZED;
    }

    const MotorPWM_Status_t status =
            MotorPWM_Stop(
                    esc->motor_pwm);

    if (status != MOTOR_PWM_OK) {
        return ESC_ERR_MOTOR_PWM;
    }

    return ESC_OK;
}
