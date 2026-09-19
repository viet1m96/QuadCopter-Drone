/*
 * motor_mixer.c
 *
 *  Created on: Jul 27, 2026
 *      Author: vietht-hl
 */

#include "motor_mixer.h"
#include "math.h"
#include "stddef.h"
#include "stdint.h"

static void
motor_mixer_calculate_corrections(const AxisCorrection_t *axis,
                                  float correction[MOTOR_PWM_QUANTITY]) {
  correction[MOTOR_FRONT_LEFT] = axis->roll - axis->pitch - axis->yaw;
  correction[MOTOR_FRONT_RIGHT] = -axis->roll - axis->pitch + axis->yaw;
  correction[MOTOR_REAR_RIGHT] = -axis->roll + axis->pitch - axis->yaw;
  correction[MOTOR_REAR_LEFT] = axis->roll + axis->pitch + axis->yaw;
}

static float motor_mixer_clamp(float value, float min, float max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

static void
motor_mixer_desaturate_corrections(float base_throttle,
                                   float correction[MOTOR_PWM_QUANTITY]) {
  float min_tmp = correction[0];
  float max_tmp = correction[0];
  for (uint32_t i = 1; i < MOTOR_PWM_QUANTITY; i++) {
    min_tmp = (min_tmp < correction[i]) ? min_tmp : correction[i];
    max_tmp = (max_tmp > correction[i]) ? max_tmp : correction[i];
  }

  float scale = 1.0f;
  float range = max_tmp - min_tmp;

  if (range > 1.0f) {
    scale = 1.0f / range;
    min_tmp *= scale;
    max_tmp *= scale;
  }

  float adjusted_throttle = base_throttle;

  if (adjusted_throttle + max_tmp > 1.0f) {
    adjusted_throttle = 1.0f - max_tmp;
  }

  if (adjusted_throttle + min_tmp < 0.0f && min_tmp < 0.0f) {
    float lower_scale = adjusted_throttle / -min_tmp;
    scale *= motor_mixer_clamp(lower_scale, 0.0f, 1.0f);
  }

  for (uint32_t i = 0; i < MOTOR_PWM_QUANTITY; i++) {
    correction[i] = adjusted_throttle + correction[i] * scale;
    correction[i] = motor_mixer_clamp(correction[i], 0.0f, 1.0f);
  }
}

static uint8_t motor_mixer_is_axis_valid(float value) {
  return isfinite(value) && value >= -1.0f && value <= 1.0f;
}
static uint8_t motor_mixer_is_input_valid(float base_throttle,
                                          const AxisCorrection_t *correction) {
  if (!isfinite(base_throttle)) {
    return 0U;
  }

  if ((base_throttle < 0.0f) || (base_throttle > 1.0f)) {
    return 0U;
  }

  if (!motor_mixer_is_axis_valid(correction->roll) ||
      !motor_mixer_is_axis_valid(correction->pitch) ||
      !motor_mixer_is_axis_valid(correction->yaw)) {
    return 0U;
  }

  return 1U;
}

MotorMixer_Status_t MotorMixer_Mix(float base_throttle,
                                   const AxisCorrection_t *correction,
                                   MotorMixer_Output_t *output) {
  if (correction == NULL || output == NULL) {
    return MIX_ERR_NULL;
  }
  if (!motor_mixer_is_input_valid(base_throttle, correction)) {
    return MIX_ERR_INVALID_CONFIG;
  }
  motor_mixer_calculate_corrections(correction, output->motor);
  motor_mixer_desaturate_corrections(base_throttle, output->motor);
  return MIX_OK;
}
