/*
 * rc_input.c
 *
 *  Created on: Jul 27, 2026
 *      Author: vietht-hl
 */

#include "rc_input.h"
#include "control_common.h"


static uint8_t rc_input_validate_throttle(const RCInput_ThrottleConfig_t* config) {
	if(config->channel_idx >= IBUS_CHANNEL_COUNT) return 0U;
	if(config->min >= config->max) return 0U;
	return 1U;
}

static uint8_t rc_input_validate_axis(const RCInput_AxisConfig_t* config) {
	if(config->channel_idx >= IBUS_CHANNEL_COUNT) return 0U;
	if (config->min >= config->center) {
	    return 0U;
	}
	if (config->center >= config->max) {
	    return 0U;
	}
	if (config->deadband >= (config->center - config->min)) {
	    return 0U;
	}
	if (config->deadband >= (config->max - config->center)) {
	    return 0U;
	}
	return 1U;
}

static uint8_t rc_input_validate_dup_channel(const RCInput_Config_t* config) {
	return(
		   config->throttle.channel_idx != config->roll.channel_idx &&
		   config->throttle.channel_idx != config->pitch.channel_idx &&
		   config->throttle.channel_idx != config->yaw.channel_idx &&
		   config->roll.channel_idx != config->pitch.channel_idx &&
		   config->roll.channel_idx != config->yaw.channel_idx &&
		   config->pitch.channel_idx != config->yaw.channel_idx);
}


RCInput_Status_t RCInput_Init(
		RCInput_Handle_t* rc,
		const RCInput_Config_t* config) {
	if(rc == NULL || config == NULL) return RC_INPUT_ERR_NULL;
	rc->initialized = 0U;
	if(!(rc_input_validate_throttle(&config->throttle))) {
		return RC_INPUT_ERR_INVALID_THROTTLE_CONF;
	}
	if(!(rc_input_validate_axis(&config->roll))) {
		return RC_INPUT_ERR_INVALID_AXIS_CONF;
	}
	if(!(rc_input_validate_axis(&config->pitch))) {
		return RC_INPUT_ERR_INVALID_AXIS_CONF;
	}
	if(!(rc_input_validate_axis(&config->yaw))) {
		return RC_INPUT_ERR_INVALID_AXIS_CONF;
	}
	if(!(rc_input_validate_dup_channel(config))) {
		return RC_INPUT_ERR_DUP_CHANNEL;
	}
	rc->config = *config;
	rc->initialized = 1U;
	return RC_INPUT_OK;
}

static float rc_input_clamp(
		float value,
		float min,
		float max) {
	if(value < min) return min;
	if(value > max) return max;
	return value;
}

static float rc_input_normalize_throttle(
		const RCInput_ThrottleConfig_t* throttle,
		uint16_t raw) {
	float numerator =
	    (float)((int32_t)raw - (int32_t)throttle->min);

	float denominator =
	    (float)((int32_t)throttle->max - (int32_t)throttle->min);

	float normalized = numerator / denominator;

	normalized = rc_input_clamp(normalized, 0.0f, 1.0f);
	if (throttle->reversed != 0U) {
		normalized = 1.0f - normalized;
	}
	return normalized;
}

static float rc_input_normalize_axis(
		const RCInput_AxisConfig_t* axis,
		uint16_t raw) {
	int32_t raw_value = (int32_t)raw;
	int32_t center = (int32_t)axis->center;
	int32_t deadband = (int32_t)axis->deadband;

	int32_t lower_deadband = center - deadband;
	int32_t upper_deadband = center + deadband;

	float normalized;

	if ((raw_value >= lower_deadband) &&
		(raw_value <= upper_deadband)) {
		normalized = 0.0f;
	} else if (raw_value < lower_deadband) {
		normalized =
			(float)(raw_value - lower_deadband) /
			(float)(lower_deadband - (int32_t)axis->min);
	} else {
		normalized =
			(float)(raw_value - upper_deadband) /
			(float)((int32_t)axis->max - upper_deadband);
	}

	normalized = rc_input_clamp(normalized, -1.0f, 1.0f);
	if (axis->reversed != 0U) {
		normalized = -normalized;
	}
	return normalized;
}

static uint8_t rc_input_detect_failsafe(uint16_t raw) {
	return (raw == FAILSAFE_THROTTLE) ? 1U : 0U;
}

static RCInput_Mode_t rc_input_detect_mode(uint16_t raw) {
	if(raw == FLIGHT_MODE_RATE) return RC_MODE_RATE;
	if(raw == FLIGHT_MODE_ANGLE) return RC_MODE_ANGLE;
	if(raw == FLIGHT_MODE_ALTITUDE_HOLD) return RC_MODE_ALTITUDE_HOLD;
	return 0;
}

RCInput_Status_t RCInput_Convert(
		const RCInput_Handle_t* rc,
		const IBUS_Data_t* data,
		RCInput_Command_t* command) {
	if(rc == NULL || data == NULL || command == NULL) {
		return RC_INPUT_ERR_NULL;
	}
	if(rc->initialized != 1U) return RC_INPUT_ERR_UNINITIALIZED;
	command->throttle = rc_input_normalize_throttle(
													&rc->config.throttle,
													data->channels[rc->config.throttle.channel_idx]);

	command->roll = rc_input_normalize_axis(
										    &rc->config.roll,
											data->channels[rc->config.roll.channel_idx]);

	command->pitch = rc_input_normalize_axis(
											&rc->config.pitch,
											data->channels[rc->config.pitch.channel_idx]);
	command->yaw = rc_input_normalize_axis(
											&rc->config.yaw,
											data->channels[rc->config.yaw.channel_idx]);

	command->failsafe_active = rc_input_detect_failsafe(data->channels[FAILSAFE_CHANNEL_IDX]);
	command->mode = rc_input_detect_mode(data->channels[FLIGHT_MODE_CHANNEL_IDX]);
	return RC_INPUT_OK;
}
