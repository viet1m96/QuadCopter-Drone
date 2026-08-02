/*
 * pid_controller.c
 *
 *  Created on: Jul 28, 2026
 *      Author: vietht-hl
 */

#include "pid_controller.h"
#include "math.h"



static float pid_derivative_low_pass_filter(
		float prev_filtered,
		float derivative,
		float cut_of_hz,
		float dt_s) {
	if(cut_of_hz <= 0.0f) {
		return derivative;
	}
	float time_constant = 1.0f / (PID_CONTROLLER_TWO_PI * cut_of_hz);
	float alpha = dt_s / (time_constant + dt_s);

	return prev_filtered + alpha *(derivative - prev_filtered);
}

static float pid_clamp_sym(float num, float bound) {
	if(num > bound) return bound;
	if(num < -bound) return -bound;
	return num;
}

static void pid_clear_state(PID_Handle_t* pid) {
	pid->integral = 0.0f;
	pid->prev_measurement = 0.0f;
	pid->derivative_filtered = 0.0f;
	pid->has_prev_measurement = 0U;
}

static PID_Status_t pid_is_valid_config(const PID_Config_t* config) {
	if (!isfinite(config->Kp) ||
		!isfinite(config->Ki) ||
		!isfinite(config->Kd) ||
		!isfinite(config->integral_limit) ||
		!isfinite(config->output_limit) ||
		!isfinite(config->derivative_cut_of_hz)) {
		return 0U;
	}

	if (config->Kp < 0.0f ||
		config->Ki < 0.0f ||
		config->Kd < 0.0f) {
		return 0U;
	}

	if (config->integral_limit < 0.0f) {
		return 0U;
	}

	if (config->output_limit <= 0.0f) {
		return 0U;
	}

	if (config->derivative_cut_of_hz < 0.0f) {
		return 0U;
	}

	return 1U;
}
PID_Status_t PID_Update(
		PID_Handle_t* pid,
		float set_point,
		float measurement,
		float dt_s,
		float* output) {
	if(pid == NULL || output == NULL) return PID_ERR_NULL;
	if(pid->initialized == 0U) return PID_ERR_UNINITIALIZED;
	*output = 0.0f;
	if(!isfinite(set_point) || !isfinite(measurement)) {
		return PID_ERR_INVALID_INPUT;
	}
	if(!isfinite(dt_s) || dt_s <= 0.0f) {
		return PID_ERR_INVALID_DTS;
	}
	float error = set_point - measurement;

	float proportional = pid->config.Kp * error;

	pid->integral += pid->config.Ki * error * dt_s;
	pid->integral = pid_clamp_sym(pid->integral, pid->config.integral_limit);

	if(pid->has_prev_measurement == 0U) {
		pid->prev_measurement = measurement;
		pid->derivative_filtered = 0U;
		pid->has_prev_measurement = 1U;
	} else {
		float measurement_derivative = (measurement - pid->prev_measurement) / dt_s;
		float derivative = -pid->config.Kd * measurement_derivative;
		pid->derivative_filtered = pid_derivative_low_pass_filter(
				pid->derivative_filtered,
				derivative,
				pid->config.derivative_cut_of_hz,
				dt_s);
		pid->prev_measurement = measurement;
	}
	float result = proportional + pid->integral + pid->derivative_filtered;
	result = pid_clamp_sym(result, pid->config.output_limit);
	*output = result;
	return PID_OK;
}

PID_Status_t PID_Reset(PID_Handle_t* pid) {
	if(pid == NULL) return PID_ERR_NULL;
	if(pid->initialized == 0U) return PID_ERR_UNINITIALIZED;
	pid_clear_state(pid);
	return PID_OK;
}

PID_Status_t PID_Init(
		PID_Handle_t* pid,
		const PID_Config_t* config) {
	if(pid == NULL || config == NULL) return PID_ERR_NULL;
	pid->initialized = 0U;
	if(!pid_is_valid_config(config)) return PID_ERR_INVALID_CONFIG;
	pid->config = *config;
	pid_clear_state(pid);
	pid->initialized = 1U;
	return PID_OK;
}



