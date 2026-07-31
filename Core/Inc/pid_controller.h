/*
 * pid_controller.h
 *
 *  Created on: Jul 28, 2026
 *      Author: vietht-hl
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#include "stdint.h"

#define PID_CONTROLLER_TWO_PI    6.28318530718f

typedef enum {
	PID_OK = 0,
	PID_ERR_NULL,
	PID_ERR_INVALID_INPUT,
	PID_ERR_INVALID_DTS,
	PID_ERR_UNINITIALIZED,
	PID_ERR_INVALID_CONFIG
} PID_Status_t;


typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float integral_limit;
	float output_limit;
	float derivative_cut_of_hz;
} PID_Config_t;

typedef struct {
	PID_Config_t config;
	float integral;
	float prev_measurement;
	float derivative_filtered;
	uint8_t has_prev_measurement;
	uint8_t initialized;
} PID_Handle_t;


PID_Status_t PID_Update(
		PID_Handle_t* pid,
		float set_point,
		float measurement,
		float dt_s,
		float* output);
PID_Status_t PID_Reset(PID_Handle_t* pid);
PID_Status_t PID_Init(
		PID_Handle_t* pid,
		const PID_Config_t* config);




#endif /* INC_PID_CONTROLLER_H_ */
