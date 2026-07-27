/*
 * motor_mixer.h
 *
 *  Created on: Jul 27, 2026
 *      Author: vietht-hl
 */

#ifndef INC_MOTOR_MIXER_H_
#define INC_MOTOR_MIXER_H_

#include "motor_common.h"
#include "control_common.h"

typedef enum {
	MIX_OK = 0,
	MIX_ERR_NULL,
	MIX_ERR_INVALID_CONFIG
} MotorMixer_Status_t;

typedef struct {
	float motor[MOTOR_PWM_QUANTITY];
} MotorMixer_Output_t;


MotorMixer_Status_t MotorMixer_Mix(
		float base_throttle,
		const AxisCorrection_t* correction,
		MotorMixer_Output_t* output);



#endif /* INC_MOTOR_MIXER_H_ */
