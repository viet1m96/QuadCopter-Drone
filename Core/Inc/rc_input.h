/*
 * rc_input.h
 *
 *  Created on: Jul 27, 2026
 *      Author: vietht-hl
 */

#ifndef INC_RC_INPUT_H_
#define INC_RC_INPUT_H_

#include "stdint.h"
#include "ibus.h"
typedef enum {
	RC_INPUT_OK = 0,
	RC_INPUT_ERR_NULL,
	RC_INPUT_ERR_INVALID_THROTTLE_CONF,
	RC_INPUT_ERR_INVALID_AXIS_CONF,
	RC_INPUT_ERR_DUP_CHANNEL,
	RC_INPUT_ERR_UNINITIALIZED
} RCInput_Status_t;

typedef enum {
	RC_MODE_RATE = 0,
	RC_MODE_ANGLE,
	RC_MODE_ALTITUDE_HOLD,
	RC_MODE_FAILSAFE
} RCInput_Mode_t;

typedef struct {
	uint8_t channel_idx;
	uint16_t min;
	uint16_t max;
	uint8_t reversed;
} RCInput_ThrottleConfig_t;

typedef struct {
	uint8_t channel_idx;
	uint16_t min;
	uint16_t center;
	uint16_t max;

	uint16_t deadband;
	uint8_t reversed;
} RCInput_AxisConfig_t;

typedef struct {
	RCInput_ThrottleConfig_t throttle;
	RCInput_AxisConfig_t roll;
	RCInput_AxisConfig_t pitch;
	RCInput_AxisConfig_t yaw;
} RCInput_Config_t;

typedef struct {
	float throttle;
	float roll;
	float pitch;
	float yaw;
	RCInput_Mode_t mode;
	uint8_t failsafe_active;
	uint32_t timestamp_ms;
} RCInput_Command_t;

typedef struct {
	RCInput_Config_t config;
	uint8_t initialized;
} RCInput_Handle_t;

RCInput_Status_t RCInput_Init(
		RCInput_Handle_t* rc,
		const RCInput_Config_t* config);

RCInput_Status_t RCInput_Convert(
		const RCInput_Handle_t* rc,
		const IBUS_Data_t* data,
		RCInput_Command_t* command);


#endif /* INC_RC_INPUT_H_ */
