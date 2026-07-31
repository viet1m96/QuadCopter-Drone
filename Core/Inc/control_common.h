/*
 * control_common.h
 *
 *  Created on: Jul 27, 2026
 *      Author: vietht-hl
 */

#ifndef INC_CONTROL_COMMON_H_
#define INC_CONTROL_COMMON_H_

#define MAX_THROTTLE 2000U
#define MIN_THROTTLE 1000U
#define CENTER_THROTTLE 1500U
#define COMMON_DEADBAND 0U;

#define THROTTLE_CHANNEL_IDX 2U
#define ROLL_CHANNEL_IDX 0U
#define PITCH_CHANNEL_IDX 1U
#define YAW_CHANNEL_IDX   3U


typedef struct {
	float roll;
	float pitch;
	float yaw;
} AxisCorrection_t;


#endif /* INC_CONTROL_COMMON_H_ */
