/*
 * tasks.h
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#ifndef TASKS_INC_RECEIVER_TASK_H_
#define TASKS_INC_RECEIVER_TASK_H_
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "ibus.h"
#include "rc_input.h"
#include "hal_ibus_transport.h"



typedef struct {
	HAL_IBUS_Transport_t *transport;
	QueueHandle_t raw_frame_queue;
	QueueHandle_t command_queue;
	RCInput_Handle_t* rc_input;
	TickType_t timeout_ticks;
} ReceiverTask_Context_t;


BaseType_t ReceiverTask_Create(ReceiverTask_Context_t* receiver_ctx);


#endif /* TASKS_INC_RECEIVER_TASK_H_ */
