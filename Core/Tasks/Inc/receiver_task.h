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


#define RECEIVER_EVENT_FRAME_READY (1UL << 0)
#define RECEIVER_EVENT_UART_ERROR  (1UL << 1)



typedef struct {
	HAL_IBUS_Transport_t *transport;
	QueueHandle_t raw_frame_queue;
	QueueHandle_t command_queue;
	RCInput_Handle_t* rc_input;
	TickType_t timeout_ticks;
	TaskHandle_t task_handle;
} ReceiverTask_Context_t;


BaseType_t ReceiverTask_Create(ReceiverTask_Context_t* receiver_ctx);


#endif /* TASKS_INC_RECEIVER_TASK_H_ */
