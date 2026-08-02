/*
 * tasks.h
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#ifndef INC_TASKS_LIST_H_
#define INC_TASKS_LIST_H_
#include "FreeRTOS.h"
#include "task.h"
#include "ibus.h"
#include "queue.h"
#include "rc_input.h"

typedef struct {
	IBUS_Handle_t* receiver_ibus;
	RCInput_Handle_t* rc_inp;
	QueueHandle_t receiver_queue;
	QueueHandle_t process_queue;
} ReceiverTask_Context_t;

BaseType_t ReceiverTask_Create(ReceiverTask_Context_t* receiver_ctx);

#endif /* INC_TASKS_LIST_H_ */
