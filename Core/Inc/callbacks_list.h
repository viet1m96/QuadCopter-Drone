/*
 * callbacks_list.h
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#ifndef INC_CALLBACKS_LIST_H_
#define INC_CALLBACKS_LIST_H_
#include "ibus.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "tasks_list.h"

void Callbacks_Init(ReceiverTask_Context_t rcv_ctx);

#endif /* INC_CALLBACKS_LIST_H_ */
