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

void CallBack_Init(
		IBUS_Handle_t* ibus,
		QueueHandle_t* queue);

#endif /* INC_CALLBACKS_LIST_H_ */
