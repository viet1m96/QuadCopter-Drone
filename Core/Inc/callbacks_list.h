/*
 * callbacks_list.h
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#ifndef INC_CALLBACKS_LIST_H_
#define INC_CALLBACKS_LIST_H_
#include "FreeRTOS.h"
#include "ibus.h"
#include "queue.h"
#include "receiver_task.h"
#include "sensor_task.h"
#include "task.h"

void Callbacks_Init(ReceiverTask_Context_t *rcv_ctx,
                    SensorTask_Context_t *sen_ctx);

#endif /* INC_CALLBACKS_LIST_H_ */
