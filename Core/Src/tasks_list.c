/*
 * tasks.c
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */


#include "tasks_list.h"


static void ReceiverTask(void *argument) {
	IBUS_Data_t frame;
	RCInput_Command_t command;

	ReceiverTask_Context_t* ctx = (ReceiverTask_Context_t*) argument;

	if(IBUS_Start(ctx->receiver_ibus) != IBUS_OK) {
		//TODO
		vTaskDelete(NULL);
	}

	for(;;) {
		BaseType_t received = xQueueReceive(
										ctx->receiver_queue,
										&frame,
										pdMS_TO_TICKS(IBUS_TIMEOUT_MS));
		if(received == pdPASS) {
			RCInput_Status_t status = RCInput_Convert(
												ctx->rc_inp,
												&frame,
												&command);
			if(status != RC_INPUT_OK) {
				//TODO
				continue;
			}
			xQueueOverwrite(ctx->process_queue, &command);
		} else {
			uint32_t now_ms = HAL_GetTick();

			taskENTER_CRITICAL();

			IBUS_Status_t status = IBUS_Update(
			        ctx->receiver_ibus,
			        now_ms);

			taskEXIT_CRITICAL();
			if(status != IBUS_OK) {
				//TODO
			}
		}
	}

}

BaseType_t ReceiverTask_Create(ReceiverTask_Context_t* receiver_ctx) {
	if(receiver_ctx == NULL ||
	   receiver_ctx->receiver_ibus == NULL ||
	   receiver_ctx->rc_inp == NULL ||
	   receiver_ctx->receiver_queue == NULL ||
	   receiver_ctx->process_queue == NULL) {
		return pdFALSE;
	}
	return xTaskCreate(
				ReceiverTask,
				"Receiver",
				256U,
				receiver_ctx,
				3U,
				NULL);
}


