/*
 * tasks.c
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#include "receiver_task.h"
#include "stddef.h"
#include "string.h"
#include "stdio.h"

#define RECEIVER_START_RETRY_MS    100U

static void receiver_make_failsafe_command(
		RCInput_Command_t* command) {
	if(command == NULL) return;
	memset(command, 0, sizeof(*command));

	command->throttle = 0.0f;
	command->roll = 0.0f;
	command->pitch = 0.0f;
	command->yaw = 0.0f;
	command->mode = RC_MODE_FAILSAFE;
}

static void receiver_publish_command(
		ReceiverTask_Context_t* ctx,
		const RCInput_Command_t* cmd) {
	if(ctx == NULL || cmd == NULL) return;
	(void)xQueueOverwrite(ctx->command_queue, cmd);
}


static void ReceiverTask(void *argument) {

	ReceiverTask_Context_t *context = (ReceiverTask_Context_t *)argument;
	IBUS_RawFrame_t raw;
	IBUS_Data_t data;
	RCInput_Command_t command;
	receiver_make_failsafe_command(&command);
	receiver_publish_command(context, &command);

	while(HAL_IBUS_TransportStart(context->transport) != HAL_IBUS_TRANSPORT_OK) {
		vTaskDelay(pdMS_TO_TICKS(RECEIVER_START_RETRY_MS));
	}
	TickType_t last_valid_frame_ticks = xTaskGetTickCount();
	IBUS_Status_t decode_status;
	for(;;) {
		TickType_t wait_ticks;

		if(command.mode == RC_MODE_FAILSAFE) {
			wait_ticks = portMAX_DELAY;
		} else {
			const TickType_t now = xTaskGetTickCount();
			const TickType_t elapsed = now - last_valid_frame_ticks;
			if(elapsed >= context->timeout_ticks) {
				wait_ticks = 0U;
			} else {
				wait_ticks = context->timeout_ticks - elapsed;
			}
		}

		const BaseType_t received = xQueueReceive(
				context->raw_frame_queue,
				&raw,
				wait_ticks);
		if(received != pdPASS) {
			receiver_make_failsafe_command(&command);
			receiver_publish_command(context, &command);
			continue;
		}

		decode_status = IBUS_DecodeFrame(
						raw.bytes,
						IBUS_FRAME_SIZE,
						&data);
		if(decode_status != IBUS_OK) {
			continue;
		}

		last_valid_frame_ticks = xTaskGetTickCount();
		(void)RCInput_Convert(
				context->rc_input,
				&data,
				&command);
		printf("Throttle: %f | Roll: %f | Pitch: %f | Yaw:  %f \r\n",
				command.throttle,
				command.roll,
				command.pitch,
				command.yaw);
		receiver_publish_command(context, &command);
	}

}



BaseType_t ReceiverTask_Create(ReceiverTask_Context_t* receiver_ctx) {
	if(receiver_ctx == NULL ||
	   receiver_ctx->transport == NULL ||
	   receiver_ctx->rc_input == NULL ||
	   receiver_ctx->raw_frame_queue == NULL ||
	   receiver_ctx->command_queue == NULL ||
	   receiver_ctx->timeout_ticks == 0U) {
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


