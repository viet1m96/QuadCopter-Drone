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

static void receiver_start_transport(ReceiverTask_Context_t* context) {
	if(context == NULL ||
	   context->transport == NULL) {
		return;
	}

	(void)HAL_IBUS_TransportStop(context->transport);
	(void)xQueueReset(context->raw_frame_queue);
	while (HAL_IBUS_TransportStart(context->transport) != HAL_IBUS_TRANSPORT_OK) {
		vTaskDelay(
				pdMS_TO_TICKS(
						RECEIVER_START_RETRY_MS));
	}

}

static void start_failsafe_protocol(
		ReceiverTask_Context_t* context,
		RCInput_Command_t* command,
		TickType_t* last_sent_frame) {
	printf("Sending failsafe...\r\n");
	receiver_make_failsafe_command(command);
	receiver_publish_command(context, command);
	receiver_start_transport(context);
	*last_sent_frame = xTaskGetTickCount();
}


static void ReceiverTask(void *argument) {

	ReceiverTask_Context_t *context = (ReceiverTask_Context_t *)argument;
	IBUS_RawFrame_t raw;
	IBUS_Data_t data;
	RCInput_Command_t command;
	TickType_t last_sent_frame;
	start_failsafe_protocol(context, &command, &last_sent_frame);
	TickType_t wait_ticks;
	TickType_t now;
	TickType_t elapsed;
	IBUS_Status_t decode_status;
	for(;;) {
		wait_ticks = context->timeout_ticks;
		now = xTaskGetTickCount();
		elapsed = now - last_sent_frame;
		if(elapsed >= context->timeout_ticks) {
			start_failsafe_protocol(context, &command, &last_sent_frame);
		} else {
			wait_ticks = context->timeout_ticks - elapsed;
		}
		uint32_t events = 0U;

		const BaseType_t notified = xTaskNotifyWait(
													0U,
													RECEIVER_EVENT_FRAME_READY |
													RECEIVER_EVENT_UART_ERROR,
													&events,
													wait_ticks);
		if(notified != pdPASS) {
			start_failsafe_protocol(context, &command, &last_sent_frame);
			continue;
		}
		if((events & RECEIVER_EVENT_UART_ERROR) != 0U) {
			start_failsafe_protocol(context, &command, &last_sent_frame);
		}
		if((events & RECEIVER_EVENT_FRAME_READY) != 0U) {
			if(xQueueReceive(
					context->raw_frame_queue,
					&raw,
					0U) == pdPASS) {
				decode_status = IBUS_DecodeFrame(
				            			raw.bytes,
										IBUS_FRAME_SIZE,
										&data);
				if (decode_status != IBUS_OK) {
					continue;
				}
				(void)RCInput_Convert(
						context->rc_input,
						&data,
						&command);
				receiver_publish_command(context, &command);
				last_sent_frame = xTaskGetTickCount();
//				printf("Throttle: %f | Roll: %f | Pitch: %f | Yaw: %f \r\n",
//						command.throttle,
//						command.roll,
//						command.pitch,
//						command.yaw);
			}
		}




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
				&receiver_ctx->task_handle);
}


