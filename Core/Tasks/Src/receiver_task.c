#include "receiver_task.h"

#include "stddef.h"
#include "stdio.h"
#include "string.h"

#define RECEIVER_START_RETRY_MS 100U

static void receiver_make_failsafe_command(RCInput_Command_t *command) {
  if (command == NULL) {
    return;
  }
  // printf("h\r\n");
  memset(command, 0, sizeof(*command));
  command->throttle = 0.0f;
  command->roll = 0.0f;
  command->pitch = 0.0f;
  command->yaw = 0.0f;
  command->failsafe_active = 1U;
}

static void receiver_publish_command(ReceiverTask_Context_t *context,
                                     const RCInput_Command_t *command) {
  if (context == NULL || command == NULL) {
    return;
  }

  (void)xQueueOverwrite(context->command_queue, command);
}

static void receiver_enter_failsafe(ReceiverTask_Context_t *context,
                                    RCInput_Command_t *command,
                                    TickType_t *last_sent_frame) {
  if (context == NULL || command == NULL || last_sent_frame == NULL) {
    return;
  }

  TickType_t now = xTaskGetTickCount();

  receiver_make_failsafe_command(command);

  command->timestamp_tick = now;

  receiver_publish_command(context, command);

  *last_sent_frame = now;
}

static void receiver_start_transport(ReceiverTask_Context_t *context) {
  if (context == NULL || context->transport == NULL) {
    return;
  }

  (void)xQueueReset(context->raw_frame_queue);

  while (HAL_IBUS_TransportStart(context->transport) != HAL_IBUS_TRANSPORT_OK) {
    vTaskDelay(pdMS_TO_TICKS(RECEIVER_START_RETRY_MS));
  }
}

static void ReceiverTask(void *argument) {
  ReceiverTask_Context_t *context = (ReceiverTask_Context_t *)argument;

  IBUS_RawFrame_t raw;
  IBUS_Data_t data;
  RCInput_Command_t command;

  TickType_t last_sent_frame;

  receiver_enter_failsafe(context, &command, &last_sent_frame);
  receiver_start_transport(context);

  for (;;) {

    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed = now - last_sent_frame;

    if (elapsed >= context->timeout_ticks) {
      receiver_enter_failsafe(context, &command, &last_sent_frame);
      continue;
    }

    TickType_t wait_ticks = context->timeout_ticks - elapsed;

    uint32_t events = 0U;

    BaseType_t notified = xTaskNotifyWait(
        0U, RECEIVER_EVENT_FRAME_READY | RECEIVER_EVENT_UART_ERROR, &events,
        wait_ticks);

    if (notified != pdPASS) {
      receiver_enter_failsafe(context, &command, &last_sent_frame);
      continue;
    }

    if ((events & RECEIVER_EVENT_UART_ERROR) != 0U) {
      receiver_enter_failsafe(context, &command, &last_sent_frame);

      receiver_start_transport(context);

      continue;
    }

    if ((events & RECEIVER_EVENT_FRAME_READY) == 0U) {
      continue;
    }

    if (xQueueReceive(context->raw_frame_queue, &raw, 0U) != pdPASS) {
      continue;
    }

    IBUS_Status_t decode_status =
        IBUS_DecodeFrame(raw.bytes, IBUS_FRAME_SIZE, &data);

    if (decode_status != IBUS_OK) {
      continue;
    }

    (void)RCInput_Convert(context->rc_input, &data, &command);

    command.timestamp_tick = xTaskGetTickCount();

    receiver_publish_command(context, &command);

    last_sent_frame = command.timestamp_tick;
  }
}

BaseType_t ReceiverTask_Create(ReceiverTask_Context_t *receiver_ctx) {
  if (receiver_ctx == NULL || receiver_ctx->transport == NULL ||
      receiver_ctx->rc_input == NULL || receiver_ctx->raw_frame_queue == NULL ||
      receiver_ctx->command_queue == NULL ||
      receiver_ctx->timeout_ticks == 0U) {
    return pdFALSE;
  }

  return xTaskCreate(ReceiverTask, "Receiver", 256U, receiver_ctx, 3U,
                     &receiver_ctx->task_handle);
}
