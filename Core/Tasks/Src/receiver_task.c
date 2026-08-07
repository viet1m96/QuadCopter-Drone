#include "receiver_task.h"
#include "stddef.h"
#include "string.h"
#include "stdio.h"

#define RECEIVER_START_RETRY_MS    100U

static void receiver_make_failsafe_command(
        RCInput_Command_t *command)
{
    if (command == NULL) {
        return;
    }

    memset(
            command,
            0,
            sizeof(*command));

    command->throttle = 0.0f;
    command->roll = 0.0f;
    command->pitch = 0.0f;
    command->yaw = 0.0f;
    command->mode = RC_MODE_FAILSAFE;
}

static void receiver_publish_command(
        ReceiverTask_Context_t *context,
        const RCInput_Command_t *command)
{
    if (context == NULL
            || command == NULL) {
        return;
    }

    (void)xQueueOverwrite(
            context->command_queue,
            command);
}

static void receiver_start_transport(
        ReceiverTask_Context_t *context)
{
    if (context == NULL
            || context->transport == NULL) {
        return;
    }

    (void)HAL_IBUS_TransportStop(
            context->transport);

    (void)xQueueReset(
            context->raw_frame_queue);

    while (HAL_IBUS_TransportStart(
            context->transport)
            != HAL_IBUS_TRANSPORT_OK) {
        vTaskDelay(
                pdMS_TO_TICKS(
                        RECEIVER_START_RETRY_MS));
    }
}

static void start_failsafe_protocol(
        ReceiverTask_Context_t *context,
        RCInput_Command_t *command,
        TickType_t *last_sent_frame)
{

    receiver_make_failsafe_command(
            command);

    receiver_publish_command(
            context,
            command);

    receiver_start_transport(
            context);

    *last_sent_frame =
            xTaskGetTickCount();
}

static uint8_t receiver_start_next_frame(
        ReceiverTask_Context_t *context)
{
    if (context == NULL
            || context->transport == NULL) {
        return 0U;
    }

    return HAL_IBUS_TransportStart(
            context->transport)
            == HAL_IBUS_TRANSPORT_OK;
}

static void ReceiverTask(
        void *argument)
{
    ReceiverTask_Context_t *context =
            (ReceiverTask_Context_t *)argument;

    IBUS_RawFrame_t raw;
    IBUS_Data_t data;
    RCInput_Command_t command;

    TickType_t last_sent_frame;

    start_failsafe_protocol(
            context,
            &command,
            &last_sent_frame);

    for (;;) {
        TickType_t now =
                xTaskGetTickCount();

        TickType_t elapsed =
                now - last_sent_frame;

        TickType_t wait_ticks =
                context->timeout_ticks;

        if (elapsed >= context->timeout_ticks) {
            start_failsafe_protocol(
                    context,
                    &command,
                    &last_sent_frame);

            continue;
        }

        wait_ticks =
                context->timeout_ticks
                - elapsed;

        uint32_t events = 0U;

        BaseType_t notified =
                xTaskNotifyWait(
                        0U,
                        RECEIVER_EVENT_FRAME_READY
                        | RECEIVER_EVENT_UART_ERROR,
                        &events,
                        wait_ticks);

        if (notified != pdPASS) {
            start_failsafe_protocol(
                    context,
                    &command,
                    &last_sent_frame);

            continue;
        }

        if ((events & RECEIVER_EVENT_UART_ERROR) != 0U) {
            start_failsafe_protocol(
                    context,
                    &command,
                    &last_sent_frame);

            continue;
        }

        if ((events & RECEIVER_EVENT_FRAME_READY) == 0U) {
            continue;
        }

        if (xQueueReceive(
                context->raw_frame_queue,
                &raw,
                0U) == pdPASS) {
            IBUS_Status_t decode_status =
                    IBUS_DecodeFrame(
                            raw.bytes,
                            IBUS_FRAME_SIZE,
                            &data);

            if (decode_status == IBUS_OK) {
                (void)RCInput_Convert(
                        context->rc_input,
                        &data,
                        &command);

                receiver_publish_command(
                        context,
                        &command);

                last_sent_frame =
                        xTaskGetTickCount();
            }
        }

        if (!receiver_start_next_frame(context)) {
            start_failsafe_protocol(
                    context,
                    &command,
                    &last_sent_frame);
        }
    }
}

BaseType_t ReceiverTask_Create(
        ReceiverTask_Context_t *receiver_ctx)
{
    if (receiver_ctx == NULL
            || receiver_ctx->transport == NULL
            || receiver_ctx->rc_input == NULL
            || receiver_ctx->raw_frame_queue == NULL
            || receiver_ctx->command_queue == NULL
            || receiver_ctx->timeout_ticks == 0U) {
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
