/*
 * callbacks_list.c
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#include "stm32f4xx_hal.h"
#include "callbacks_list.h"

static IBUS_Handle_t callback_ibus;
static QueueHandle_t callback_queue;


void CallBack_Init(
		IBUS_Handle_t* ibus,
		QueueHandle_t* queue) {
	if(ibus == NULL || queue == NULL) {
		//TODO
		return;
	}
	callback_ibus = *ibus;
	callback_queue = *queue;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM6) {
		HAL_IncTick();
	}
}

void HAL_UARTEx_RxEventCallback(
        UART_HandleTypeDef *huart,
        uint16_t size)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    IBUS_Status_t status = IBUS_OnRxEvent(
        &callback_ibus,
        HAL_GetTick(),
        size
    );
    if (status != IBUS_OK)
    {
        return;
    }


    if (callback_queue == NULL)
    {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    xQueueOverwriteFromISR(
        callback_queue,
        &callback_ibus.latest_valid_data,
        &higher_priority_task_woken
    );
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
