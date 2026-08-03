/*
 * callbacks_list.c
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#include "stm32f4xx_hal.h"
#include "callbacks_list.h"

static ReceiverTask_Context_t receiver_ctx;


void Callbacks_Init(
		ReceiverTask_Context_t rcv_ctx) {

	if(rcv_ctx.receiver_ibus == NULL ||
	   rcv_ctx.receiver_queue == NULL) {
		//TODO
		return;
	}
	receiver_ctx = rcv_ctx;
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
    if (huart->Instance == USART1)
    {
    	IBUS_Status_t status = IBUS_OnRxEvent(
    		receiver_ctx.receiver_ibus,
			HAL_GetTick(),
			size
		);
		if (status != IBUS_OK)
		{
			return;
		}


		if (receiver_ctx.receiver_queue == NULL)
		{
			return;
		}

		BaseType_t higher_priority_task_woken = pdFALSE;
		xQueueOverwriteFromISR(
			receiver_ctx.receiver_queue,
			&receiver_ctx.receiver_ibus->latest_valid_data,
			&higher_priority_task_woken
		);
		portYIELD_FROM_ISR(higher_priority_task_woken);
    }


}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {

}

