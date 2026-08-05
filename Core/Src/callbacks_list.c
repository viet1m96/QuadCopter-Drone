/*
 * callbacks_list.c
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#include "stm32f4xx_hal.h"
#include "callbacks_list.h"
#include "string.h"
#include "hal_ibus_transport.h"
#include "stdio.h"

static ReceiverTask_Context_t* receiver_ctx;


void Callbacks_Init(
		ReceiverTask_Context_t* rcv_ctx) {
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
	printf("Here");
    if(receiver_ctx == NULL ||
       receiver_ctx->transport == NULL) {
    	return;
    }
    if(huart == receiver_ctx->transport->huart) {
    	IBUS_RawFrame_t frame;
    	BaseType_t woken = pdFALSE;
    	if(size == IBUS_FRAME_SIZE) {
    		memcpy(frame.bytes,
    				receiver_ctx->transport->rx_buffer,
					IBUS_FRAME_SIZE);
    		(void) HAL_IBUS_TransportStart(receiver_ctx->transport);
    		(void) xQueueOverwriteFromISR(receiver_ctx->raw_frame_queue,
    									  &frame,
										  &woken);
    	} else {
    		HAL_IBUS_TransportStart(receiver_ctx->transport);
    	}
    	portYIELD_FROM_ISR(woken);
    }

}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {

}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {

}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {

}
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef* hi2c) {

}

