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
#include "peripherals.h"

static ReceiverTask_Context_t* receiver_ctx;
static SensorTask_Context_t* sensor_ctx;
void Callbacks_Init(
		ReceiverTask_Context_t* rcv_ctx,
		SensorTask_Context_t* sen_ctx) {
	receiver_ctx = rcv_ctx;
	sensor_ctx = sen_ctx;
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
    if(receiver_ctx != NULL &&
       receiver_ctx->transport != NULL &&
	   receiver_ctx->transport->huart &&
	   receiver_ctx->transport->huart == huart) {
    	BaseType_t higher_priority_task_woken = pdFALSE;
    	uint32_t events = 0U;
    	if(size == IBUS_FRAME_SIZE) {
    		IBUS_RawFrame_t frame;
    		memcpy(frame.bytes,
    			   receiver_ctx->transport->rx_buffer,
				   IBUS_FRAME_SIZE);
    		(void)xQueueOverwriteFromISR(
    									receiver_ctx->raw_frame_queue,
										&frame,
										&higher_priority_task_woken);
    		events |= RECEIVER_EVENT_FRAME_READY;
    	}
    	if(HAL_IBUS_TransportStart(receiver_ctx->transport) != HAL_IBUS_TRANSPORT_OK) {
    		events |= RECEIVER_EVENT_UART_ERROR;
    	}
    	if(events != 0U && receiver_ctx->task_handle != NULL) {
    		(void)xTaskNotifyFromISR(
    								receiver_ctx->task_handle,
									events,
									eSetBits,
									&higher_priority_task_woken);
    	}
    	portYIELD_FROM_ISR(higher_priority_task_woken);
    }

}

void HAL_UART_ErrorCallback(
        UART_HandleTypeDef *huart)
{
	if(receiver_ctx != NULL &&
	   receiver_ctx->transport != NULL &&
	   receiver_ctx->transport->huart &&
	   receiver_ctx->transport->huart == huart) {
	    BaseType_t higher_priority_task_woken = pdFALSE;
		if (receiver_ctx->task_handle != NULL) {
			(void)xTaskNotifyFromISR(
					receiver_ctx->task_handle,
					RECEIVER_EVENT_UART_ERROR,
					eSetBits,
					&higher_priority_task_woken);
		}
		portYIELD_FROM_ISR(higher_priority_task_woken);
	}


}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {
	if(gpio_pin == GPIO_PIN_12) {
		if(sensor_ctx == NULL) return;
		sensor_ctx->imu_pending = 1U;
	}
}

void HAL_I2C_MemRxCpltCallback(
        I2C_HandleTypeDef *hi2c)
{

}

void HAL_I2C_ErrorCallback(
        I2C_HandleTypeDef *hi2c)
{

}

void HAL_I2C_AbortCpltCallback(
		I2C_HandleTypeDef* hi2c) {

}

