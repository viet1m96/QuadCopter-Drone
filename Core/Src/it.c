/*
 * init.c
 *
 *  Created on: Jun 22, 2026
 *      Author: vietht-hl
 */

#include "stm32f4xx_hal.h"


extern UART_HandleTypeDef husart1;
extern DMA_HandleTypeDef hdma2_usart1_rx;


void Error_Handler(void) {

}


void DMA2_Stream2_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma2_usart1_rx);
}

void USART1_IRQHandler(void) {
	HAL_UART_IRQHandler(&husart1);
}
