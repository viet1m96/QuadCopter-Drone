/*
 * init.c
 *
 *  Created on: Jun 22, 2026
 *      Author: vietht-hl
 */

#include "stm32f4xx_hal.h"
#include "peripherals.h"


void Error_Handler(void) {

}


void DMA2_Stream2_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma2_usart1_rx);
}

void USART1_IRQHandler(void) {
	HAL_UART_IRQHandler(&husart1);
}

void TIM6_DAC_IRQHandler(void) {
	HAL_TIM_IRQHandler(&hal_tick_timer);
}

void EXTI15_10_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
}

void I2C1_EV_IRQHandler(void) {
	HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void) {
	HAL_I2C_ER_IRQHandler(&hi2c1);
}
