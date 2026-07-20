/*
 * init.c
 *
 *  Created on: Jun 22, 2026
 *      Author: vietht-hl
 */

#include "stm32f4xx_hal.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void EXTI1_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}
