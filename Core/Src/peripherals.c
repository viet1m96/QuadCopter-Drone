/*
 * peripherals.c
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#include "peripherals.h"

UART_HandleTypeDef husart2;
UART_HandleTypeDef husart1;
DMA_HandleTypeDef hdma2_usart1_rx;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef hal_tick_timer;
TIM_HandleTypeDef htim5;
I2C_HandleTypeDef hi2c1;

uint32_t PrecisionTimer_GetUs(void) { return __HAL_TIM_GET_COUNTER(&htim5); }
