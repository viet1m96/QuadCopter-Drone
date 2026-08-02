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
