/*
 * peripherals.h
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#ifndef INC_PERIPHERALS_H_
#define INC_PERIPHERALS_H_
#include "stm32f4xx_hal.h"
extern UART_HandleTypeDef husart2;
extern UART_HandleTypeDef husart1;
extern DMA_HandleTypeDef hdma2_usart1_rx;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef hal_tick_timer;
extern I2C_HandleTypeDef hi2c1;

uint32_t PrecisionTimer_GetUs(void);
#endif /* INC_PERIPHERALS_H_ */
