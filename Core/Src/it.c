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
