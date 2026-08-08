/*
 * main.h
 *
 *  Created on: Jun 22, 2026
 *      Author: vietht-hl
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "float.h"
#include "math.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

#define I2C_CLOCK_SPEED_SM 100000
#define I2C_CLOCK_SPEED_FM 400000
#define TIM3_PRESCALER 89U
#define TIM3_PERIOD 19999U

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
