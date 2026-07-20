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

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "stdio.h"
#include "float.h"
#include "math.h"


extern UART_HandleTypeDef husart2;
extern I2C_HandleTypeDef hi2c1;

#define I2C_CLOCK_SPEED_SM 100000
#define I2C_CLOCK_SPEED_FM 400000

#define MPU6050_ADDRESS 0x68U
#define BMP180_ADDRESS 0x77U
#define HMC5883L_ADDRESS 0x1EU


void Error_Handler(void);
void SystemClockConfig(void);
void I2C1_Init(void);
void USART2_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
