/*
 * init.h
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#ifndef INC_SETUP_H_
#define INC_SETUP_H_
#include "stm32f4xx_hal.h"

#include "ibus.h"
#include "rc_input.h"
#include "control_common.h"
#include "motor_pwm.h"


extern UART_HandleTypeDef husart2;
extern UART_HandleTypeDef husart1;
extern DMA_HandleTypeDef hdma2_usart1_rx;
extern TIM_HandleTypeDef htim3;

extern IBUS_Handle_t ibus;
extern RCInput_Handle_t rc_inp;
extern MotorPWM_Handle_t motor_pwm;

void SystemClockConfig();
void USART2_UART_Init();
void USART1_UART_Init();
void DMA_UART1_Init();
void TIM3_Init();
uint8_t IBUS_Setup();
uint8_t RCInput_Setup();
uint8_t MotorPWM_Setup();


#endif /* INC_SETUP_H_ */
