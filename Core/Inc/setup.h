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
#include "peripherals.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "tasks_list.h"

void SystemClockConfig();
void USART2_UART_Init();
void USART1_UART_Init();
void DMA_UART1_Init();
void TIM3_Init();
uint8_t IBUS_Setup(IBUS_Handle_t* ibus);
uint8_t RCInput_Setup(RCInput_Handle_t* rc_inp);
uint8_t MotorPWM_Setup(MotorPWM_Handle_t* motor_pwm);
uint8_t ReceiverTask_Setup(
		ReceiverTask_Context_t* receiver_ctx,
		IBUS_Handle_t* receiver_ibus,
		RCInput_Handle_t* rc_inp,
		QueueHandle_t receiver_queue,
		QueueHandle_t process_queue);

#endif /* INC_SETUP_H_ */
