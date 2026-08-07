/*
 * init.h
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#ifndef INC_SETUP_H_
#define INC_SETUP_H_
#include <receiver_task.h>
#include "stm32f4xx_hal.h"

#include "ibus.h"
#include "rc_input.h"
#include "control_common.h"
#include "motor_pwm.h"
#include "peripherals.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "mpu6050.h"
#include "hal_ibus_transport.h"
#include "sensor_task.h"

void SystemClockConfig();
void USART2_UART_Init();
void USART1_UART_Init();
void DMA_UART1_Init();
void TIM3_Init();
uint8_t IBUS_Setup(HAL_IBUS_Transport_t* transport);
uint8_t RCInput_Setup(RCInput_Handle_t* rc_inp);
uint8_t MotorPWM_Setup(MotorPWM_Handle_t* motor_pwm);
uint8_t ReceiverTask_Setup(
		ReceiverTask_Context_t* ctx,
		RCInput_Handle_t* rc_inp,
		QueueHandle_t raw_frame_queue,
		QueueHandle_t command_queue,
		HAL_IBUS_Transport_t* transport);
void I2C1_Init();
void MPU6050_DRDY_GPIO_Init();
uint8_t MPU6050_Setup(
		MPU6050_Handle_t* mpu,
		DeviceIO_t* device_io);
uint8_t SensorTask_Setup(
		SensorTask_Context_t* sensor_ctx,
		MPU6050_Handle_t* mpu,
		QueueHandle_t data_queue_to_control);

#endif /* INC_SETUP_H_ */
