/*
 * msp.c
 *
 *  Created on: Jun 22, 2026
 *      Author: vietht-hl
 */

#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_usart.h"




void HAL_MspInit() {
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	//Set the priority group for interrupts
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	//Enable some of system exceptions
	SCB -> SHCSR |= 0x7 << 16;

	//Set the priority for enabled exceptions above
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);

}


void Sensor_EXTI_Init(void) {
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef gpiob_config;
	gpiob_config.Pin = GPIO_PIN_1;
	gpiob_config.Mode = GPIO_MODE_IT_FALLING;
	gpiob_config.Pull = GPIO_NOPULL;
	gpiob_config.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(GPIOB, &gpiob_config);

	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
	HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}





void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {

	if(hi2c -> Instance == I2C1) {
		__HAL_RCC_I2C1_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		GPIO_InitTypeDef gpiob_config;
		gpiob_config.Pin = GPIO_PIN_6;
		gpiob_config.Mode = GPIO_MODE_AF_OD;
		gpiob_config.Pull = GPIO_PULLUP;
		gpiob_config.Speed = GPIO_SPEED_FAST;
		gpiob_config.Alternate = GPIO_AF4_I2C1;

		HAL_GPIO_Init(GPIOB, &gpiob_config);
		gpiob_config.Pin = GPIO_PIN_7;
		HAL_GPIO_Init(GPIOB, &gpiob_config);

		HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
		HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
		HAL_NVIC_SetPriority(I2C1_EV_IRQn, 15, 0);
		HAL_NVIC_SetPriority(I2C1_ER_IRQn, 14, 0);
	}

}

void HAL_UART_MspInit(UART_HandleTypeDef* husart) {

	if(husart -> Instance == USART2) {
		__HAL_RCC_USART2_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitTypeDef gpioa_config;
		gpioa_config.Pin = GPIO_PIN_2;
		gpioa_config.Mode = GPIO_MODE_AF_PP;
		gpioa_config.Pull = GPIO_NOPULL;
		gpioa_config.Speed = GPIO_SPEED_FAST;
		gpioa_config.Alternate = GPIO_AF7_USART2;

		HAL_GPIO_Init(GPIOA, &gpioa_config);
		gpioa_config.Pin = GPIO_PIN_3;
		HAL_GPIO_Init(GPIOA, &gpioa_config);

		HAL_NVIC_EnableIRQ(USART2_IRQn);
		HAL_NVIC_SetPriority(USART2_IRQn, 13, 0);
	}

}


