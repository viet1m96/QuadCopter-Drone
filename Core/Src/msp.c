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

DMA_HandleTypeDef hdma_usart1_rx;

void HAL_MspInit() {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  // Set the priority group for interrupts
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  // Enable some of system exceptions
  SCB->SHCSR |= 0x7 << 16;

  // Set the priority for enabled exceptions above
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
}

void HAL_UART_MspInit(UART_HandleTypeDef *husart) {

  if (husart->Instance == USART2) {
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
  } else if (husart->Instance == USART1) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpiob_config;
    gpiob_config.Pin = GPIO_PIN_7;
    gpiob_config.Mode = GPIO_MODE_AF_PP;
    gpiob_config.Pull = GPIO_NOPULL;
    gpiob_config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpiob_config.Alternate = GPIO_AF7_USART1;

    HAL_GPIO_Init(GPIOB, &gpiob_config);
  }
}

void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *tim) {
  if (tim->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpiob_config;
    gpiob_config.Pin = GPIO_PIN_0;
    gpiob_config.Mode = GPIO_MODE_AF_PP;
    gpiob_config.Pull = GPIO_NOPULL;
    gpiob_config.Speed = GPIO_SPEED_FREQ_HIGH;
    gpiob_config.Alternate = GPIO_AF2_TIM3;

    HAL_GPIO_Init(GPIOB, &gpiob_config);

    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpioc_config;
    gpioc_config.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_9;
    gpioc_config.Mode = GPIO_MODE_AF_PP;
    gpioc_config.Pull = GPIO_NOPULL;
	gpioc_config.Speed = GPIO_SPEED_FREQ_HIGH;
	gpioc_config.Alternate = GPIO_AF2_TIM3;

	HAL_GPIO_Init(GPIOC, &gpioc_config);
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpiob_config;
    gpiob_config.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpiob_config.Mode = GPIO_MODE_AF_OD;
    gpiob_config.Pull = GPIO_PULLUP;
    gpiob_config.Speed = GPIO_SPEED_FREQ_HIGH;
    gpiob_config.Alternate = GPIO_AF4_I2C1;

    HAL_GPIO_Init(GPIOB, &gpiob_config);
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  }
}
