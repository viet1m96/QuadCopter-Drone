/*
 * stm32f4xx_hal_timebase_tim.c
 *
 *  Created on: Aug 1, 2026
 *      Author: vietht-hl
 */

#include "peripherals.h"
#include "stm32f4xx_hal.h"

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
  if (TickPriority >= (1 << __NVIC_PRIO_BITS)) {
    return HAL_ERROR;
  }
  RCC_ClkInitTypeDef clk_config;
  uint32_t flash_latency;
  uint32_t pclk1_freq;
  uint32_t tim6_freq;
  uint32_t tim6_presc;
  __HAL_RCC_TIM6_CLK_ENABLE();
  HAL_RCC_GetClockConfig(&clk_config, &flash_latency);
  pclk1_freq = HAL_RCC_GetPCLK1Freq();
  if (clk_config.APB1CLKDivider == RCC_HCLK_DIV1) {
    tim6_freq = pclk1_freq;
  } else {
    tim6_freq = 2U * pclk1_freq;
  }

  tim6_presc = (tim6_freq / 1000000U) - 1;

  hal_tick_timer.Instance = TIM6;
  hal_tick_timer.Init.Prescaler = tim6_presc;
  hal_tick_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
  hal_tick_timer.Init.Period = 999U;
  hal_tick_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  hal_tick_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_StatusTypeDef status = HAL_TIM_Base_Init(&hal_tick_timer);
  if (status != HAL_OK) {
    return status;
  }
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  status = HAL_TIM_Base_Start_IT(&hal_tick_timer);
  if (status == HAL_OK) {
    uwTickPrio = TickPriority;
  }
  return status;
}

void HAL_SuspendTick(void) {
  __HAL_TIM_DISABLE_IT(&hal_tick_timer, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void) {
  __HAL_TIM_ENABLE_IT(&hal_tick_timer, TIM_IT_UPDATE);
}
