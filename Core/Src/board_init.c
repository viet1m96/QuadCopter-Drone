#include "board_init.h"

#include "main.h"
#include "peripherals.h"
#include "stm32f4xx_hal.h"

void SystemClockConfig(void) {
  RCC_OscInitTypeDef osc_config = {0};
  RCC_ClkInitTypeDef clk_config = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc_config.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc_config.HSEState = RCC_HSE_OFF;
  osc_config.LSEState = RCC_LSE_OFF;
  osc_config.HSIState = RCC_HSI_ON;
  osc_config.LSIState = RCC_LSI_OFF;
  osc_config.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

  osc_config.PLL.PLLState = RCC_PLL_ON;
  osc_config.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc_config.PLL.PLLM = 8U;
  osc_config.PLL.PLLN = 180U;
  osc_config.PLL.PLLP = RCC_PLLP_DIV2;
  osc_config.PLL.PLLQ = 8U;
  osc_config.PLL.PLLR = 2U;

  if (HAL_RCC_OscConfig(&osc_config) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }

  clk_config.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk_config.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk_config.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_config.APB1CLKDivider = RCC_HCLK_DIV4;
  clk_config.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&clk_config, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

void USART2_UART_Init(void) {
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200U;
  husart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husart2.Init.WordLength = UART_WORDLENGTH_8B;
  husart2.Init.StopBits = UART_STOPBITS_1;
  husart2.Init.Parity = UART_PARITY_NONE;
  husart2.Init.Mode = UART_MODE_TX_RX;
  husart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&husart2) != HAL_OK) {
    Error_Handler();
  }
}

void USART1_UART_Init(void) {
  husart1.Instance = USART1;
  husart1.Init.BaudRate = 115200U;
  husart1.Init.WordLength = UART_WORDLENGTH_8B;
  husart1.Init.StopBits = UART_STOPBITS_1;
  husart1.Init.Parity = UART_PARITY_NONE;
  husart1.Init.Mode = UART_MODE_RX;
  husart1.Init.OverSampling = UART_OVERSAMPLING_16;
  husart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);

  if (HAL_UART_Init(&husart1) != HAL_OK) {
    Error_Handler();
  }
}

void DMA_UART1_Init(void) {
  __HAL_RCC_DMA2_CLK_ENABLE();

  hdma2_usart1_rx.Instance = DMA2_Stream2;
  hdma2_usart1_rx.Init.Channel = DMA_CHANNEL_4;
  hdma2_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma2_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma2_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma2_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma2_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma2_usart1_rx.Init.Mode = DMA_CIRCULAR;
  hdma2_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma2_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma2_usart1_rx) != HAL_OK) {
    Error_Handler();
  }

  __HAL_LINKDMA(&husart1, hdmarx, hdma2_usart1_rx);

  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

void TIM3_Init(void) {
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = TIM3_PRESCALER;
  htim3.Init.Period = TIM3_PERIOD;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  TIM_OC_InitTypeDef pwm_config = {0};
  pwm_config.OCMode = TIM_OCMODE_PWM1;
  pwm_config.Pulse = 0U;
  pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm_config.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
}

void TIM5_Init(void) {
  RCC_ClkInitTypeDef clk_config = {0};
  uint32_t flash_latency;
  uint32_t pclk1_freq;
  uint32_t tim5_freq;

  __HAL_RCC_TIM5_CLK_ENABLE();

  HAL_RCC_GetClockConfig(&clk_config, &flash_latency);
  pclk1_freq = HAL_RCC_GetPCLK1Freq();

  if (clk_config.APB1CLKDivider == RCC_HCLK_DIV1) {
    tim5_freq = pclk1_freq;
  } else {
    tim5_freq = 2U * pclk1_freq;
  }

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = (tim5_freq / 1000000U) - 1U;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFFU;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
    Error_Handler();
  }

  __HAL_TIM_SET_COUNTER(&htim5, 0U);

  if (HAL_TIM_Base_Start(&htim5) != HAL_OK) {
    Error_Handler();
  }
}
