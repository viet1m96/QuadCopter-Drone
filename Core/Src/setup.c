/*
 * init.c
 *
 *  Created on: Aug 2, 2026
 *      Author: vietht-hl
 */

#include "setup.h"
#include "stdio.h"
#include "main.h"

void SystemClockConfig(void)
{
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

	if(HAL_RCC_OscConfig(&osc_config) != HAL_OK) {
		Error_Handler();
	}

	if(HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	clk_config.ClockType = RCC_CLOCKTYPE_SYSCLK |
						   RCC_CLOCKTYPE_HCLK   |
						   RCC_CLOCKTYPE_PCLK1  |
						   RCC_CLOCKTYPE_PCLK2;

	clk_config.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clk_config.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clk_config.APB1CLKDivider = RCC_HCLK_DIV4;
	clk_config.APB2CLKDivider = RCC_HCLK_DIV2;

	if(HAL_RCC_ClockConfig(&clk_config, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}


void USART2_UART_Init(void)
{
	husart2.Instance = USART2;
	husart2.Init.BaudRate = 115200U;
	husart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	husart2.Init.WordLength = UART_WORDLENGTH_8B;
	husart2.Init.StopBits = UART_STOPBITS_1;
	husart2.Init.Parity = UART_PARITY_NONE;
	husart2.Init.Mode = UART_MODE_TX;
	husart2.Init.OverSampling = UART_OVERSAMPLING_16;

	if(HAL_UART_Init(&husart2) != HAL_OK) {
		Error_Handler();
	}
}


void USART1_UART_Init(void)
{
	husart1.Instance = USART1;
	husart1.Init.BaudRate = 115200U;
	husart1.Init.WordLength = UART_WORDLENGTH_8B;
	husart1.Init.StopBits = UART_STOPBITS_1;
	husart1.Init.Parity = UART_PARITY_NONE;
	husart1.Init.Mode = UART_MODE_RX;
	husart1.Init.OverSampling = UART_OVERSAMPLING_16;
	husart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;

	if(HAL_UART_Init(&husart1) != HAL_OK) {
		Error_Handler();
	}
}


void DMA_UART1_Init(void)
{
	__HAL_RCC_DMA2_CLK_ENABLE();

	hdma2_usart1_rx.Instance = DMA2_Stream2;
	hdma2_usart1_rx.Init.Channel = DMA_CHANNEL_4;
	hdma2_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma2_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma2_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma2_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma2_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma2_usart1_rx.Init.Mode = DMA_NORMAL;
	hdma2_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma2_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	if(HAL_DMA_Init(&hdma2_usart1_rx) != HAL_OK) {
		Error_Handler();
	}

	__HAL_LINKDMA(&husart1, hdmarx, hdma2_usart1_rx);

	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6U, 0U);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}





void TIM3_Init(void)
{
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = TIM3_PRESCALER;
	htim3.Init.Period = TIM3_PERIOD;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;


	if(HAL_TIM_OC_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}

	TIM_OC_InitTypeDef pwm_config = {0};

	pwm_config.OCMode = TIM_OCMODE_PWM1;
	pwm_config.Pulse = 0U;
	pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	pwm_config.OCFastMode = TIM_OCFAST_DISABLE;

	if(HAL_TIM_PWM_ConfigChannel(
			&htim3,
			&pwm_config,
			TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}

	if(HAL_TIM_PWM_ConfigChannel(
			&htim3,
			&pwm_config,
			TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}

	if(HAL_TIM_PWM_ConfigChannel(
			&htim3,
			&pwm_config,
			TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}

	if(HAL_TIM_PWM_ConfigChannel(
			&htim3,
			&pwm_config,
			TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
}


uint8_t IBUS_Setup(void)
{
	IBUS_Status_t status = IBUS_Init(&ibus, &husart1);

	if(status != IBUS_OK) {
		printf("IBUS_Init failed: %d\r\n", (int)status);
		return 0U;
	}

	status = IBUS_Start(&ibus);

	if(status != IBUS_OK) {
		printf("IBUS_Start failed: %d\r\n", (int)status);
		return 0U;
	}

	return 1U;
}


uint8_t RCInput_Setup(void)
{
	RCInput_Config_t config = {0};

	config.throttle.channel_idx = THROTTLE_CHANNEL_IDX;
	config.throttle.min = MIN_THROTTLE;
	config.throttle.max = MAX_THROTTLE;
	config.throttle.reversed = 0U;

	config.roll.channel_idx = ROLL_CHANNEL_IDX;
	config.roll.min = MIN_THROTTLE;
	config.roll.center = CENTER_THROTTLE;
	config.roll.max = MAX_THROTTLE;
	config.roll.deadband = COMMON_DEADBAND;
	config.roll.reversed = 0U;

	config.pitch.channel_idx = PITCH_CHANNEL_IDX;
	config.pitch.min = MIN_THROTTLE;
	config.pitch.center = CENTER_THROTTLE;
	config.pitch.max = MAX_THROTTLE;
	config.pitch.deadband = COMMON_DEADBAND;
	config.pitch.reversed = 0U;

	config.yaw.channel_idx = YAW_CHANNEL_IDX;
	config.yaw.min = MIN_THROTTLE;
	config.yaw.center = CENTER_THROTTLE;
	config.yaw.max = MAX_THROTTLE;
	config.yaw.deadband = COMMON_DEADBAND;
	config.yaw.reversed = 0U;

	RCInput_Status_t status = RCInput_Init(&rc_inp, &config);

	if(status != RC_INPUT_OK) {
		printf("RCInput_Init failed: %d\r\n", (int)status);
		return 0U;
	}

	return 1U;
}


uint8_t MotorPWM_Setup(void)
{
	MotorPWM_Config_t config = {
		.channels = {
			TIM_CHANNEL_1,
			TIM_CHANNEL_2,
			TIM_CHANNEL_3,
			TIM_CHANNEL_4
		},
		.frame_period_us = (uint16_t)(htim3.Init.Period + 1U)
	};

	MotorPWM_Status_t status = MotorPWM_Init(
										&motor_pwm,
										&htim3,
										&config);

	if(status != MOTOR_PWM_OK) {
		printf("MotorPWM_Init failed: %d\r\n", (int)status);
		return 0U;
	}

	status = MotorPWM_Start(&motor_pwm, MIN_THROTTLE);

	if(status != MOTOR_PWM_OK) {
		printf("MotorPWM_Start failed: %d\r\n", (int)status);
		return 0U;
	}

	return 1U;
}
