#include "stm32f4xx.h"

#include "main.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "msp.h"
#include "ibus.h"
#include "rc_input.h"
#include "control_common.h"

#include <math.h>
#include <stdio.h>






UART_HandleTypeDef husart2;
UART_HandleTypeDef husart1;
DMA_HandleTypeDef hdma2_usart1_rx;
TIM_HandleTypeDef htim3;
IBUS_Handle_t ibus;
RCInput_Handle_t rc_inp;
static volatile uint8_t ibus_frame_ready = 0U;
static volatile IBUS_Status_t last_rx_status = IBUS_OK;







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

    if (HAL_UART_Init(&husart2) != HAL_OK) {
        while (1) {
        }
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
	if(HAL_UART_Init(&husart1) != HAL_OK) {
		printf("usart1 init failed!/r/n");
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
	hdma2_usart1_rx.Init.Mode = DMA_NORMAL;
	hdma2_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma2_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	if(HAL_DMA_Init(&hdma2_usart1_rx) != HAL_OK) {
		printf("usart1 init failed!\r\n");
	}

	__HAL_LINKDMA(&husart1, hdmarx, hdma2_usart1_rx);
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}



void HAL_UARTEx_RxEventCallback(
		UART_HandleTypeDef *huart,
		uint16_t Size) {
	if(huart -> Instance == USART1) {
		last_rx_status = IBUS_OnRxEvent(&ibus, HAL_GetTick(), Size);
		if(last_rx_status == IBUS_OK) {
			ibus_frame_ready = 1U;
		}
	}
}


void TIM3_Init() {
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = TIM3_PRESCALER;
	htim3.Init.Period = TIM3_PERIOD;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	if(HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}

	TIM_OC_InitTypeDef pwm_config;
	pwm_config.OCMode = TIM_OCMODE_PWM1;
	pwm_config.Pulse = 0U;
	pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	pwm_config.OCFastMode = TIM_OCFAST_DISABLE;

	if(HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if(HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	if(HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	if(HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}

}


uint8_t IBUS_Setup() {
	IBUS_Status_t status;
	status = IBUS_Init(&ibus, &husart1);
	if(status != IBUS_OK) {
		printf("IBUS_Init failed! Code: %d\r\n", (int)(status));
		return 0;
	}
	status = IBUS_Start(&ibus);
	if (status != IBUS_OK)
	{
	   printf("IBUS_Start failed: %d\r\n", (int)status);
	   return 0;
	}
	return 1;
}

uint8_t RCInput_Setup() {
	RCInput_Config_t config = {0};
	config.throttle.channel_idx = THROTTLE_CHANNEL_IDX;
	config.throttle.min = MIN_THROTTLE;
	config.throttle.max = MAX_THROTTLE;
	config.throttle.reversed = 0U;

	config.roll.channel_idx = ROLL_CHANNEL_IDX;
	config.roll.min = MIN_THROTTLE;
	config.roll.max = MAX_THROTTLE;
	config.roll.reversed = 0U;

	config.pitch.channel_idx = PITCH_CHANNEL_IDX;
	config.pitch.min = MIN_THROTTLE;
	config.pitch.max = MAX_THROTTLE;
	config.pitch.reversed = 0U;

	config.yaw.channel_idx = YAW_CHANNEL_IDX;
	config.yaw.min = MIN_THROTTLE;
	config.yaw.max = MAX_THROTTLE;
	config.yaw.reversed = 0U;

	config.roll.center = CENTER_THROTTLE;
	config.roll.deadband = COMMON_DEADBAND;
	config.pitch.center = CENTER_THROTTLE;
	config.pitch.deadband = COMMON_DEADBAND;
	config.yaw.center = CENTER_THROTTLE;
	config.yaw.deadband = COMMON_DEADBAND;

	RCInput_Status_t status = RCInput_Init(&rc_inp, &config);
	if(status != RC_INPUT_OK) {
		return 0;
	}
	return 1;
}





int main(void)
{
	IBUS_Status_t status;
    HAL_Init();
    SystemClockConfig();
    USART2_UART_Init();
    USART1_UART_Init();
    DMA_UART1_Init();
    TIM3_Init();
    if(!IBUS_Setup()) {
    	return 0;
    }
    if(!RCInput_Setup()) {
    	return 0;
    }

    while (1) {
    	uint32_t now_ms = HAL_GetTick();
    	status = IBUS_Update(&ibus, now_ms);
    	if(ibus_frame_ready == 1U) {
    		uint16_t throttle;
    		ibus_frame_ready = 0U;
    		throttle = ibus.latest_valid_data.channels[2U];
    		printf(
				"CH1=%u CH2=%u CH3=%u CH4=%u | "
				"Throttle=%u us\r\n",
				ibus.latest_valid_data.channels[0],
				ibus.latest_valid_data.channels[1],
				ibus.latest_valid_data.channels[2],
				ibus.latest_valid_data.channels[3],
				throttle);
    	}
    	if (status == IBUS_ERR_TIMEOUT)
		{
			static uint32_t last_timeout_print_ms = 0U;

			if ((now_ms - last_timeout_print_ms) >= 500U)
			{
				last_timeout_print_ms = now_ms;
				printf("iBus signal lost\r\n");
			}
		}
    	HAL_Delay(10U);
    }
}
