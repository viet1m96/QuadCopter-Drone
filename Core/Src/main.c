#include "stm32f4xx.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_uart.h"

#include "main.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "msp.h"
#include "ibus.h"

#include <math.h>
#include <stdio.h>






UART_HandleTypeDef husart2;
UART_HandleTypeDef husart1;
DMA_HandleTypeDef hdma2_usart1_rx;
IBUS_Handle_t ibus;

static volatile uint8_t ibus_frame_ready = 0U;
static volatile IBUS_Status_t last_rx_status = IBUS_OK;







void SystemClockConfig(void)
{
    /* Dùng clock mặc định sau HAL_Init(). */
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






int main(void)
{
	IBUS_Status_t status;
    uint16_t throttle_min = UINT16_MAX;
	uint16_t throttle_max = 0U;
    HAL_Init();
    SystemClockConfig();
    USART2_UART_Init();
    USART1_UART_Init();
    DMA_UART1_Init();

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
    while (1) {
    	uint32_t now_ms = HAL_GetTick();
    	status = IBUS_Update(&ibus, now_ms);
    	if(ibus_frame_ready == 1U) {
    		uint16_t throttle;
    		ibus_frame_ready = 0U;
    		throttle = ibus.latest_valid_data.channels[2U];
    		throttle_min = (throttle_min < throttle) ? throttle_min : throttle;
    		throttle_max = (throttle_max > throttle) ? throttle_max : throttle;
    		printf(
				"CH1=%u CH2=%u CH3=%u CH4=%u | "
				"Throttle=%u us | Min=%u us | Max=%u us\r\n",
				ibus.latest_valid_data.channels[0],
				ibus.latest_valid_data.channels[1],
				ibus.latest_valid_data.channels[2],
				ibus.latest_valid_data.channels[3],
				throttle,
				throttle_min,
				throttle_max);
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
