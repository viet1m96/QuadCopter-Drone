#include "stm32f4xx.h"

#include "main.h"
#include "msp.h"

#include "motor_mixer.h"


#include <stdio.h>
#include "setup.h"


UART_HandleTypeDef husart2;
UART_HandleTypeDef husart1;
DMA_HandleTypeDef hdma2_usart1_rx;
TIM_HandleTypeDef htim3;

IBUS_Handle_t ibus;
RCInput_Handle_t rc_inp;
MotorPWM_Handle_t motor_pwm;

static volatile uint8_t ibus_frame_ready = 0U;
static volatile IBUS_Status_t last_rx_status = IBUS_OK;


void HAL_UARTEx_RxEventCallback(
		UART_HandleTypeDef *huart,
		uint16_t size)
{
	if(huart->Instance != USART1) {
		return;
	}

	last_rx_status = IBUS_OnRxEvent(
							&ibus,
							HAL_GetTick(),
							size);

	if(last_rx_status == IBUS_OK) {
		ibus_frame_ready = 1U;
	}
}





int main(void)
{
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

	if(!MotorPWM_Setup()) {
		return 0;
	}



}
