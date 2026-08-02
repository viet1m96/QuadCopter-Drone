#include "stm32f4xx.h"

#include "main.h"
#include "msp.h"

#include "motor_mixer.h"


#include "stdio.h"
#include "setup.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "tasks_list.h"




static IBUS_Handle_t ibus;
static RCInput_Handle_t rc_inp;
static MotorPWM_Handle_t motor_pwm;
static QueueHandle_t receiver_queue = NULL;
static QueueHandle_t process_queue = NULL;
static ReceiverTask_Context_t receiver_ctx;




int main(void)
{
	HAL_Init();
	SystemClockConfig();

	USART2_UART_Init();
	USART1_UART_Init();
	DMA_UART1_Init();
	TIM3_Init();

	if(!IBUS_Setup(&ibus)) {
		return 0;
	}

	if(!RCInput_Setup(&rc_inp)) {
		return 0;
	}

	if(!MotorPWM_Setup(&motor_pwm)) {
		return 0;
	}



	if(ReceiverTask_Setup(
			&receiver_ctx,
			&ibus,
			&rc_inp,
			receiver_queue,
			process_queue) != pdPASS) {
		return 0;
	}

	vTaskStartScheduler();
	for(;;) {

	}

}
