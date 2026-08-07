#include <receiver_task.h>
#include "stm32f4xx.h"

#include "main.h"

#include "motor_mixer.h"


#include "stdio.h"
#include "setup.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "callbacks_list.h"
#include "mpu6050.h"
#include "device_IO.h"



static RCInput_Handle_t rc_inp;
static HAL_IBUS_Transport_t ibus_transport;
static DeviceIO_t device_io;
static MPU6050_Handle_t mpu;
static QueueHandle_t raw_ibus_queue;
static QueueHandle_t command_queue;
static ReceiverTask_Context_t receiver_ctx;







int main(void)
{
	HAL_Init();
	SystemClockConfig();

	USART2_UART_Init();

	USART1_UART_Init();
	DMA_UART1_Init();
	TIM3_Init();
	I2C1_Init();
	MPU6050_DRDY_GPIO_Init();

	if(!RCInput_Setup(&rc_inp)) {
		return 0;
	}

	if(!IBUS_Setup(&ibus_transport)) {
		return 0;
	}



	if(!MPU6050_Setup(&mpu, &device_io)) {
		return 0;
	}
	if(!ReceiverTask_Setup(
			&receiver_ctx,
			&rc_inp,
			raw_ibus_queue,
			command_queue,
			&ibus_transport)) {
		return 0;
	}
//	Callbacks_Init(&receiver_ctx);
//	if(ReceiverTask_Create(&receiver_ctx) != pdPASS) {
//		return 0;
//	}
	vTaskStartScheduler();
	for(;;) {

	}

}
