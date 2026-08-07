/*
 * sensor_task.c
 *
 *  Created on: Aug 7, 2026
 *      Author: vietht-hl
 */


#include "sensor_task.h"

static void HandleMPU6050() {

}


static void HandleEvents(SensorTask_Context_t* context) {
	switch(context->i2c1_manager.owner) {
	case MPU6050:
		HandleMPU6050();
		break;
	default:
		return;
	}
}

static void SensorTask(void* argument) {
	SensorTask_Context_t* context = (SensorTask_Context_t*) argument;
	MPU6050_RawData_t raw = {0};
	MPU6050_Data_t physical = {0};
	uint32_t events = 0U;
	for(;;) {
		HandleEvents(context);
	}
}

BaseType_t SensorTask_Create(SensorTask_Context_t* sensor_ctx) {

}
