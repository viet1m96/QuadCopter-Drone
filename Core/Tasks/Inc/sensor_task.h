/*
 * sensor_task.h
 *
 *  Created on: Aug 7, 2026
 *      Author: vietht-hl
 */

#ifndef TASKS_INC_SENSOR_TASK_H_
#define TASKS_INC_SENSOR_TASK_H_
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "mpu6050.h"

#define SENSOR_BUS_I2C1_FRAME_READY (1UL << 0)
#define SENSOR_BUS_I2C1_ERROR  (1UL << 1)

typedef enum {
	I2C1_STATE_IDLE = 0,
	I2C1_STATE_BUSY,
	I2C1_STATE_ABORTING
} I2C1_State_t;


typedef enum {
	MPU6050 = 0
} I2C1_Owner_t;

typedef struct {
	I2C1_Owner_t owner;
	DeviceIO_t* device_io;

	TickType_t state_started_tick;
	TickType_t transaction_timeout_ticks;
	TickType_t abort_timeout_ticks;
} I2C1_BusManager_t;

typedef struct {
	MPU6050_Handle_t* imu;
	uint8_t imu_pending;

	I2C1_BusManager_t i2c1_manager;
	QueueHandle_t data_queue_from_callback;
	QueueHandle_t data_queue_to_control;
} SensorTask_Context_t;

BaseType_t SensorTask_Create(SensorTask_Context_t* sensor_ctx);

#endif /* TASKS_INC_SENSOR_TASK_H_ */
