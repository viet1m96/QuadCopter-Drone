/*
 * sensor_i2c_bus.h
 *
 *  Created on: Aug 17, 2026
 *      Author: vietht-hl
 */

#ifndef TASKS_INC_SENSOR_I2C_BUS_H_
#define TASKS_INC_SENSOR_I2C_BUS_H_

#include "FreeRTOS.h"
#include "device_IO.h"

typedef enum {
  SENSOR_BUS_IDLE = 0,
  SENSOR_BUS_BUSY,
  SENSOR_BUS_ABORTING
} SensorBusState_t;

typedef enum { SENSOR_OWNER_NONE = 0, SENSOR_OWNER_MPU6050 } SensorOwner_t;

typedef struct {
  SensorBusState_t state;
  SensorOwner_t owner;

  DeviceIO_t *active_io;
  uint8_t active_device_address;

  TickType_t state_started_tick;
  TickType_t transaction_timeout_ticks;
  TickType_t abort_timeout_ticks;
} SensorBusManager_t;

void SensorI2CBus_Init(SensorBusManager_t *bus,
                       TickType_t transaction_timeout_ticks,
                       TickType_t abort_timeout_ticks);

uint8_t SensorI2CBus_IsIdle(const SensorBusManager_t *bus);

void SensorI2CBus_Start(SensorBusManager_t *bus, SensorOwner_t owner,
                        DeviceIO_t *io, uint8_t device_address, TickType_t now);

SensorOwner_t SensorI2CBus_HandleEvents(SensorBusManager_t *bus,
                                        uint32_t events);

void SensorI2CBus_CheckTimeout(SensorBusManager_t *bus, TickType_t now);

TickType_t SensorI2CBus_CalculateWaitTime(const SensorBusManager_t *bus,
                                          TickType_t now);

#endif /* TASKS_INC_SENSOR_I2C_BUS_H_ */
