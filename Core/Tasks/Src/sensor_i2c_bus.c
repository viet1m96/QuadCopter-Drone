/*
 * sensor_i2c_bus.c
 *
 *  Created on: Aug 17, 2026
 *      Author: vietht-hl
 */

#include "sensor_i2c_bus.h"
#include "board_i2c1.h"
#include "sensor_task.h"

static void SetBusIdle(SensorBusManager_t *bus) {
  bus->state = SENSOR_BUS_IDLE;
  bus->owner = SENSOR_OWNER_NONE;
  bus->state_started_tick = 0U;
}

static void SetBusFault(SensorBusManager_t *bus) {
  bus->state = SENSOR_BUS_FAULT;
  bus->owner = SENSOR_OWNER_NONE;
  bus->state_started_tick = 0U;
}

static void RecoverI2C1(SensorBusManager_t *bus) {
  SetBusFault(bus);

  if (I2C1_Recover() != 0U) {
    SetBusIdle(bus);
  }
}

void SensorI2CBus_Init(SensorBusManager_t *bus,
                       TickType_t transaction_timeout_ticks) {
  bus->state = SENSOR_BUS_IDLE;
  bus->owner = SENSOR_OWNER_NONE;
  bus->state_started_tick = 0U;
  bus->transaction_timeout_ticks = transaction_timeout_ticks;
}

uint8_t SensorI2CBus_IsIdle(const SensorBusManager_t *bus) {
  return bus->state == SENSOR_BUS_IDLE;
}

void SensorI2CBus_Start(SensorBusManager_t *bus, SensorOwner_t owner,
                        TickType_t now) {
  bus->owner = owner;
  bus->state = SENSOR_BUS_BUSY;
  bus->state_started_tick = now;
}

SensorOwner_t SensorI2CBus_HandleEvents(SensorBusManager_t *bus,
                                        uint32_t events) {
  if (bus->state != SENSOR_BUS_BUSY) {
    return SENSOR_OWNER_NONE;
  }

  if ((events & SENSOR_EVENT_I2C1_ERROR) != 0U) {
    RecoverI2C1(bus);
    return SENSOR_OWNER_NONE;
  }

  if ((events & SENSOR_EVENT_I2C1_RX_DONE) == 0U) {
    return SENSOR_OWNER_NONE;
  }

  SensorOwner_t completed_owner = bus->owner;

  SetBusIdle(bus);

  return completed_owner;
}

void SensorI2CBus_CheckTimeout(SensorBusManager_t *bus, TickType_t now) {
  if (bus->state != SENSOR_BUS_BUSY) {
    return;
  }

  TickType_t elapsed = now - bus->state_started_tick;

  if (elapsed < bus->transaction_timeout_ticks) {
    return;
  }

  RecoverI2C1(bus);
}

TickType_t SensorI2CBus_CalculateWaitTime(const SensorBusManager_t *bus,
                                          TickType_t now) {
  if (bus->state != SENSOR_BUS_BUSY) {
    return portMAX_DELAY;
  }

  TickType_t elapsed = now - bus->state_started_tick;

  if (elapsed >= bus->transaction_timeout_ticks) {
    return 0U;
  }

  return bus->transaction_timeout_ticks - elapsed;
}
