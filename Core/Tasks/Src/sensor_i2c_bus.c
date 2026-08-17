/*
 * sensor_i2c_bus.c
 *
 *  Created on: Aug 17, 2026
 *      Author: vietht-hl
 */

#include "sensor_i2c_bus.h"
#include "setup.h"
static void SetBusIdle(SensorBusManager_t *bus) {
  bus->state = SENSOR_BUS_IDLE;
  bus->owner = SENSOR_OWNER_NONE;
  bus->active_io = NULL;
  bus->active_device_address = 0U;
}

static void RecoverI2C1(SensorBusManager_t *bus) {
  if (bus == NULL) {
    return;
  }

  bus->active_io = NULL;
  bus->active_device_address = 0U;
  bus->owner = SENSOR_OWNER_NONE;

  if (I2C1_RecoverBus() == 0U) {
    return;
  }
  I2C1_Init();
  return;
}

void SensorI2CBus_Init(SensorBusManager_t *bus,
                       TickType_t transaction_timeout_ticks,
                       TickType_t abort_timeout_ticks) {
  bus->state = SENSOR_BUS_IDLE;
  bus->owner = SENSOR_OWNER_NONE;

  bus->active_io = NULL;
  bus->active_device_address = 0U;

  bus->state_started_tick = 0U;

  bus->transaction_timeout_ticks = transaction_timeout_ticks;
  bus->abort_timeout_ticks = abort_timeout_ticks;
}

uint8_t SensorI2CBus_IsIdle(const SensorBusManager_t *bus) {
  return bus->state == SENSOR_BUS_IDLE;
}

void SensorI2CBus_Start(SensorBusManager_t *bus, SensorOwner_t owner,
                        DeviceIO_t *io, uint8_t device_address,
                        TickType_t now) {
  bus->owner = owner;
  bus->active_io = io;
  bus->active_device_address = device_address;

  bus->state = SENSOR_BUS_BUSY;
  bus->state_started_tick = now;
}

SensorOwner_t SensorI2CBus_HandleEvents(SensorBusManager_t *bus,
                                        uint32_t events) {
  if (bus->state == SENSOR_BUS_ABORTING) {
    if ((events & SENSOR_EVENT_I2C1_ABORT_DONE) != 0U) {
      SetBusIdle(bus);
    }

    return SENSOR_OWNER_NONE;
  }

  if (bus->state != SENSOR_BUS_BUSY) {
    return SENSOR_OWNER_NONE;
  }

  if ((events & SENSOR_EVENT_I2C1_ERROR) != 0U) {
    DeviceIO_Status_t status = bus->active_io->ops->abort_it(
        bus->active_io->context, bus->active_device_address);

    if (status == DEVICE_IO_OK) {
      bus->state = SENSOR_BUS_ABORTING;
      bus->state_started_tick = xTaskGetTickCount();
    } else {
      RecoverI2C1(bus);
      SetBusIdle(bus);
    }
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
  if (bus->state == SENSOR_BUS_IDLE) {
    return;
  }

  TickType_t elapsed = now - bus->state_started_tick;

  if (bus->state == SENSOR_BUS_BUSY) {
    if (elapsed < bus->transaction_timeout_ticks) {
      return;
    }

    DeviceIO_Status_t status = bus->active_io->ops->abort_it(
        bus->active_io->context, bus->active_device_address);

    if (status == DEVICE_IO_OK) {
      bus->state = SENSOR_BUS_ABORTING;
      bus->state_started_tick = now;
      return;
    }

    RecoverI2C1(bus);

    SetBusIdle(bus);

    return;
  }

  if (bus->state == SENSOR_BUS_ABORTING) {
    if (elapsed < bus->abort_timeout_ticks) {
      return;
    }

    RecoverI2C1(bus);

    SetBusIdle(bus);
  }
}

TickType_t SensorI2CBus_CalculateWaitTime(const SensorBusManager_t *bus,
                                          TickType_t now) {
  TickType_t timeout;

  if (bus->state == SENSOR_BUS_BUSY) {
    timeout = bus->transaction_timeout_ticks;
  } else if (bus->state == SENSOR_BUS_ABORTING) {
    timeout = bus->abort_timeout_ticks;
  } else {
    return portMAX_DELAY;
  }

  TickType_t elapsed = now - bus->state_started_tick;

  if (elapsed >= timeout) {
    return 0U;
  }

  return timeout - elapsed;
}
