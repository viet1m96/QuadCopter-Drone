/*
 * hal_i2c_transport.h
 *
 *  Created on: Aug 7, 2026
 *      Author: vietht-hl
 */

#ifndef TRANSPORTS_INC_HAL_I2C_TRANSPORT_H_
#define TRANSPORTS_INC_HAL_I2C_TRANSPORT_H_

#include "stm32f4xx_hal.h"
#include "device_IO.h"

DeviceIO_Status_t HAL_I2C_DeviceIO_Init(
		I2C_HandleTypeDef* hi2c,
		DeviceIO_t* device);

#endif /* TRANSPORTS_INC_HAL_I2C_TRANSPORT_H_ */
