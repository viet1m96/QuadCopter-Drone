/*
 * device_IO.h
 *
 *  Created on: Aug 7, 2026
 *      Author: vietht-hl
 */

#ifndef DEVICES_INC_DEVICE_IO_H_
#define DEVICES_INC_DEVICE_IO_H_

#include "stdint.h"

typedef enum {
	DEVICE_IO_OK = 0,
	DEVICE_IO_BUSY,
	DEVICE_IO_TIMEOUT,
	DEVICE_IO_ERROR,
	DEVICE_IO_INVALID_ARGUMENT
} DeviceIO_Status_t;


typedef struct {
	DeviceIO_Status_t (*read_registers) (
			void* context,
			uint8_t device_address,
			uint8_t start_register,
			uint8_t* data,
			uint16_t length,
			uint32_t timeout_ms);
	DeviceIO_Status_t (*write_registers)(
			void* context,
			uint8_t device_address,
			uint8_t start_register,
			const uint8_t* data,
			uint16_t length,
			uint32_t timeout_ms);
	DeviceIO_Status_t (*read_registers_it)(
			void *context,
			uint8_t device_address,
			uint8_t start_register,
			uint8_t *data,
			uint16_t length);

	void (*delay_ms)(
			void *context,
			uint32_t delay_ms);

	uint32_t (*get_tick_ms)(
			void *context);

	DeviceIO_Status_t (*abort_it) (
			void* context,
			uint8_t device_address);
} DeviceIO_Ops_t;

typedef struct {
	void* context;
	const DeviceIO_Ops_t* ops;
} DeviceIO_t;


#endif /* DEVICES_INC_DEVICE_IO_H_ */
