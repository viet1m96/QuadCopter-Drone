/*
 * hal_i2c_transport.c
 *
 *  Created on: Aug 7, 2026
 *      Author: vietht-hl
 */

#include "hal_i2c_transport.h"

#include <stddef.h>

static DeviceIO_Status_t hal_i2c_convert_status(
        HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return DEVICE_IO_OK;

    case HAL_BUSY:
        return DEVICE_IO_BUSY;

    case HAL_TIMEOUT:
        return DEVICE_IO_TIMEOUT;

    case HAL_ERROR:
    default:
        return DEVICE_IO_ERROR;
    }
}

static DeviceIO_Status_t hal_i2c_write_registers(
        void *context,
        uint8_t device_address,
        uint8_t start_register,
        const uint8_t *data,
        uint16_t length,
        uint32_t timeout_ms)
{
    I2C_HandleTypeDef *hi2c =
            (I2C_HandleTypeDef *)context;

    if (hi2c == NULL
            || data == NULL
            || length == 0U) {
        return DEVICE_IO_INVALID_ARGUMENT;
    }

    HAL_StatusTypeDef status =
            HAL_I2C_Mem_Write(
                    hi2c,
                    (uint16_t)(device_address << 1U),
                    start_register,
                    I2C_MEMADD_SIZE_8BIT,
                    (uint8_t *)data,
                    length,
                    timeout_ms);

    return hal_i2c_convert_status(status);
}

static DeviceIO_Status_t hal_i2c_read_registers(
        void *context,
        uint8_t device_address,
        uint8_t start_register,
        uint8_t *data,
        uint16_t length,
        uint32_t timeout_ms)
{
    I2C_HandleTypeDef *hi2c =
            (I2C_HandleTypeDef *)context;

    if (hi2c == NULL
            || data == NULL
            || length == 0U) {
        return DEVICE_IO_INVALID_ARGUMENT;
    }

    HAL_StatusTypeDef status =
            HAL_I2C_Mem_Read(
                    hi2c,
                    (uint16_t)(device_address << 1U),
                    start_register,
                    I2C_MEMADD_SIZE_8BIT,
                    data,
                    length,
                    timeout_ms);

    return hal_i2c_convert_status(status);
}

static DeviceIO_Status_t hal_i2c_read_registers_it(
        void *context,
        uint8_t device_address,
        uint8_t start_register,
        uint8_t *data,
        uint16_t length)
{
    I2C_HandleTypeDef *hi2c =
            (I2C_HandleTypeDef *)context;

    if (hi2c == NULL
            || data == NULL
            || length == 0U) {
        return DEVICE_IO_INVALID_ARGUMENT;
    }

    HAL_StatusTypeDef status =
            HAL_I2C_Mem_Read_IT(
                    hi2c,
                    (uint16_t)(device_address << 1U),
                    start_register,
                    I2C_MEMADD_SIZE_8BIT,
                    data,
                    length);

    return hal_i2c_convert_status(status);
}

static void hal_i2c_delay_ms(
        void *context,
        uint32_t delay_ms)
{
    (void)context;

    HAL_Delay(delay_ms);
}

static uint32_t hal_i2c_get_tick_ms(
        void *context)
{
    (void)context;

    return HAL_GetTick();
}

static DeviceIO_Status_t hal_i2c_abort_it(
		void* context,
		uint8_t device_address) {
	I2C_HandleTypeDef* hi2c = (I2C_HandleTypeDef*) context;
	if(hi2c == NULL) return DEVICE_IO_INVALID_ARGUMENT;
	HAL_StatusTypeDef status =
	            HAL_I2C_Master_Abort_IT(
	                    hi2c,
	                    (uint16_t)(device_address << 1U));
	return hal_i2c_convert_status(status);
}

static const DeviceIO_Ops_t hal_i2c_device_io_ops = {
    .write_registers = hal_i2c_write_registers,
    .read_registers = hal_i2c_read_registers,
    .read_registers_it = hal_i2c_read_registers_it,
    .delay_ms = hal_i2c_delay_ms,
    .get_tick_ms = hal_i2c_get_tick_ms,
	.abort_it = hal_i2c_abort_it
};

DeviceIO_Status_t HAL_I2C_DeviceIO_Init(
		I2C_HandleTypeDef *hi2c,
        DeviceIO_t *device_io)
{
    if (device_io == NULL || hi2c == NULL) {
        return DEVICE_IO_INVALID_ARGUMENT;
    }

    device_io->context = hi2c;
    device_io->ops = &hal_i2c_device_io_ops;

    return DEVICE_IO_OK;
}
