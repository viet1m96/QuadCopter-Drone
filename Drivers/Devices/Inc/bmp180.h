/*
 * bmp180.h
 *
 *  Created on: Jul 11, 2026
 *      Author: vietht-hl
 */

#ifndef DEVICES_INC_BMP180_H_
#define DEVICES_INC_BMP180_H_


typedef struct {
	I2C_HandleTypeDef* hi2c;
	uint8_t address;
} BMP180_Handle_t;


#endif /* DEVICES_INC_BMP180_H_ */
