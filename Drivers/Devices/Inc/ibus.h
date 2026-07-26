/*
 * ibus.h
 *
 *  Created on: Jul 26, 2026
 *      Author: vietht-hl
 */

#ifndef DEVICES_INC_IBUS_H_
#define DEVICES_INC_IBUS_H_

#include "stm32f4xx_hal.h"

#define IBUS_CHANNEL_QUANTITY 14U
#define IBUS_FRAME_SIZE 32U
#define IBUS_FRAME_CHECKSUM 0xFFFFU
#define IBUS_BYTE_SIZE 0x20U
#define IBUS_BYTE_TYPE 0x40U
#define IBUS_CHANNEL_BYTE_INDEX(i)    (((i) * 2U) + 2U)
#define IBUS_TIMEOUT_MS 50U

typedef enum {
	IBUS_OK = 0,
	IBUS_ERR_NULL,
	IBUS_ERR_UNINITIALIZED,
	IBUS_ERR_INVALID_STATE,
	IBUS_ERR_UART,
	IBUS_ERR_INVALID_FRAME,
	IBUS_ERR_INVALID_CHECKSUM,
	IBUS_ERR_INVALID_FRAME_HEADER,
	IBUS_ERR_TIMEOUT
} IBUS_Status_t;

typedef enum {
	IBUS_LINK_STOPPED = 0,
	IBUS_LINK_WAITING,
	IBUS_LINK_ACTIVE,
	IBUS_LINK_LOST
} IBUS_LinkState_t;

typedef struct {
	uint16_t channels[IBUS_CHANNEL_QUANTITY];
	uint32_t timestamp_ms;
} IBUS_Data_t;

typedef struct {
	UART_HandleTypeDef* huart;
	IBUS_Data_t latest_valid_data;
	volatile IBUS_LinkState_t state;
	uint8_t rxBuffer[IBUS_FRAME_SIZE];
	uint8_t initialized;
} IBUS_Handle_t;

IBUS_Status_t IBUS_Start(
		IBUS_Handle_t* ibus);
IBUS_Status_t IBUS_Stop(
		IBUS_Handle_t* ibus);
IBUS_Status_t IBUS_OnRxEvent(
		IBUS_Handle_t* ibus,
		uint32_t now_ms,
		uint16_t received_size);
IBUS_Status_t IBUS_Update(
		IBUS_Handle_t* ibus,
		uint32_t now_ms);
IBUS_Status_t IBUS_Init(
		IBUS_Handle_t* ibus,
		UART_HandleTypeDef* huart);


#endif /* DEVICES_INC_IBUS_H_ */
