/*
 * ibus.c
 *
 *  Created on: Jul 26, 2026
 *      Author: vietht-hl
 */

#include "ibus.h"
#include "byte_utils.h"
#include "string.h"


static IBUS_Status_t ibus_start_receive(IBUS_Handle_t* ibus) {
	if(ibus == NULL || ibus->huart == NULL || ibus->huart->hdmarx == NULL) return IBUS_ERR_NULL;
	HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
								ibus->huart,
								ibus->rxBuffer,
								IBUS_FRAME_SIZE);
	if(status != HAL_OK) return IBUS_ERR_UART;
	__HAL_DMA_DISABLE_IT(
	    ibus->huart->hdmarx,
	    DMA_IT_HT);
	return IBUS_OK;
}

IBUS_Status_t IBUS_Start(IBUS_Handle_t* ibus) {
	if(ibus == NULL ||
	   ibus->huart == NULL) {
		return IBUS_ERR_NULL;
	}
	if(ibus->initialized == 0U) {
		return IBUS_ERR_UNINITIALIZED;
	}
	if(ibus->state != IBUS_LINK_STOPPED) {
		return IBUS_ERR_INVALID_STATE;
	}
	IBUS_Status_t status = ibus_start_receive(ibus);
	if(status != IBUS_OK) return status;
	ibus->state = IBUS_LINK_WAITING;
	return IBUS_OK;
}

IBUS_Status_t IBUS_Stop(IBUS_Handle_t *ibus)
{
    if (ibus == NULL || ibus->huart == NULL) {
        return IBUS_ERR_NULL;
    }

    if (ibus->initialized == 0U) {
        return IBUS_ERR_UNINITIALIZED;
    }

    if (ibus->state == IBUS_LINK_STOPPED) {
        return IBUS_ERR_INVALID_STATE;
    }

    HAL_StatusTypeDef status =
            HAL_UART_AbortReceive(ibus->huart);

    if (status != HAL_OK) {
        return IBUS_ERR_UART;
    }

    ibus->state = IBUS_LINK_STOPPED;

    return IBUS_OK;
}

static IBUS_Status_t ibus_decode_channels(
		uint16_t* data,
		const uint8_t* rxBuffer) {
	if(data == NULL || rxBuffer == NULL) return IBUS_ERR_NULL;
	for(uint32_t i = 0; i < IBUS_CHANNEL_QUANTITY; i++) {
		uint32_t idx = IBUS_CHANNEL_BYTE_INDEX(i);
		data[i] = byte_utils_u16_from_be(rxBuffer[idx + 1], rxBuffer[idx]);
	}
	return IBUS_OK;
}

static uint16_t ibus_cal_checksum(
		const uint8_t* rxBuffer) {
	uint16_t checksum = IBUS_FRAME_CHECKSUM;
	for(uint32_t i = 0; i < IBUS_FRAME_SIZE - 2U; i++) {
		checksum = (uint16_t)(checksum - rxBuffer[i]);
	}
	return checksum;
}





IBUS_Status_t IBUS_OnRxEvent(
        IBUS_Handle_t *ibus,
        uint32_t now_ms,
        uint16_t received_size)
{
    if ((ibus == NULL) || (ibus->huart == NULL)) {
        return IBUS_ERR_NULL;
    }

    if (ibus->initialized == 0U) {
        return IBUS_ERR_UNINITIALIZED;
    }

    if (ibus->state == IBUS_LINK_STOPPED || ibus->state == IBUS_LINK_LOST) {
        return IBUS_ERR_INVALID_STATE;
    }

    IBUS_Status_t frame_status = IBUS_OK;

    if (received_size != IBUS_FRAME_SIZE) {
        frame_status = IBUS_ERR_INVALID_FRAME;
    }
    else if ((ibus->rxBuffer[0] != IBUS_BYTE_SIZE) ||
             (ibus->rxBuffer[1] != IBUS_BYTE_TYPE)) {
        frame_status = IBUS_ERR_INVALID_FRAME_HEADER;
    }
    else {
        uint16_t received_checksum =
                (uint16_t)ibus->rxBuffer[30] |
                ((uint16_t)ibus->rxBuffer[31] << 8U);

        uint16_t calculated_checksum =
                ibus_cal_checksum(ibus->rxBuffer);

        if (received_checksum != calculated_checksum) {
            frame_status = IBUS_ERR_INVALID_CHECKSUM;
        }
        else {
            frame_status = ibus_decode_channels(
                    ibus->latest_valid_data.channels,
                    ibus->rxBuffer);

            if (frame_status == IBUS_OK) {
                ibus->latest_valid_data.timestamp_ms = now_ms;
                ibus->state = IBUS_LINK_ACTIVE;
            }
        }
    }

    IBUS_Status_t restart_status = ibus_start_receive(ibus);

    if (restart_status != IBUS_OK) {
        ibus->state = IBUS_LINK_STOPPED;
        return restart_status;
    }

    return frame_status;
}

IBUS_Status_t IBUS_Update(
		IBUS_Handle_t* ibus,
		uint32_t now_ms) {
	if(ibus == NULL) return IBUS_ERR_NULL;
	if(ibus->initialized == 0U) return IBUS_ERR_UNINITIALIZED;
	if(ibus->state == IBUS_LINK_ACTIVE) {
		if((now_ms - ibus->latest_valid_data.timestamp_ms) >= IBUS_TIMEOUT_MS) {
			ibus->state = IBUS_LINK_LOST;
			return IBUS_ERR_TIMEOUT;
		}
	} else if(ibus->state == IBUS_LINK_LOST) {
		return IBUS_ERR_TIMEOUT;
	}

	return IBUS_OK;
}

IBUS_Status_t IBUS_Init(
		IBUS_Handle_t* ibus,
		UART_HandleTypeDef* huart) {
	if(ibus == NULL || huart == NULL || huart->Instance == NULL) return IBUS_ERR_NULL;
	memset(ibus, 0, sizeof(*ibus));
	ibus->state = IBUS_LINK_STOPPED;
	ibus->huart = huart;
	ibus->initialized = 1U;
	return IBUS_OK;
}





