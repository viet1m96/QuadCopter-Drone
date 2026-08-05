/*
 * hal_ibus_transport.c
 *
 *  Created on: Aug 5, 2026
 *      Author: vietht-hl
 */


#include "hal_ibus_transport.h"

HAL_IBUS_TransportStatus_t HAL_IBUS_TransportInit(
		HAL_IBUS_Transport_t* transport,
		UART_HandleTypeDef* huart) {
	if(transport == NULL || huart == NULL) {
		return HAL_IBUS_TRANSPORT_ERR_NULL;
	}
	if(huart->hdmarx == NULL) {
		return HAL_IBUS_TRANSPORT_ERR_CONFIG;
	}
	transport->huart = huart;
	return HAL_IBUS_TRANSPORT_OK;
}

HAL_IBUS_TransportStatus_t HAL_IBUS_TransportStart(
		HAL_IBUS_Transport_t* transport) {
	if(transport == NULL ||
	   transport->huart == NULL) {
		return HAL_IBUS_TRANSPORT_ERR_NULL;
	}
	const HAL_StatusTypeDef status =
			HAL_UARTEx_ReceiveToIdle_DMA(
					transport->huart,
					transport->rx_buffer,
					IBUS_FRAME_SIZE);
	if(status != HAL_OK) {
		return HAL_IBUS_TRANSPORT_ERR_HAL;
	}
	__HAL_DMA_DISABLE_IT(
	            transport->huart->hdmarx,
	            DMA_IT_HT);
	return HAL_IBUS_TRANSPORT_OK;
}

HAL_IBUS_TransportStatus_t HAL_IBUS_TransportStop(
		HAL_IBUS_Transport_t* transport) {
	if(transport == NULL ||
	   transport->huart == NULL) {
		return HAL_IBUS_TRANSPORT_ERR_NULL;
	}
	if(HAL_UART_DMAStop(transport->huart) != HAL_OK) {
		return HAL_IBUS_TRANSPORT_ERR_HAL;
	}
	return HAL_IBUS_TRANSPORT_OK;
}
























