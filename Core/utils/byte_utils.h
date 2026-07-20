/*
 * byte_utils.h
 *
 *  Created on: Jul 17, 2026
 *      Author: vietht-hl
 */

#ifndef UTILS_BYTE_UTILS_H_
#define UTILS_BYTE_UTILS_H_

#include "stdint.h"

static inline uint16_t byte_utils_u16_from_be(
        uint8_t msb,
        uint8_t lsb)
{
    return ((uint16_t)msb << 8)
         |  (uint16_t)lsb;
}

static inline int16_t byte_utils_i16_from_be(
		uint8_t msb,
		uint8_t lsb) {
	return (int16_t) byte_utils_u16_from_be(msb, lsb);
}


#endif /* UTILS_BYTE_UTILS_H_ */
