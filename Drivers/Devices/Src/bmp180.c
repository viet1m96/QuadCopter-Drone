/*
 * bmp180.c
 *
 *  Created on: Jul 11, 2026
 *      Author: vietht-hl
 */

#include "bmp180.h"


static BMP180_Status_t bmp180_write_reg(BMP180_Handle_t* bmp, uint8_t reg, uint8_t value) {
	if (bmp == NULL || bmp -> hi2c == NULL) {
		return BMP180_ERR_NULL;
	}
	uint16_t DevAddress = (bmp -> address << 1);
	HAL_StatusTypeDef wStatus = HAL_I2C_Mem_Write(
			bmp -> hi2c,
			DevAddress,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			&value,
			1,
			100);
	if(wStatus == HAL_BUSY) {
		return BMP180_I2C_BUSY;
	} else if(wStatus == HAL_ERROR) {
		return BMP180_I2C_ERROR;
	} else if(wStatus == HAL_TIMEOUT) {
		return BMP180_I2C_TIMEOUT;
	}
	return BMP180_OK;
}

static BMP180_Status_t bmp180_read_reg(BMP180_Handle_t* bmp, uint8_t reg, uint8_t* result) {
	if (bmp == NULL || bmp -> hi2c == NULL || result == NULL) {
		return BMP180_ERR_NULL;
	}
	uint16_t DevAddress = (bmp -> address << 1);
	HAL_StatusTypeDef rStatus = HAL_I2C_Mem_Read(
			bmp -> hi2c,
			DevAddress,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			result,
			1,
			100);
	if(rStatus == HAL_BUSY) {
			return BMP180_I2C_BUSY;
		} else if(rStatus == HAL_ERROR) {
			return BMP180_I2C_ERROR;
		} else if(rStatus == HAL_TIMEOUT) {
			return BMP180_I2C_TIMEOUT;
		}
	return BMP180_OK;
}

static BMP180_Status_t bmp180_read_regs(BMP180_Handle_t* bmp, uint8_t start_reg, uint8_t len, uint8_t* result) {
	if(bmp == NULL || bmp -> hi2c == NULL || result == NULL) {
		return BMP180_ERR_NULL;
	}
	if(len == 0) {
		return BMP180_INVALID_CONFIG;
	}
	uint16_t DevAddress = (bmp -> address << 1);
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
			bmp->hi2c,
			DevAddress,
			start_reg,
			I2C_MEMADD_SIZE_8BIT,
			result,
			len,
			100
		);

   if (status == HAL_BUSY) {
		return BMP180_I2C_BUSY;
   } else if (status == HAL_TIMEOUT) {
		return BMP180_I2C_TIMEOUT;
   } else {
		if(status == HAL_OK) return BMP180_OK;
		return BMP180_I2C_ERROR;
   }
}

BMP180_Status_t BMP180_Init(BMP180_Handle_t* bmp, I2C_HandleTypeDef* hi2c, uint8_t address) {
	if(bmp == NULL || hi2c == NULL) return BMP180_ERR_NULL;
	if (address > 0x7FU) {
		return BMP180_INVALID_CONFIG;
	}

	bmp -> hi2c = hi2c;
	bmp -> address = address;
	bmp -> pending_oss = BMP180_PRESSURE_OSS0;
	bmp -> startup_pressure_pa = 0U;
	return BMP180_CheckDeviceID(bmp);
}

static uint16_t bmp180_make_u16(uint8_t msb, uint8_t lsb)
{
    return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

static int16_t bmp180_make_s16(uint8_t msb, uint8_t lsb)
{
    return (int16_t)bmp180_make_u16(msb, lsb);
}

BMP180_Status_t BMP180_CheckDeviceID(BMP180_Handle_t* bmp) {
	if(bmp == NULL) return BMP180_ERR_NULL;
	uint8_t result = 0;
	BMP180_Status_t status = bmp180_read_reg(bmp, BMP180_REG_CHIP_ID, &result);
	if(status == BMP180_OK) {
		if(result == BMP180_CHIP_ID_VALUE) {
			return BMP180_OK;
		}
		return BMP180_ERR_BAD_DEVICE_ID;
	}
	return status;
}

BMP180_Status_t BMP180_ReadCalibrationOffsets(BMP180_Handle_t* bmp, BMP180_Calibration_t* offsets) {
	if(bmp == NULL || offsets == NULL) return BMP180_ERR_NULL;
	uint8_t data[22];
	BMP180_Status_t status = bmp180_read_regs(bmp, BMP180_REG_AC1, sizeof(data), data);
	if(status != BMP180_OK) return status;
	offsets -> ac1 = bmp180_make_s16(data[0],  data[1]);
	offsets -> ac2 = bmp180_make_s16(data[2],  data[3]);
	offsets -> ac3 = bmp180_make_s16(data[4],  data[5]);

	offsets -> ac4 = bmp180_make_u16(data[6],  data[7]);
	offsets -> ac5 = bmp180_make_u16(data[8],  data[9]);
	offsets -> ac6 = bmp180_make_u16(data[10], data[11]);

	offsets -> b1 = bmp180_make_s16(data[12], data[13]);
	offsets -> b2 = bmp180_make_s16(data[14], data[15]);
	offsets -> mb = bmp180_make_s16(data[16], data[17]);
	offsets -> mc = bmp180_make_s16(data[18], data[19]);
	offsets -> md = bmp180_make_s16(data[20], data[21]);
	return BMP180_OK;
}

BMP180_Status_t BMP180_SendControlCmd(BMP180_Handle_t* bmp, uint8_t cmd_val, uint8_t oss_value) {
	BMP180_Status_t status;
	switch(cmd_val) {
	case BMP180_CMD_PRESSURE: {
		if(oss_value > BMP180_PRESSURE_OSS3) return BMP180_INVALID_CONFIG;
		uint8_t cmd = cmd_val | (oss_value << 6);
		status = bmp180_write_reg(bmp, BMP180_REG_CTRL_MEAS, cmd);
		if(status != BMP180_OK) return status;
		bmp -> pending_oss = oss_value;
		return BMP180_OK;
	}

	case BMP180_CMD_TEMPERATURE: {
		return bmp180_write_reg(bmp, BMP180_REG_CTRL_MEAS, BMP180_CMD_TEMPERATURE);
	}
	default:
		return BMP180_INVALID_CONFIG;
	}
}

BMP180_Status_t BMP180_ReadRawTemperature(BMP180_Handle_t* bmp, BMP180_RawData_t* raw) {
	if(bmp == NULL || raw == NULL) return BMP180_ERR_NULL;
	uint8_t result[2];
	BMP180_Status_t status = bmp180_read_regs(bmp, BMP180_REG_MSB, 2, result);
	if(status != BMP180_OK) return status;
	raw -> temperature = (uint16_t)(result[0] << 8) | (uint16_t)result[1];
	return BMP180_OK;
}
BMP180_Status_t BMP180_ReadRawPressure(BMP180_Handle_t* bmp, BMP180_RawData_t* raw) {
	if(bmp == NULL || raw == NULL) return BMP180_ERR_NULL;
	if (bmp -> pending_oss > BMP180_PRESSURE_OSS3) {
		return BMP180_INVALID_CONFIG;
	}
	uint8_t result[3];
	BMP180_Status_t status = bmp180_read_regs(bmp, BMP180_REG_MSB, 3, result);
	if(status != BMP180_OK) return status;
	uint32_t raw24 = ((uint32_t)result[0] << 16) | ((uint32_t)result[1] << 8) | ((uint32_t)result[2]);
	raw -> pressure = raw24 >> (8U - bmp -> pending_oss);
	return BMP180_OK;
}

static BMP180_Status_t bmp180_calculate_b5(
        const BMP180_RawData_t *raw,
        const BMP180_Calibration_t *offsets,
        int32_t *b5,
        int32_t *temperature_01c)
{
    if (raw == NULL || offsets == NULL || b5 == NULL || temperature_01c == NULL) {
        return BMP180_ERR_NULL;
    }

    int32_t x1;
    int32_t x2;
    int32_t denominator;

    x1 = (int32_t)(
    				(((int64_t)raw -> temperature - (int64_t)offsets->ac6) * (int64_t)offsets -> ac5) >> 15
    );

    denominator = x1 + (int32_t)offsets -> md;

    if (denominator == 0) {
        return BMP180_INVALID_CONFIG;
    }


    x2 = (int32_t)(
        ((int64_t)offsets -> mc * 2048LL) / (int64_t)denominator
    );

    *b5 = x1 + x2;

    *temperature_01c = (*b5 + 8) >> 4;

    return BMP180_OK;
}

BMP180_Status_t BMP180_ConvertRawTempToScaled(
        BMP180_Handle_t *bmp,
        const BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets)
{
    if (bmp == NULL || raw == NULL || scaled == NULL || offsets == NULL) {
        return BMP180_ERR_NULL;
    }

    int32_t b5;
    int32_t temperature_01c;

    BMP180_Status_t status = bmp180_calculate_b5(
        raw,
        offsets,
        &b5,
        &temperature_01c
    );

    if (status != BMP180_OK) {
        return status;
    }

    scaled -> temperature_c = (float)temperature_01c / 10.0f;

    return BMP180_OK;
}
static BMP180_Status_t bmp180_calculate_pressure_pa(
        const BMP180_Handle_t *bmp,
        const BMP180_RawData_t *raw,
        const BMP180_Calibration_t *offsets,
        int32_t b5,
        uint32_t *pressure_pa)
{
    if (bmp == NULL || raw == NULL || offsets == NULL || pressure_pa == NULL) {
        return BMP180_ERR_NULL;
    }

    if (bmp->pending_oss > 3U) {
        return BMP180_INVALID_CONFIG;
    }

    uint8_t oss = bmp -> pending_oss;

    int32_t b6;
    int32_t b3;
    uint32_t b4;
    uint64_t b7;

    int32_t x1;
    int32_t x2;
    int32_t x3;

    b6 = b5 - 4000;

    /*
     * X1 = (B2 * (B6 * B6 / 2^12)) / 2^11
     */
    int32_t b6_squared_div_4096 = (int32_t)(
        ((int64_t)b6 * (int64_t)b6) >> 12
    );

    x1 = (int32_t)(
        ((int64_t)offsets -> b2 *
         (int64_t)b6_squared_div_4096) >> 11
    );

    /*
     * X2 = AC2 * B6 / 2^11
     */
    x2 = (int32_t)(
        ((int64_t)offsets -> ac2 *
         (int64_t)b6) >> 11
    );

    x3 = x1 + x2;

    /*
     * B3 = (((AC1 * 4 + X3) << OSS) + 2) / 4
     */
    b3 = (int32_t)(
        ((((int64_t)offsets -> ac1 * 4LL +
           (int64_t)x3) *
          (1LL << oss)) + 2LL) >> 2
    );

    /*
     * X1 = AC3 * B6 / 2^13
     */
    x1 = (int32_t)(
        ((int64_t)offsets -> ac3 *
         (int64_t)b6) >> 13
    );

    /*
     * X2 = (B1 * (B6 * B6 / 2^12)) / 2^16
     */
    x2 = (int32_t)(
        ((int64_t)offsets -> b1 *
         (int64_t)b6_squared_div_4096) >> 16
    );

    /*
     * X3 = (X1 + X2 + 2) / 2^2
     */
    x3 = (x1 + x2 + 2) >> 2;

    int32_t b4_factor = x3 + 32768;

    if (b4_factor <= 0) {
        return BMP180_INVALID_CONFIG;
    }

    /*
     * B4 = AC4 * (unsigned long)(X3 + 32768) / 2^15
     */
    b4 = (uint32_t)(
        ((uint64_t)offsets -> ac4 *
         (uint64_t)(uint32_t)b4_factor) >> 15
    );

    if (b4 == 0U) {
        return BMP180_INVALID_CONFIG;
    }

    /*
     * B7 = ((unsigned long)UP - B3) * (50000 >> OSS)
     */
    int64_t up_minus_b3 =
        (int64_t)raw -> pressure - (int64_t)b3;

    if (up_minus_b3 <= 0) {
        return BMP180_INVALID_CONFIG;
    }

    b7 = (uint64_t)up_minus_b3 *
         (uint64_t)(50000U >> oss);

    uint32_t p;

    if (b7 < 0x80000000ULL) {
        p = (uint32_t)((b7 * 2ULL) / b4);
    } else {
        p = (uint32_t)((b7 / b4) * 2ULL);
    }

    /*
     * X1 = (p / 2^8) * (p / 2^8)
     * X1 = X1 * 3038 / 2^16
     */
    int32_t p_div_256 = (int32_t)(p >> 8);

    x1 = (int32_t)(
        (((int64_t)p_div_256 *
          (int64_t)p_div_256) *
         3038LL) >> 16
    );

    /*
     * X2 = (-7357 * p) / 2^16
     */
    x2 = (int32_t)(
        (-(int64_t)7357 * (int64_t)p) >> 16
    );

    /*
     * p = p + (X1 + X2 + 3791) / 2^4
     */
    int64_t final_pressure =
        (int64_t)p +
        (int64_t)((x1 + x2 + 3791) >> 4);

    if (final_pressure <= 0) {
        return BMP180_INVALID_CONFIG;
    }

    *pressure_pa = (uint32_t)final_pressure;

    return BMP180_OK;
}

BMP180_Status_t BMP180_ConvertRawPressToScaled(
        BMP180_Handle_t *bmp,
        const BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets)
{
    if (bmp == NULL || raw == NULL || scaled == NULL || offsets == NULL) {
        return BMP180_ERR_NULL;
    }

    int32_t b5;
    int32_t temperature_01c;

    BMP180_Status_t status = bmp180_calculate_b5(
        raw,
        offsets,
        &b5,
        &temperature_01c
    );

    if (status != BMP180_OK) {
        return status;
    }

    return bmp180_calculate_pressure_pa(
        bmp,
        raw,
        offsets,
        b5,
        &scaled -> pressure_pa
    );
}


BMP180_Status_t BMP180_ConvertRawToScaled(
        BMP180_Handle_t *bmp,
        const BMP180_RawData_t *raw,
        BMP180_Data_t *scaled,
        const BMP180_Calibration_t *offsets)
{
    if (bmp == NULL || raw == NULL || scaled == NULL || offsets == NULL) {
        return BMP180_ERR_NULL;
    }

    int32_t b5;
    int32_t temperature_01c;

    BMP180_Status_t status = bmp180_calculate_b5(
        raw,
        offsets,
        &b5,
        &temperature_01c
    );

    if (status != BMP180_OK) {
        return status;
    }

    scaled -> temperature_c = (float)temperature_01c / 10.0f;

    status = bmp180_calculate_pressure_pa(
        bmp,
        raw,
        offsets,
        b5,
        &scaled->pressure_pa
    );

    return status;
}

BMP180_Status_t BMP180_CalculateAltitude(uint32_t pressure_pa, float *altitude_m)
{
    if (altitude_m == NULL) {
        return BMP180_ERR_NULL;
    }

    if (pressure_pa == 0U) {
        return BMP180_INVALID_CONFIG;
    }

    float pressure_ratio = (float)pressure_pa / BMP180_STANDARD_SEA_LEVEL_PRESSURE_PA;

    *altitude_m = 44330.0f * (1.0f - powf(pressure_ratio, BMP180_ALTITUDE_EXPONENT));

    return BMP180_OK;
}


BMP180_Status_t BMP180_CalculateRelativeAltitude(
        float current_pressure_pa,
        float startup_pressure_pa,
        float *relative_altitude_m)
{
    if(relative_altitude_m == NULL) {
        return BMP180_ERR_NULL;
    }

    if(current_pressure_pa <= 0.0f ||
       startup_pressure_pa <= 0.0f)
    {
        return BMP180_INVALID_CONFIG;
    }

    float pressure_ratio =
        current_pressure_pa / startup_pressure_pa;

    *relative_altitude_m =
        44330.0f *
        (1.0f - powf(
            pressure_ratio,
            BMP180_ALTITUDE_EXPONENT
        ));

    return BMP180_OK;
}

void BMP180_NotiStatus(BMP180_Status_t status, char* str) {

	printf("%s\r\n", str);
	switch(status) {
	case BMP180_I2C_BUSY:
		printf("I2C is busy now %d\r\n", status);
		break;
	case BMP180_I2C_ERROR:
		printf("There is some problem with I2C communication %d\r\n", status);
		break;
	case BMP180_I2C_TIMEOUT:
		printf("I2C communication is time out %d\r\n", status);
		break;
	case BMP180_ERR_NULL:
		printf("There is a null object %d\r\n", status);
		break;
	case BMP180_ERR_BAD_DEVICE_ID:
		printf("The device id is incorrect %d\r\n", status);
		break;
	case BMP180_INVALID_CONFIG:
		printf("There are some invalid config %d\r\n", status);
		break;
	default:
		return;
	}
	return;

}

BMP180_Status_t BMP180_MeasureData(
		BMP180_Handle_t* bmp,
		BMP180_RawData_t* raw,
		BMP180_Data_t* scaled,
		const BMP180_Calibration_t* offsets,
		BMP180_PressureWindow_t* window
		) {
	if(bmp == NULL
			|| raw == NULL
			|| scaled == NULL
			|| offsets == NULL
			) {
		return BMP180_ERR_NULL;
	}
	BMP180_Status_t status = BMP180_SendControlCmd(bmp, BMP180_CMD_TEMPERATURE, 0U);
	if(status != BMP180_OK) {
		BMP180_NotiStatus(status, "Temp command failed");
		return status;
	}
	HAL_Delay(5U);
	status = BMP180_ReadRawTemperature(bmp, raw);
	if(status != BMP180_OK) {
		BMP180_NotiStatus(status, "Temp read failed");
		return status;
	}
	status = BMP180_SendControlCmd(bmp, BMP180_CMD_PRESSURE, BMP180_PRESSURE_OSS3);
	if(status != BMP180_OK) {
		BMP180_NotiStatus(status, "Pressure command failed");
		return status;
	}
	HAL_Delay(BMP180_OSS3_DELAY_MS);

	status = BMP180_ReadRawPressure(bmp, raw);
	if(status != BMP180_OK) {
		BMP180_NotiStatus(status, "Pressure read failed");
		return status;
	}
	status = BMP180_ConvertRawToScaled(bmp, raw, scaled, offsets);
	if(status != BMP180_OK) {
		BMP180_NotiStatus(status, "Convert failed %d\r\n");
		return status;
	}
	BMP180_WindowPush(window, scaled -> pressure_pa);
	return status;
}

static int comp(const void *a, const void *b)
{
    float x = *(const float *)a;
    float y = *(const float *)b;

    if (x < y) return -1;
    if (x > y) return 1;
    return 0;
}

BMP180_Status_t BMP180_SetStartupPressurePa(
		BMP180_Handle_t* bmp,
		BMP180_RawData_t* raw,
		BMP180_Data_t* scaled,
		const BMP180_Calibration_t* offsets,
		BMP180_PressureWindow_t* window) {
	float pressures[64];
	BMP180_Status_t status;
	for(uint8_t i = 0U; i < 64U; i++) {

	    status = BMP180_MeasureData(
	        bmp,
	        raw,
	        scaled,
	        offsets,
	        window
	    );

	    if(status != BMP180_OK) {
	        BMP180_NotiStatus(
	            status,
	            "Set startup pressure failed"
	        );
	        return status;
	    }

	    pressures[i] = (float)scaled->pressure_pa;

	    HAL_Delay(100U);
	}
	qsort(pressures, sizeof(pressures) / sizeof(pressures[0]), sizeof(pressures[0]), comp);
	float sum = 0.0f;
	for(uint8_t i = 8; i < 56; i++) {
		sum += pressures[i];
	}
	sum /= 48;
	bmp -> startup_pressure_pa = sum;
	return BMP180_OK;
}


void BMP180_WindowInit(BMP180_PressureWindow_t* window) {
	if(window == NULL) {
		BMP180_NotiStatus(BMP180_ERR_NULL, "Window is null");
		return;
	}
	window -> count = 0U;
	window -> write_index = 0U;
	window -> sum = 0.0f;
	for(uint8_t i = 0; i < BMP180_WINDOW_SIZE; i++) {
		window -> pressure_pa[i] = 0.0f;
	}
}
void BMP180_WindowPush(BMP180_PressureWindow_t* window, float pressure_pa) {
	if(window == NULL) {
		BMP180_NotiStatus(BMP180_ERR_NULL, "Window is null");
		return;
	}
	if(window -> count == BMP180_WINDOW_SIZE) {
		window -> sum -= window -> pressure_pa[window -> write_index];
	} else {
		window -> count++;
	}
	window -> pressure_pa[window -> write_index] = pressure_pa;
	window -> sum += pressure_pa;
	window -> write_index++;
	if(window -> write_index >= BMP180_WINDOW_SIZE) {
		window -> write_index = 0U;
	}
	return;
}
float BMP180_WindowGetAvg(BMP180_PressureWindow_t* window) {
	if(window == NULL || window -> count == 0.0f) {
		return 0.0f;
	}
	return window -> sum / (float)window -> count;
}

void BMP180_FilterInit(BMP180_Filter_t* filter)
{
    if(filter == NULL) {
        BMP180_NotiStatus(BMP180_ERR_NULL, "Filter is null");
        return;
    }

    filter->alpha_low = BMP180_FILTER_ALPHA_LOW;
    filter->alpha_medium = BMP180_FILTER_ALPHA_MEDIUM;
    filter->alpha_high = BMP180_FILTER_ALPHA_HIGH;

    filter->threshold_low_pa =
        BMP180_FILTER_THRESHOLD_LOW_PA;

    filter->threshold_high_pa =
        BMP180_FILTER_THRESHOLD_HIGH_PA;

    filter->last_pressure_pa = 0.0f;
    filter->active_alpha = filter->alpha_low;
    filter->first_data = 1U;
}
float BMP180_EMAFilter(
        BMP180_Filter_t* filter,
        float current_pressure_pa)
{
    if(filter == NULL) {
        BMP180_NotiStatus(
            BMP180_ERR_NULL,
            "Filter is null"
        );
        return 0.0f;
    }

    if(current_pressure_pa <= 0.0f) {
        return filter->last_pressure_pa;
    }

    if(filter->first_data) {
        filter->last_pressure_pa = current_pressure_pa;
        filter->active_alpha = filter->alpha_low;
        filter->first_data = 0U;

        return current_pressure_pa;
    }

    float pressure_error = fabsf(
        current_pressure_pa -
        filter->last_pressure_pa
    );

    if(pressure_error < filter->threshold_low_pa) {
        filter->active_alpha = filter->alpha_low;
    }
    else if(pressure_error < filter->threshold_high_pa) {
        filter->active_alpha = filter->alpha_medium;
    }
    else {
        filter->active_alpha = filter->alpha_high;
    }

    filter->last_pressure_pa +=
        filter->active_alpha *
        (current_pressure_pa -
         filter->last_pressure_pa);

    return filter->last_pressure_pa;
}









;
