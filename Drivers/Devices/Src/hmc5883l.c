/*
 * hmc5883l.c
 *
 *  Created on: Jul 15, 2026
 *      Author: vietht-hl
 */

#include "hmc5883l.h"

static HMC5883L_Status_t hmc5883l_write_reg(HMC5883L_Handle_t* hmc, uint8_t reg, uint8_t value) {
	if (hmc == NULL || hmc -> hi2c == NULL) {
		return HMC5883L_ERR_NULL;
	}
	uint16_t DevAddress = (hmc -> address << 1);
	HAL_StatusTypeDef wStatus = HAL_I2C_Mem_Write(
			hmc -> hi2c,
			DevAddress,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			&value,
			1,
			100);
	if(wStatus == HAL_BUSY) {
		return HMC5883L_I2C_BUSY;
	} else if(wStatus == HAL_ERROR) {
		return HMC5883L_I2C_ERROR;
	} else if(wStatus == HAL_TIMEOUT) {
		return HMC5883L_I2C_TIMEOUT;
	}
	return HMC5883L_OK;

}

static HMC5883L_Status_t hmc5883l_read_reg(HMC5883L_Handle_t* hmc, uint8_t reg, uint8_t* result) {
	if (hmc == NULL || hmc -> hi2c == NULL || result == NULL) {
		return HMC5883L_ERR_NULL;
	}
	uint16_t DevAddress = (hmc -> address << 1);
	HAL_StatusTypeDef rStatus = HAL_I2C_Mem_Read(
			hmc -> hi2c,
			DevAddress,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			result,
			1,
			100);
	if(rStatus == HAL_BUSY) {
			return HMC5883L_I2C_BUSY;
		} else if(rStatus == HAL_ERROR) {
			return HMC5883L_I2C_ERROR;
		} else if(rStatus == HAL_TIMEOUT) {
			return HMC5883L_I2C_TIMEOUT;
		}
	return HMC5883L_OK;
}


static HMC5883L_Status_t hmc5883l_read_regs(HMC5883L_Handle_t* hmc, uint8_t start_reg, uint8_t len, uint8_t* result) {
	if(hmc == NULL || hmc -> hi2c == NULL || result == NULL) {
		return HMC5883L_ERR_NULL;
	}
	if(len == 0) {
		return HMC5883L_INVALID_CONFIG;
	}
	uint16_t DevAddress = (hmc -> address << 1);
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
	        hmc->hi2c,
	        DevAddress,
	        start_reg,
	        I2C_MEMADD_SIZE_8BIT,
	        result,
	        len,
	        100
	    );

   if (status == HAL_BUSY) {
		return HMC5883L_I2C_BUSY;
   } else if (status == HAL_TIMEOUT) {
		return HMC5883L_I2C_TIMEOUT;
   } else {
	   	if(status == HAL_OK) return HMC5883L_OK;
		return HMC5883L_I2C_ERROR;
   }
}

void HMC5883L_NotiStatus(HMC5883L_Status_t status, const char* str) {
	printf("%s\r\n", str);
	switch(status) {
	case HMC5883L_OK:
		printf("HMC5883L_OK\r\n");
		return;
	case HMC5883L_ERR_NULL:
		printf("HMC5883L_ERR_NULL\r\n");
		return;

	case HMC5883L_I2C_TIMEOUT:
		printf("HMC5883L_I2C_TIMEOUT\r\n");
		return;
	case HMC5883L_I2C_ERROR:
		printf("HMC5883L_I2C_ERROR\r\n");
		return;
	case HMC5883L_I2C_BUSY:
		printf("HMC5883L_I2C_BUSY\r\n");
		return;
	case HMC5883L_INVALID_CONFIG:
		printf("HMC5883L_INVALID_CONFIG\r\n");
		return;
	case HMC5883L_BAD_DEVICE_ID:
		printf("HMC5883L_BAD_DEVICE_ID\r\n");
		return;
	case HMC5883L_NOT_READY_TO_READ:
		printf("HMC5883L_NOT_READY_TO_READ\r\n");
		return;
	case HMC5883L_DATA_TIMEOUT:
		printf("HMC5883L_DATA_TIMEOUT\r\n");
		return;
	case HMC5883L_DATA_OVERFLOW:
	    printf("HMC5883L_DATA_OVERFLOW\r\n");
	    return;
	case HMC5883L_INVALID_CALIBRATION:
		printf("HMC5883L_INVALID_CALIBRATION\r\n");
		return;
	case HMC5883L_NOT_ENOUGH_SAMPLES:
		printf("HMC5883L_NOT_ENOUGH_SAMPLES\r\n");
		return;
	case HMC5883L_DATA_LOCKED:
		printf("HMC5883L_DATA_LOCKED\r\n");
		return;
	default:
		printf("Unknown status\r\n");
		return;
	}
}

HMC5883L_Status_t HMC5883L_CheckDeviceID(HMC5883L_Handle_t* hmc) {
	uint8_t id[HMC5883L_ID_LENGTH];
	HMC5883L_Status_t status = hmc5883l_read_regs(hmc, HMC5883L_REG_ID_A, HMC5883L_ID_LENGTH, id);
	if(status != HMC5883L_OK) return status;
	return (memcmp(id, HMC5883L_ID, HMC5883L_ID_LENGTH) == 0)
	        ? HMC5883L_OK
	        : HMC5883L_BAD_DEVICE_ID;
}

HMC5883L_Status_t HMC5883L_SetConfigurationA(
		HMC5883L_Handle_t* hmc,
		uint8_t meas_mode,
		uint8_t output_rate,
		uint8_t sample_avg
		) {
	if (hmc == NULL) {

	        return HMC5883L_ERR_NULL;
	    }
	if(meas_mode > HMC5883L_MEAS_MODE_NEG_BIAS) {

		return HMC5883L_INVALID_CONFIG;
	}
	if(output_rate > HMC5883L_OUTPUT_RATE_75) {

		return HMC5883L_INVALID_CONFIG;
	}
	if(sample_avg > HMC5883L_SAMPLE_AVG_8) {

		return HMC5883L_INVALID_CONFIG;
	}
	uint8_t value =
	          ((sample_avg & HMC5883L_CONF_A_SAMPLE_AVG_MASK)
	            << HMC5883L_CONF_A_MA0)

	        | ((output_rate & HMC5883L_CONF_A_OUTPUT_RATE_MASK)
	            << HMC5883L_CONF_A_DO0)

	        | ((meas_mode & HMC5883L_CONF_A_MEAS_MODE_MASK)
	            << HMC5883L_CONF_A_MS0);
	HMC5883L_Status_t status =
	        hmc5883l_write_reg(hmc, HMC5883L_REG_CONF_A, value);

	if (status == HMC5883L_OK) {
	    hmc -> config.meas_mode = meas_mode;
	    hmc -> config.output_rate = output_rate;
	    hmc -> config.sample_avg = sample_avg;
	}

	return status;

}

static void hmc5883l_set_lsb_per_gauss(
		HMC5883L_Handle_t* hmc,
		uint8_t device_gain) {
	switch(device_gain) {
	case HMC5883L_DEVICE_GAIN_0_88:
		hmc -> lsb_per_gauss = HMC5883L_LSB_PER_G_0_88;
		return;
	case HMC5883L_DEVICE_GAIN_1_3:
		hmc -> lsb_per_gauss = HMC5883L_LSB_PER_G_1_3;
		return;
	case HMC5883L_DEVICE_GAIN_1_9:
		hmc -> lsb_per_gauss = HMC5883L_LSB_PER_G_1_9;
		return;
	case HMC5883L_DEVICE_GAIN_2_5:
		hmc -> lsb_per_gauss = HMC5883L_LSB_PER_G_2_5;
		return;
	case HMC5883L_DEVICE_GAIN_4_0:
		hmc -> lsb_per_gauss = HMC5883L_LSB_PER_G_4_0;
		return;
	case HMC5883L_DEVICE_GAIN_4_7:
		hmc -> lsb_per_gauss = HMC5883L_LSB_PER_G_4_7;
		return;
	case HMC5883L_DEVICE_GAIN_5_6:
		hmc -> lsb_per_gauss = HMC5883L_LSB_PER_G_5_6;
		return;
	case HMC5883L_DEVICE_GAIN_8_1:
		hmc -> lsb_per_gauss = HMC5883L_LSB_PER_G_8_1;
		return;
	default:
		return;
	}
}

HMC5883L_Status_t HMC5883L_SetConfigurationB(
		HMC5883L_Handle_t* hmc,
		uint8_t device_gain) {
	if (hmc == NULL) {
		return HMC5883L_ERR_NULL;
	}
	if(device_gain > HMC5883L_DEVICE_GAIN_8_1) {

		return HMC5883L_INVALID_CONFIG;
	}
	uint8_t value = 0;
	value |= ((HMC5883L_CONF_B_DEVICE_GAIN_MASK & device_gain)
			<< HMC5883L_CONF_B_CRB5);
	HMC5883L_Status_t status =
	        hmc5883l_write_reg(hmc, HMC5883L_REG_CONF_B, value);

	if (status == HMC5883L_OK) {
	    hmc -> config.device_gain = device_gain;
	    hmc5883l_set_lsb_per_gauss(hmc, device_gain);
	}
	return status;
}

HMC5883L_Status_t HMC5883L_SetMode(
        HMC5883L_Handle_t* hmc,
        uint8_t mode)
{
    if (hmc == NULL) {
        return HMC5883L_ERR_NULL;
    }

    if (mode > HMC5883L_MODE_IDLE_MODE2) {
        return HMC5883L_INVALID_CONFIG;
    }

    uint8_t value = mode & HMC5883L_MODE_MASK;

    HMC5883L_Status_t status =
            hmc5883l_write_reg(hmc, HMC5883L_REG_MODE, value);

    if (status == HMC5883L_OK) {
        hmc -> config.mode = mode;
    }

    return status;
}

HMC5883L_Status_t HMC5883L_IsDataReady(
        HMC5883L_Handle_t* hmc,
        uint8_t* ready)
{
    if (hmc == NULL || ready == NULL) {
        return HMC5883L_ERR_NULL;
    }

    uint8_t status_reg = 0U;

    HMC5883L_Status_t status = hmc5883l_read_reg(
            hmc,
            HMC5883L_REG_STATUS,
            &status_reg);

    if (status != HMC5883L_OK) {
        return status;
    }

    *ready = ((status_reg & (1U << HMC5883L_STATUS_RDY)) != 0U)
            ? 1U
            : 0U;

    return HMC5883L_OK;
}


HMC5883L_Status_t HMC5883L_IsDataLocked(
        HMC5883L_Handle_t* hmc,
        uint8_t* locked)
{
    if (hmc == NULL || locked == NULL) {
        return HMC5883L_ERR_NULL;
    }

    uint8_t status_reg = 0U;

    HMC5883L_Status_t status = hmc5883l_read_reg(
            hmc,
            HMC5883L_REG_STATUS,
            &status_reg);

    if (status != HMC5883L_OK) {
        return status;
    }

    *locked = ((status_reg & (1U << HMC5883L_STATUS_LOCK)) != 0U)
            ? 1U
            : 0U;

    return HMC5883L_OK;
}

HMC5883L_Status_t HMC5883L_Init(
        HMC5883L_Handle_t* hmc,
        I2C_HandleTypeDef* hi2c,
        uint8_t address,
        const HMC5883L_Config_t* config)
{
    if (hmc == NULL || hi2c == NULL || config == NULL) {
        return HMC5883L_ERR_NULL;
    }

    hmc->hi2c = hi2c;
    hmc->address = address;

    HMC5883L_Status_t status = HMC5883L_CheckDeviceID(hmc);
    if (status != HMC5883L_OK) {
        return status;
    }

    hmc->config = *config;

    status = HMC5883L_SetConfigurationA(
            hmc,
            config->meas_mode,
            config->output_rate,
            config->sample_avg);
    if (status != HMC5883L_OK) {
        return status;
    }

    status = HMC5883L_SetConfigurationB(
            hmc,
            config->device_gain);
    if (status != HMC5883L_OK) {
        return status;
    }

    status = HMC5883L_CalibrationSetDefault(&hmc->calibration);
    if (status != HMC5883L_OK) {
        return status;
    }

    return HMC5883L_SetMode(hmc, HMC5883L_MODE_IDLE_MODE1);
}

HMC5883L_Status_t HMC5883L_StartContinuousMeasurement(
        HMC5883L_Handle_t* hmc)
{
    if (hmc == NULL) {
        return HMC5883L_ERR_NULL;
    }

    return HMC5883L_SetMode(
            hmc,
            HMC5883L_MODE_CONTINUOUS_MEAS);
}

HMC5883L_Status_t HMC5883L_StartSingleMeasurement(
        HMC5883L_Handle_t* hmc)
{
    if (hmc == NULL) {
        return HMC5883L_ERR_NULL;
    }

    return HMC5883L_SetMode(
            hmc,
            HMC5883L_MODE_SINGLE_MEAS);
}

HMC5883L_Status_t HMC5883L_StopMeasurement(
        HMC5883L_Handle_t* hmc)
{
    if (hmc == NULL) {
        return HMC5883L_ERR_NULL;
    }

    return HMC5883L_SetMode(
            hmc,
            HMC5883L_MODE_IDLE_MODE1);
}

static HMC5883L_Status_t hmc5883l_wait_data_ready(
        HMC5883L_Handle_t* hmc,
        uint32_t timeout_ms)
{
    if (hmc == NULL) {
        return HMC5883L_ERR_NULL;
    }

    uint32_t start_tick = HAL_GetTick();

    do {
        uint8_t ready = 0U;

        HMC5883L_Status_t status =
                HMC5883L_IsDataReady(hmc, &ready);

        if (status != HMC5883L_OK) {
            return status;
        }

        if (ready != 0U) {
            return HMC5883L_OK;
        }

    } while ((HAL_GetTick() - start_tick) < timeout_ms);

    return HMC5883L_DATA_TIMEOUT;
}

static HMC5883L_Status_t hmc5883l_decode_raw_data(
        const uint8_t* registers,
        HMC5883L_RawData_t* raw)
{
    if (registers == NULL || raw == NULL) {
        return HMC5883L_ERR_NULL;
    }

    raw->x = byte_utils_i16_from_be(registers[0], registers[1]);
    raw->z = byte_utils_i16_from_be(registers[2], registers[3]);
    raw->y = byte_utils_i16_from_be(registers[4], registers[5]);

    if (raw->x == HMC5883L_OVERFLOW_VALUE ||
        raw->y == HMC5883L_OVERFLOW_VALUE ||
        raw->z == HMC5883L_OVERFLOW_VALUE) {
        return HMC5883L_DATA_OVERFLOW;
    }

    return HMC5883L_OK;
}

HMC5883L_Status_t HMC5883L_ReadRawData(
        HMC5883L_Handle_t* hmc,
        HMC5883L_RawData_t* raw)
{
    if (hmc == NULL || raw == NULL) {
        return HMC5883L_ERR_NULL;
    }

    uint8_t registers[6] = {0U};

    HMC5883L_Status_t status = hmc5883l_read_regs(
            hmc,
            HMC5883L_REG_X_MSB,
            (uint8_t)sizeof(registers),
            registers);

    if (status != HMC5883L_OK) {
        return status;
    }

    return hmc5883l_decode_raw_data(registers, raw);
}

HMC5883L_Status_t HMC5883L_ReadRawDataBlocking(
        HMC5883L_Handle_t* hmc,
        HMC5883L_RawData_t* raw,
        uint32_t timeout_ms)
{
    if (hmc == NULL || raw == NULL) {
        return HMC5883L_ERR_NULL;
    }

    HMC5883L_Status_t status;

    switch (hmc->config.mode) {
    case HMC5883L_MODE_CONTINUOUS_MEAS:
        break;

    case HMC5883L_MODE_SINGLE_MEAS:
        status = HMC5883L_StartSingleMeasurement(hmc);
        if (status != HMC5883L_OK) {
            return status;
        }
        break;

    case HMC5883L_MODE_IDLE_MODE1:
    case HMC5883L_MODE_IDLE_MODE2:
        return HMC5883L_NOT_READY_TO_READ;

    default:
        return HMC5883L_INVALID_CONFIG;
    }

    status = hmc5883l_wait_data_ready(hmc, timeout_ms);
    if (status != HMC5883L_OK) {
        return status;
    }

    return HMC5883L_ReadRawData(hmc, raw);
}

HMC5883L_Status_t HMC5883L_ConvertRawToScaled(
		HMC5883L_Handle_t* hmc,
		HMC5883L_RawData_t* raw,
		HMC5883L_Data_t* scaled) {
	if(hmc == NULL || raw == NULL || scaled == NULL) {
		return HMC5883L_ERR_NULL;
	}
	if(hmc -> lsb_per_gauss <= 0 || !isfinite(hmc -> lsb_per_gauss)) return HMC5883L_INVALID_CONFIG;
	scaled -> x_g = (float)raw -> x / hmc -> lsb_per_gauss;
	scaled -> y_g = (float)raw -> y / hmc -> lsb_per_gauss;
	scaled -> z_g = (float)raw -> z / hmc -> lsb_per_gauss;
	return HMC5883L_OK;
}


HMC5883L_Status_t HMC5883L_CalibrationSetDefault(
		HMC5883L_Calibration_t* calibration) {
	if(calibration == NULL) return HMC5883L_ERR_NULL;

	calibration->sensor_scale.x = 1.0f;
	calibration->sensor_scale.y = 1.0f;
	calibration->sensor_scale.z = 1.0f;

	calibration -> hard_iron_bias_g.x = 0.0f;
	calibration -> hard_iron_bias_g.y = 0.0f;
	calibration -> hard_iron_bias_g.z = 0.0f;

	calibration -> soft_iron_matrix[0][0] = 1.0f;
	calibration -> soft_iron_matrix[0][1] = 0.0f;
	calibration -> soft_iron_matrix[0][2] = 0.0f;

	calibration -> soft_iron_matrix[1][0] = 0.0f;
	calibration -> soft_iron_matrix[1][1] = 1.0f;
	calibration -> soft_iron_matrix[1][2] = 0.0f;

	calibration -> soft_iron_matrix[2][0] = 0.0f;
	calibration -> soft_iron_matrix[2][1] = 0.0f;
	calibration -> soft_iron_matrix[2][2] = 1.0f;

	calibration -> valid_flags = 0U;

	return HMC5883L_OK;
}

HMC5883L_Status_t HMC5883L_CalibrationBegin(
		HMC5883L_CalibrationSession_t* session) {
		if(session == NULL) return HMC5883L_ERR_NULL;

		session -> min_g.x = FLT_MAX;
		session -> min_g.y = FLT_MAX;
		session -> min_g.z = FLT_MAX;

		session -> max_g.x = -FLT_MAX;
		session -> max_g.y = -FLT_MAX;
		session -> max_g.z = -FLT_MAX;

		session -> accepted_samples = 0;
		session -> rejected_samples = 0;
		return HMC5883L_OK;
}

HMC5883L_Status_t HMC5883L_CalibrationAddSample(
		HMC5883L_CalibrationSession_t* session,
		const HMC5883L_Data_t* sample) {
	if(session == NULL || sample == NULL) return HMC5883L_ERR_NULL;
	if(!isfinite(sample -> x_g) ||
	   !isfinite(sample -> y_g) ||
	   !isfinite(sample -> z_g)) {
		session->rejected_samples++;
		return HMC5883L_INVALID_CALIBRATION;
	}


	session -> min_g.x = (session -> min_g.x < sample -> x_g) ?
							session -> min_g.x :
							sample -> x_g;
	session -> min_g.y = (session -> min_g.y < sample -> y_g) ?
			                session -> min_g.y :
							sample -> y_g;
	session -> min_g.z = (session -> min_g.z < sample -> z_g) ?
							session -> min_g.z :
							sample -> z_g;
	session -> max_g.x = (session -> max_g.x > sample -> x_g) ?
							session -> max_g.x :
							sample -> x_g;
	session -> max_g.y = (session -> max_g.y > sample -> y_g) ?
								session -> max_g.y :
								sample -> y_g;
	session -> max_g.z = (session -> max_g.z > sample -> z_g) ?
								session -> max_g.z :
								sample -> z_g;
	session -> accepted_samples++;
	return HMC5883L_OK;
}

HMC5883L_Status_t HMC5883L_CalibrationFinish(
		HMC5883L_Handle_t* hmc,
		const HMC5883L_CalibrationSession_t* session) {
	if(hmc == NULL || session == NULL) return HMC5883L_ERR_NULL;
	if(session -> accepted_samples < HMC5883L_CAL_MIN_SAMPLES) return HMC5883L_NOT_ENOUGH_SAMPLES;

	float bias_x = (session -> min_g.x + session -> max_g.x) * 0.5f;
	float bias_y = (session -> min_g.y + session -> max_g.y) * 0.5f;
	float bias_z = (session -> min_g.z + session -> max_g.z) * 0.5f;


	float range_x = session -> max_g.x - session -> min_g.x;
	float range_y = session -> max_g.y - session -> min_g.y;
	float range_z = session -> max_g.z - session -> min_g.z;

	if(range_x < HMC5883L_CAL_MIN_AXIS_RANGE_G ||
	   range_y < HMC5883L_CAL_MIN_AXIS_RANGE_G ||
	   range_z < HMC5883L_CAL_MIN_AXIS_RANGE_G) {
		return HMC5883L_INVALID_CALIBRATION;
	}

	float avg_range = (range_x + range_y + range_z) / 3.0f;
	float scale_x = avg_range / range_x;
	float scale_y = avg_range / range_y;
	float scale_z = avg_range / range_z;

	if(!isfinite(bias_x) ||
	   !isfinite(bias_y) ||
	   !isfinite(bias_z) ||
	   !isfinite(scale_x) ||
	   !isfinite(scale_y) ||
	   !isfinite(scale_z)) {

		return HMC5883L_INVALID_CALIBRATION;
	}


	hmc->calibration.hard_iron_bias_g.x = bias_x;
	hmc->calibration.hard_iron_bias_g.y = bias_y;
	hmc->calibration.hard_iron_bias_g.z = bias_z;

	hmc->calibration.soft_iron_matrix[0][0] = scale_x;
	hmc->calibration.soft_iron_matrix[1][1] = scale_y;
	hmc->calibration.soft_iron_matrix[2][2] = scale_z;

	hmc->calibration.soft_iron_matrix[0][1] = 0.0f;
	hmc->calibration.soft_iron_matrix[0][2] = 0.0f;
	hmc->calibration.soft_iron_matrix[1][0] = 0.0f;
	hmc->calibration.soft_iron_matrix[1][2] = 0.0f;
	hmc->calibration.soft_iron_matrix[2][0] = 0.0f;
	hmc->calibration.soft_iron_matrix[2][1] = 0.0f;

	hmc->calibration.valid_flags |=
			HMC5883L_CAL_HARD_IRON_VALID |
			HMC5883L_CAL_SOFT_IRON_VALID;
	return HMC5883L_OK;
}


HMC5883L_Status_t HMC5883L_ApplyCalibration(
        HMC5883L_Handle_t* hmc,
        const HMC5883L_Data_t* input,
        HMC5883L_Data_t* output)
{
    if (hmc == NULL || input == NULL || output == NULL) {
        return HMC5883L_ERR_NULL;
    }

    float x = input->x_g - hmc->calibration.hard_iron_bias_g.x;
    float y = input->y_g - hmc->calibration.hard_iron_bias_g.y;
    float z = input->z_g - hmc->calibration.hard_iron_bias_g.z;

    output->x_g =
            hmc->calibration.soft_iron_matrix[0][0] * x +
            hmc->calibration.soft_iron_matrix[0][1] * y +
            hmc->calibration.soft_iron_matrix[0][2] * z;

    output->y_g =
            hmc->calibration.soft_iron_matrix[1][0] * x +
            hmc->calibration.soft_iron_matrix[1][1] * y +
            hmc->calibration.soft_iron_matrix[1][2] * z;

    output->z_g =
            hmc->calibration.soft_iron_matrix[2][0] * x +
            hmc->calibration.soft_iron_matrix[2][1] * y +
            hmc->calibration.soft_iron_matrix[2][2] * z;

    return HMC5883L_OK;
}
