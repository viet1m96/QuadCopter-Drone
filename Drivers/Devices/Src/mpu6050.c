/*
 * mpu6050.c
 *
 *  Created on: Jun 24, 2026
 *      Author: vietht-hl
 */



#include "mpu6050.h"

typedef struct {
	int64_t sum_gyro_x;
	int64_t sum_gyro_y;
	int64_t sum_gyro_z;
	int64_t sum_accel_x;
	int64_t sum_accel_y;
	int64_t sum_accel_z;
} MPU6050_CalibWindow_t;

static MPU6050_Status_t mpu6050_write_reg(MPU6050_Handle_t* mpu, uint8_t reg, uint8_t value) {
	if (mpu == NULL || mpu -> hi2c == NULL) {
		return MPU6050_ERR_NULL;
	}
	uint16_t DevAddress = (mpu -> address << 1);
	HAL_StatusTypeDef wStatus = HAL_I2C_Mem_Write(
			mpu -> hi2c,
			DevAddress,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			&value,
			1,
			100);
	if(wStatus == HAL_BUSY) {
		return MPU6050_I2C_BUSY;
	} else if(wStatus == HAL_ERROR) {
		return MPU6050_I2C_ERROR;
	} else if(wStatus == HAL_TIMEOUT) {
		return MPU6050_I2C_TIMEOUT;
	}
	return MPU6050_OK;

}

static MPU6050_Status_t mpu6050_read_reg(MPU6050_Handle_t* mpu, uint8_t reg, uint8_t* result) {
	if (mpu == NULL || mpu -> hi2c == NULL || result == NULL) {
		return MPU6050_ERR_NULL;
	}
	uint16_t DevAddress = (mpu -> address << 1);
	HAL_StatusTypeDef rStatus = HAL_I2C_Mem_Read(
			mpu -> hi2c,
			DevAddress,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			result,
			1,
			100);
	if(rStatus == HAL_BUSY) {
			return MPU6050_I2C_BUSY;
		} else if(rStatus == HAL_ERROR) {
			return MPU6050_I2C_ERROR;
		} else if(rStatus == HAL_TIMEOUT) {
			return MPU6050_I2C_TIMEOUT;
		}
	return MPU6050_OK;
}


static MPU6050_Status_t mpu6050_read_regs(MPU6050_Handle_t* mpu, uint8_t start_reg, uint8_t len, uint8_t* result) {
	if(mpu == NULL || mpu -> hi2c == NULL || result == NULL) {
		return MPU6050_ERR_NULL;
	}
	if(len == 0) {
		return MPU6050_INVALID_CONFIG;
	}
	uint16_t DevAddress = (mpu -> address << 1);
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
	        mpu->hi2c,
	        DevAddress,
	        start_reg,
	        I2C_MEMADD_SIZE_8BIT,
	        result,
	        len,
	        100
	    );

   if (status == HAL_BUSY) {
		return MPU6050_I2C_BUSY;
   } else if (status == HAL_TIMEOUT) {
		return MPU6050_I2C_TIMEOUT;
   } else {
	   	if(status == HAL_OK) return MPU6050_OK;
		return MPU6050_I2C_ERROR;
   }
}



MPU6050_Status_t MPU6050_Init(
		MPU6050_Handle_t* mpu,
		I2C_HandleTypeDef* hi2c ,
		uint8_t address,
		uint8_t clksrc,
		uint8_t dlpf_config,
		uint8_t fs_sel_config,
		uint8_t accel_sel_config,
		uint8_t sample_rate_div) {
	if (mpu == NULL || hi2c == NULL) {
	    return MPU6050_ERR_NULL;
	}
	mpu -> hi2c = hi2c;
	mpu -> address = address;
	MPU6050_Status_t status = MPU6050_CheckDeviceID(mpu);
	if(status != MPU6050_OK) return status;
	status = MPU6050_WakeUpChip(mpu);
	if(status != MPU6050_OK) return status;
	status = MPU6050_SetClockSource(mpu, clksrc);
	if(status != MPU6050_OK) return status;
	status = MPU6050_SetDLPF(mpu, dlpf_config);
	if(status != MPU6050_OK) return status;
	status = MPU6050_SetGyroConfig(mpu, fs_sel_config);
	if(status != MPU6050_OK) return status;
	status = MPU6050_SetAccelConfig(mpu, accel_sel_config);
	if(status != MPU6050_OK) return status;
	status = MPU6050_SetSampleRate(mpu, sample_rate_div);
	if(status != MPU6050_OK) return status;
	status = MPU6050_ResetOffsets(mpu);
	return status;

}

MPU6050_Status_t MPU6050_CheckDeviceID(MPU6050_Handle_t* mpu) {
	uint8_t who = 0;
	MPU6050_Status_t readWhoAmI = mpu6050_read_reg(mpu, MPU6050_REG_WHO_AM_I, &who);
	if(readWhoAmI == MPU6050_OK) {
		if(who == MPU6050_WHO_AM_I_VALUE) {
			return MPU6050_OK;
		}
		return MPU6050_ERR_BAD_DEVICE_ID;
	}
	return readWhoAmI;
}

MPU6050_Status_t MPU6050_WakeUpChip(MPU6050_Handle_t* mpu) {
	uint8_t value = 0x00;
	return mpu6050_write_reg(mpu, MPU6050_REG_PWR_MGMT_1, value);
}

MPU6050_Status_t MPU6050_SetClockSource(MPU6050_Handle_t* mpu, uint8_t clksrc) {
	uint8_t cur_res = 0;
	MPU6050_Status_t status = mpu6050_read_reg(mpu, MPU6050_REG_PWR_MGMT_1, &cur_res);
	if(status != MPU6050_OK) return status;
	cur_res &= ~(MPU6050_PWR1_CLKSEL_MASK << MPU6050_PWR1_CLKSEL_POS);
	cur_res |= ((clksrc & MPU6050_PWR1_CLKSEL_MASK) << MPU6050_PWR1_CLKSEL_POS);
	return mpu6050_write_reg(mpu, MPU6050_REG_PWR_MGMT_1, cur_res);
}

MPU6050_Status_t MPU6050_SetDLPF(MPU6050_Handle_t* mpu, uint8_t dlpf_config) {
	if(dlpf_config > 6) return MPU6050_INVALID_CONFIG;
	uint8_t cur_res = 0;
	MPU6050_Status_t status = mpu6050_read_reg(mpu, MPU6050_REG_CONFIG, &cur_res);
	if(status != MPU6050_OK) return status;
	cur_res &= ~(MPU6050_CONFIG_DLPF_CFG_MASK << MPU6050_CONFIG_DLPF_CFG_POS);
	cur_res |= ((dlpf_config & MPU6050_CONFIG_DLPF_CFG_MASK) << MPU6050_CONFIG_DLPF_CFG_POS);
	return mpu6050_write_reg(mpu, MPU6050_REG_CONFIG, cur_res);
}

MPU6050_Status_t MPU6050_SetGyroConfig(MPU6050_Handle_t* mpu, uint8_t fs_sel_config) {
	if(fs_sel_config > 3) return MPU6050_INVALID_CONFIG;
	uint8_t cur_res = 0;
	MPU6050_Status_t status = mpu6050_read_reg(mpu, MPU6050_REG_GYRO_CONFIG, &cur_res);
	if(status != MPU6050_OK) return status;
	cur_res &= ~(MPU6050_GYRO_CONFIG_FS_SEL_MASK << MPU6050_GYRO_CONFIG_FS_SEL_POS);
	cur_res |= ((fs_sel_config & MPU6050_GYRO_CONFIG_FS_SEL_MASK) << MPU6050_GYRO_CONFIG_FS_SEL_POS);
	status = mpu6050_write_reg(mpu, MPU6050_REG_GYRO_CONFIG, cur_res);
	if(status != MPU6050_OK) return status;
	if(fs_sel_config == MPU6050_GYRO_CONFIG_FS_250DPS) {
		mpu -> gyro_scale = MPU6050_GYRO_CONFIG_FS_SEN0;
	} else if(fs_sel_config == MPU6050_GYRO_CONFIG_FS_500DPS) {
		mpu -> gyro_scale = MPU6050_GYRO_CONFIG_FS_SEN1;
	} else if(fs_sel_config == MPU6050_GYRO_CONFIG_FS_1000DPS) {
		mpu -> gyro_scale = MPU6050_GYRO_CONFIG_FS_SEN2;
	} else if(fs_sel_config == MPU6050_GYRO_CONFIG_FS_2000DPS) {
		mpu -> gyro_scale = MPU6050_GYRO_CONFIG_FS_SEN3;
	} else {
		return MPU6050_INVALID_CONFIG;
	}
	return MPU6050_OK;
}

MPU6050_Status_t MPU6050_SetAccelConfig(MPU6050_Handle_t* mpu, uint8_t accel_sel_config) {
	if(accel_sel_config > 3) return MPU6050_INVALID_CONFIG;
	uint8_t cur_res = 0;
	MPU6050_Status_t status = mpu6050_read_reg(mpu, MPU6050_REG_ACCEL_CONFIG, &cur_res);
	if(status != MPU6050_OK) return status;
	cur_res &= ~(MPU6050_ACCEL_CONFIG_AFS_SEL_MASK << MPU6050_ACCEL_CONFIG_AFS_SEL_POS);
	cur_res |= ((accel_sel_config & MPU6050_ACCEL_CONFIG_AFS_SEL_MASK) << MPU6050_ACCEL_CONFIG_AFS_SEL_POS);
	status = mpu6050_write_reg(mpu, MPU6050_REG_ACCEL_CONFIG, cur_res);
	if(status != MPU6050_OK) return status;
	if(accel_sel_config == MPU6050_ACCEL_CONFIG_AFS_2G) {
		mpu -> accel_scale = MPU6050_ACCEL_CONFIG_AFS_SEN0;
	} else if(accel_sel_config == MPU6050_ACCEL_CONFIG_AFS_4G) {
		mpu -> accel_scale = MPU6050_ACCEL_CONFIG_AFS_SEN1;
	} else if(accel_sel_config == MPU6050_ACCEL_CONFIG_AFS_8G) {
		mpu -> accel_scale = MPU6050_ACCEL_CONFIG_AFS_SEN2;
	} else if(accel_sel_config == MPU6050_ACCEL_CONFIG_AFS_16G) {
		mpu -> accel_scale = MPU6050_ACCEL_CONFIG_AFS_SEN3;
	} else {
		return MPU6050_INVALID_CONFIG;
	}
	return MPU6050_OK;
}

MPU6050_Status_t MPU6050_SetSampleRate(MPU6050_Handle_t* mpu, uint8_t sample_rate_div) {
	return mpu6050_write_reg(mpu, MPU6050_REG_SMPRT_DIV, sample_rate_div);
}

MPU6050_Status_t MPU6050_ReadRawData(MPU6050_Handle_t* mpu, MPU6050_RawData_t* raw) {
	if (raw == NULL) return MPU6050_ERR_NULL;
	uint8_t result[14];
	MPU6050_Status_t status = mpu6050_read_regs(mpu, MPU6050_REG_ACCEL_XOUTH, 14, result);
	if(status != MPU6050_OK) return status;
	uint16_t temp = ((uint16_t)result[0] << 8) | result[1];
	raw -> accel_x = (int16_t) temp;
	temp = ((uint16_t)result[2] << 8) | result[3];
	raw -> accel_y = (int16_t) temp;
	temp = ((uint16_t)result[4] << 8) | result[5];
	raw -> accel_z = (int16_t) temp;
	temp = ((uint16_t)result[6] << 8) | result[7];
	raw -> temp = (int16_t) temp;
	temp = ((uint16_t)result[8] << 8) | result[9];
	raw -> gyro_x = (int16_t) temp;
	temp = ((uint16_t)result[10] << 8) | result[11];
	raw -> gyro_y = (int16_t) temp;
	temp = ((uint16_t)result[12] << 8) | result[13];
	raw -> gyro_z = (int16_t) temp;
	return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadRawAccel(MPU6050_Handle_t* mpu, MPU6050_RawData_t* raw) {
	if(raw == NULL) return MPU6050_ERR_NULL;
	uint8_t result[6];
	MPU6050_Status_t status = mpu6050_read_regs(mpu, MPU6050_REG_ACCEL_XOUTH, 6, result);
	if(status != MPU6050_OK) return status;
	uint16_t temp = ((uint16_t)result[0] << 8) | result[1];
	raw -> accel_x = (int16_t) temp;
	temp = ((uint16_t)result[2] << 8) | result[3];
	raw -> accel_y = (int16_t) temp;
	temp = ((uint16_t)result[4] << 8) | result[5];
	raw -> accel_z = (int16_t) temp;
	return MPU6050_OK;
}
MPU6050_Status_t MPU6050_ReadRawGyro(MPU6050_Handle_t* mpu, MPU6050_RawData_t* raw) {
	if(raw == NULL) return MPU6050_ERR_NULL;
	uint8_t result[6];
	MPU6050_Status_t status = mpu6050_read_regs(mpu, MPU6050_REG_GYRO_XOUTH, 6, result);
	if(status != MPU6050_OK) return status;
	uint16_t temp = ((uint16_t)result[0] << 8) | result[1];
	raw -> gyro_x = (int16_t) temp;
	temp = ((uint16_t)result[2] << 8) | result[3];
	raw -> gyro_y = (int16_t) temp;
	temp = ((uint16_t)result[4] << 8) | result[5];
	raw -> gyro_z = (int16_t) temp;
	return MPU6050_OK;
}
MPU6050_Status_t MPU6050_ReadRawTemp(MPU6050_Handle_t* mpu, MPU6050_RawData_t* raw) {
	if(raw == NULL) return MPU6050_ERR_NULL;
	uint8_t result[2];
	MPU6050_Status_t status = mpu6050_read_regs(mpu, MPU6050_REG_TEMP_OUTH, 2, result);
	if(status != MPU6050_OK) return status;
	uint16_t temp = ((uint16_t)result[0] << 8) | result[1];
	raw -> temp = (int16_t) temp;
	return MPU6050_OK;
}


MPU6050_Status_t MPU6050_ReadScaledData(MPU6050_Handle_t *mpu,
                                         MPU6050_Data_t *scaled)
{
    if (mpu == NULL || scaled == NULL) {
        return MPU6050_ERR_NULL;
    }

    MPU6050_RawData_t raw;
    MPU6050_Status_t status = MPU6050_ReadRawData(mpu, &raw);
    if (status != MPU6050_OK) {
        return status;
    }

    return MPU6050_ConvertRawToScaled(mpu, &raw, scaled);
}

MPU6050_Status_t MPU6050_ConvertRawToScaled(MPU6050_Handle_t *mpu,
                                             const MPU6050_RawData_t *raw,
                                             MPU6050_Data_t *scaled)
{
    if (mpu == NULL || raw == NULL || scaled == NULL) {
        return MPU6050_ERR_NULL;
    }

    scaled -> accel_x_g = raw -> accel_x / mpu -> accel_scale - mpu -> accel_offset_x;
    scaled -> accel_y_g = raw -> accel_y / mpu -> accel_scale - mpu -> accel_offset_y;
    scaled -> accel_z_g = raw -> accel_z / mpu -> accel_scale - mpu -> accel_offset_z;

    scaled -> temp_c = raw -> temp / mpu -> temp_scale + mpu -> temp_offset;

    scaled -> gyro_x_dps = raw -> gyro_x / mpu -> gyro_scale - mpu -> gyro_offset_x;
    scaled -> gyro_y_dps = raw -> gyro_y / mpu -> gyro_scale - mpu -> gyro_offset_y;
    scaled -> gyro_z_dps = raw -> gyro_z / mpu -> gyro_scale - mpu -> gyro_offset_z;

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ResetOffsets(MPU6050_Handle_t* mpu) {
	if(mpu == NULL) return MPU6050_ERR_NULL;
	mpu -> temp_scale = MPU6050_TEMP_SCALE;
	mpu -> temp_offset = MPU6050_TEMP_OFFSET;
	mpu -> accel_offset_x = 0;
	mpu -> accel_offset_y = 0;
	mpu -> accel_offset_z = 0;
	mpu -> gyro_offset_x = 0;
	mpu -> gyro_offset_y = 0;
	mpu -> gyro_offset_z = 0;
	return MPU6050_OK;
}

MPU6050_Status_t MPU6050_SetStillnessConfig(MPU6050_Handle_t* mpu, MPU6050_StillnessConfig_t* still) {
    if(mpu == NULL || still == NULL) return MPU6050_ERR_NULL;
    if(mpu->gyro_scale <= 0.0f || mpu->accel_scale <= 0.0f) return MPU6050_INVALID_CONFIG;

    still -> sample_count = MPU6050_STILLNESS_SMPL_CNT200;
    still -> sample_delay_ms = MPU6050_STILLNESS_SAMPLE_DELAY_MS;
    still -> timeout_ms = MPU6050_STILLNESS_TIMEOUT5000;

    still -> gyro_threshold = mpu -> gyro_scale ;


    still -> accel_threshold = mpu->accel_scale;
    still -> accel_axis_threshold = mpu->accel_scale * 0.05f;
    still -> accel_allowed_gap = mpu->accel_scale * 0.1f;

    return MPU6050_OK;
}

static MPU6050_Status_t mpu6050_collect_still_window(
		MPU6050_Handle_t* mpu,
		MPU6050_RawData_t* raw,
		MPU6050_StillnessConfig_t* still,
		MPU6050_CalibWindow_t* window) {
	if(mpu == NULL || raw == NULL || still == NULL || window == NULL) return MPU6050_ERR_NULL;
	if(still -> sample_count == 0) return MPU6050_INVALID_CONFIG;

	int16_t mx_raw_gyro_x = INT16_MIN;
	int16_t mn_raw_gyro_x = INT16_MAX;
	int16_t mx_raw_gyro_y = INT16_MIN;
	int16_t mn_raw_gyro_y = INT16_MAX;
	int16_t mx_raw_gyro_z = INT16_MIN;
	int16_t mn_raw_gyro_z = INT16_MAX;

	int16_t mx_raw_accel_x = INT16_MIN;
	int16_t mn_raw_accel_x = INT16_MAX;
	int16_t mx_raw_accel_y = INT16_MIN;
	int16_t mn_raw_accel_y = INT16_MAX;
	int16_t mx_raw_accel_z = INT16_MIN;
	int16_t mn_raw_accel_z = INT16_MAX;

	window -> sum_gyro_x = 0;
	window -> sum_gyro_y = 0;
	window -> sum_gyro_z = 0;
	window -> sum_accel_x = 0;
	window -> sum_accel_y = 0;
	window -> sum_accel_z = 0;

	MPU6050_Status_t status;
	for(uint32_t i = 0; i < still -> sample_count; i++) {
		status = MPU6050_ReadRawData(mpu, raw);
		if(status != MPU6050_OK) return status;

		mx_raw_gyro_x = (mx_raw_gyro_x > raw -> gyro_x) ? mx_raw_gyro_x : raw -> gyro_x;
		mn_raw_gyro_x = (mn_raw_gyro_x < raw -> gyro_x) ? mn_raw_gyro_x : raw -> gyro_x;
		mx_raw_gyro_y = (mx_raw_gyro_y > raw -> gyro_y) ? mx_raw_gyro_y : raw -> gyro_y;
		mn_raw_gyro_y = (mn_raw_gyro_y < raw -> gyro_y) ? mn_raw_gyro_y : raw -> gyro_y;
		mx_raw_gyro_z = (mx_raw_gyro_z > raw -> gyro_z) ? mx_raw_gyro_z : raw -> gyro_z;
		mn_raw_gyro_z = (mn_raw_gyro_z < raw -> gyro_z) ? mn_raw_gyro_z : raw -> gyro_z;

		mx_raw_accel_x = (mx_raw_accel_x > raw -> accel_x) ? mx_raw_accel_x : raw -> accel_x;
		mn_raw_accel_x = (mn_raw_accel_x < raw -> accel_x) ? mn_raw_accel_x : raw -> accel_x;
		mx_raw_accel_y = (mx_raw_accel_y > raw -> accel_y) ? mx_raw_accel_y : raw -> accel_y;
		mn_raw_accel_y = (mn_raw_accel_y < raw -> accel_y) ? mn_raw_accel_y : raw -> accel_y;
		mx_raw_accel_z = (mx_raw_accel_z > raw -> accel_z) ? mx_raw_accel_z : raw -> accel_z;
		mn_raw_accel_z = (mn_raw_accel_z < raw -> accel_z) ? mn_raw_accel_z : raw -> accel_z;

		window -> sum_gyro_x += raw -> gyro_x;
		window -> sum_gyro_y += raw -> gyro_y;
		window -> sum_gyro_z += raw -> gyro_z;
		window -> sum_accel_x += raw -> accel_x;
		window -> sum_accel_y += raw -> accel_y;
		window -> sum_accel_z += raw -> accel_z;

		if(still -> sample_delay_ms > 0U) {
			HAL_Delay(still -> sample_delay_ms);
		}
	}

	int32_t gyro_range_x = (int32_t)mx_raw_gyro_x - (int32_t)mn_raw_gyro_x;
	int32_t gyro_range_y = (int32_t)mx_raw_gyro_y - (int32_t)mn_raw_gyro_y;
	int32_t gyro_range_z = (int32_t)mx_raw_gyro_z - (int32_t)mn_raw_gyro_z;

	if((gyro_range_x > still -> gyro_threshold)
	  || (gyro_range_y > still -> gyro_threshold)
	  || (gyro_range_z > still -> gyro_threshold)) {
		return MPU6050_ERR_MOVING;
	}

	int32_t accel_range_x = (int32_t)mx_raw_accel_x - (int32_t)mn_raw_accel_x;
	int32_t accel_range_y = (int32_t)mx_raw_accel_y - (int32_t)mn_raw_accel_y;
	int32_t accel_range_z = (int32_t)mx_raw_accel_z - (int32_t)mn_raw_accel_z;

	if((accel_range_x > still -> accel_axis_threshold)
	  || (accel_range_y > still -> accel_axis_threshold)
	  || (accel_range_z > still -> accel_axis_threshold)) {
		return MPU6050_ERR_MOVING;
	}

	float avg_raw_accel_x = (float)window -> sum_accel_x / (float)still -> sample_count;
	float avg_raw_accel_y = (float)window -> sum_accel_y / (float)still -> sample_count;
	float avg_raw_accel_z = (float)window -> sum_accel_z / (float)still -> sample_count;

	float avg_accel_mag = sqrtf(
			avg_raw_accel_x * avg_raw_accel_x
		  + avg_raw_accel_y * avg_raw_accel_y
		  + avg_raw_accel_z * avg_raw_accel_z);

	if(fabsf(avg_accel_mag - still -> accel_threshold) > still -> accel_allowed_gap) {
		return MPU6050_ERR_MOVING;
	}

	return MPU6050_OK;
}

MPU6050_Status_t MPU6050_CalibrateGyroOffset(MPU6050_Handle_t* mpu, MPU6050_RawData_t* raw, MPU6050_StillnessConfig_t* still) {
	if(mpu == NULL || raw == NULL || still == NULL) return MPU6050_ERR_NULL;
	if(still -> sample_count == 0 || mpu -> gyro_scale <= 0.0f) return MPU6050_INVALID_CONFIG;

	MPU6050_CalibWindow_t window;
	MPU6050_Status_t status;

	uint32_t start_tick = HAL_GetTick();
	do {
		status = mpu6050_collect_still_window(mpu, raw, still, &window);
		if(status == MPU6050_OK) {
			float avg_gyro_x_raw = (float)window.sum_gyro_x / (float)still -> sample_count;
			float avg_gyro_y_raw = (float)window.sum_gyro_y / (float)still -> sample_count;
			float avg_gyro_z_raw = (float)window.sum_gyro_z / (float)still -> sample_count;

			mpu -> gyro_offset_x = avg_gyro_x_raw / mpu -> gyro_scale;
			mpu -> gyro_offset_y = avg_gyro_y_raw / mpu -> gyro_scale;
			mpu -> gyro_offset_z = avg_gyro_z_raw / mpu -> gyro_scale;

			return MPU6050_OK;
		}

		if(status != MPU6050_ERR_MOVING) return status;
		if(still -> timeout_ms == 0U) return status;
	} while((HAL_GetTick() - start_tick) < still -> timeout_ms);

	return MPU6050_ERR_TIMEOUT;
}

MPU6050_Status_t MPU6050_CalibrateAccelOffset(
		MPU6050_Handle_t* mpu,
		MPU6050_RawData_t* raw,
		MPU6050_StillnessConfig_t* still,
		float accel_ref_x_g,
		float accel_ref_y_g,
		float accel_ref_z_g) {
	if(mpu == NULL || raw == NULL || still == NULL) return MPU6050_ERR_NULL;
	if(still -> sample_count == 0 || mpu -> accel_scale <= 0.0f) return MPU6050_INVALID_CONFIG;

	MPU6050_CalibWindow_t window;
	MPU6050_Status_t status;

	uint32_t start_tick = HAL_GetTick();
	do {
		status = mpu6050_collect_still_window(mpu, raw, still, &window);
		if(status == MPU6050_OK) {
			float avg_accel_x_g = ((float)window.sum_accel_x / (float)still -> sample_count) / mpu -> accel_scale;
			float avg_accel_y_g = ((float)window.sum_accel_y / (float)still -> sample_count) / mpu -> accel_scale;
			float avg_accel_z_g = ((float)window.sum_accel_z / (float)still -> sample_count) / mpu -> accel_scale;

			mpu -> accel_offset_x = avg_accel_x_g - accel_ref_x_g;
			mpu -> accel_offset_y = avg_accel_y_g - accel_ref_y_g;
			mpu -> accel_offset_z = avg_accel_z_g - accel_ref_z_g;

			return MPU6050_OK;
		}

		if(status != MPU6050_ERR_MOVING) return status;
		if(still -> timeout_ms == 0U) return status;
	} while((HAL_GetTick() - start_tick) < still -> timeout_ms);

	return MPU6050_ERR_TIMEOUT;
}

const char* MPU6050_ConvertStatusToString(MPU6050_Status_t status) {
    switch (status) {
        case MPU6050_OK:
            return "MPU6050_OK";

        case MPU6050_ERROR:
            return "MPU6050_ERROR";

        case MPU6050_I2C_ERROR:
            return "MPU6050_I2C_ERROR";

        case MPU6050_I2C_BUSY:
            return "MPU6050_I2C_BUSY";

        case MPU6050_I2C_TIMEOUT:
            return "MPU6050_I2C_TIMEOUT";

        case MPU6050_ERR_TIMEOUT:
            return "MPU6050_ERR_TIMEOUT";

        case MPU6050_ERR_NULL:
            return "MPU6050_ERR_NULL";

        case MPU6050_ERR_BAD_DEVICE_ID:
            return "MPU6050_ERR_BAD_DEVICE_ID";

        case MPU6050_ERR_MOVING:
            return "MPU6050_ERR_MOVING";

        case MPU6050_INVALID_CONFIG:
            return "MPU6050_INVALID_CONFIG";

        default:
            return "MPU6050_UNKNOWN_STATUS";
    }
}
