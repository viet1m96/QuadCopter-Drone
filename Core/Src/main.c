/*
 * main.c
 *
 *  Created on: Jun 22, 2026
 *      Author: vietht-hl
 */


#include "stm32f4xx.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_uart.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "mpu6050.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef husart2;
MPU6050_Handle_t mpu6050;
MPU6050_RawData_t raw;
MPU6050_Data_t scaled;
MPU6050_StillnessConfig_t still;
uint8_t mpu6050_ready = 0;

float data_x[1000] = {0.0f};
float data_y[1000] = {0.0f};
float data_z[1000] = {0.0f};

float find_min(const float* data, uint32_t sz) {
	float res = data[0];
	for(uint32_t i = 1; i < sz; i++) {
		res = fminf(res, data[i]);
	}
	return res;
}

float find_max(const float* data, uint32_t sz) {
	float res = data[0];
	for(uint32_t i = 1; i < sz; i++) {
		res = fmaxf(res, data[i]);
	}
	return res;
}

float calc_mean(const float* data, uint32_t sz) {
	float sum = 0;
	for(uint32_t i = 0; i < sz; i++) {
		sum += data[i];
	}
	return sum / sz;
}

float calc_stdDev(const float* data, uint32_t sz) {
	float sum = 0;
	for(uint32_t i = 0; i < sz; i++) {
		sum += data[i];
	}
	sum /= sz;
	float stdDev = 0;
	for(uint32_t i = 0; i < sz; i++) {
		stdDev += (data[i] - sum) * (data[i] - sum);
	}
	stdDev /= sz - 1;
	return sqrtf(stdDev);
}



void SystemClockConfig(void) {
	//when we do not use HSI
}

void I2C1_Init(void)
{
    hi2c1.Instance = I2C1;

    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(&hi2c1);
}

void USART2_UART_Init(void) {
	husart2.Instance = USART2;
	husart2.Init.BaudRate = 115200;
	husart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	husart2.Init.WordLength = UART_WORDLENGTH_8B;
	husart2.Init.StopBits = UART_STOPBITS_1;
	husart2.Init.Parity = UART_PARITY_NONE;
	husart2.Init.Mode = UART_MODE_TX;
	husart2.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(&husart2);
}

void MPU6050_Setup(void) {
	MPU6050_Status_t status = MPU6050_Init(&mpu6050,
			&hi2c1,
			MPU6050_ADDRESS,
			MPU6050_CLKSRC_PLL_X,
			MPU6050_DLPF_CFG_3,
			MPU6050_GYRO_CONFIG_FS_250DPS,
			MPU6050_ACCEL_CONFIG_AFS_2G,
			9);
	if(status != MPU6050_OK) {
		printf("MPU6050_Init failed: %s\r\n", MPU6050_ConvertStatusToString(status));
		return;
	}
	status = MPU6050_SetStillnessConfig(&mpu6050, &still);
	if(status != MPU6050_OK) {
		printf("MPU6050_SetStillnessConfig failed: %s\r\n", MPU6050_ConvertStatusToString(status));
		return;
	}
	status = MPU6050_CalibrateGyroOffset(&mpu6050, &raw, &still);
	if(status != MPU6050_OK) {
		printf("MPU6050_CalibrateGyroOffset failed: %s\r\n", MPU6050_ConvertStatusToString(status));
		return;
	}
	status = MPU6050_CalibrateAccelOffset(&mpu6050, &raw, &still, 0.0f, 0.0f, 1.0f);
	if(status != MPU6050_OK) {
		printf("MPU6050_CalibrateAccelOffset failed: %s\r\n", MPU6050_ConvertStatusToString(status));
		return;
	}
	if(status == MPU6050_OK) mpu6050_ready = 1;

}

void I2C_Scan(void) {
    printf("I2C scan start\r\n");

    for(uint8_t addr = 1; addr < 128; addr++) {
        if(HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 3, 100) == HAL_OK) {
            printf("Found I2C device at 0x%02X\r\n", addr);
        }
    }

    printf("I2C scan done\r\n");
}

int main(void) {

	HAL_Init();
	SystemClockConfig();
	I2C1_Init();
	USART2_UART_Init();
	printf("\r\nBOOT\r\n");
	I2C_Scan();
	MPU6050_Setup();
	int num = 0;
	while (1)
	{
		if(mpu6050_ready == 1 && num < 1000) {
			MPU6050_ReadScaledData(&mpu6050, &scaled);
//			printf("accel_x %f\r\n", scaled.accel_x_g);
//			printf("accel_y %f\r\n", scaled.accel_y_g);
//			printf("accel_z %f\r\n", scaled.accel_z_g);
//			printf("gyro_x %f\r\n", scaled.gyro_x_dps);
//			printf("gyro_y %f\r\n", scaled.gyro_y_dps);
//			printf("gyro_z %f\r\n", scaled.gyro_z_dps);
//			printf("temp_c %f\r\n", scaled.temp_c);
//			printf("\r\n");
			printf("%d\r\n", num);
			data_x[num] = scaled.accel_x_g;
			data_y[num] = scaled.accel_y_g;
			data_z[num] = scaled.accel_z_g;
			num++;

		}
		if(num == 1000) {
			printf("min_x %f\r\n", find_min(data_x, num));
			printf("min_y %f\r\n", find_min(data_y, num));
			printf("min_z %f\r\n", find_min(data_z, num));
			printf("max_x %f\r\n", find_max(data_x, num));
			printf("max_y %f\r\n", find_max(data_y, num));
			printf("max_z %f\r\n", find_max(data_z, num));
			printf("mean_x %f\r\n", calc_mean(data_x, num));
			printf("mean_y %f\r\n", calc_mean(data_y, num));
			printf("mean_z %f\r\n", calc_mean(data_z, num));
			printf("std_x %f\r\n", calc_stdDev(data_x, num));
			printf("std_y %f\r\n", calc_stdDev(data_y, num));
			printf("std_z %f\r\n", calc_stdDev(data_z, num));
			num = 1001;
		}
		HAL_Delay(300);
	}
	return 0;
}
