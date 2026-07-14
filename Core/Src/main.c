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
#include "bmp180.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef husart2;
MPU6050_Handle_t mpu6050;
MPU6050_RawData_t raw;
MPU6050_Data_t scaled;
MPU6050_StillnessConfig_t still;
uint8_t mpu6050_ready = 0;

BMP180_Handle_t bmp180;
BMP180_Calibration_t offsets;
BMP180_RawData_t bmp_raw;
BMP180_Data_t bmp_scaled;
BMP180_PressureWindow_t bmp_window;
BMP180_Filter_t filter;
uint8_t bmp180_ready = 0;

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
		printf("MPU6050_CalibrateGyroOffset failed:%s\r\n", MPU6050_ConvertStatusToString(status));
		return;
	}
	status = MPU6050_CalibrateAccelOffset(&mpu6050, &raw, &still, 0.0f, 0.0f, 1.0f);
	if(status != MPU6050_OK) {
		printf("MPU6050_CalibrateAccelOffset failed: %s\r\n", MPU6050_ConvertStatusToString(status));
		return;
	}
	if(status == MPU6050_OK) mpu6050_ready = 1;

}

void BMP180_Setup(void)
{
    BMP180_Status_t status =
        BMP180_Init(
            &bmp180,
            &hi2c1,
            BMP180_ADDRESS
        );

    if(status != BMP180_OK) {
        printf("BMP180 init failed\r\n");
        return;
    }

    status = BMP180_ReadCalibrationOffsets(
        &bmp180,
        &offsets
    );

    if(status != BMP180_OK) {
        printf("Reading calibration offsets failed\r\n");
        return;
    }

    BMP180_WindowInit(&bmp_window);

    printf("BMP180 active warming up...\r\n");

    uint32_t warmup_start_tick = HAL_GetTick();

    while((HAL_GetTick() - warmup_start_tick) < 30000U) {

        status = BMP180_MeasureData(
            &bmp180,
            &bmp_raw,
            &bmp_scaled,
            &offsets,
            &bmp_window
        );

        if(status != BMP180_OK) {
            BMP180_NotiStatus(
                status,
                "Warm-up measurement failed"
            );
            return;
        }


        HAL_Delay(100U);
    }


    BMP180_WindowInit(&bmp_window);

    status = BMP180_SetStartupPressurePa(
        &bmp180,
        &bmp_raw,
        &bmp_scaled,
        &offsets,
        &bmp_window
    );

    if(status != BMP180_OK) {
        BMP180_NotiStatus(
            status,
            "Startup pressure failed"
        );
        return;
    }

    BMP180_WindowInit(&bmp_window);

    BMP180_FilterInit(&filter);


    filter.last_pressure_pa =
        bmp180.startup_pressure_pa;

    filter.first_data = 0U;

    bmp180_ready = 1U;

    printf(
        "Startup pressure: %.2f Pa\r\n",
        bmp180.startup_pressure_pa
    );
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
	BMP180_Setup();
	BMP180_Status_t status;
	while (1)
	{
		status = BMP180_MeasureData(
		    &bmp180,
		    &bmp_raw,
		    &bmp_scaled,
		    &offsets,
		    &bmp_window
		);

		if(status != BMP180_OK) {
		    continue;
		}

		float raw_pressure_pa =
		    (float)bmp_scaled.pressure_pa;

		float average_pressure_pa =
		    BMP180_WindowGetAvg(&bmp_window);

		float filtered_pressure_pa =
		    BMP180_EMAFilter(
		        &filter,
		        average_pressure_pa
		    );

		float raw_height = 0.0f;
		float average_height = 0.0f;
		float filtered_height = 0.0f;

		BMP180_CalculateRelativeAltitude(
		    raw_pressure_pa,
		    bmp180.startup_pressure_pa,
		    &raw_height
		);

		BMP180_CalculateRelativeAltitude(
		    average_pressure_pa,
		    bmp180.startup_pressure_pa,
		    &average_height
		);

		BMP180_CalculateRelativeAltitude(
		    filtered_pressure_pa,
		    bmp180.startup_pressure_pa,
		    &filtered_height
		);

		printf(
		    "t=%lu T=%.1f P=%lu "
		    "RawH=%.2f AvgH=%.2f EmaH=%.2f Alpha=%.2f\r\n",

		    (unsigned long)HAL_GetTick(),
		    bmp_scaled.temperature_c,
		    (unsigned long)bmp_scaled.pressure_pa,
		    raw_height,
		    average_height,
		    filtered_height,
		    filter.active_alpha
		);

		HAL_Delay(100U);
	}
	return 0;
}
