/*
 * lcd_i2c.c
 *
 *  Created on: Jun 23, 2026
 *      Author: vietht-hl
 */

#include "stm32f4xx_hal.h"
#include "stdint.h"
#define SLAVE_ADDR_LCD 0x4E



void lcd_parse_8bit(uint8_t* pdata, uint8_t data, uint8_t rs)
{
    uint8_t data_u = data & 0xF0;
    uint8_t data_l = (data << 4) & 0xF0;

    uint8_t en_high = 0x0C | rs;
    uint8_t en_low  = 0x08 | rs;

    pdata[0] = data_u | en_high;
    pdata[1] = data_u | en_low;
    pdata[2] = data_l | en_high;
    pdata[3] = data_l | en_low;
}


void lcd_send_cmd(I2C_HandleTypeDef* hi2c, char cmd) {
	uint8_t data_cmd[4];
	lcd_parse_8bit(data_cmd, cmd, 0);
	HAL_I2C_Master_Transmit(hi2c, SLAVE_ADDR_LCD, data_cmd, 4, 100);
}



void lcd_send_data(I2C_HandleTypeDef* hi2c, char data) {
	uint8_t data_pass[4];
	lcd_parse_8bit(data_pass, data, 1);
	HAL_I2C_Master_Transmit(hi2c, SLAVE_ADDR_LCD, data_pass, 4, 100);
}

void lcd_send_string(I2C_HandleTypeDef* hi2c, char* str) {
	while(*str) {
		lcd_send_data(hi2c, *str++);
	}
}

void lcd_set_cursor(I2C_HandleTypeDef* hi2c, uint8_t row, uint8_t col) {
	static const uint8_t row_addr[] = {0x00, 0x40, 0x14, 0x54};
	char cmd = 0x80 | (row_addr[row] + col);
	lcd_send_cmd(hi2c, cmd);
}


void lcd_init(I2C_HandleTypeDef* hi2c1)
{
    HAL_Delay(50);
    lcd_send_cmd(hi2c1, 0x30);
    HAL_Delay(5);
    lcd_send_cmd(hi2c1, 0x30);
    HAL_Delay(1);
    lcd_send_cmd(hi2c1, 0x30);
    HAL_Delay(10);
    lcd_send_cmd(hi2c1, 0x20);

    lcd_send_cmd(hi2c1, 0x28);
    HAL_Delay(1);
    lcd_send_cmd(hi2c1, 0x08);
    HAL_Delay(1);
    lcd_send_cmd(hi2c1, 0x01);
    HAL_Delay(2);
    lcd_send_cmd(hi2c1, 0x06);
    HAL_Delay(1);
    lcd_send_cmd(hi2c1, 0x0C);
}
