/*
 * lcd_i2c.h
 *
 *  Created on: Jun 24, 2026
 *      Author: vietht-hl
 */

#ifndef DEVICES_INC_LCD_I2C_H_
#define DEVICES_INC_LCD_I2C_H_

void lcd_parse_8bit(uint8_t* pdata, char data);
void lcd_send_cmd(I2C_HandleTypeDef* hi2c, char cmd);
void lcd_send_data(I2C_HandleTypeDef* hi2c, char data);
void lcd_send_string(I2C_HandleTypeDef* hi2c, char* str);
void lcd_set_cursor(I2C_HandleTypeDef* hi2c, uint8_t row, uint8_t col);
void lcd_init(I2C_HandleTypeDef* hi2c);


#endif /* DEVICES_INC_LCD_I2C_H_ */
