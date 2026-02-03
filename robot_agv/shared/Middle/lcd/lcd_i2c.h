/*******************************************************************************
 *
 * Copyright (c) 2025
 * All Rights Reserved
 *
 * Description: I2C LCD (HD44780 via PCF8574)
 *
 * Author: Developer embedded team
 *
 ******************************************************************************/
#ifndef LCD_I2C_H
#define LCD_I2C_H
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void lcd_i2c_init(uint8_t addr_7bit);
void lcd_i2c_clear(void);
void lcd_i2c_set_cursor(uint8_t row, uint8_t col);
void lcd_i2c_print(const char *text);
void lcd_i2c_write_line(uint8_t row, const char *text);
void lcd_i2c_set_backlight(bool on);

#endif /* LCD_I2C_H */

/* END_FILE */
