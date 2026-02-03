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
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stddef.h>
#include <string.h>
#include "lcd_i2c.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "system_stm32f10x.h"
#include "timer.h"
#include "stm32f10x.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define LCD_I2C                    I2C1
#define LCD_I2C_RCC                RCC_APB1Periph_I2C1
#define LCD_I2C_GPIO_RCC           RCC_APB2Periph_GPIOB
#define LCD_I2C_GPIO               GPIOB
#define LCD_I2C_SCL_PIN            GPIO_Pin_6
#define LCD_I2C_SDA_PIN            GPIO_Pin_7

#define LCD_I2C_SPEED_HZ           100000u
#define LCD_I2C_TIMEOUT_MS         50u

#define LCD_I2C_BL                 0x08u
#define LCD_I2C_EN                 0x04u
#define LCD_I2C_RW                 0x02u
#define LCD_I2C_RS                 0x01u

#define LCD_CMD_CLEAR              0x01u
#define LCD_CMD_RETURN_HOME        0x02u
#define LCD_CMD_ENTRY_MODE         0x06u
#define LCD_CMD_DISPLAY_ON         0x0Cu
#define LCD_CMD_FUNCTION_SET       0x28u

#define LCD_LINE_LEN               16u

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static bool lcd_i2c_write_byte(uint8_t data);
static bool lcd_i2c_wait_event(uint32_t event);
static void lcd_i2c_delay_us(uint32_t delay_us);
static void lcd_i2c_delay_ms(uint32_t delay_ms);
static void lcd_i2c_pulse_enable(uint8_t data);
static void lcd_i2c_write4(uint8_t nibble, uint8_t mode);
static void lcd_i2c_send(uint8_t value, uint8_t mode);
static void lcd_i2c_command(uint8_t cmd);
static void lcd_i2c_write_char(char c);

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
static uint8_t s_lcd_addr = 0x27u;
static uint8_t s_lcd_backlight = LCD_I2C_BL;
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void lcd_i2c_init(uint8_t addr_7bit)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef i2c;

    if (addr_7bit != 0u) {
        s_lcd_addr = addr_7bit;
    }

    RCC_APB2PeriphClockCmd(LCD_I2C_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(LCD_I2C_RCC, ENABLE);

    gpio.GPIO_Pin = LCD_I2C_SCL_PIN | LCD_I2C_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LCD_I2C_GPIO, &gpio);

    I2C_DeInit(LCD_I2C);
    I2C_StructInit(&i2c);
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x00;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = LCD_I2C_SPEED_HZ;
    I2C_Init(LCD_I2C, &i2c);
    I2C_Cmd(LCD_I2C, ENABLE);

    SystemCoreClockUpdate();
    lcd_i2c_delay_ms(50);

    lcd_i2c_write4(0x30, 0);
    lcd_i2c_delay_ms(5);
    lcd_i2c_write4(0x30, 0);
    lcd_i2c_delay_us(200);
    lcd_i2c_write4(0x30, 0);
    lcd_i2c_delay_us(200);
    lcd_i2c_write4(0x20, 0);
    lcd_i2c_delay_us(200);

    lcd_i2c_command(LCD_CMD_FUNCTION_SET);
    lcd_i2c_command(LCD_CMD_DISPLAY_ON);
    lcd_i2c_command(LCD_CMD_CLEAR);
    lcd_i2c_delay_ms(2);
    lcd_i2c_command(LCD_CMD_ENTRY_MODE);
}

void lcd_i2c_clear(void)
{
    lcd_i2c_command(LCD_CMD_CLEAR);
    lcd_i2c_delay_ms(2);
}

void lcd_i2c_set_cursor(uint8_t row, uint8_t col)
{
    static const uint8_t row_offsets[] = {0x00u, 0x40u, 0x14u, 0x54u};
    if (row > 3u) {
        row = 0u;
    }
    lcd_i2c_command((uint8_t)(0x80u | (col + row_offsets[row])));
}

void lcd_i2c_print(const char *text)
{
    if (text == NULL) {
        return;
    }
    while (*text != '\0') {
        lcd_i2c_write_char(*text++);
    }
}

void lcd_i2c_write_line(uint8_t row, const char *text)
{
    char buf[LCD_LINE_LEN + 1u];
    size_t len = 0u;

    if (text == NULL) {
        return;
    }

    memset(buf, ' ', LCD_LINE_LEN);
    buf[LCD_LINE_LEN] = '\0';

    while (text[len] != '\0' && len < LCD_LINE_LEN) {
        buf[len] = text[len];
        len++;
    }

    lcd_i2c_set_cursor(row, 0u);
    lcd_i2c_print(buf);
}

void lcd_i2c_set_backlight(bool on)
{
    s_lcd_backlight = on ? LCD_I2C_BL : 0u;
    lcd_i2c_write_byte(s_lcd_backlight);
}

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static bool lcd_i2c_write_byte(uint8_t data)
{
    uint32_t start = GetMilSecTick();

    while (I2C_GetFlagStatus(LCD_I2C, I2C_FLAG_BUSY) != RESET) {
        if ((GetMilSecTick() - start) > LCD_I2C_TIMEOUT_MS) {
            return false;
        }
    }

    I2C_GenerateSTART(LCD_I2C, ENABLE);
    if (!lcd_i2c_wait_event(I2C_EVENT_MASTER_MODE_SELECT)) {
        return false;
    }

    I2C_Send7bitAddress(LCD_I2C, (uint8_t)(s_lcd_addr << 1u), I2C_Direction_Transmitter);
    if (!lcd_i2c_wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        I2C_GenerateSTOP(LCD_I2C, ENABLE);
        return false;
    }

    I2C_SendData(LCD_I2C, data);
    if (!lcd_i2c_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        I2C_GenerateSTOP(LCD_I2C, ENABLE);
        return false;
    }

    I2C_GenerateSTOP(LCD_I2C, ENABLE);
    return true;
}

static bool lcd_i2c_wait_event(uint32_t event)
{
    uint32_t start = GetMilSecTick();
    while (!I2C_CheckEvent(LCD_I2C, event)) {
        if ((GetMilSecTick() - start) > LCD_I2C_TIMEOUT_MS) {
            return false;
        }
    }
    return true;
}

static void lcd_i2c_delay_ms(uint32_t delay_ms)
{
    uint32_t start = GetMilSecTick();
    while ((GetMilSecTick() - start) < delay_ms) {}
}

static void lcd_i2c_delay_us(uint32_t delay_us)
{
    uint32_t ticks_per_us = SystemCoreClock / 1000000u;
    uint32_t load;
    uint32_t start;
    uint32_t now;
    uint32_t elapsed;
    uint64_t remaining;

    if (delay_us == 0u || ticks_per_us == 0u) {
        return;
    }

    if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 0u) {
        uint32_t wait_ms = (delay_us + 999u) / 1000u;
        lcd_i2c_delay_ms(wait_ms);
        return;
    }

    load = SysTick->LOAD + 1u;
    start = SysTick->VAL;
    remaining = (uint64_t)delay_us * (uint64_t)ticks_per_us;
    while (remaining > 0u) {
        now = SysTick->VAL;
        elapsed = (start >= now) ? (start - now) : (start + (load - now));
        if ((uint64_t)elapsed >= remaining) {
            break;
        }
        remaining -= elapsed;
        start = now;
    }
}

static void lcd_i2c_pulse_enable(uint8_t data)
{
    lcd_i2c_write_byte((uint8_t)(data | LCD_I2C_EN | s_lcd_backlight));
    lcd_i2c_delay_us(1);
    lcd_i2c_write_byte((uint8_t)((data & ~LCD_I2C_EN) | s_lcd_backlight));
    lcd_i2c_delay_us(50);
}

static void lcd_i2c_write4(uint8_t nibble, uint8_t mode)
{
    uint8_t data = (uint8_t)(nibble & 0xF0u);
    data |= (mode ? LCD_I2C_RS : 0u);
    data |= s_lcd_backlight;
    lcd_i2c_pulse_enable(data);
}

static void lcd_i2c_send(uint8_t value, uint8_t mode)
{
    lcd_i2c_write4(value, mode);
    lcd_i2c_write4((uint8_t)(value << 4u), mode);
}

static void lcd_i2c_command(uint8_t cmd)
{
    lcd_i2c_send(cmd, 0u);
}

static void lcd_i2c_write_char(char c)
{
    lcd_i2c_send((uint8_t)c, 1u);
}

/* END_FILE */
