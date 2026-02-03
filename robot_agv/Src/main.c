/*******************************************************************************
 *
 * Copyright (c) 2025
 * All Rights Reserved
 *
 * Description: Sensors reader (MQ-7 + MH-Z19)
 *
 * Author: Developer embedded team
 *
 * Last Changed By:  $Author: Dev team $
 * Revision:         $Revision: 2.0.0 $
 * Last Changed:     $Date:  $
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdbool.h>
#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "logger.h"
#include "lcd_i2c.h"
#include "motor_mosfet.h"
#include "sensor_dht11.h"
#include "sensor_photodiode.h"
#include "sensor_gp2y.h"
#include "sensor_sc8.h"
#include "sensor_mq7.h"
#include "stm32f10x_usart.h"
#include "timer.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
typedef enum {
    SENSOR_MODE_MQ7 = 0,
    SENSOR_MODE_SC8,
    SENSOR_MODE_GP2Y,
    SENSOR_MODE_PHOTO,
    SENSOR_MODE_DHT11,
    SENSOR_MODE_BOTH,
    SENSOR_MODE_PHOTO_DHT11,
    SENSOR_MODE_ALL
} eSensorMode;

#ifndef SENSOR_MODE_ACTIVE
// Select SENSOR_MODE_MQ7, SENSOR_MODE_SC8, SENSOR_MODE_GP2Y, SENSOR_MODE_PHOTO,
// SENSOR_MODE_DHT11, SENSOR_MODE_BOTH, SENSOR_MODE_PHOTO_DHT11, or SENSOR_MODE_ALL
#define SENSOR_MODE_ACTIVE           SENSOR_MODE_PHOTO_DHT11
#endif

#ifndef SC8_IF_ACTIVE
#define SC8_IF_ACTIVE                SC8_IF_PWM 
#endif

#define MQ7_SAMPLE_PERIOD_MS         1100u
#define SC8_SAMPLE_PERIOD_MS         1200u
#define GP2Y_SAMPLE_PERIOD_MS        1300u
#define PHOTO_SAMPLE_PERIOD_MS       1000u
#define DHT11_SAMPLE_PERIOD_MS       2000u
#define LCD_UPDATE_PERIOD_MS         1000u
#define LCD_I2C_ADDR_DEFAULT         0x27u
#define MOTOR_MOSFET_DEFAULT_ON      0u

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
static SSwTimer s_mq7_timer_id = TIMER_ID_INVALID;
static SSwTimer s_sc8_timer_id = TIMER_ID_INVALID;
static SSwTimer s_gp2y_timer_id = TIMER_ID_INVALID;
static SSwTimer s_photo_timer_id = TIMER_ID_INVALID;
static SSwTimer s_dht11_timer_id = TIMER_ID_INVALID;
static SSwTimer s_lcd_timer_id = TIMER_ID_INVALID;
static mq7_config_t s_mq7_cfg;
static gp2y_config_t s_gp2y_cfg;
static photodiode_config_t s_photo_cfg;
static mq7_reading_t s_mq7_last;
static sc8_reading_t s_sc8_last;
static gp2y_reading_t s_gp2y_last;
static photodiode_reading_t s_photo_last;
static dht11_reading_t s_dht11_last;
static bool s_mq7_valid = false;
static bool s_sc8_valid = false;
static bool s_gp2y_valid = false;
static bool s_photo_valid = false;
static bool s_dht11_valid = false;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void mq7_read_timer_cb(void *arg);
static void sc8_read_timer_cb(void *arg);
static void gp2y_read_timer_cb(void *arg);
static void photodiode_read_timer_cb(void *arg);
static void dht11_read_timer_cb(void *arg);
static void lcd_update_timer_cb(void *arg);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

int main(void)
{
    debug_init();
    debug_reconfig(UART_BAUDRATE_FMR, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
    TimerInit();
    lcd_i2c_init(LCD_I2C_ADDR_DEFAULT);
    lcd_i2c_clear();
    motor_mosfet_init();
    if (MOTOR_MOSFET_DEFAULT_ON != 0u) {
        motor_mosfet_on();
    } else {
        motor_mosfet_off();
    }

    if ((SENSOR_MODE_ACTIVE == SENSOR_MODE_MQ7) ||
        (SENSOR_MODE_ACTIVE == SENSOR_MODE_BOTH) ||
        (SENSOR_MODE_ACTIVE == SENSOR_MODE_ALL)) {
        s_mq7_cfg = mq7_default_config();
        mq7_init();

        s_mq7_timer_id = TimerStart("MQ7_READ",
                                    MQ7_SAMPLE_PERIOD_MS,
                                    TIMER_REPEAT_FOREVER,
                                    mq7_read_timer_cb,
                                    NULL);
        if (s_mq7_timer_id == TIMER_ID_INVALID) {
            debug_error("MQ7_SS: Failed to start timer");
        } else {
            debug_info("MQ7_SS: Sampling every %u ms", MQ7_SAMPLE_PERIOD_MS);
        }
    }

    if ((SENSOR_MODE_ACTIVE == SENSOR_MODE_SC8) ||
        (SENSOR_MODE_ACTIVE == SENSOR_MODE_ALL)) {
        sc8_init(SC8_IF_ACTIVE);
        s_sc8_timer_id = TimerStart("SC8_READ",
                                      SC8_SAMPLE_PERIOD_MS,
                                      TIMER_REPEAT_FOREVER,
                                      sc8_read_timer_cb,
                                      NULL);
        if (s_sc8_timer_id == TIMER_ID_INVALID) {
            debug_error("SC8_SS: Failed to start timer");
        } else {
            debug_info("SC8_SS: Sampling every %u ms", SC8_SAMPLE_PERIOD_MS);
        }
    }

    if ((SENSOR_MODE_ACTIVE == SENSOR_MODE_GP2Y) ||
        (SENSOR_MODE_ACTIVE == SENSOR_MODE_BOTH) ||
        (SENSOR_MODE_ACTIVE == SENSOR_MODE_ALL)) {
        s_gp2y_cfg = gp2y_default_config();
        gp2y_init();

        s_gp2y_timer_id = TimerStart("GP2Y_READ",
                                     GP2Y_SAMPLE_PERIOD_MS,
                                     TIMER_REPEAT_FOREVER,
                                     gp2y_read_timer_cb,
                                     NULL);
        if (s_gp2y_timer_id == TIMER_ID_INVALID) {
            debug_error("GP2Y_SS: Failed to start timer");
        } else {
            debug_info("GP2Y_SS: Sampling every %u ms", GP2Y_SAMPLE_PERIOD_MS);
        }
    }

    if ((SENSOR_MODE_ACTIVE == SENSOR_MODE_PHOTO) ||
        (SENSOR_MODE_ACTIVE == SENSOR_MODE_PHOTO_DHT11) ||
        (SENSOR_MODE_ACTIVE == SENSOR_MODE_ALL)) {
        s_photo_cfg = photodiode_default_config();
        photodiode_init();

        s_photo_timer_id = TimerStart("PHOTO_READ",
                                      PHOTO_SAMPLE_PERIOD_MS,
                                      TIMER_REPEAT_FOREVER,
                                      photodiode_read_timer_cb,
                                      NULL);
        if (s_photo_timer_id == TIMER_ID_INVALID) {
            debug_error("PHOTO_SS: Failed to start timer");
        } else {
            debug_info("PHOTO_SS: Sampling every %u ms", PHOTO_SAMPLE_PERIOD_MS);
        }
    }

    if ((SENSOR_MODE_ACTIVE == SENSOR_MODE_DHT11) ||
        (SENSOR_MODE_ACTIVE == SENSOR_MODE_PHOTO_DHT11) ||
        (SENSOR_MODE_ACTIVE == SENSOR_MODE_ALL)) {
        dht11_init();
        s_dht11_timer_id = TimerStart("DHT11_READ",
                                      DHT11_SAMPLE_PERIOD_MS,
                                      TIMER_REPEAT_FOREVER,
                                      dht11_read_timer_cb,
                                      NULL);
        if (s_dht11_timer_id == TIMER_ID_INVALID) {
            debug_error("DHT11_SS: Failed to start timer");
        } else {
            debug_info("DHT11_SS: Sampling every %u ms", DHT11_SAMPLE_PERIOD_MS);
        }
    }

    if ((SENSOR_MODE_ACTIVE != SENSOR_MODE_MQ7) &&
        (SENSOR_MODE_ACTIVE != SENSOR_MODE_SC8) &&
        (SENSOR_MODE_ACTIVE != SENSOR_MODE_GP2Y) &&
        (SENSOR_MODE_ACTIVE != SENSOR_MODE_PHOTO) &&
        (SENSOR_MODE_ACTIVE != SENSOR_MODE_DHT11) &&
        (SENSOR_MODE_ACTIVE != SENSOR_MODE_BOTH) &&
        (SENSOR_MODE_ACTIVE != SENSOR_MODE_PHOTO_DHT11) &&
        (SENSOR_MODE_ACTIVE != SENSOR_MODE_ALL)) {
        debug_error("SENSOR_SS: Invalid SENSOR_MODE_ACTIVE=%d", SENSOR_MODE_ACTIVE);
    }

    s_lcd_timer_id = TimerStart("LCD_UPDATE",
                                LCD_UPDATE_PERIOD_MS,
                                TIMER_REPEAT_FOREVER,
                                lcd_update_timer_cb,
                                NULL);
    if (s_lcd_timer_id == TIMER_ID_INVALID) {
        debug_error("LCD_SS: Failed to start timer");
    }

    while (1)
    {
        debug_rx_poll();
    	processTimerScheduler();
    }
}

static void mq7_read_timer_cb(void *arg)
{
    (void)arg;
    mq7_reading_t reading;
    if (!mq7_read(&reading, &s_mq7_cfg)) {
        debug_error("MQ7_SS: Read failed");
        return;
    }

    debug_info("MQ7_SS: raw=%u, voltage=%.3f V, ppm=%.2f",
               reading.raw, reading.voltage_v, reading.ppm);
    s_mq7_last = reading;
    s_mq7_valid = true;
}

static void sc8_read_timer_cb(void *arg)
{
    (void)arg;
    sc8_reading_t reading;
    if (!sc8_read(&reading)) {
        debug_error("SC8_SS: Read failed");
        s_sc8_valid = false;
        return;
    }

    debug_info("SC8_SS: co2=%u ppm", reading.co2_ppm);
    s_sc8_last = reading;
    s_sc8_valid = true;
}

static void gp2y_read_timer_cb(void *arg)
{
    (void)arg;
    gp2y_reading_t reading;
    if (!gp2y_read(&reading, &s_gp2y_cfg)) {
        debug_error("GP2Y_SS: Read failed");
        s_gp2y_valid = false;
        return;
    }

    debug_info("GP2Y_SS: raw=%u, voltage=%.3f V, dust=%.3f mg/m3",
               reading.raw, reading.voltage_v, reading.dust_mg_m3);
    s_gp2y_last = reading;
    s_gp2y_valid = true;
}

static void photodiode_read_timer_cb(void *arg)
{
    (void)arg;
    photodiode_reading_t reading;
    if (!photodiode_read(&reading, &s_photo_cfg)) {
        debug_error("PHOTO_SS: Read failed");
        s_photo_valid = false;
        return;
    }

    debug_info("PHOTO_SS: raw=%u, voltage=%.3f V", reading.raw, reading.voltage_v);
    s_photo_last = reading;
    s_photo_valid = true;
}

static void dht11_read_timer_cb(void *arg)
{
    (void)arg;
    dht11_reading_t reading;
    if (!dht11_read(&reading)) {
        debug_error("DHT11_SS: Read failed");
        s_dht11_valid = false;
        return;
    }

    debug_info("DHT11_SS: temp=%.1f C, hum=%.1f %%", reading.temperature, reading.humidity);
    s_dht11_last = reading;
    s_dht11_valid = true;
}

static void lcd_update_timer_cb(void *arg)
{
    char line1[17];
    char line2[17];
    char co_buf[5];
    char co2_buf[5];
    char dust_buf[8];
    unsigned int co_ppm = 0u;
    unsigned int co2_ppm = 0u;
    float dust = 0.0f;

    (void)arg;

    if ((SENSOR_MODE_ACTIVE == SENSOR_MODE_PHOTO) ||
        (SENSOR_MODE_ACTIVE == SENSOR_MODE_DHT11) ||
        (SENSOR_MODE_ACTIVE == SENSOR_MODE_PHOTO_DHT11)) {
        char temp_buf[6];
        char hum_buf[5];
        char light_buf[7];

        if (s_dht11_valid) {
            (void)snprintf(temp_buf, sizeof(temp_buf), "%4.1f", s_dht11_last.temperature);
            (void)snprintf(hum_buf, sizeof(hum_buf), "%3.0f", s_dht11_last.humidity);
        } else {
            (void)snprintf(temp_buf, sizeof(temp_buf), "----");
            (void)snprintf(hum_buf, sizeof(hum_buf), "---");
        }

        if (s_photo_valid) {
            (void)snprintf(light_buf, sizeof(light_buf), "%5.3f", s_photo_last.voltage_v);
        } else {
            (void)snprintf(light_buf, sizeof(light_buf), "-----");
        }

        (void)snprintf(line1, sizeof(line1), "T:%4sC H:%3s%%", temp_buf, hum_buf);
        (void)snprintf(line2, sizeof(line2), "Light:%5sV", light_buf);
        lcd_i2c_write_line(0u, line1);
        lcd_i2c_write_line(1u, line2);
        return;
    }

    if (s_mq7_valid) {
        co_ppm = (unsigned int)(s_mq7_last.ppm + 0.5f);
        (void)snprintf(co_buf, sizeof(co_buf), "%4u", co_ppm);
    } else {
        (void)snprintf(co_buf, sizeof(co_buf), "----");
    }

    if (s_sc8_valid) {
        co2_ppm = (unsigned int)s_sc8_last.co2_ppm;
        (void)snprintf(co2_buf, sizeof(co2_buf), "%4u", co2_ppm);
    } else {
        (void)snprintf(co2_buf, sizeof(co2_buf), "----");
    }

    (void)snprintf(line1, sizeof(line1), "CO:%4s CO2:%4s", co_buf, co2_buf);

    if (s_gp2y_valid) {
        dust = s_gp2y_last.dust_mg_m3;
        (void)snprintf(dust_buf, sizeof(dust_buf), "%5.3f", dust);
    } else {
        (void)snprintf(dust_buf, sizeof(dust_buf), " ----");
    }

    (void)snprintf(line2, sizeof(line2), "PM2.5:%s", dust_buf);
    lcd_i2c_write_line(0u, line1);
    lcd_i2c_write_line(1u, line2);
}

void logger_cmd_handler(const char *line)
{
    char buf[64];
    size_t i = 0u;

    if (line == NULL) {
        return;
    }

    while (line[i] != '\0' && i < (sizeof(buf) - 1u)) {
        buf[i] = (char)tolower((unsigned char)line[i]);
        i++;
    }
    buf[i] = '\0';

    if ((strstr(buf, "fan") == NULL) && (strstr(buf, "status") == NULL)) {
        return;
    }

    if (strstr(buf, "off") != NULL) {
        debug_info("MOTOR_SS: Turning motor OFF");
        motor_mosfet_off();
    } else if (strstr(buf, "on") != NULL) {
        debug_info("MOTOR_SS: Turning motor ON");
        motor_mosfet_on();
    }
}

/* END_FILE */
