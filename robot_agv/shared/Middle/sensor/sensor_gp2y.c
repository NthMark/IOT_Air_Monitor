/*******************************************************************************
 *
 * Copyright (c) 2025
 * All Rights Reserved
 *
 * Description: GP2Y1010AU0F dust sensor driver (ADC + LED GPIO)
 *
 * Author: Developer embedded team
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stddef.h>
#include "sensor_gp2y.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "system_stm32f10x.h"
#include "timer.h"
#include "stm32f10x.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define GP2Y_ADC                     ADC1
#define GP2Y_ADC_CHANNEL             ADC_Channel_1
#define GP2Y_ADC_GPIO                GPIOA
#define GP2Y_ADC_PIN                 GPIO_Pin_1

#define GP2Y_LED_GPIO                GPIOA
#define GP2Y_LED_PIN                 GPIO_Pin_4

#define GP2Y_DEFAULT_VREF_V          3.3f
#define GP2Y_DEFAULT_ADC_MAX         4095.0f
#define GP2Y_DEFAULT_ZERO_V          0.6f
#define GP2Y_DEFAULT_SLOPE_V_PER_MG  0.5f

#define GP2Y_DEFAULT_LED_ON_US       320u
#define GP2Y_DEFAULT_SAMPLE_US       280u
#define GP2Y_DEFAULT_LED_OFF_US      9680u
#define GP2Y_DEFAULT_DISCONNECT_MIN_RAW 20u
#define GP2Y_DEFAULT_DISCONNECT_MAX_RAW 4075u
#define GP2Y_DEFAULT_DISCONNECT_DELTA_RAW 20u
#define GP2Y_DEFAULT_DISCONNECT_LIMIT   3u

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void gp2y_delay_us(uint32_t delay_us);
static void gp2y_led_set(bool on, bool active_low);
static uint16_t gp2y_read_raw(void);

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
static uint8_t s_gp2y_disconnect_count = 0u;
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
gp2y_config_t gp2y_default_config(void)
{
    gp2y_config_t cfg;
    cfg.vref_v = GP2Y_DEFAULT_VREF_V;
    cfg.adc_max = GP2Y_DEFAULT_ADC_MAX;
    cfg.dust_zero_v = GP2Y_DEFAULT_ZERO_V;
    cfg.dust_slope_v_per_mg_m3 = GP2Y_DEFAULT_SLOPE_V_PER_MG;
    cfg.led_active_low = true;
    cfg.led_on_us = GP2Y_DEFAULT_LED_ON_US;
    cfg.sample_delay_us = GP2Y_DEFAULT_SAMPLE_US;
    cfg.led_off_us = GP2Y_DEFAULT_LED_OFF_US;
    cfg.disconnect_raw_min = GP2Y_DEFAULT_DISCONNECT_MIN_RAW;
    cfg.disconnect_raw_max = GP2Y_DEFAULT_DISCONNECT_MAX_RAW;
    cfg.disconnect_delta_raw_min = GP2Y_DEFAULT_DISCONNECT_DELTA_RAW;
    cfg.disconnect_limit = GP2Y_DEFAULT_DISCONNECT_LIMIT;
    return cfg;
}

void gp2y_init(void)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef adc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    gpio.GPIO_Pin = GP2Y_ADC_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GP2Y_ADC_GPIO, &gpio);

    gpio.GPIO_Pin = GP2Y_LED_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GP2Y_LED_GPIO, &gpio);
    GPIO_SetBits(GP2Y_LED_GPIO, GP2Y_LED_PIN);

    ADC_DeInit(GP2Y_ADC);
    ADC_StructInit(&adc);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(GP2Y_ADC, &adc);

    ADC_Cmd(GP2Y_ADC, ENABLE);
    ADC_ResetCalibration(GP2Y_ADC);
    while (ADC_GetResetCalibrationStatus(GP2Y_ADC) != RESET) {}

    ADC_StartCalibration(GP2Y_ADC);
    while (ADC_GetCalibrationStatus(GP2Y_ADC) != RESET) {}

    ADC_RegularChannelConfig(GP2Y_ADC, GP2Y_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);

    SystemCoreClockUpdate();
}

bool gp2y_read(gp2y_reading_t *out, const gp2y_config_t *cfg)
{
    uint32_t post_sample_us;

    if (out == NULL || cfg == NULL || cfg->adc_max <= 0.0f) {
        return false;
    }

    uint16_t raw_off;
    uint16_t raw_on;
    uint16_t delta;
    bool disconnect = false;

    gp2y_led_set(false, cfg->led_active_low);
    raw_off = gp2y_read_raw();

    gp2y_led_set(true, cfg->led_active_low);
    gp2y_delay_us(cfg->sample_delay_us);

    raw_on = gp2y_read_raw();
    out->raw = raw_on;
    delta = (raw_on > raw_off) ? (raw_on - raw_off) : (raw_off - raw_on);
    if ((raw_on <= cfg->disconnect_raw_min) ||
        (raw_on >= cfg->disconnect_raw_max) ||
        (delta < cfg->disconnect_delta_raw_min)) {
        disconnect = true;
    }

    if (disconnect) {
        if (s_gp2y_disconnect_count < 0xFFu) {
            s_gp2y_disconnect_count++;
        }
        if (s_gp2y_disconnect_count >= cfg->disconnect_limit) {
            return false;
        }
    } else {
        s_gp2y_disconnect_count = 0u;
    }
    out->voltage_v = ((float)out->raw * cfg->vref_v) / cfg->adc_max;

    if (cfg->dust_slope_v_per_mg_m3 > 0.0f) {
        out->dust_mg_m3 = (out->voltage_v - cfg->dust_zero_v) / cfg->dust_slope_v_per_mg_m3;
        if (out->dust_mg_m3 < 0.0f) {
            out->dust_mg_m3 = 0.0f;
        }
    } else {
        out->dust_mg_m3 = 0.0f;
    }

    post_sample_us = (cfg->led_on_us > cfg->sample_delay_us) ?
                     (cfg->led_on_us - cfg->sample_delay_us) : 0u;
    if (post_sample_us > 0u) {
        gp2y_delay_us(post_sample_us);
    }

    gp2y_led_set(false, cfg->led_active_low);
    if (cfg->led_off_us > 0u) {
        gp2y_delay_us(cfg->led_off_us);
    }

    return true;
}

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void gp2y_delay_us(uint32_t delay_us)
{
    uint32_t ticks_per_us = SystemCoreClock / 1000000u;
    uint32_t load;
    uint32_t start;
    uint32_t now;
    uint32_t elapsed;
    uint64_t remaining;

    if (delay_us == 0u) {
        return;
    }

    if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 0u || ticks_per_us == 0u) {
        uint32_t start_ms = GetMilSecTick();
        uint32_t wait_ms = (delay_us + 999u) / 1000u;
        while ((GetMilSecTick() - start_ms) < wait_ms) {}
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

static void gp2y_led_set(bool on, bool active_low)
{
    bool drive_low = active_low ? on : !on;
    if (drive_low) {
        GPIO_ResetBits(GP2Y_LED_GPIO, GP2Y_LED_PIN);
    } else {
        GPIO_SetBits(GP2Y_LED_GPIO, GP2Y_LED_PIN);
    }
}

static uint16_t gp2y_read_raw(void)
{
    ADC_RegularChannelConfig(GP2Y_ADC, GP2Y_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(GP2Y_ADC, ENABLE);
    while (ADC_GetFlagStatus(GP2Y_ADC, ADC_FLAG_EOC) == RESET) {}
    return ADC_GetConversionValue(GP2Y_ADC);
}

/* END_FILE */
