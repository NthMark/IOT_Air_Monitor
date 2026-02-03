/*******************************************************************************
 *
 * Copyright (c) 2025
 * All Rights Reserved
 *
 * Description: MQ-7 sensor driver (ADC1_CH0)
 *
 * Author: Developer embedded team
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <math.h>
#include "sensor_mq7.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define MQ7_ADC                      ADC1
#define MQ7_ADC_CHANNEL              ADC_Channel_0
#define MQ7_ADC_GPIO                 GPIOA
#define MQ7_ADC_PIN                  GPIO_Pin_0

#define MQ7_DEFAULT_VREF_V           3.3f
#define MQ7_DEFAULT_ADC_MAX          4095.0f
#define MQ7_DEFAULT_RL_OHMS          10000.0f
#define MQ7_DEFAULT_R0_OHMS          10000.0f

// CO curve approximation: tune to your calibration data.
#define MQ7_DEFAULT_LOG_M            (-0.77f)
#define MQ7_DEFAULT_LOG_B            (1.699f)
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
mq7_config_t mq7_default_config(void)
{
    mq7_config_t cfg;
    cfg.vref_v = MQ7_DEFAULT_VREF_V;
    cfg.adc_max = MQ7_DEFAULT_ADC_MAX;
    cfg.rl_ohms = MQ7_DEFAULT_RL_OHMS;
    cfg.r0_ohms = MQ7_DEFAULT_R0_OHMS;
    cfg.log_m = MQ7_DEFAULT_LOG_M;
    cfg.log_b = MQ7_DEFAULT_LOG_B;
    return cfg;
}

void mq7_init(void)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef adc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    gpio.GPIO_Pin  = MQ7_ADC_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(MQ7_ADC_GPIO, &gpio);

    ADC_DeInit(MQ7_ADC);
    ADC_StructInit(&adc);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(MQ7_ADC, &adc);

    ADC_Cmd(MQ7_ADC, ENABLE);
    ADC_ResetCalibration(MQ7_ADC);
    while (ADC_GetResetCalibrationStatus(MQ7_ADC) != RESET) {}

    ADC_StartCalibration(MQ7_ADC);
    while (ADC_GetCalibrationStatus(MQ7_ADC) != RESET) {}

    ADC_RegularChannelConfig(MQ7_ADC, MQ7_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
}

uint16_t mq7_read_raw(void)
{
    ADC_RegularChannelConfig(MQ7_ADC, MQ7_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(MQ7_ADC, ENABLE);
    while (ADC_GetFlagStatus(MQ7_ADC, ADC_FLAG_EOC) == RESET) {}
    return ADC_GetConversionValue(MQ7_ADC);
}

float mq7_raw_to_voltage(uint16_t raw, const mq7_config_t *cfg)
{
    if (cfg == NULL || cfg->adc_max <= 0.0f) {
        return 0.0f;
    }
    return ((float)raw * cfg->vref_v) / cfg->adc_max;
}

float mq7_voltage_to_ppm(float voltage_v, const mq7_config_t *cfg)
{
    if (cfg == NULL) {
        return 0.0f;
    }
    if (voltage_v <= 0.001f || voltage_v >= (cfg->vref_v - 0.001f)) {
        return 0.0f;
    }

    float rs = (cfg->vref_v - voltage_v) * cfg->rl_ohms / voltage_v;
    float ratio = rs / cfg->r0_ohms;
    if (ratio <= 0.0f) {
        return 0.0f;
    }

    return powf(10.0f, (log10f(ratio) - cfg->log_b) / cfg->log_m);
}

bool mq7_read(mq7_reading_t *out, const mq7_config_t *cfg)
{
    if (out == NULL || cfg == NULL) {
        return false;
    }

    out->raw = mq7_read_raw();
    out->voltage_v = mq7_raw_to_voltage(out->raw, cfg);
    out->ppm = mq7_voltage_to_ppm(out->voltage_v, cfg);
    return true;
}

/* END_FILE */
