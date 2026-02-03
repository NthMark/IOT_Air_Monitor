/*******************************************************************************
 *
 * Copyright (c) 2025
 * All Rights Reserved
 *
 * Description: Photodiode light sensor driver (ADC)
 *
 * Author: Developer embedded team
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stddef.h>
#include "sensor_photodiode.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define PHOTODIODE_ADC            ADC1
#define PHOTODIODE_ADC_CHANNEL    ADC_Channel_6
#define PHOTODIODE_ADC_GPIO       GPIOA
#define PHOTODIODE_ADC_PIN        GPIO_Pin_6

#define PHOTODIODE_DEFAULT_VREF_V 3.3f
#define PHOTODIODE_DEFAULT_MAX    4095.0f
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
photodiode_config_t photodiode_default_config(void)
{
    photodiode_config_t cfg;
    cfg.vref_v = PHOTODIODE_DEFAULT_VREF_V;
    cfg.adc_max = PHOTODIODE_DEFAULT_MAX;
    return cfg;
}

void photodiode_init(void)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef adc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    gpio.GPIO_Pin  = PHOTODIODE_ADC_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(PHOTODIODE_ADC_GPIO, &gpio);

    ADC_DeInit(PHOTODIODE_ADC);
    ADC_StructInit(&adc);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(PHOTODIODE_ADC, &adc);

    ADC_Cmd(PHOTODIODE_ADC, ENABLE);
    ADC_ResetCalibration(PHOTODIODE_ADC);
    while (ADC_GetResetCalibrationStatus(PHOTODIODE_ADC) != RESET) {}

    ADC_StartCalibration(PHOTODIODE_ADC);
    while (ADC_GetCalibrationStatus(PHOTODIODE_ADC) != RESET) {}

    ADC_RegularChannelConfig(PHOTODIODE_ADC, PHOTODIODE_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
}

uint16_t photodiode_read_raw(void)
{
    ADC_RegularChannelConfig(PHOTODIODE_ADC, PHOTODIODE_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(PHOTODIODE_ADC, ENABLE);
    while (ADC_GetFlagStatus(PHOTODIODE_ADC, ADC_FLAG_EOC) == RESET) {}
    return ADC_GetConversionValue(PHOTODIODE_ADC);
}

float photodiode_raw_to_voltage(uint16_t raw, const photodiode_config_t *cfg)
{
    if (cfg == NULL || cfg->adc_max <= 0.0f) {
        return 0.0f;
    }
    return ((float)raw * cfg->vref_v) / cfg->adc_max;
}

bool photodiode_read(photodiode_reading_t *out, const photodiode_config_t *cfg)
{
    if (out == NULL || cfg == NULL) {
        return false;
    }

    out->raw = photodiode_read_raw();
    out->voltage_v = photodiode_raw_to_voltage(out->raw, cfg);
    return true;
}

/* END_FILE */
