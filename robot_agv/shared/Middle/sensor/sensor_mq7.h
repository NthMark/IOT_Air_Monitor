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
#ifndef SENSOR_MQ7_H
#define SENSOR_MQ7_H
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
typedef struct {
    float vref_v;
    float adc_max;
    float rl_ohms;
    float r0_ohms;
    float log_m;
    float log_b;
} mq7_config_t;

typedef struct {
    uint16_t raw;
    float voltage_v;
    float ppm;
} mq7_reading_t;

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
mq7_config_t mq7_default_config(void);
void mq7_init(void);
uint16_t mq7_read_raw(void);
float mq7_raw_to_voltage(uint16_t raw, const mq7_config_t *cfg);
float mq7_voltage_to_ppm(float voltage_v, const mq7_config_t *cfg);
bool mq7_read(mq7_reading_t *out, const mq7_config_t *cfg);

#endif /* SENSOR_MQ7_H */

/* END_FILE */
