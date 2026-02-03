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
#ifndef SENSOR_PHOTODIODE_H
#define SENSOR_PHOTODIODE_H
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
} photodiode_config_t;

typedef struct {
    uint16_t raw;
    float voltage_v;
} photodiode_reading_t;

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
photodiode_config_t photodiode_default_config(void);
void photodiode_init(void);
uint16_t photodiode_read_raw(void);
float photodiode_raw_to_voltage(uint16_t raw, const photodiode_config_t *cfg);
bool photodiode_read(photodiode_reading_t *out, const photodiode_config_t *cfg);

#endif /* SENSOR_PHOTODIODE_H */

/* END_FILE */
