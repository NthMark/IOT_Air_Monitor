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
#ifndef SENSOR_GP2Y_H
#define SENSOR_GP2Y_H
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
    float dust_zero_v;
    float dust_slope_v_per_mg_m3;
    bool led_active_low;
    uint32_t led_on_us;
    uint32_t sample_delay_us;
    uint32_t led_off_us;
    uint16_t disconnect_raw_min;
    uint16_t disconnect_raw_max;
    uint16_t disconnect_delta_raw_min;
    uint8_t disconnect_limit;
} gp2y_config_t;

typedef struct {
    uint16_t raw;
    float voltage_v;
    float dust_mg_m3;
} gp2y_reading_t;

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
gp2y_config_t gp2y_default_config(void);
void gp2y_init(void);
bool gp2y_read(gp2y_reading_t *out, const gp2y_config_t *cfg);

#endif /* SENSOR_GP2Y_H */

/* END_FILE */
