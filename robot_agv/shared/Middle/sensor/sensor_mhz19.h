/*******************************************************************************
 *
 * Copyright (c) 2025
 * All Rights Reserved
 *
 * Description: MH-Z19 CO2 sensor driver (USART1 on PA9/PA10)
 *
 * Author: Developer embedded team
 *
 ******************************************************************************/
#ifndef SENSOR_MHZ19_H
#define SENSOR_MHZ19_H
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
typedef struct {
    uint16_t co2_ppm;
    int8_t temperature_c;
    uint8_t status;
} mhz19_reading_t;

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void mhz19_init(void);
bool mhz19_read(mhz19_reading_t *out);

#endif /* SENSOR_MHZ19_H */

/* END_FILE */
