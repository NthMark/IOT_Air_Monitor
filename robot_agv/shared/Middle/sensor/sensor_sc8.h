/*******************************************************************************
 *
 * Copyright (c) 2025
 * All Rights Reserved
 *
 * Description: SC8 (C8) CO2 sensor driver (UART, active output mode)
 *
 * Author: Developer embedded team
 *
 ******************************************************************************/
#ifndef SENSOR_SC8_H
#define SENSOR_SC8_H
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
typedef struct {
    uint16_t co2_ppm;
} sc8_reading_t;

typedef enum {
    SC8_IF_UART = 0,
    SC8_IF_PWM
} sc8_interface_t;

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void sc8_init(sc8_interface_t iface);
bool sc8_read(sc8_reading_t *out);

#endif /* SENSOR_SC8_H */

/* END_FILE */
