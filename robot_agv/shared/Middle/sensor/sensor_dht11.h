/*******************************************************************************
 *
 * Copyright (c) 2025
 * All Rights Reserved
 *
 * Description: DHT11 temperature/humidity sensor (single-wire)
 *
 * Author: Developer embedded team
 *
 ******************************************************************************/
#ifndef SENSOR_DHT11_H
#define SENSOR_DHT11_H
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
typedef struct {
    uint8_t humidity_int;
    uint8_t humidity_dec;
    uint8_t temperature_int;
    uint8_t temperature_dec;
    uint8_t checksum;
    float humidity;
    float temperature;
} dht11_reading_t;

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void dht11_init(void);
bool dht11_read(dht11_reading_t *out);

#endif /* SENSOR_DHT11_H */

/* END_FILE */
