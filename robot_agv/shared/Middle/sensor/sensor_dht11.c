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
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stddef.h>
#include "sensor_dht11.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "timer.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define DHT11_GPIO            GPIOA
#define DHT11_GPIO_RCC        RCC_APB2Periph_GPIOA
#define DHT11_PIN             GPIO_Pin_8

#define DHT11_START_LOW_MS    18u
#define DHT11_START_HIGH_US   30u
#define DHT11_TIMEOUT_US      100u
#define DHT11_BIT_HIGH_THRESH 50u
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void dht11_pin_output(void);
static void dht11_pin_input(void);
static void dht11_write_low(void);
static void dht11_write_high(void);
static uint8_t dht11_read_pin(void);
static void dht11_delay_us(uint32_t delay_us);
static uint32_t dht11_get_us(void);
static bool dht11_wait_level(uint8_t level, uint32_t timeout_us, uint32_t *elapsed_us);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void dht11_init(void)
{
    RCC_APB2PeriphClockCmd(DHT11_GPIO_RCC, ENABLE);
    dht11_pin_input();
    dht11_write_high();
}

bool dht11_read(dht11_reading_t *out)
{
    uint8_t data[5] = {0};
    uint8_t bit_index = 0u;
    uint8_t byte_index = 0u;
    uint32_t elapsed = 0u;

    if (out == NULL) {
        return false;
    }

    SystemCoreClockUpdate();

    dht11_pin_output();
    dht11_write_low();
    for (uint32_t i = 0; i < DHT11_START_LOW_MS; i++) {
        uint32_t start = GetMilSecTick();
        while ((GetMilSecTick() - start) < 1u) {}
    }
    dht11_write_high();
    dht11_delay_us(DHT11_START_HIGH_US);
    dht11_pin_input();

    if (!dht11_wait_level(0u, DHT11_TIMEOUT_US, NULL)) {
        return false;
    }
    if (!dht11_wait_level(1u, DHT11_TIMEOUT_US, NULL)) {
        return false;
    }
    if (!dht11_wait_level(0u, DHT11_TIMEOUT_US, NULL)) {
        return false;
    }

    for (bit_index = 0u; bit_index < 40u; bit_index++) {
        if (!dht11_wait_level(1u, DHT11_TIMEOUT_US, NULL)) {
            return false;
        }
        if (!dht11_wait_level(0u, DHT11_TIMEOUT_US, &elapsed)) {
            return false;
        }

        byte_index = (uint8_t)(bit_index / 8u);
        data[byte_index] <<= 1u;
        if (elapsed > DHT11_BIT_HIGH_THRESH) {
            data[byte_index] |= 1u;
        }
    }

    out->humidity_int = data[0];
    out->humidity_dec = data[1];
    out->temperature_int = data[2];
    out->temperature_dec = data[3];
    out->checksum = data[4];

    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) {
        return false;
    }

    out->humidity = (float)out->humidity_int + ((float)out->humidity_dec * 0.1f);
    out->temperature = (float)out->temperature_int + ((float)out->temperature_dec * 0.1f);
    return true;
}

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void dht11_pin_output(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = DHT11_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DHT11_GPIO, &gpio);
}

static void dht11_pin_input(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = DHT11_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DHT11_GPIO, &gpio);
}

static void dht11_write_low(void)
{
    GPIO_ResetBits(DHT11_GPIO, DHT11_PIN);
}

static void dht11_write_high(void)
{
    GPIO_SetBits(DHT11_GPIO, DHT11_PIN);
}

static uint8_t dht11_read_pin(void)
{
    return (GPIO_ReadInputDataBit(DHT11_GPIO, DHT11_PIN) != Bit_RESET) ? 1u : 0u;
}

static void dht11_delay_us(uint32_t delay_us)
{
    uint32_t ticks_per_us = SystemCoreClock / 1000000u;
    uint32_t load;
    uint32_t start;
    uint32_t now;
    uint32_t elapsed;
    uint64_t remaining;

    if (delay_us == 0u || ticks_per_us == 0u) {
        return;
    }

    if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 0u) {
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

static uint32_t dht11_get_us(void)
{
    uint32_t ms1;
    uint32_t ms2;
    uint32_t val;
    uint32_t load;
    uint32_t ticks_per_us;

    ticks_per_us = SystemCoreClock / 1000000u;
    if (ticks_per_us == 0u) {
        return 0u;
    }

    ms1 = GetMilSecTick();
    val = SysTick->VAL;
    ms2 = GetMilSecTick();
    if (ms2 != ms1) {
        ms1 = ms2;
        val = SysTick->VAL;
    }

    load = SysTick->LOAD + 1u;
    return (ms1 * 1000u) + (uint32_t)((load - val) / ticks_per_us);
}

static bool dht11_wait_level(uint8_t level, uint32_t timeout_us, uint32_t *elapsed_us)
{
    uint32_t start = dht11_get_us();
    uint32_t now;

    while (dht11_read_pin() != level) {
        now = dht11_get_us();
        if ((now - start) > timeout_us) {
            return false;
        }
    }

    start = dht11_get_us();
    now = start;
    while (dht11_read_pin() == level) {
        now = dht11_get_us();
        if ((now - start) > timeout_us) {
            return false;
        }
    }

    if (elapsed_us != NULL) {
        *elapsed_us = (now - start);
    }
    return true;
}

/* END_FILE */
