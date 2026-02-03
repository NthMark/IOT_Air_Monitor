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
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stddef.h>
#include "sensor_mhz19.h"
#include "logger.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "timer.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define MHZ19_USART                 USART1
#define MHZ19_BAUDRATE              9600u
#define MHZ19_RESPONSE_DELAY_MS     50u
#define MHZ19_TIMEOUT_MS            1000u

#define MHZ19_GPIO                  GPIOA
#define MHZ19_TX_PIN                GPIO_Pin_9
#define MHZ19_RX_PIN                GPIO_Pin_10

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void mhz19_uart_flush_rx(void);
static void mhz19_delay_ms(uint32_t delay_ms);
static void mhz19_uart_write(const uint8_t *data, uint16_t len);
static bool mhz19_uart_read_byte(uint8_t *out, uint32_t timeout_ms);
static bool mhz19_uart_read_frame(uint8_t *frame, uint16_t len, uint32_t timeout_ms, uint16_t *out_bytes);
static uint8_t mhz19_checksum(const uint8_t *frame);
static void mhz19_log_frame(const uint8_t *frame, const char *tag);

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void mhz19_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    gpio.GPIO_Pin   = MHZ19_TX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MHZ19_GPIO, &gpio);
    GPIO_SetBits(MHZ19_GPIO, MHZ19_TX_PIN);

    gpio.GPIO_Pin   = MHZ19_RX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MHZ19_GPIO, &gpio);
    GPIO_SetBits(MHZ19_GPIO, MHZ19_RX_PIN);

    USART_StructInit(&usart);
    usart.USART_BaudRate            = MHZ19_BAUDRATE;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(MHZ19_USART, &usart);

    USART_Cmd(MHZ19_USART, ENABLE);
}

bool mhz19_read(mhz19_reading_t *out)
{
    uint8_t cmd[9] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t resp[9];
    uint16_t bytes = 0u;

    if (out == NULL) {
        return false;
    }

    cmd[8] = mhz19_checksum(cmd);

    mhz19_uart_flush_rx();
    mhz19_uart_write(cmd, (uint16_t)sizeof(cmd));
    mhz19_delay_ms(MHZ19_RESPONSE_DELAY_MS);

    if (!mhz19_uart_read_frame(resp, (uint16_t)sizeof(resp), MHZ19_TIMEOUT_MS, &bytes)) {
        debug_error("MHZ19_SS: RX frame timeout (bytes=%u)", (unsigned)bytes);
        return false;
    }

    if (resp[0] != 0xFF || resp[1] != 0x86) {
        debug_error("MHZ19_SS: Bad header 0x%02X 0x%02X", resp[0], resp[1]);
        mhz19_log_frame(resp, "bad_header");
        return false;
    }

    if (resp[8] != mhz19_checksum(resp)) {
        debug_error("MHZ19_SS: Bad checksum");
        mhz19_log_frame(resp, "bad_checksum");
        return false;
    }

    out->co2_ppm = (uint16_t)((resp[2] << 8) | resp[3]);
    out->temperature_c = (int8_t)resp[4] - 40;
    out->status = resp[5];
    return true;
}

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void mhz19_uart_flush_rx(void)
{
    while (USART_GetFlagStatus(MHZ19_USART, USART_FLAG_RXNE) != RESET) {
        (void)USART_ReceiveData(MHZ19_USART);
    }
}

static void mhz19_delay_ms(uint32_t delay_ms)
{
    uint32_t start = GetMilSecTick();
    while ((GetMilSecTick() - start) < delay_ms) {}
}

static void mhz19_uart_write(const uint8_t *data, uint16_t len)
{
    if (data == NULL) {
        return;
    }

    for (uint16_t i = 0; i < len; i++) {
        while (USART_GetFlagStatus(MHZ19_USART, USART_FLAG_TXE) == RESET) {}
        USART_SendData(MHZ19_USART, data[i]);
    }

    while (USART_GetFlagStatus(MHZ19_USART, USART_FLAG_TC) == RESET) {}
}

static bool mhz19_uart_read_byte(uint8_t *out, uint32_t timeout_ms)
{
    uint32_t start = GetMilSecTick();

    if (out == NULL) {
        return false;
    }

    while (USART_GetFlagStatus(MHZ19_USART, USART_FLAG_RXNE) == RESET) {
        if ((GetMilSecTick() - start) >= timeout_ms) {
            return false;
        }
    }

    *out = (uint8_t)USART_ReceiveData(MHZ19_USART);
    return true;
}

static bool mhz19_uart_read_frame(uint8_t *frame, uint16_t len, uint32_t timeout_ms, uint16_t *out_bytes)
{
    uint8_t b = 0;
    uint32_t start = GetMilSecTick();
    uint8_t state = 0u;
    uint16_t idx = 0;
    uint16_t bytes = 0u;
    uint8_t history[8] = { 0 };
    uint8_t hist_len = 0u;

    if (frame == NULL || len < 2u) {
        debug_error("MHZ19_SS: Invalid frame buffer");
        return false;
    }

    while ((GetMilSecTick() - start) < timeout_ms) {
        uint32_t elapsed = GetMilSecTick() - start;
        uint32_t remaining = (elapsed >= timeout_ms) ? 0u : (timeout_ms - elapsed);
        if (remaining == 0u) {
            break;
        }
        if (!mhz19_uart_read_byte(&b, remaining)) {
            continue;
        }
        bytes++;
        if (hist_len < (uint8_t)sizeof(history)) {
            history[hist_len++] = b;
        }

        if (state == 0u) {
            if (b == 0xFF) {
                frame[0] = b;
                state = 1u;
            }
            continue;
        }

        if (state == 1u) {
            if (b == 0x86) {
                frame[1] = b;
                idx = 2u;
                state = 2u;
            } else if (b == 0xFF) {
                frame[0] = b;
                state = 1u;
            } else {
                state = 0u;
            }
            continue;
        }

        frame[idx++] = b;
        if (idx >= len) {
            if (out_bytes != NULL) {
                *out_bytes = bytes;
            }
            return true;
        }
    }

    if (out_bytes != NULL) {
        *out_bytes = bytes;
    }
    if (hist_len > 0u) {
        debug_error("MHZ19_SS: RX bytes(first %u)=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                    (unsigned)hist_len,
                    history[0], history[1], history[2], history[3],
                    history[4], history[5], history[6], history[7],history[8],
                    history[9], history[10], history[11], history[12],
                    history[13], history[14], history[15]);
    }
    return false;
}

static uint8_t mhz19_checksum(const uint8_t *frame)
{
    uint8_t sum = 0;
    if (frame == NULL) {
        return 0;
    }

    for (uint8_t i = 1; i < 8; i++) {
        sum = (uint8_t)(sum + frame[i]);
    }
    return (uint8_t)(0xFFu - sum + 1u);
}

static void mhz19_log_frame(const uint8_t *frame, const char *tag)
{
    if (frame == NULL) {
        return;
    }

    debug_error("MHZ19_SS: frame(%s)=%02X %02X %02X %02X %02X %02X %02X %02X %02X",
                (tag != NULL) ? tag : "?",
                frame[0], frame[1], frame[2], frame[3], frame[4],
                frame[5], frame[6], frame[7], frame[8]);
}

/* END_FILE */
