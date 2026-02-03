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
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stddef.h>
#include "sensor_sc8.h"
#include "logger.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "timer.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define SC8_USART                  USART1
#define SC8_BAUDRATE               9600u
#define SC8_FRAME_LEN              16u
#define SC8_RESPONSE_DELAY_MS      0u
#define SC8_TIMEOUT_MS             1000u

#define SC8_HEADER0                0x42u
#define SC8_HEADER1                0x4Du

#define SC8_GPIO                   GPIOA
#define SC8_TX_PIN                 GPIO_Pin_9
#define SC8_RX_PIN                 GPIO_Pin_10

#define SC8_PWM_TIM                TIM3
#define SC8_PWM_TIM_RCC            RCC_APB1Periph_TIM3
#define SC8_PWM_TIM_IRQn           TIM3_IRQn
#define SC8_PWM_TIM_IRQHandler     TIM3_IRQHandler
#define SC8_PWM_TIM_CHANNEL        TIM_Channel_2
#define SC8_PWM_TIM_TRIGGER        TIM_TS_TI2FP2
#define SC8_PWM_TIM_IT             TIM_IT_CC2
#define SC8_PWM_CAP_PERIOD()       TIM_GetCapture2(SC8_PWM_TIM)
#define SC8_PWM_CAP_HIGH()         TIM_GetCapture1(SC8_PWM_TIM)

#define SC8_PWM_GPIO               GPIOA
#define SC8_PWM_GPIO_RCC           RCC_APB2Periph_GPIOA
#define SC8_PWM_PIN                GPIO_Pin_7

#define SC8_PWM_COUNTER_HZ         10000u
#define SC8_PWM_RANGE_PPM          5000u
#define SC8_PWM_OFFSET_US          2000u
#define SC8_PWM_US_PER_MS          1000u
#define SC8_PWM_TIMEOUT_MS         2000u

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void sc8_uart_init(void);
static bool sc8_uart_read(sc8_reading_t *out);
static void sc8_pwm_init(void);
static bool sc8_pwm_read(sc8_reading_t *out);
static uint32_t sc8_pwm_get_tim_clk_hz(void);
static uint32_t sc8_pwm_ticks_to_us(uint16_t ticks);
static void sc8_uart_flush_rx(void);
static void sc8_delay_ms(uint32_t delay_ms);
static bool sc8_uart_read_byte(uint8_t *out, uint32_t timeout_ms);
static bool sc8_uart_read_frame(uint8_t *frame, uint16_t len, uint32_t timeout_ms);
static uint8_t sc8_checksum(const uint8_t *frame, uint16_t len);
static void sc8_log_frame(const uint8_t *frame, const char *tag);

static sc8_interface_t s_sc8_iface = SC8_IF_UART;
static volatile uint16_t s_sc8_pwm_period_ticks = 0u;
static volatile uint16_t s_sc8_pwm_high_ticks = 0u;
static volatile uint32_t s_sc8_pwm_last_update_ms = 0u;
static uint16_t s_sc8_pwm_last_ppm = 0u;
static bool s_sc8_pwm_has_last = false;

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void sc8_init(sc8_interface_t iface)
{
    s_sc8_iface = iface;
    if (iface == SC8_IF_PWM) {
        sc8_pwm_init();
    } else {
        sc8_uart_init();
    }
}

static void sc8_uart_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    gpio.GPIO_Pin   = SC8_TX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SC8_GPIO, &gpio);
    GPIO_SetBits(SC8_GPIO, SC8_TX_PIN);

    gpio.GPIO_Pin   = SC8_RX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SC8_GPIO, &gpio);
    GPIO_SetBits(SC8_GPIO, SC8_RX_PIN);

    USART_StructInit(&usart);
    usart.USART_BaudRate            = SC8_BAUDRATE;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(SC8_USART, &usart);

    USART_Cmd(SC8_USART, ENABLE);
}

bool sc8_read(sc8_reading_t *out)
{
    if (s_sc8_iface == SC8_IF_PWM) {
        return sc8_pwm_read(out);
    }

    return sc8_uart_read(out);
}

static bool sc8_uart_read(sc8_reading_t *out)
{
    uint8_t frame[SC8_FRAME_LEN];

    if (out == NULL) {
        return false;
    }

    sc8_uart_flush_rx();
    sc8_delay_ms(SC8_RESPONSE_DELAY_MS);

    if (!sc8_uart_read_frame(frame, SC8_FRAME_LEN, SC8_TIMEOUT_MS)) {
        debug_error("SC8_SS: RX frame timeout");
        return false;
    }

    if (frame[0] != SC8_HEADER0 || frame[1] != SC8_HEADER1) {
        debug_error("SC8_SS: Bad header 0x%02X 0x%02X", frame[0], frame[1]);
        sc8_log_frame(frame, "bad_header");
        return false;
    }

    if (frame[SC8_FRAME_LEN - 1] != sc8_checksum(frame, SC8_FRAME_LEN)) {
        debug_error("SC8_SS: Bad checksum");
        sc8_log_frame(frame, "bad_checksum");
        return false;
    }

    {
        uint16_t co2_b45 = (uint16_t)((frame[4] << 8) | frame[5]);
        uint16_t co2_b67 = (uint16_t)((frame[6] << 8) | frame[7]);
        uint16_t co2_b89 = (uint16_t)((frame[8] << 8) | frame[9]);

        debug_info("SC8_SS: co2_b45=%u ppm, co2_b67=%u ppm, co2_b89=%u ppm",
                   co2_b45, co2_b67, co2_b89);
        out->co2_ppm = co2_b89;
    }
    sc8_log_frame(frame, "ok");
    return true;
}

static void sc8_pwm_init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_ICInitTypeDef ic;
    NVIC_InitTypeDef nvic;
    uint32_t tim_clk_hz = sc8_pwm_get_tim_clk_hz();

    RCC_APB1PeriphClockCmd(SC8_PWM_TIM_RCC, ENABLE);
    RCC_APB2PeriphClockCmd(SC8_PWM_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);

    gpio.GPIO_Pin = SC8_PWM_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SC8_PWM_GPIO, &gpio);
    GPIO_SetBits(SC8_PWM_GPIO, SC8_PWM_PIN);

    if (tim_clk_hz == 0u || SC8_PWM_COUNTER_HZ == 0u) {
        return;
    }

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = (uint16_t)((tim_clk_hz / SC8_PWM_COUNTER_HZ) - 1u);
    tim.TIM_Period = 0xFFFFu;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(SC8_PWM_TIM, &tim);

    TIM_ICStructInit(&ic);
    ic.TIM_Channel = SC8_PWM_TIM_CHANNEL;
    ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter = 0x0;
    TIM_PWMIConfig(SC8_PWM_TIM, &ic);

    TIM_SelectInputTrigger(SC8_PWM_TIM, SC8_PWM_TIM_TRIGGER);
    TIM_SelectSlaveMode(SC8_PWM_TIM, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(SC8_PWM_TIM, TIM_MasterSlaveMode_Enable);

    nvic.NVIC_IRQChannel = SC8_PWM_TIM_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_ITConfig(SC8_PWM_TIM, SC8_PWM_TIM_IT, ENABLE);
    TIM_Cmd(SC8_PWM_TIM, ENABLE);

    s_sc8_pwm_period_ticks = 0u;
    s_sc8_pwm_high_ticks = 0u;
    s_sc8_pwm_last_update_ms = 0u;
}

static bool sc8_pwm_read(sc8_reading_t *out)
{
    uint32_t now = GetMilSecTick();
    uint16_t period_ticks = s_sc8_pwm_period_ticks;
    uint16_t high_ticks = s_sc8_pwm_high_ticks;
    uint32_t period_us;
    uint32_t high_us;
    uint64_t ppm;

    if (out == NULL) {
        return false;
    }

    if ((now - s_sc8_pwm_last_update_ms) > SC8_PWM_TIMEOUT_MS) {
        debug_error("SC8_SS: PWM timeout");
        return false;
    }

    if (period_ticks == 0u) {
        debug_error("SC8_SS: PWM period=0");
        return false;
    }

    period_us = sc8_pwm_ticks_to_us(period_ticks);
    high_us = sc8_pwm_ticks_to_us(high_ticks);
    if (high_us == 0u) {
        debug_error("SC8_SS: PWM invalid capture (period_ticks=%u, high_ticks=%u)",
                    (unsigned)period_ticks, (unsigned)high_ticks);
        return false;
    }

    if (high_us <= SC8_PWM_OFFSET_US) {
        debug_error("SC8_SS: PWM high too short (high_us=%lu, period_us=%lu)",
                    (unsigned long)high_us, (unsigned long)period_us);
        if (s_sc8_pwm_has_last) {
            out->co2_ppm = s_sc8_pwm_last_ppm;
            return true;
        }
        return false;
    }

    ppm = ((uint64_t)(high_us - SC8_PWM_OFFSET_US) * (uint64_t)SC8_PWM_RANGE_PPM) /
          (uint64_t)(1000u * SC8_PWM_US_PER_MS);
    if (ppm > SC8_PWM_RANGE_PPM) {
        ppm = SC8_PWM_RANGE_PPM;
    }
    out->co2_ppm = (uint16_t)ppm;
    s_sc8_pwm_last_ppm = out->co2_ppm;
    s_sc8_pwm_has_last = true;

    debug_info("SC8_SS: pwm_period_us=%lu, pwm_high_us=%lu",
               (unsigned long)period_us, (unsigned long)high_us);
    return true;
}

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void sc8_uart_flush_rx(void)
{
    while (USART_GetFlagStatus(SC8_USART, USART_FLAG_RXNE) != RESET) {
        (void)USART_ReceiveData(SC8_USART);
    }
}

static void sc8_delay_ms(uint32_t delay_ms)
{
    uint32_t start = GetMilSecTick();
    while ((GetMilSecTick() - start) < delay_ms) {}
}

static uint32_t sc8_pwm_get_tim_clk_hz(void)
{
    RCC_ClocksTypeDef clocks;
    uint32_t pclk1;

    RCC_GetClocksFreq(&clocks);
    pclk1 = clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        return pclk1 * 2u;
    }

    return pclk1;
}

static uint32_t sc8_pwm_ticks_to_us(uint16_t ticks)
{
    if (SC8_PWM_COUNTER_HZ == 0u) {
        return 0u;
    }

    return (uint32_t)(((uint64_t)ticks * 1000000u) / SC8_PWM_COUNTER_HZ);
}

void SC8_PWM_TIM_IRQHandler(void)
{
    if (TIM_GetITStatus(SC8_PWM_TIM, SC8_PWM_TIM_IT) != RESET) {
        s_sc8_pwm_period_ticks = SC8_PWM_CAP_PERIOD();
        s_sc8_pwm_high_ticks = SC8_PWM_CAP_HIGH();
        s_sc8_pwm_last_update_ms = GetMilSecTick();
        TIM_ClearITPendingBit(SC8_PWM_TIM, SC8_PWM_TIM_IT);
    }
}

static bool sc8_uart_read_byte(uint8_t *out, uint32_t timeout_ms)
{
    uint32_t start = GetMilSecTick();

    if (out == NULL) {
        return false;
    }

    while (USART_GetFlagStatus(SC8_USART, USART_FLAG_RXNE) == RESET) {
        if ((GetMilSecTick() - start) >= timeout_ms) {
            return false;
        }
    }

    *out = (uint8_t)USART_ReceiveData(SC8_USART);
    return true;
}

static bool sc8_uart_read_frame(uint8_t *frame, uint16_t len, uint32_t timeout_ms)
{
    uint8_t b = 0;
    uint32_t start = GetMilSecTick();
    uint8_t state = 0u;
    uint16_t idx = 0;

    if (frame == NULL || len < SC8_FRAME_LEN) {
        debug_error("SC8_SS: Invalid frame buffer");
        return false;
    }

    while ((GetMilSecTick() - start) < timeout_ms) {
        uint32_t elapsed = GetMilSecTick() - start;
        uint32_t remaining = (elapsed >= timeout_ms) ? 0u : (timeout_ms - elapsed);
        if (remaining == 0u) {
            break;
        }
        if (!sc8_uart_read_byte(&b, remaining)) {
            continue;
        }

        if (state == 0u) {
            if (b == SC8_HEADER0) {
                frame[0] = b;
                state = 1u;
            }
            continue;
        }

        if (state == 1u) {
            if (b == SC8_HEADER1) {
                frame[1] = b;
                idx = 2u;
                state = 2u;
            } else if (b == SC8_HEADER0) {
                frame[0] = b;
                state = 1u;
            } else {
                state = 0u;
            }
            continue;
        }

        frame[idx++] = b;
        if (idx >= len) {
            return true;
        }
    }

    return false;
}

static uint8_t sc8_checksum(const uint8_t *frame, uint16_t len)
{
    uint16_t sum = 0;

    if (frame == NULL || len < 2u) {
        return 0u;
    }

    for (uint16_t i = 0; i < (len - 1u); i++) {
        sum = (uint16_t)(sum + frame[i]);
    }
    return (uint8_t)(sum & 0xFFu);
}

static void sc8_log_frame(const uint8_t *frame, const char *tag)
{
    if (frame == NULL) {
        return;
    }

    debug_error("SC8_SS: frame(%s)=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                (tag != NULL) ? tag : "?",
                frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frame[7],
                frame[8], frame[9], frame[10], frame[11], frame[12], frame[13], frame[14], frame[15]);
}

/* END_FILE */
