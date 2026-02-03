/*******************************************************************************
 *
 * Copyright (c) 2025
 * Lumi, JSC.
 * All Rights Reserved
 *
 * Description: Include file for application
 *
 * Author: Dev team
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 2.0.1 $
 * Last Changed:     $Date: 21/10/2025 $
 *
*******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "misc.h"
#include "logger.h"
#include "timer.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/* Shared state between main and IRQ.
 * Marked volatile because values can change inside ISRs. */
static volatile uint8_t  tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;            // producer index
static volatile uint16_t tx_tail = 0;            // consumer index
static volatile bool     tx_dma_busy = false;    // true while a DMA transfer is running
static volatile uint16_t tx_current_len = 0;     // Length of the current DMA chunk

#define RX_BUF_SIZE 128u
static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void _logger_usart2_init(void);
static void _logger_dma1_ch7_init(void);
static void _logger_usart2_irq_init(void);

static inline uint16_t rb_contig_len_from_tail(void);
static void uart_kick_dma_if_idle(void);
uint16_t uart_write_bytes_dropnew(const uint8_t* data, uint16_t len);
static void rx_push(uint8_t byte);
static bool rx_pop(uint8_t *out);

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

void debug_init(void) {
    _logger_usart2_init();
    _logger_dma1_ch7_init();
    _logger_usart2_irq_init();

    debug_info("LOGGER_SS: Logger initialized at %d bps with TX_BUF_SIZE = %d", UART_BAUDRATE_MCBUS, TX_BUF_SIZE);
}

/* Return contiguous bytes from tail to head (no wrap).
 * This lets DMA send a single linear chunk. */
static inline uint16_t rb_contig_len_from_tail(void) {
    uint16_t head = tx_head, tail = tx_tail;
    if (head == tail) return 0; // empty
    if (head > tail)  return (uint16_t)(head - tail);
    return (uint16_t)(TX_BUF_SIZE - tail);  // send to end first
}

/* DROP_NEW policy: if the ring buffer is full, stop writing and drop the rest.
 * Non-blocking: never stalls the producer. */
uint16_t uart_write_bytes_dropnew(const uint8_t* data, uint16_t len) {
    uint16_t written = 0;

    for (uint16_t i = 0; i < len; i++) {
        uint16_t next_head = (uint16_t)((tx_head + 1) % TX_BUF_SIZE);
        if (next_head == tx_tail) {
            break; // Buffer is FULL → stop, drop remaining bytes
        }
        tx_buf[tx_head] = data[i];
        tx_head = next_head;
        written++;
    }

    /* Start a DMA transfer if none is running.
     * If DMA is busy, the ISR will chain the next chunk automatically. */
    uart_kick_dma_if_idle();
    return written;
}

/* Start a single DMA transfer if idle:
 * - Sends one contiguous chunk starting from tx_tail. */
static void uart_kick_dma_if_idle(void) {
    if (tx_dma_busy) return;

    uint16_t len = rb_contig_len_from_tail();
    if (len == 0) return;

    tx_current_len = len;
    tx_dma_busy    = true; // mark busy BEFORE enabling DMA to avoid races

    /* Always disable the channel before reconfiguring its registers. */
    DMA1_Channel7->CCR &= ~DMA_CCR7_EN;

    /* Program the DMA source/destination and length for this chunk. */
    DMA1_Channel7->CPAR  = (uint32_t)&USART2->DR;      // peripheral: USART2 data register
    DMA1_Channel7->CMAR  = (uint32_t)&tx_buf[tx_tail]; // memory: start at current tail
    DMA1_Channel7->CNDTR = tx_current_len;             // number of bytes to send
    
    /* Clear any stale pending flags for channel 7 (good hygiene). */
    DMA_ClearITPendingBit(DMA1_IT_GL7);

    /* Enable the channel to start the transfer. */
    DMA1_Channel7->CCR |= DMA_CCR7_EN;
}

/* DMA1 Channel 7 (USART2_TX) interrupt:
 * - On Transfer Complete (TC), advance tail by the chunk length.
 * - Mark not busy and try to send the next chunk if available. */
void DMA1_Channel7_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TE7) != RESET) {
        DMA_ClearITPendingBit(DMA1_IT_TE7);
        tx_current_len = 0;
        tx_dma_busy    = false;
        return;
    }

    if (DMA_GetITStatus(DMA1_IT_TC7) != RESET) {
        DMA_ClearITPendingBit(DMA1_IT_TC7);

        tx_tail        = (uint16_t)((tx_tail + tx_current_len) % TX_BUF_SIZE);
        tx_current_len = 0;
        tx_dma_busy    = false;

        /* If more data arrived while this chunk was sending, start next one. */
        uart_kick_dma_if_idle();
    }
}

void debug_reconfig(uint32_t baudrate, uint16_t word_length, uint16_t stop_bits, uint16_t parity)
{
    USART_InitTypeDef USART_InitStructure;

    /* 1. Stop TX DMA */
    DMA_Cmd(DMA1_Channel7, DISABLE);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {}

    tx_dma_busy    = false;
    tx_current_len = 0;
    DMA_ClearITPendingBit(DMA1_IT_GL7);

    /* 2. Disable USART2 */
    USART_Cmd(USART2, DISABLE);

    /* 3. Re-config USART2 */
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = word_length;
    USART_InitStructure.USART_StopBits            = stop_bits;
    USART_InitStructure.USART_Parity              = parity;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &USART_InitStructure);

    /* 4. TX DMA request luôn bật cho logger */
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

    /* 5. Xử lý RX DMA theo mode */
    if (baudrate == UART_BAUDRATE_MCBUS) {
        // Mode MCBUS: cần RX DMA
        USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

        DMA_SetCurrDataCounter(DMA1_Channel6, 64u);
        DMA_Cmd(DMA1_Channel6, ENABLE);
    } else {
        // Mode FMR / DEBUG: không dùng RX DMA
        USART_DMACmd(USART2, USART_DMAReq_Rx, DISABLE);
        DMA_Cmd(DMA1_Channel6, DISABLE);
    }

    /* 6. Clear SR/DR cho chắc */
    volatile uint16_t tmp;
    tmp = USART2->SR;
    tmp = USART2->DR;
    (void)tmp;

    /* 7. Enable USART2 */
    USART_Cmd(USART2, ENABLE);

    debug_info("LOG_SS: Logger re-configured at %d bps, WL=%d, SB=%d, P=%d", baudrate, word_length, stop_bits, parity);
}

static void _logger_usart2_init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   USART_InitStructure;

    // Enable clocks for GPIOA and USART2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // PA2: USART2_TX = AF_PP
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA3: USART2_RX = input
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART2 configuration
    USART_InitStructure.USART_BaudRate            = UART_BAUDRATE_MCBUS;
    USART_InitStructure.USART_WordLength          = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_2;
    USART_InitStructure.USART_Parity              = USART_Parity_Even;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStructure);

    // Enable USART2
    USART_Cmd(USART2, ENABLE);
}

static void _logger_usart2_irq_init(void)
{
    NVIC_InitTypeDef nvic;

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

static void rx_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((rx_head + 1u) % RX_BUF_SIZE);
    if (next == rx_tail) {
        return; // drop if full
    }
    rx_buf[rx_head] = byte;
    rx_head = next;
}

static bool rx_pop(uint8_t *out)
{
    if (out == NULL) {
        return false;
    }
    if (rx_head == rx_tail) {
        return false;
    }
    *out = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) % RX_BUF_SIZE);
    return true;
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t b = (uint8_t)USART_ReceiveData(USART2);
        rx_push(b);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

void debug_rx_poll(void)
{
    static char line[64];
    static uint16_t len = 0;
    uint8_t b;

    while (rx_pop(&b)) {
        // debug_info("HC05_RX: 0x%02X '%c'", b, (b >= 32u && b < 127u) ? (char)b : '.');
        if ((b == '\n') || (b == '\r')) {
            if (len > 0u) {
                line[len] = '\0';
                debug_info("HC05_LINE: %s", line);
                logger_cmd_handler(line);
                len = 0u;
            }
            continue;
        }
        if (len < (sizeof(line) - 1u)) {
            line[len++] = (char)b;
        } else {
            len = 0u;
        }
    }
}

__attribute__((weak)) void logger_cmd_handler(const char *line)
{
    (void)line;
}
// Table 78. Summary of DMA1 requests for each channel (STM32F103xx Reference Manual RM0008)
static void _logger_dma1_ch7_init(void) {
    DMA_InitTypeDef  dma;
    NVIC_InitTypeDef nvic;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // DMA1 clock enable
    DMA_DeInit(DMA1_Channel7);

    /* Base DMA settings for USART2 TX.
     * We'll override CMAR/CNDTR on each kick to point to the current chunk. */
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;   // USART2 data register
    dma.DMA_MemoryBaseAddr     = (uint32_t)tx_buf;        // placeholder; overridden per transfer
    dma.DMA_DIR                = DMA_DIR_PeripheralDST;   // Memory -> Peripheral
    dma.DMA_BufferSize         = 0;                       // overridden via CNDTR
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;    // walk through the buffer
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Normal;         // one-shot per chunk
    dma.DMA_Priority           = DMA_Priority_Low;
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7, &dma);

    /* Enable Transfer Complete interrupt for this channel. */
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* NVIC setup for DMA1 Channel 7 interrupt. */
    nvic.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* Clear any pending flags before first use. */
    DMA_ClearITPendingBit(DMA1_IT_GL7);

    /* Enable TX DMA requests from USART2 (sets CR3.DMAT) */
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}

int fputc(int ch, FILE *f) {
    (void)f;
    uint8_t c = (uint8_t)ch;
    (void)uart_write_bytes_dropnew(&c, 1);
    return ch;
}

void run_test_logger_USART_DMA(void) {
    debug_info("LOG_SS: UART2-DMA logger (DROP_NEW) 115200-8N1");

    for (uint32_t i = 0;; i++) {
        debug_info("Tick %lu", (unsigned long)i);
        for (volatile uint32_t d = 0; d < 720000; d++) __NOP(); // ~10ms @72MHz
    }
}

static void _debug_print(const char *level, const char *fmt, va_list args)
{
    uint32_t t = GetMilSecTick();
    printf("(%lu) [%s] ", (unsigned long)t, level);
    vprintf(fmt, args);
    printf("\r\n");
}

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
void debug_verbose(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    _debug_print("VERBOSE", fmt, args);
    va_end(args);
}
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
void debug_info(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    _debug_print("INFO", fmt, args);
    va_end(args);
}
#endif

#if LOG_LEVEL >= LOG_LEVEL_ERROR
void debug_error(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    _debug_print("ERROR", fmt, args);
    va_end(args);
}
#endif

/* END FILE */
