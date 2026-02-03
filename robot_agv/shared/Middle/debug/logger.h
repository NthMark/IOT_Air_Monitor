/*******************************************************************************
 *
 * Copyright (c) 2025
 * MSFT, JSC.
 * All Rights Reserved
 *
 *
 * Description: Include file for application
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 2.0.1 $
 * Last Changed:     $Date: 21/10/2025 $
 *
 ******************************************************************************/
#ifndef _LOGGER_H_
#define _LOGGER_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define UART_BAUDRATE_FMR     230400
#define UART_BAUDRATE_MCBUS   115200
#define TX_BUF_SIZE           2048

#define LOG_LEVEL_NONE        0
#define LOG_LEVEL_ERROR       1
#define LOG_LEVEL_INFO        2
#define LOG_LEVEL_VERBOSE     3

#ifndef LOG_LEVEL
#define LOG_LEVEL             LOG_LEVEL_INFO
#endif
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

void debug_init(void);
void debug_reconfig(uint32_t baudrate, uint16_t word_length, uint16_t stop_bits, uint16_t parity);
void run_test_logger_USART_DMA(void);
void debug_rx_poll(void);
void logger_cmd_handler(const char *line);

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
void debug_verbose(const char *fmt, ...);
#else
#define debug_verbose(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
void debug_info(const char *fmt, ...);
#else
#define debug_info(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_ERROR
void debug_error(const char *fmt, ...);
#else
#define debug_error(fmt, ...) ((void)0)
#endif

#endif

/* END FILE */
