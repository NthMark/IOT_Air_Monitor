/******************************************************************************
 *
 * Copyright (c) 2025
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description:
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: hoangnh $
 * Revision:         $Revision: 2.0.0 $
 * Last Changed:     $Date: 10/12/2025 $
 *
 ******************************************************************************/
#ifndef _BUFF_H_
#define _BUFF_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define ERR_OK                         0x00 
#define ERR_BUF_FULL                   0x01
#define ERR_BUF_EMPTY                  0x02     
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
typedef struct __buff_queue__ {
    volatile uint16_t wHeadIndex;
    volatile uint16_t wTailIndex;
    uint16_t          wSize;
    uint8_t           byItemSize;
    uint8_t           _padding;
    uint8_t          *pData;
} buffqueue_t, *buffqueue_p;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
/**
 * @brief  Initializes the FIFO structure
 * @param  pBuffer: Data to be pushed into the FIFO
 * @param  pQueue: Pointer to the FIFO object
 * @param  sizeofElement: Size of a element in the buffer
 * @param  numberOfElement: Size of the buffer
 * @retval None
 */
void bufInit(
    void *pBuffer,
    buffqueue_p pQueue,
    uint8_t sizeofElement,
    uint16_t numberOfElement
);

/**
 * @brief  Determine number of uint8_ts in FIFO has not been processed
 * @param  pQueue: Pointer to the FIFO object
 * @retval Number of uint8_ts in FIFO
 */
uint16_t bufNumItems(
    buffqueue_p pQueue
);

/**
 * @brief  Checks if the FIFO is full
 * @param  pQueue: Pointer to the FIFO object
 * @retval TRUE of FALSE
 */
uint8_t bufIsFull(
    buffqueue_p pQueue
);

/**
 * @brief  Checks if the FIFO is empty
 * @param  pQueue: Pointer to the FIFO object
 * @retval TRUE of FALSE
 */
uint8_t bufIsEmpty(
    buffqueue_p pQueue
);

/**
 * @brief  Flushes the FIFO
 * @param  pQueue: Pointer to the FIFO object
 * @retval None
 */
void bufFlush(
    buffqueue_p pQueue
);

/**
 * @brief  Pushes data to the FIFO
 * @param  pQueue: Pointer to the FIFO object
 * @param  pReceiverData: Received data to be pushed into the FIFO
 * @retval ERR_OK or ERR_BUF_FUL
 */
uint8_t bufEnDat(
    buffqueue_p pQueue,
    uint8_t *pReceiverData
);

/**
 * @brief  Pops data from the FIFO
 * @param  pQueue: Pointer to the FIFO object
 * @param  pBuffer: Data in the FIFO popped into the buffer
 * @retval ERR_OK or ERR_BUF_EMPTY
 */
uint8_t bufDeDat(
    buffqueue_p pQueue,
    uint8_t *pBuffer
);

#endif /* END FILE */
