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
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <string.h>
#include "buff.h"
#include "utilities.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
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
) {
    pQueue->wSize = numberOfElement;
    pQueue->byItemSize = sizeofElement;
    pQueue->pData = (uint8_t *)pBuffer;
    bufFlush(pQueue);
}

/**
 * @brief  Returns whether a ring buffer is full
 * @param  pQueue The buffer for which it should be returned whether it is full.
 * @return 1 if full; 0 otherwise
 */
uint8_t bufIsFull(
    buffqueue_p pQueue
) {
    uint16_t mask = pQueue->wSize - 1;
    uint16_t nextHead = (pQueue->wHeadIndex + pQueue->byItemSize) & mask;
    return (nextHead == pQueue->wTailIndex);
}

/**
 * @brief  Returns whether a ring buffer is empty
 * @param buffer The buffer for which it should be returned whether it is empty
 * @return 1 if empty; 0 otherwise
 */
uint8_t bufIsEmpty(
    buffqueue_p pQueue
) {
    return (pQueue->wHeadIndex == pQueue->wTailIndex);
}

/**
 * @brief  Flushes the FIFO
 * @param  pQueue: Pointer to the FIFO object
 * @retval None
 */
void bufFlush(
    buffqueue_p pQueue
) {
    pQueue->wHeadIndex = 0;
    pQueue->wTailIndex = 0;
    
    memset(pQueue->pData, 0, pQueue->wSize);
}

/**
 * @brief  Returns the number of items in a ring buffer
 * @param  pQueue: The buffer for which the number of items should be returned
 * @return The number of items in the ring buffer
 */
uint16_t bufNumItems(
	buffqueue_p pQueue
) {
    return (uint16_t)((pQueue->wHeadIndex - pQueue->wTailIndex) & (pQueue->wSize - 1));
}

/**
 * @brief  Pushes data to the FIFO
 * @param  pQueue: Pointer to the FIFO object
 * @param  pReceiverData: Received data to be pushed into the FIFO
 * @retval ERR_OK or ERR_BUF_FUL
 */
uint8_t bufEnDat(
    buffqueue_p pQueue,
	uint8_t *pReceiverData
) {    
    uint16_t mask = pQueue->wSize - 1;
    uint16_t nextHead = pQueue->wHeadIndex;

    /* Place data (byItemSize bytes) */
    for (uint8_t i = 0; i < pQueue->byItemSize; i++) {
        pQueue->pData[nextHead] = pReceiverData[i];
        nextHead = (nextHead + 1) & mask;
    }

    /* If writing will overwrite oldest byte(s), advance tail by one item */
    if (nextHead == pQueue->wTailIndex) {
        pQueue->wTailIndex = (pQueue->wTailIndex + pQueue->byItemSize) & mask;
    }

    pQueue->wHeadIndex = nextHead;
    return ERR_OK;
}

/**
 * @brief  Pops data from the FIFO
 * @param  pQueue: Pointer to the FIFO object
 * @param  pBuffer: Data in the FIFO popped into the buffer
 * @retval ERR_OK or ERR_BUF_EMPTY
 */
uint8_t bufDeDat(
    buffqueue_p pQueue,
	uint8_t *pBuffer
) {    
    if (bufIsEmpty(pQueue)) {
        return ERR_BUF_EMPTY;
    }

    for (uint8_t i = 0; i < pQueue->byItemSize; i++) {
        pBuffer[i] = pQueue->pData[pQueue->wTailIndex];
        pQueue->wTailIndex = ((pQueue->wTailIndex + 1) & (pQueue->wSize - 1));
    }

    return ERR_OK;
}

/* END FILE */
