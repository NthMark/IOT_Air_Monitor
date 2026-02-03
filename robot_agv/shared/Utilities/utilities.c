/*******************************************************************************
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
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 2.0.0 $
 * Last Changed:     $Date: 10/10/2025 $
 *
 ******************************************************************************/
 
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "utilities.h"
#include "timer.h"
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

uint16_t crc16_ccitt(const uint8_t* data, uint32_t len)
{
    /* CCITT-FALSE: poly 0x1021, init 0xFFFF, no xorout */
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc = (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief  Check if a timeout (in ms) has occurred since the last timestamp update.
 * @param  pTimeStamp: Pointer to the timestamp variable (in ms)
 * @param  timeout: Timeout duration (in ms)
 * @return true if timeout has occurred, false otherwise
 */
bool util_is_timeout_ms(uint32_t *pTimeStamp, uint32_t timeout)
{
    if (pTimeStamp == NULL) return false;

    uint32_t time_current = GetMilSecTick();

    if ((uint32_t)(time_current - *pTimeStamp) >= timeout) {
        *pTimeStamp = time_current;
        return true;
    }
    return false;
}

/**
 * @brief   Clamp a floating-point value within a given range.
 *
 * This function ensures that the input value `v` always lies between
 * the lower bound `lo` and the upper bound `hi`.
 * - If `v` is smaller than `lo`, it returns `lo`.
 * - If `v` is greater than `hi`, it returns `hi`.
 * - Otherwise, it returns `v` unchanged.
 *
 * @param   v   Input value to be clamped
 * @param   lo  Lower bound of the range
 * @param   hi  Upper bound of the range
 *
 * @return  Clamped value within [lo, hi]
 */
float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* END FILE */
