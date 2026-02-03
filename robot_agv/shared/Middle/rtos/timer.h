/*******************************************************************************
 *
 * Copyright (c) 2025
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description: Include file for application
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 2.0.0 $
 * Last Changed:     $Date: 02/10/2025 $
 *
 ******************************************************************************/
#ifndef _TIMER_H_
#define _TIMER_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define TIMER_ID_MAX                32u
#define TIMER_ID_INVALID            255u

#define TIMER_REPEAT_ONE_TIME       1u
#define TIMER_REPEAT_FOREVER        255u

typedef uint8_t SSwTimer;
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @brief  Timer scheduler initialization
 * @param  None
 * @retval None
 */
void TimerInit(void);

/**
 * @brief  Start a timer
 * @param  name: name of timer
 * @param  wMilSecTick: timer tick
 * @param  byRepeats: number of repeater
 * @param  callback: callback function
 * @param  pcallbackData : parameters
 * @retval Index of timer
 */
uint8_t TimerStart(
    char* name,
    uint32_t dwMilSecTick,
	uint8_t byRepeats,
	void (*callback)(void *),
    void *pcallbackData
);

/**
 * @brief  Change period of timer
 * @param  byTimerId
 * @param  dwMilSecTick
 * @retval None
 */
void TimerChangePeriod(
	uint8_t byTimerId,
	uint32_t dwPeriodTicks
);
    
/**
 * @brief  Restart a timer
 * @param  byTimerId: ID of timer
 * @param  dwMilSecTick: timer tick
 * @param  byRepeats: number of repeater
 * @retval 1: success, 0: fail
 */
uint8_t TimerRestart(
	uint8_t byTimerId,
	uint32_t dwMilSecTick,
	uint8_t byRepeats
);
    
/**
 * @brief  Stop a timer
 * @param  byTimerId: ID of timer
 * @retval 1: success, 0: fail
 */
uint8_t TimerStop(
    uint8_t *byTimerId
);

/**
 * @brief  Get current tick in ms
 * @param  None
 * @retval Current tick in ms
 */
uint32_t GetMilSecTick(void);

/**
 * @brief  Process timer scheduler
 * @param  None
 * @retval None
 */
void processTimerScheduler(void);

#endif /* _TIMER_H_ */
