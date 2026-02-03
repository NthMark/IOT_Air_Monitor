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
 
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <string.h>
#include <stdio.h>
#include "timer.h"
#include "logger.h"
#include "utilities.h"
#include "stm32f10x.h"
#include "system_stm32f10x.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
typedef struct _TIMER_ {
    uint8_t id;
    uint8_t gen;
    char *name;
    uint32_t milSecStart;
    uint32_t milSecTimeout;
    uint8_t repeats;
    void (*callbackFunc)(void *);
    void *pCallbackData;
} TIMER_t;

#define TIMER_SS_DEBUG                  0
#define TIMER_SYSTICK_FREQ_HZ           1000 // 1ms
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
static TIMER_t g_pTimerHandle[TIMER_ID_MAX] = { 0 };
static volatile uint32_t g_wMilSecTickTimer = 0;

#if TIMER_SS_DEBUG
typedef struct {
    uint32_t last_ms;           // thời gian callback lần gần nhất (ms)
    uint32_t max_ms;            // lớn nhất từng đo (ms)
    uint64_t sum_ms;            // tích lũy ms (để tính avg)
    uint32_t call_count;        // số lần callback đã chạy
    uint32_t last_stack_free;   // byte stack còn trống ngay sau callback
    uint32_t min_stack_free;    // ít nhất từng thấy (high-water)
} TIMER_STATS_t;

static TIMER_STATS_t g_timerStats[TIMER_ID_MAX];
// Tổng “ms bận” cộng theo từng callback trong 1 tick (thô, <1ms => 0)
static volatile uint32_t g_busyMsAcc      = 0;
static volatile uint32_t g_busyMsSnapshot = 0;
// Linker symbols (STM32F1 ld script thường có tên này)
extern uint32_t __StackTop;    // địa chỉ cao
extern uint32_t __StackLimit;  // địa chỉ thấp

// Đọc MSP hiện tại (stack pointer)
static inline uint32_t GetMSP(void) {
    register uint32_t msp __asm("msp");
    return msp;
}

// (Tuỳ chọn) Điền mẫu stack để quét high-water chính xác
static inline void PrefillStack(void) {
    // CHỈ dùng sớm trong init, trước khi nhiều ISR chạy.
    uint8_t *p = (uint8_t *)&__StackLimit;
    while (p < (uint8_t *)&__StackTop) { *p++ = 0xA5; }
}
#endif /* TIMER_SS_DEBUG */
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
void TimerStructResetToDefault(TIMER_t *s);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @brief  Timer scheduler initialization
 * @param  None
 * @retval None
 */
void TimerInit(void) {
    SystemCoreClockUpdate();
    NVIC_SetPriority(SysTick_IRQn, 1);

    for (uint8_t i = 0; i < TIMER_ID_MAX; i++) {
        TimerStructResetToDefault(&g_pTimerHandle[i]);
    }

    #if TIMER_SS_DEBUG
    memset(g_timerStats, 0, sizeof(g_timerStats));
    // PrefillStack(); // bật nếu muốn quét high-water theo pattern 0xA5
    #endif

    if (SysTick_Config(SystemCoreClock / TIMER_SYSTICK_FREQ_HZ) != 0) {
        debug_error("TIM_SS: SysTick configuration error");
        while (1);
    }

    debug_info("TIM_SS: Timer SS initialized with freq = %u Hz with system clock freq = %ld Hz\n",
        TIMER_SYSTICK_FREQ_HZ, SystemCoreClock);
}

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
	uint32_t wMilSecTick,
	uint8_t byRepeats,
	void (*callback)(void *),
    void *pcallbackData
) {
    if (callback == NULL) return TIMER_ID_INVALID;

    for (uint8_t i = 0; i < TIMER_ID_MAX; i++)
    {
        TIMER_t *s = &g_pTimerHandle[i];
        if (s->callbackFunc == NULL)
        {
            s->id = i;
            s->gen += 1u;
            s->name = name;
            s->repeats = byRepeats;
            s->callbackFunc = callback;
            s->pCallbackData = pcallbackData;
            s->milSecStart = GetMilSecTick();
            s->milSecTimeout = wMilSecTick;
            
            #if TIMER_SS_DEBUG
            debug_info("TIM_SS: TimerStart = %s, id = %d, gen = %d, timestamp = %ld, repeat = %d, timeout = %ld\n",
                s->name, i, s->gen, s->milSecStart, s->repeats, s->milSecTimeout);
            #endif
            
            return i;
        }
    }
    
    return TIMER_ID_INVALID;
}

/**
 * @brief  Change period of timer
 * @param  byTimerId
 * @param  dwMilSecTick
 * @retval None
 */
void TimerChangePeriod(
    uint8_t byTimerId,
    uint32_t dwMilSecTick
) {
    if ((byTimerId >= TIMER_ID_MAX) || (g_pTimerHandle[byTimerId].callbackFunc == NULL))
        return;

    TIMER_t *s = &g_pTimerHandle[byTimerId];
    s->milSecStart = GetMilSecTick();
    s->milSecTimeout = dwMilSecTick;
    
    #if TIMER_SS_DEBUG
    debug_info("TIM_SS: TimerChangePeriod = %s, id = %d, gen = %d, timestamp = %ld, repeat = %d, timeout = %ld\n",
        s->name, byTimerId, s->gen, s->milSecStart, s->repeats, s->milSecTimeout);
    #endif
}

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
) {   
    if ((byTimerId >= TIMER_ID_MAX) || (g_pTimerHandle[byTimerId].callbackFunc == NULL))
        return 0;

    TIMER_t *s = &g_pTimerHandle[byTimerId];
    s->repeats = byRepeats;
    s->milSecTimeout = dwMilSecTick;
    s->milSecStart = GetMilSecTick();

    #if TIMER_SS_DEBUG
    debug_info("TIM_SS: TimerRestart = %s, id = %d, gen = %d, timestamp = %ld, repeat = %d, timeout = %ld\n",
        s->name, byTimerId, s->gen, s->milSecStart, s->repeats, s->milSecTimeout);
    #endif
    
    return 1;
}

/**
 * @brief  Stop a timer
 * @param  byTimerId: ID of timer
 * @retval 1: success, 0: fail
 */
uint8_t TimerStop(
    uint8_t *byTimerId
) {
    if ((byTimerId == NULL) || (*byTimerId >= TIMER_ID_MAX) || (g_pTimerHandle[*byTimerId].callbackFunc == NULL))
        return 0;
    
    TIMER_t *s = &g_pTimerHandle[*byTimerId];
    TimerStructResetToDefault(s);
    *byTimerId = TIMER_ID_INVALID;

    #if TIMER_SS_DEBUG
    debug_info("TIM_SS: TimerStop with id = %d, gen = %d\n", s->id, s->gen);
    #endif
    
    return 1;
}

/**
 * @brief  Get current tick in ms
 * @param  None
 * @retval Current tick in ms
 */
uint32_t GetMilSecTick(void) {
    // Atomic 32-bit only-read operation
    return g_wMilSecTickTimer;
}

/**
 * @brief  Process timer scheduler
 * @param  None
 * @retval None
 */
void processTimerScheduler(void) {
    for (uint8_t i = 0; i < TIMER_ID_MAX; i++) {
    	TIMER_t *s = &g_pTimerHandle[i];

        if (s->callbackFunc == NULL) {
            continue;
        }

        uint32_t now   = GetMilSecTick();
        uint32_t delta = now - s->milSecStart;

        if (delta >= s->milSecTimeout) {
            // Snapshot callback and parameter for ABA protection
            uint8_t gen_before = s->gen;
            void (*callback)(void *) = s->callbackFunc;
            void *param = s->pCallbackData;

#if TIMER_SS_DEBUG
            // —— ĐO THỜI GIAN CALLBACK THEO MS ——
            uint32_t t0 = GetMilSecTick();
            callback(param);
            uint32_t t1    = GetMilSecTick();
            uint32_t cb_ms = t1 - t0;                // 0 nếu callback < 1ms
            // Cập nhật thống kê theo index i (không phụ thuộc slot còn sống)
            TIMER_STATS_t *st = &g_timerStats[i];
            st->last_ms = cb_ms;
            if (cb_ms > st->max_ms) st->max_ms = cb_ms;
            st->sum_ms += cb_ms;
            st->call_count++;
            // Tích lũy ms bận (thô, vì <1ms -> 0)
            g_busyMsAcc += cb_ms;
            // —— ĐO STACK SAU CALLBACK ——
            uint32_t msp        = GetMSP();
            uint32_t stack_free = (uint32_t)&__StackTop - msp; // byte còn trống
            st->last_stack_free = stack_free;
            if (st->min_stack_free == 0 || stack_free < st->min_stack_free) {
                st->min_stack_free = stack_free;
            }
#else
            // Debug disabled, call directly
            callback(param);
#endif

            // Re-fetch timer after callback
            s = &g_pTimerHandle[i];

            // Check if timer was stopped in callback
            if (s->callbackFunc == NULL) continue;

            // Check if timer was restarted in callback
            if (s->gen != gen_before) continue;

            if (s->repeats != TIMER_REPEAT_FOREVER) {
                if (s->repeats > 0) s->repeats--;
                if (s->repeats == 0) {
                    TimerStop(&s->id);
                    continue;
                }
            }

            s->milSecStart = now;

            #if TIMER_SS_DEBUG
            debug_info("TIM_SS: TimerExpired = %s, id = %d, gen = %d, timestamp = %ld, repeat = %d, timeout = %ld\n",
                s->name, i, s->gen, s->milSecStart, s->repeats, s->milSecTimeout);
            #endif
        }
    }
    
#if TIMER_SS_DEBUG
    // Log CPU load mỗi 1000ms
    static uint32_t lastCpuLog = 0;
    if ((GetMilSecTick() - lastCpuLog) >= 1000) {
        TimerDebugPrintCpuLoadMs();
        lastCpuLog += 1000;
    }

    // Log thống kê TimerBlink mỗi 5s
    static uint32_t lastStatLog = 0;
    if ((GetMilSecTick() - lastStatLog) >= 5000) {
        for (uint8_t id = 0; id < TIMER_ID_MAX; id++) {
            if (g_pTimerHandle[id].callbackFunc != NULL) {
                TimerDebugPrintStatsMs(id);
            }
        }
        lastStatLog += 5000;
    }
#endif
}

void TimerStructResetToDefault(TIMER_t *s) {
    if (s == NULL) return;

    s->id = TIMER_ID_INVALID;
    s->gen = 0;
    s->name = NULL;
    s->milSecStart = 0;
    s->milSecTimeout = 0;
    s->repeats = 0;
    s->callbackFunc = NULL;
    s->pCallbackData = NULL;
}

/**
 * @brief  System tick handler
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) {
    // Atomic 32-bit read-modify-write operation
    g_wMilSecTickTimer++;

#if TIMER_SS_DEBUG
    // snapshot tổng ms bận (thô) – nếu cần tính CPU thô ở nơi khác
    g_busyMsSnapshot = g_busyMsAcc;
    g_busyMsAcc = 0;
#endif
}

#if TIMER_SS_DEBUG
/**
 * In “CPU load” thô theo ms callback trong mỗi tick (rất thô vì <1ms = 0)
 * Khuyến nghị: tự cộng dồn g_busyMsSnapshot qua cửa sổ dài hơn (ví dụ 100ms)
 * để xem tổng ms bận trong 100ms.
 */
void TimerDebugPrintCpuLoadMs(void) {
    debug_info("TIM_SS: [CPU/ms] busy (ms in last tick) = %lu (ms)\n",
           (unsigned long)g_busyMsSnapshot);
}

/**
 * In thống kê của slot id (ms + stack)
 */
void TimerDebugPrintStatsMs(uint8_t id) {
    if (id >= TIMER_ID_MAX) return;
    TIMER_STATS_t *st = &g_timerStats[id];
    uint32_t avg_ms = (st->call_count == 0) ? 0u :
                      (uint32_t)(st->sum_ms / st->call_count);
    debug_info("TIM_SS: [STAT/ms] id = %u, calls = %lu, last = %lu ms, max = %lu ms, "
           "avg = %lu ms, stack(last = %lu B, min = %lu B)\n",
           (unsigned)id,
           (unsigned long)st->call_count,
           (unsigned long)st->last_ms,
           (unsigned long)st->max_ms,
           (unsigned long)avg_ms,
           (unsigned long)st->last_stack_free,
           (unsigned long)st->min_stack_free);
}
#endif /* TIMER_SS_DEBUG */

/* END FILE */
