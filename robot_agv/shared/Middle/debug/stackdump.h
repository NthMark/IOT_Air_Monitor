/**
 * @file    crashlog_f1.h
 * @brief   Minimal, fast crash logger for STM32F103 (Cortex-M3).
 *
 * Save a compact stackdump record to on-chip Flash in the event of a fault,
 * in ~1–2 ms, without erasing during the fault. Writing is done from RAM.
 *
 * How it works (high level):
 *  - Reserve one Flash page as an append-only "crash log" area.
 *  - At boot (normal runtime), you may erase this page when needed.
 *  - On fault, the CPU pushes a hardware stack frame [R0..R3, R12, LR, PC, xPSR].
 *  - The naked fault handler picks the right SP (MSP/PSP) and calls the writer.
 *  - The writer packs a small record (PC/LR/xPSR/CFSR/... + a few frame words)
 *    and "programs" it into the next free half-words (0xFFFF) of the log page.
 *  - No erase is performed inside the fault; only programming half-words.
 *
 * Linker requirement (GCC): place .ramfunc in SRAM:
 *  SECTIONS {
 *    .ramfunc : { *(.ramfunc*) } >RAM
 *  }
 *
 * Notes:
 *  - Do NOT call printf/heap/IRQ/DMA in the fault path.
 *  - Keep the record small for speed.
 *  - Program-from-RAM is mandatory on F1, because Flash fetch stalls during program.
 */
#ifndef _STACKDUMP_H_
#define _STACKDUMP_H_

#include <stdint.h>
#include "stm32f10x.h"   /* CMSIS + device registers */

/* ---------- User configuration ---------- */

/** Select your device flash size and crash page address.
 *  F103C8 (64KB):  last page = 0x0800FC00
 *  F103xB (128KB): last page = 0x0801FC00
 *  Adjust as needed.
 */
#ifndef CRASHLOG_FLASH_PAGE_ADDR
#define CRASHLOG_FLASH_PAGE_ADDR   (0x0800FC00UL)   /* default for 64KB devices */
#endif

#ifndef CRASHLOG_FLASH_PAGE_SIZE
#define CRASHLOG_FLASH_PAGE_SIZE   (0x400UL)        /* 1 KB page on STM32F103C8 */
#endif

/** Enable hooking additional faults (besides HardFault) */
#ifndef CRASHLOG_HOOK_ALL_FAULTS
#define CRASHLOG_HOOK_ALL_FAULTS   (1)              /* 1 = also hook Bus/Mem/Usage faults */
#endif

/** Number of words from HW frame to store (4 = R0..R3; 8 = R0..R3,R12,LR,PC,xPSR) */
#ifndef CRASHLOG_FRAME_WORDS
#define CRASHLOG_FRAME_WORDS       (8)              /* keep small for speed; set 8 if you want full frame */
#endif

/** Optional: pet IWDG at fault entry to gain a few ms margin */
#ifndef CRASHLOG_PET_IWDG_ON_FAULT
#define CRASHLOG_PET_IWDG_ON_FAULT (1)
#endif

/* ---------- Types ---------- */

/* Packed crash record, sized for speed (~48B @ FRAME_WORDS=4; ~64B @ 8) */
typedef struct __attribute__((packed)) {
    uint16_t open_magic;    /* 0xA11C – written FIRST */
    uint16_t len;           /* payload length (bytes) from 'sp' to end of 'frame' */
    uint32_t sp;            /* address of the HW frame (MSP or PSP) */
    uint32_t pc;            /* faulting PC */
    uint32_t lr;            /* caller return address */
    uint32_t xpsr;          /* xPSR (usually 0x21000000) */
    uint32_t cfsr;          /* SCB->CFSR */
    uint32_t hfsr;          /* SCB->HFSR */
    uint32_t bfar;          /* SCB->BFAR */
    uint32_t mmfar;         /* SCB->MMFAR */
    uint32_t frame[CRASHLOG_FRAME_WORDS]; /* subset of HW frame for quick match */
    uint16_t crc16;         /* CRC16-CCITT over the payload [sp..frame] */
    uint16_t close_magic;   /* 0xC10S – written LAST (marks completion) */
} crashlog_rec_t;

/* Result codes */
typedef enum {
    CRASHLOG_OK = 0,
    CRASHLOG_ERR_NO_SPACE = -1,
    CRASHLOG_ERR_FLASH_BUSY = -2,
    CRASHLOG_ERR_INVALID = -3,
} crashlog_result_t;

/* ---------- API (boot/runtime) ---------- */

/**
 * @brief  Initialize crashlog system at boot (optional, non-fault path).
 *         - Optionally erase page if full (policy up to caller).
 *         - Typically cheap: may only scan the page for last record.
 */
void crashlog_init(void);

/**
 * @brief  Erase the crashlog flash page (non-fault path only).
 * @note   Takes ~20–40 ms; never call inside a fault.
 */
void crashlog_erase_page(void);

/**
 * @brief  Find and load the most recent valid crash record.
 * @param  out   Pointer to struct to fill.
 * @return 1 if found a valid record (CRC OK & closed), 0 if none.
 */
int crashlog_read_last(crashlog_rec_t *out);

/**
 * @brief  Optional: test path without crash by taking an SVC snapshot.
 *         This triggers SVC (CPU pushes a HW frame) and stores a record.
 */
void crashlog_test_svc_snapshot(void);

/* ---------- Fault handlers (installed by this module) ---------- */
/* HardFault is always hooked. Additional faults are optional via CRASHLOG_HOOK_ALL_FAULTS. */
void HardFault_Handler(void);
#if CRASHLOG_HOOK_ALL_FAULTS
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
#endif

/* ---------- Weak I/O hooks (override to integrate with your logger) ---------- */
__attribute__((weak)) void crashlog_puts(const char *s);

#endif
