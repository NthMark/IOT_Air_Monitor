/**
 * @file    crashlog_f1.c
 * @brief   Minimal, fast crash logger for STM32F103 (Cortex-M3).
 *
 *  - Fault handlers are naked: they pick MSP/PSP and call the writer.
 *  - Writer runs small, writes from RAM, and never erases during fault.
 *  - One Flash page acts as an append-only journal.
 */

#include "stackdump.h"
#include "iwdg.h"
#include "utilities.h"
#include <stddef.h>

/* ---------- Magic values ---------- */
#define CRASHLOG_OPEN_MAGIC   (0xA11CUL)
#define CRASHLOG_CLOSE_MAGIC  (0xC10EUL)   /* visually distinct */

/* ---- Flash primitives (MUST run from RAM on F1 when programming) ---- */

__attribute__((section(".ramfunc")))
static void flash_wait_not_busy(void)
{
    while (FLASH->SR & FLASH_SR_BSY) { /* spin */ }
}

__attribute__((section(".ramfunc")))
static void flash_unlock_if_needed(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123UL;
        FLASH->KEYR = 0xCDEF89ABUL;
    }
}

__attribute__((section(".ramfunc")))
static void flash_prog_halfword(uint32_t addr, uint16_t data)
{
    flash_wait_not_busy();
    flash_unlock_if_needed();
    FLASH->CR |= FLASH_CR_PG;
    *(__IO uint16_t*)addr = data;
    flash_wait_not_busy();
    FLASH->CR &= ~FLASH_CR_PG;
}

__attribute__((section(".ramfunc")))
static void flash_write_seq(uint32_t dst, const void* src, uint32_t len)
{
    const uint16_t* p = (const uint16_t*)src;
    uint32_t halfwords = (len + 1U) / 2U;
    for (uint32_t i = 0; i < halfwords; ++i) {
        uint32_t a = dst + 2U*i;
        if (*(__IO uint16_t*)a != 0xFFFFU) break;   /* stop if no free space */
        flash_prog_halfword(a, p[i]);
    }
}

/* ---- Page mgmt (non-fault path) ---- */

void crashlog_erase_page(void)
{
    /* Non-fault path only (takes tens of ms). */
    flash_wait_not_busy();
    flash_unlock_if_needed();
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = CRASHLOG_FLASH_PAGE_ADDR;
    FLASH->CR |= FLASH_CR_STRT;
    flash_wait_not_busy();
    FLASH->CR &= ~FLASH_CR_PER;
}

static inline uint32_t page_end_addr(void)
{
    return CRASHLOG_FLASH_PAGE_ADDR + CRASHLOG_FLASH_PAGE_SIZE;
}

static uint32_t find_free_offset(void)
{
    /* Scan for first half-word == 0xFFFF (start of free slot) aligned to record size. */
    for (uint32_t off = 0; off + sizeof(crashlog_rec_t) <= CRASHLOG_FLASH_PAGE_SIZE; off += sizeof(crashlog_rec_t)) {
        uint16_t w = *(__IO uint16_t*)(CRASHLOG_FLASH_PAGE_ADDR + off);
        if (w == 0xFFFFU) return off;
    }
    return 0xFFFFFFFFUL;
}

/* Validate a record (open/close markers + CRC) */
static int is_valid_record_at(uint32_t addr, crashlog_rec_t *out)
{
    const crashlog_rec_t *r = (const crashlog_rec_t*)addr;
    if (r->open_magic != CRASHLOG_OPEN_MAGIC) return 0;
    if (r->close_magic != CRASHLOG_CLOSE_MAGIC) return 0;

    uint16_t calc = crc16_ccitt((const uint8_t*)&r->sp,
        sizeof(crashlog_rec_t) - offsetof(crashlog_rec_t, sp) - sizeof(r->crc16) - sizeof(r->close_magic));
    if (calc != r->crc16) return 0;

    if (out) *out = *r;
    return 1;
}

void crashlog_init(void)
{
    /* Optional: nothing mandatory. User may choose to erase if full. */
    (void)page_end_addr();
}

/* Read the last valid record (if any). Return 1 if found, 0 if none. */
int crashlog_read_last(crashlog_rec_t *out)
{
    uint32_t last_valid = 0;
    for (uint32_t off = 0; off + sizeof(crashlog_rec_t) <= CRASHLOG_FLASH_PAGE_SIZE; off += sizeof(crashlog_rec_t)) {
        uint32_t addr = CRASHLOG_FLASH_PAGE_ADDR + off;
        const crashlog_rec_t *r = (const crashlog_rec_t*)addr;
        if (r->open_magic == 0xFFFFU) break; /* never written beyond this point */
        if (is_valid_record_at(addr, NULL)) last_valid = addr;
    }
    if (last_valid) {
        if (out) *out = *(const crashlog_rec_t*)last_valid;
        return 1;
    }
    return 0;
}

/* ---- Fault capture path ---- */

/* Forward decl: called from naked handlers with SP frame pointer */
void crashlog_capture_and_store(uint32_t *sp_frame);

/* HardFault always hooked */
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "TST lr, #4            \n"   /* EXC_RETURN bit2: 0=MSP, 1=PSP */
        "ITE EQ                \n"
        "MRSEQ r0, MSP         \n"   /* r0 = SP to HW frame [R0..xPSR] */
        "MRSNE r0, PSP         \n"
        "B crashlog_capture_and_store \n"
    );
}

#if CRASHLOG_HOOK_ALL_FAULTS
__attribute__((naked)) void MemManage_Handler(void)
{
    __asm volatile(
        "TST lr, #4 \n"
        "ITE EQ     \n"
        "MRSEQ r0, MSP \n"
        "MRSNE r0, PSP \n"
        "B crashlog_capture_and_store \n"
    );
}
__attribute__((naked)) void BusFault_Handler(void)
{
    __asm volatile(
        "TST lr, #4 \n"
        "ITE EQ     \n"
        "MRSEQ r0, MSP \n"
        "MRSNE r0, PSP \n"
        "B crashlog_capture_and_store \n"
    );
}
__attribute__((naked)) void UsageFault_Handler(void)
{
    __asm volatile(
        "TST lr, #4 \n"
        "ITE EQ     \n"
        "MRSEQ r0, MSP \n"
        "MRSNE r0, PSP \n"
        "B crashlog_capture_and_store \n"
    );
}
#endif /* HOOK_ALL_FAULTS */

/* Optional SVC snapshot for testing path without a crash */
__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile(
        "TST lr, #4 \n"
        "ITE EQ     \n"
        "MRSEQ r0, MSP \n"
        "MRSNE r0, PSP \n"
        "B crashlog_capture_and_store \n"
    );
}

void crashlog_test_svc_snapshot(void)
{
    __asm volatile("svc 0");
}

/* Writer: gather minimal context and append to Flash (runs partly from RAM) */
__attribute__((optimize("O0")))
void crashlog_capture_and_store(uint32_t *sp_frame)
{
#if CRASHLOG_PET_IWDG_ON_FAULT
    iwdg_pet();
#endif

    crashlog_rec_t rec = {0};
    rec.open_magic = (uint16_t)CRASHLOG_OPEN_MAGIC;
    rec.len = (uint16_t)(sizeof(crashlog_rec_t) - offsetof(crashlog_rec_t, sp));

    /* Read SCB fault status quickly */
    rec.cfsr  = SCB->CFSR;
    rec.hfsr  = SCB->HFSR;
    rec.bfar  = SCB->BFAR;
    rec.mmfar = SCB->MMFAR;

    /* HW frame layout: [0]=R0,[1]=R1,[2]=R2,[3]=R3,[4]=R12,[5]=LR,[6]=PC,[7]=xPSR */
    uint32_t r0  = sp_frame[0U];
    uint32_t r1  = sp_frame[1U];
    uint32_t r2  = sp_frame[2U];
    uint32_t r3  = sp_frame[3U];
    uint32_t r12 = sp_frame[4U];
    uint32_t lr  = sp_frame[5U];
    uint32_t pc  = sp_frame[6U];
    uint32_t xpsr= sp_frame[7U];

    /* Store minimal set */
    rec.sp   = (uint32_t)sp_frame;
    rec.pc   = pc;
    rec.lr   = lr;
    rec.xpsr = xpsr;

#if CRASHLOG_FRAME_WORDS >= 4
    rec.frame[0] = r0;
    rec.frame[1] = r1;
    rec.frame[2] = r2;
    rec.frame[3] = r3;
#endif
#if CRASHLOG_FRAME_WORDS >= 8
    rec.frame[4] = r12;
    rec.frame[5] = lr;
    rec.frame[6] = pc;
    rec.frame[7] = xpsr;
#endif

    /* Compute CRC over payload [sp..frame] */
    rec.crc16 = crc16_ccitt((const uint8_t*)&rec.sp,
                sizeof(crashlog_rec_t) - offsetof(crashlog_rec_t, sp) - sizeof(rec.crc16) - sizeof(rec.close_magic));
    rec.close_magic = (uint16_t)CRASHLOG_CLOSE_MAGIC;

    /* Disable IRQs during Flash program to minimize interference */
    __disable_irq();

    uint32_t off = find_free_offset();
    if (off != 0xFFFFFFFFUL) {
        uint32_t base = CRASHLOG_FLASH_PAGE_ADDR + off;
        /* Write in safe order: header -> payload -> crc -> close marker */
        flash_write_seq(base + offsetof(crashlog_rec_t, open_magic), &rec.open_magic, sizeof(rec.open_magic) + sizeof(rec.len));
        flash_write_seq(base + offsetof(crashlog_rec_t, sp),
                        &rec.sp,
                        sizeof(crashlog_rec_t) - offsetof(crashlog_rec_t, sp) - sizeof(rec.crc16) - sizeof(rec.close_magic));
        flash_write_seq(base + offsetof(crashlog_rec_t, crc16), &rec.crc16, sizeof(rec.crc16));
        flash_write_seq(base + offsetof(crashlog_rec_t, close_magic), &rec.close_magic, sizeof(rec.close_magic));
    }

    __enable_irq();

    /* Optional: brief UART note via weak hooks (safe, polling assumed) */
    crashlog_puts("[crashlog] record written");

    /* Decide what to do next: loop for debugger or reset. */
#ifdef NDEBUG
    /* right before NVIC_SystemReset(); add: */
    while (FLASH->SR & FLASH_SR_BSY) { /* wait */ }
    __DSB(); __ISB();
    NVIC_SystemReset();
#else
    while (1) { __NOP(); }
#endif
}

/* ---------- Weak I/O hooks ---------- */
__attribute__((weak)) void crashlog_puts(const char *s) { (void)s; }

/* End of file */
