/*
 * Shared bootloader <-> application IPC region.
 *
 * Layout MUST match Arduino-DroneCAN's app_bootloader_comms (lib/libArduinoDroneCAN/src/dronecan.h)
 * and AP_Periph's app_comms.h. The struct lives at BOOTCOM_ADDR (0x20000000), 256 bytes
 * carved out of RAM by the linker scripts on both sides.
 */

#pragma once

#include <stdint.h>

#define APP_BOOTLOADER_COMMS_MAGIC 0xc544ad9a
#define BOOTCOM_ADDR               0x20000000UL

/* Written to `reserved` once the bootloader has acted on a hand-over, so a
   reset mid-download doesn't start the same download again on every boot.
   Both AP_Periph and Arduino-DroneCAN zero the whole struct before a
   hand-over whenever `magic` isn't already set, and the bootloader always
   clears `magic` before jumping, so a fresh hand-over has reserved == 0.
   Neither app reads the field. */
#define BOOTCOM_HANDOVER_USED      0x55534544UL  /* "USED" */

struct app_bootloader_comms {
    uint32_t magic;
    uint32_t ip;
    uint32_t netmask;
    uint32_t gateway;
    uint32_t reserved;
    uint8_t  server_node_id;
    uint8_t  my_node_id;
    uint8_t  path[201];
};

static inline volatile struct app_bootloader_comms *bootcom(void)
{
    return (volatile struct app_bootloader_comms *)BOOTCOM_ADDR;
}

/*
 * Make sure every write to BOOTCOM has reached RAM. On STM32H7, BOOTCOM is
 * in DTCM, which has ECC: a write narrower than an ECC word waits in a
 * buffer for the next write to the same RAM, and a reset discards it. An
 * aligned 64-bit write to a spare word at the end of the 256-byte region
 * commits it without leaving anything new pending. Call after changing
 * BOOTCOM and before any reset or jump. Harmless on the L4 (no ECC).
 */
/*
 * Arduino-DroneCAN apps (libArduinoDroneCAN) mark "ran for 30 s" here, with
 * the same RTC_BOOT_FWOK value AP_Periph writes to RTC backup register 0:
 * a 64-bit word at BOOTCOM + 0xF0, past the struct, which RAM keeps across a
 * watchdog reset. The bootloader reads and clears it on every boot.
 */
#define BOOTCOM_FWOK_WORD ((volatile uint64_t *)(BOOTCOM_ADDR + 0xF0UL))

static inline void bootcom_commit(void);

static inline uint32_t bootcom_fwok_take(void)
{
    const uint32_t v = (uint32_t)*BOOTCOM_FWOK_WORD;
    *BOOTCOM_FWOK_WORD = 0;
    bootcom_commit();
    return v;
}

static inline void bootcom_commit(void)
{
    volatile uint64_t *spare = (volatile uint64_t *)(BOOTCOM_ADDR + 0x100UL - sizeof(uint64_t));
    *spare = 0;
    (void)*spare;
    __asm__ volatile("dsb" ::: "memory");
}
