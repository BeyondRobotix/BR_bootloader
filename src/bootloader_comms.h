/* Layout must match Arduino-DroneCAN's app_bootloader_comms and AP_Periph's
   app_comms.h within the linker-reserved 256-byte BOOTCOM region. */

#pragma once

#include <stdint.h>

#define APP_BOOTLOADER_COMMS_MAGIC 0xc544ad9a
#define BOOTCOM_ADDR               0x20000000UL

/* In `reserved`, prevents retrying a hand-over after reset; apps initialize it to zero. */
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

/* Arduino-DroneCAN's RTC_BOOT_FWOK marker, outside the struct and preserved by reset. */
#define BOOTCOM_FWOK_WORD ((volatile uint64_t *)(BOOTCOM_ADDR + 0xF0UL))

static inline void bootcom_commit(void);

static inline uint32_t bootcom_fwok_take(void)
{
    const uint32_t v = (uint32_t)*BOOTCOM_FWOK_WORD;
    *BOOTCOM_FWOK_WORD = 0;
    bootcom_commit();
    return v;
}

/* Drain H7 DTCM's ECC write buffer with a full-width spare-word write before reset/jump. */
static inline void bootcom_commit(void)
{
    volatile uint64_t *spare = (volatile uint64_t *)(BOOTCOM_ADDR + 0x100UL - sizeof(uint64_t));
    *spare = 0;
    (void)*spare;
    __asm__ volatile("dsb" ::: "memory");
}
