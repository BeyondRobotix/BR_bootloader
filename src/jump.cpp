#include <Arduino.h>
#include "board_config.h"
#include "jump.h"

/*
 * Where an app's initial stack pointer may be. An initial SP is the top of
 * a stack, so it may equal a region's end. Arduino apps put the stack in
 * DTCM on H7; ArduPilot (AP_Periph) puts it at the start of its RAM map,
 * which on the H743 is SRAM1 (0x30000000), so every H7 SRAM is allowed.
 */
struct ram_region { uint32_t start, end; };
static const struct ram_region k_app_ram[] = {
#if defined(CANH7) || defined(ARDUINO_NUCLEO_H723ZG)
    {0x20000000UL, 0x20020000UL},  /* DTCM, 128 KB */
    {0x24000000UL, 0x24080000UL},  /* AXI SRAM, up to 512 KB */
    {0x30000000UL, 0x30048000UL},  /* SRAM1-3 (D2), up to 288 KB */
    {0x38000000UL, 0x38010000UL},  /* SRAM4 (D3), 64 KB */
#else
    {0x20000000UL, 0x20010000UL},  /* L431 SRAM1 + SRAM2, 64 KB contiguous */
    {0x10000000UL, 0x10004000UL},  /* L431 SRAM2 alias, 16 KB */
#endif
};

static bool sp_in_app_ram(uint32_t sp)
{
    for (const auto &r : k_app_ram) {
        if (sp > r.start && sp <= r.end) {
            return true;
        }
    }
    return false;
}

bool app_is_valid(uint32_t app_address)
{
    const uint32_t *vt = (const uint32_t *)app_address;
    uint32_t sp    = vt[0];
    uint32_t reset = vt[1];

    if (!sp_in_app_ram(sp)) {
        return false;
    }
    /* Reset handler must have the Thumb bit set and lie inside the app: an
       image linked at 0x08000000 (a no-bootloader build loaded over CAN by
       mistake) would otherwise jump into the bootloader's own code. */
    if ((reset & 1U) == 0U) {
        return false;
    }
    if (reset < app_address || reset >= 0x08000000UL + BOARD_FLASH_BYTES) {
        return false;
    }
    return true;
}

void jump_to_app(uint32_t app_address)
{
    if (!app_is_valid(app_address)) {
        return;
    }

    const uint32_t *vt = (const uint32_t *)app_address;
    uint32_t sp    = vt[0];
    uint32_t reset = vt[1];

    /* Disable interrupts and clear pending so the app starts clean. */
    __disable_irq();
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    /* Point the vector table at the app and force the change to be visible. */
    SCB->VTOR = app_address;
    __DSB();
    __ISB();

    /* Load SP and branch -- noreturn. */
    __set_MSP(sp);
    __enable_irq();
    ((void (*)(void))reset)();

    /* Should never reach here. */
    while (1) { }
}
