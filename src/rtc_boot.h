/*
 * RTC backup register 0: the boot flag ArduPilot's AP_Bootloader and AP_Periph
 * share, read and written the same way here so AP_Periph behaves identically
 * on BR_bootloader. Values from AP_HAL_ChibiOS/hwdef/common/stm32_util.h.
 *
 * The app writes RTC_BOOT_FWOK once it has run for 30 s (AP_Periph, and
 * Arduino-DroneCAN apps from libArduinoDroneCAN). The bootloader reads the
 * flag and then resets it to RTC_BOOT_OFF on every boot, so FWOK means "the
 * previous run lasted at least 30 s".
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

#define RTC_BOOT_OFF   0x00000000UL
#define RTC_BOOT_HOLD  0xb0070001UL  /* stay in the bootloader */
#define RTC_BOOT_FWOK  0xb0093a26UL  /* the app ran for 30 s */

static inline void rtc_boot_enable_access(void)
{
#if defined(STM32H7xx) || defined(STM32H7)
    RCC->APB4ENR |= RCC_APB4ENR_RTCAPBEN;
    (void)RCC->APB4ENR;
#else
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN;
    (void)RCC->APB1ENR1;
#endif
    PWR->CR1 |= PWR_CR1_DBP;
    while ((PWR->CR1 & PWR_CR1_DBP) == 0) {
    }
}

static inline uint32_t rtc_boot_read(void)
{
    rtc_boot_enable_access();
    return RTC->BKP0R;
}

static inline void rtc_boot_write(uint32_t v)
{
    rtc_boot_enable_access();
    if (RTC->BKP0R != v) {
        RTC->BKP0R = v;
    }
}
