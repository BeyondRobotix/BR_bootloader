#include <Arduino.h>
#include "board_config.h"
#include "bootloader_comms.h"
#include "bootloader_dronecan.h"
#include "bootloader_flash.h"
#include "jump.h"
#include "rtc_boot.h"

static void clear_bootcom_magic()
{
    bootcom()->magic = 0;
    bootcom()->reserved = 0;
    bootcom_commit();
}

static bool g_watchdog_reset;

static void capture_reset_cause()
{
#if defined(STM32H7xx) || defined(STM32H7)
    g_watchdog_reset = (RCC->RSR & RCC_RSR_IWDG1RSTF) != 0;
#else
    g_watchdog_reset = (RCC->CSR & RCC_CSR_IWDGRSTF) != 0;
#endif
}

static void clear_reset_flags()
{
#if defined(STM32H7xx) || defined(STM32H7)
    RCC->RSR |= RCC_RSR_RMVF;
#else
    RCC->CSR |= RCC_CSR_RMVF;
#endif
}

void setup()
{
    capture_reset_cause();
#ifdef BOOTLOADER_LED_PIN
    pinMode(BOOTLOADER_LED_PIN, OUTPUT);
#endif

    volatile struct app_bootloader_comms *bc = bootcom();
    const bool update_requested = (bc->magic == APP_BOOTLOADER_COMMS_MAGIC);
    /* Recovery must not depend on a DNA allocator; node ID zero cannot advertise status. */
    uint8_t node_id = BL_FALLBACK_NODE_ID;
    if (update_requested && bc->my_node_id > 0 && bc->my_node_id <= 127) {
        node_id = bc->my_node_id;
    }

    const uint32_t rtc_boot = rtc_boot_read();
    rtc_boot_write(RTC_BOOT_OFF);
    bool fw_ok = rtc_boot == RTC_BOOT_FWOK;
    if (!fw_ok) {
        fw_ok = bootcom_fwok_take() == RTC_BOOT_FWOK;
    }
    const bool crashed_early = g_watchdog_reset && !fw_ok;
    const bool stay_in_bootloader = update_requested || rtc_boot == RTC_BOOT_HOLD || crashed_early;

    if (!stay_in_bootloader && app_is_valid(APP_START_ADDRESS)) {
        jump_to_app(APP_START_ADDRESS);
    }
    /* Preserve reset flags for the app when jumping directly to it. */
    clear_reset_flags();

    /* Do not retry the same hand-over after a reset mid-download. */
    uint8_t handover_server = 0;
    char handover_path[sizeof(bc->path)] = {0};
    if (update_requested && bc->reserved != BOOTCOM_HANDOVER_USED &&
        bc->server_node_id > 0 && bc->server_node_id <= 127) {
        size_t n = 0;
        while (n < sizeof(handover_path) - 1 && bc->path[n] != 0) {
            handover_path[n] = (char)bc->path[n];
            n++;
        }
        handover_path[n] = 0;
        if (n > 0) {
            handover_server = bc->server_node_id;
        }
    }

    /* Starting a download re-arms this request with BOOTCOM_HANDOVER_USED. */
    clear_bootcom_magic();

    bl_can_start(node_id, crashed_early);
    if (handover_server != 0) {
        bl_can_start_handover_download(handover_server, handover_path);
    }
}

/* Errors blink the flash error code, followed by a one-second pause. */
#ifdef BOOTLOADER_LED_PIN
static void drive_led()
{
    const uint32_t now = millis();
    static uint32_t toggle_at;
    static bool     led_on;

    if (bl_can_update_failed()) {
        const uint8_t code = (uint8_t)bootloader_flash_last_error();
        static uint32_t phase_start;
        static uint8_t  blinks_done;
        static bool     initialized;
        if (!initialized) {
            initialized = true;
            phase_start = now;
            blinks_done = 0;
            led_on = false;
            digitalWrite(BOOTLOADER_LED_PIN, LOW);
        }
        const uint32_t in_phase = now - phase_start;
        if (blinks_done < code) {
            if (in_phase < 200 && !led_on) {
                led_on = true;
                digitalWrite(BOOTLOADER_LED_PIN, HIGH);
            } else if (in_phase >= 200 && in_phase < 400 && led_on) {
                led_on = false;
                digitalWrite(BOOTLOADER_LED_PIN, LOW);
            } else if (in_phase >= 400) {
                blinks_done++;
                phase_start = now;
                led_on = false;
            }
        } else {
            digitalWrite(BOOTLOADER_LED_PIN, LOW);
            if (in_phase >= 1000) {
                blinks_done = 0;
                phase_start = now;
            }
        }
        return;
    }

    const uint32_t period = bl_can_update_in_progress() ? 100 : 1000;
    if ((int32_t)(now - toggle_at) >= 0) {
        led_on = !led_on;
        digitalWrite(BOOTLOADER_LED_PIN, led_on);
        toggle_at = now + period;
    }
}
#else
static inline void drive_led() {}
#endif

void loop()
{
    bl_can_cycle();

    if (bl_can_update_done() && app_is_valid(APP_START_ADDRESS)) {
        /* NRST preserves RAM: a completed update must not leave a pending request. */
        clear_bootcom_magic();
        jump_to_app(APP_START_ADDRESS);
    }

    drive_led();
}
