/*
 * Per-MCU configuration constants for the BR bootloader.
 *
 * Selected at build time via the platformio.ini build flag for the target:
 *   -DCANL431                 -- MicroNode (STM32L431)
 *   -DARDUINO_NUCLEO_H723ZG   -- MicroNodePlus (STM32H723)
 *   -DCANH7                   -- CoreNode (STM32H743)
 *   -DCANC532                 -- MicroNodeC5 (STM32C532), UNVERIFIED
 */

#pragma once

#include <Arduino.h>   /* for LED_BUILTIN / PNUM_NOT_DEFINED on H7 boards */
#include <stdint.h>

#if defined(CANL431)

/* STM32L431 (MicroNode) ----------------------------------------------------*/

/* Bootloader occupies the first 40 KB of flash; app starts on the next 2 KB
   page boundary so the bootloader is safe to leave in place during app erase. */
#define APP_START_ADDRESS   0x0800A000UL
#define BL_FLASH_PAGE_SIZE  0x800UL          /* 2 KB pages on L4 */
#define BOARD_FLASH_BYTES   (256UL * 1024UL) /* L431CC */

/* Arduino pin 19 == PB3 on the MicroNode variant header. */
#define BOOTLOADER_LED_PIN  19

/* 96-bit unique ID base address (per RM0394). */
#define UDID_BASE           0x1FFF7590UL

/* Node ID claimed by the bootloader when BOOTCOM didn't hand one over
   (i.e. cold-boot recovery scenarios -- power-cycle after a bricked update).
   Matches PREFERRED_NODE_ID in libArduinoDroneCAN so the bootloader appears
   on the bus at the same ID the app would normally use, no DNA needed. */
#define BL_FALLBACK_NODE_ID 100

#elif defined(CANH7) || defined(ARDUINO_NUCLEO_H723ZG)

/* STM32H7 family (CoreNode H743 / MicroNodePlus H723) ----------------------*/

/* Both H7 boards use 128 KB sectors; the app starts at sector 1 (0x08020000)
   so erasing the app never touches the bootloader. */
#define APP_START_ADDRESS   0x08020000UL
#define BL_FLASH_PAGE_SIZE  0x20000UL

/* 96-bit unique ID base on STM32H7 (RM0433 / RM0468). */
#define UDID_BASE           0x1FF1E800UL

#define BL_FALLBACK_NODE_ID 100

#if defined(ARDUINO_NUCLEO_H723ZG)
  /* MicroNodePlus: STM32H723ZG, 1024 KB flash. */
  #define BOARD_FLASH_BYTES (1024UL * 1024UL)
#else
  /* CoreNode: STM32H743XI, 2048 KB flash. */
  #define BOARD_FLASH_BYTES (2048UL * 1024UL)
#endif

/* MicroNodePlus has LED_BUILTIN (PB0); CoreNode has none yet. When no LED
   is wired, leave BOOTLOADER_LED_PIN undefined and main.cpp / drive_led()
   skip all GPIO calls. The DroneCAN debug.LogMessage stream and the
   NodeStatus health/mode override remain as the diagnostic channels. */
#if defined(LED_BUILTIN) && (LED_BUILTIN != PNUM_NOT_DEFINED)
  #define BOOTLOADER_LED_PIN  LED_BUILTIN
#endif

#elif defined(CANC532)

/* STM32C532 (MicroNodeC5) -- UNVERIFIED, no build/hardware testing done. -----
 * Its flash controller happens to share MicroNode's (STM32L431) geometry:
 * single bank, 2 KB pages, 256 KB total. Same 40 KB bootloader reserve /
 * app-start-address split is reused here for consistency with MicroNode. */

#define APP_START_ADDRESS   0x0800A000UL
#define BL_FLASH_PAGE_SIZE  0x800UL          /* 2 KB pages, single bank */
#define BOARD_FLASH_BYTES   (256UL * 1024UL) /* C532CC */

/* No LED confirmed on this board yet -- leave undefined so main.cpp / drive_led()
   skip all GPIO calls, same as CoreNode today. */

/* 96-bit unique ID base address (per the STM32C5 CMSIS device header,
   UID_BASE in stm32c532xx.h) -- verify against the STM32C5 reference manual. */
#define UDID_BASE           0x08FFF800UL

#define BL_FALLBACK_NODE_ID 100

#else
#error "Define CANL431, CANH7, ARDUINO_NUCLEO_H723ZG, or CANC532 for the target MCU."
#endif
