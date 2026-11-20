/* Append-only image writes, buffered in 32-byte granules with lazy page erasure. */

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Prepare for a new image: reset offset, erase the first app page. */
bool bootloader_flash_begin(void);

/* Append a chunk at the given linear offset (0 == start of app image). The
   offset is checked against the internal cursor; out-of-order chunks fail. */
bool bootloader_flash_write(uint32_t offset, const uint8_t *data, uint16_t len);

/* Pad the tail with 0xFF, then commit the deferred vector table. */
bool bootloader_flash_finalize(void);

void bootloader_flash_write_cb(uint32_t offset, const uint8_t *data, uint16_t len);

/* True if any begin/write/finalize call has failed since the last begin(). */
bool bootloader_flash_failed(void);

/* Write progress, excluding buffered tail bytes and including final alignment padding. */
uint32_t bootloader_flash_bytes_written(void);

/* First failure since the last begin() attempt; values also encode LED blink counts. */
typedef enum {
    BL_FLASH_OK                = 0,
    BL_FLASH_ERR_BEGIN_ERASE   = 1,
    BL_FLASH_ERR_NOT_STARTED   = 2,  /* write called before successful begin */
    BL_FLASH_ERR_OFFSET        = 3,  /* chunk arrived at unexpected offset */
    BL_FLASH_ERR_OVERFLOW      = 4,  /* chunk + pending > internal buffer */
    BL_FLASH_ERR_PAGE_ERASE    = 5,  /* mid-image page erase failed */
    BL_FLASH_ERR_WRITE         = 6,  /* stm32_flash_write reported failure */
    BL_FLASH_ERR_FINALIZE      = 7,  /* finalize called without a successful begin */
    BL_FLASH_ERR_OUT_OF_RANGE  = 8,  /* page address outside app region */
} bl_flash_err_t;

bl_flash_err_t bootloader_flash_last_error(void);

/* STM32L4 register snapshot, captured only when the initial page erase fails. */
struct bl_flash_diag {
    uint32_t sr;       /* FLASH_SR after the failed op (WRPERR/PGSERR/etc.) */
    uint32_t cr;       /* FLASH_CR after the failed op (PER/PG/LOCK state) */
    uint32_t optr;     /* FLASH_OPTR (RDP level, dual-bank, etc.) */
    uint32_t wrp1ar;   /* FLASH_WRP1AR (write-protected area A bounds) */
    uint32_t wrp1br;   /* FLASH_WRP1BR (write-protected area B bounds) */
    uint32_t pcrop1sr; /* FLASH_PCROP1SR (PCROP zone start) */
    uint32_t pcrop1er; /* FLASH_PCROP1ER (PCROP zone end + RDP_ERASE flag) */
    uint32_t acr;      /* FLASH_ACR (cache + prefetch state) */
};
void bootloader_flash_get_diag(struct bl_flash_diag *out);

#ifdef __cplusplus
}
#endif
