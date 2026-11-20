#include "bootloader_flash.h"
#include "board_config.h"
#include <flash.h>
#include <string.h>

#define APP_FIRST_PAGE  ((APP_START_ADDRESS - 0x08000000UL) / BL_FLASH_PAGE_SIZE)
/* H7 parameter sectors are outside the writable app region. */
#if (defined(CANH7) || defined(ARDUINO_NUCLEO_H723ZG)) && defined(STORAGE_FLASH_PAGE)
#define APP_LAST_PAGE   (STORAGE_FLASH_PAGE - 1)
#else
#define APP_LAST_PAGE   ((BOARD_FLASH_BYTES / BL_FLASH_PAGE_SIZE) - 1)
#endif
/* H7 requires 32-byte writes; L4 accepts multiples of 8 bytes. */
#define BL_WRITE_ALIGN  32U

static uint32_t cursor;
static uint8_t  pending[BL_WRITE_ALIGN];
static uint8_t  pending_len;
static uint32_t highest_page_erased;
static bool     started;
static bool     failed;
static uint32_t bytes_written;
static bl_flash_err_t last_err = BL_FLASH_OK;
static struct bl_flash_diag diag;

/* Keep the vector table erased until finalize so interrupted images cannot boot.
   Defer the whole granule: H7 cannot reprogram a partially written granule. */
static uint8_t  deferred_vt[BL_WRITE_ALIGN];
static bool     deferred_vt_armed;

/* STM32L4 FLASH peripheral register addresses (RM0394 Table 51). */
#define BL_FLASH_BASE   0x40022000UL
#define BL_FLASH_ACR    (*(volatile uint32_t *)(BL_FLASH_BASE + 0x00))
#define BL_FLASH_KEYR   (*(volatile uint32_t *)(BL_FLASH_BASE + 0x08))
#define BL_FLASH_SR     (*(volatile uint32_t *)(BL_FLASH_BASE + 0x10))
#define BL_FLASH_CR     (*(volatile uint32_t *)(BL_FLASH_BASE + 0x14))
#define BL_FLASH_OPTR   (*(volatile uint32_t *)(BL_FLASH_BASE + 0x20))
#define BL_FLASH_WRP1AR  (*(volatile uint32_t *)(BL_FLASH_BASE + 0x2C))
#define BL_FLASH_WRP1BR  (*(volatile uint32_t *)(BL_FLASH_BASE + 0x30))
#define BL_FLASH_PCROP1SR (*(volatile uint32_t *)(BL_FLASH_BASE + 0x24))
#define BL_FLASH_PCROP1ER (*(volatile uint32_t *)(BL_FLASH_BASE + 0x28))

static void capture_diag(void)
{
    diag.sr       = BL_FLASH_SR;
    diag.cr       = BL_FLASH_CR;
    diag.optr     = BL_FLASH_OPTR;
    diag.wrp1ar   = BL_FLASH_WRP1AR;
    diag.wrp1br   = BL_FLASH_WRP1BR;
    diag.pcrop1sr = BL_FLASH_PCROP1SR;
    diag.pcrop1er = BL_FLASH_PCROP1ER;
    diag.acr      = BL_FLASH_ACR;
}

static void fail(bl_flash_err_t err)
{
    failed = true;
    if (last_err == BL_FLASH_OK) {
        last_err = err;  /* keep the first failure -- root cause, not consequences */
    }
}

static uint32_t addr_to_page(uint32_t addr)
{
    return (addr - 0x08000000UL) / BL_FLASH_PAGE_SIZE;
}

static bool ensure_page_erased(uint32_t addr)
{
    uint32_t page = addr_to_page(addr);
    if (page < APP_FIRST_PAGE || page > APP_LAST_PAGE) {
        fail(BL_FLASH_ERR_OUT_OF_RANGE);
        return false;
    }
    if (started && page <= highest_page_erased) {
        return true;
    }
    if (!stm32_flash_erasepage(page)) {
        fail(BL_FLASH_ERR_PAGE_ERASE);
        return false;
    }
    highest_page_erased = page;
    return true;
}

static bool flush_aligned(const uint8_t *src, uint32_t len)
{
    uint32_t remaining = len;
    uint32_t addr      = cursor;
    const uint8_t *p   = src;
    while (remaining > 0) {
        if (!ensure_page_erased(addr)) {
            return false;
        }
        uint32_t page_end   = (addr_to_page(addr) + 1) * BL_FLASH_PAGE_SIZE + 0x08000000UL;
        uint32_t chunk_room = page_end - addr;
        uint32_t chunk      = remaining < chunk_room ? remaining : chunk_room;
        chunk &= ~(BL_WRITE_ALIGN - 1U);
        if (chunk == 0) {
            fail(BL_FLASH_ERR_WRITE);
            return false;
        }
        if (!stm32_flash_write(addr, p, chunk)) {
            fail(BL_FLASH_ERR_WRITE);
            return false;
        }
        addr      += chunk;
        p         += chunk;
        remaining -= chunk;
    }
    cursor = addr;
    return true;
}

bool bootloader_flash_begin(void)
{
    cursor = APP_START_ADDRESS;
    pending_len = 0;
    highest_page_erased = 0;
    bytes_written = 0;
    started = false;
    failed = false;
    last_err = BL_FLASH_OK;
    deferred_vt_armed = false;
    memset(deferred_vt, 0xFF, sizeof(deferred_vt));

    if (!stm32_flash_erasepage(APP_FIRST_PAGE)) {
        capture_diag();
        fail(BL_FLASH_ERR_BEGIN_ERASE);
        return false;
    }
    highest_page_erased = APP_FIRST_PAGE;
    started = true;
    return true;
}

bool bootloader_flash_write(uint32_t offset, const uint8_t *data, uint16_t len)
{
    if (!started) {
        fail(BL_FLASH_ERR_NOT_STARTED);
        return false;
    }
    uint32_t expected_offset = (cursor - APP_START_ADDRESS) + pending_len;
    if (offset != expected_offset) {
        fail(BL_FLASH_ERR_OFFSET);
        return false;
    }
    uint8_t buf[BL_WRITE_ALIGN + 512];
    uint32_t total = pending_len + len;
    if (total > sizeof(buf)) {
        fail(BL_FLASH_ERR_OVERFLOW);
        return false;
    }
    memcpy(buf, pending, pending_len);
    memcpy(buf + pending_len, data, len);

    /* The flash primitives skip matching 0xFF, leaving this granule erased. */
    if (!deferred_vt_armed && offset == 0 && total >= BL_WRITE_ALIGN) {
        memcpy(deferred_vt, buf, BL_WRITE_ALIGN);
        memset(buf, 0xFF, BL_WRITE_ALIGN);
        deferred_vt_armed = true;
    }

    uint32_t aligned = total & ~(BL_WRITE_ALIGN - 1U);
    uint32_t tail    = total - aligned;

    if (aligned > 0 && !flush_aligned(buf, aligned)) {
        return false;
    }
    memcpy(pending, buf + aligned, tail);
    pending_len = (uint8_t)tail;
    bytes_written = (cursor - APP_START_ADDRESS);
    return true;
}

bool bootloader_flash_finalize(void)
{
    if (!started) {
        fail(BL_FLASH_ERR_FINALIZE);
        return false;
    }
    if (failed) {
        return false;
    }
    if (pending_len > 0) {
        uint8_t padded[BL_WRITE_ALIGN];
        memset(padded, 0xFF, sizeof(padded));
        memcpy(padded, pending, pending_len);
        if (!flush_aligned(padded, BL_WRITE_ALIGN)) {
            return false;
        }
        pending_len = 0;
    }

    /* Commit the vector table only after every other write succeeds. */
    if (deferred_vt_armed) {
        if (!stm32_flash_write(APP_START_ADDRESS, deferred_vt, BL_WRITE_ALIGN)) {
            fail(BL_FLASH_ERR_WRITE);
            return false;
        }
        deferred_vt_armed = false;
    }

    bytes_written = (cursor - APP_START_ADDRESS);
    started = false;
    return true;
}

bool bootloader_flash_failed(void)
{
    return failed;
}

uint32_t bootloader_flash_bytes_written(void)
{
    return bytes_written;
}

bl_flash_err_t bootloader_flash_last_error(void)
{
    return last_err;
}

void bootloader_flash_get_diag(struct bl_flash_diag *out)
{
    if (out != NULL) {
        *out = diag;
    }
}

void bootloader_flash_write_cb(uint32_t offset, const uint8_t *data, uint16_t len)
{
    bootloader_flash_write(offset, data, len);
}
