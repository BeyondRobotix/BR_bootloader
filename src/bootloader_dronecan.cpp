#include "bootloader_dronecan.h"
#include "bootloader_flash.h"
#include "bootloader_comms.h"
#include "board_config.h"

#include <dronecan.h>
#include <dronecan_msgs.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define BOOTLOADER_NODE_NAME "org.beyondrobotix.bootloader"

/* DroneCAN LogMessage level constants -- match uavcan.protocol.debug.LogLevel. */
#define LOG_DEBUG   0
#define LOG_INFO    1
#define LOG_WARNING 2
#define LOG_ERROR   3

/* How long a hand-over download waits for the server's first reply before
   giving up and waiting for a BeginFirmwareUpdate instead. The FileRead
   client retries every 750 ms, so this is several attempts. */
#define BL_HANDOVER_TIMEOUT_MS 5000

static DroneCAN g_dronecan;
static bool     g_last_chunk_seen = false;
static uint32_t g_last_chunk_offset = 0;
static bl_flash_err_t g_logged_err = BL_FLASH_OK;
static uint32_t g_last_progress_log_kb = 0;

/* A hand-over download that hasn't had its first reply yet: flash is not
   armed, and the old app is still intact. */
static bool     g_handover_pending = false;
static uint8_t  g_handover_server = 0;
static uint32_t g_handover_started_ms = 0;

/* Format a short message and broadcast via DroneCAN debug.LogMessage. The
   wire-level limit is ~90 bytes; we cap the buffer well below that. */
static void bl_logf(uint8_t level, const char *fmt, ...)
{
    char msg[80];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    g_dronecan.debug(msg, level);
}

/* Reset per-download bookkeeping. */
static void reset_download_state(void)
{
    g_last_chunk_seen      = false;
    g_last_chunk_offset    = 0;
    g_last_progress_log_kb = 0;
    g_logged_err           = BL_FLASH_OK;
}

/* Record the download in BOOTCOM so a watchdog/reset mid-update stays in
   the bootloader, marked used so that boot doesn't start it again. */
static void mirror_bootcom(uint8_t server, const char *path, uint16_t plen)
{
    volatile struct app_bootloader_comms *bc = bootcom();
    bc->magic          = APP_BOOTLOADER_COMMS_MAGIC;
    bc->reserved       = BOOTCOM_HANDOVER_USED;
    bc->server_node_id = server;
    bc->my_node_id     = canardGetLocalNodeID(&g_dronecan.canard);
    memcpy((void *)bc->path, path, plen);
    bc->path[plen] = 0;
}

/* Log why flash couldn't be armed. */
static void log_flash_begin_failure(void)
{
    bl_logf(LOG_ERROR, "flash begin FAILED err=%u",
            (unsigned)bootloader_flash_last_error());
    struct bl_flash_diag d;
    bootloader_flash_get_diag(&d);
    /* Top byte of ACR is repurposed as a path tag: 0xAA=raw worked,
       0xBB=HAL worked, 0xFF=neither (and that's why we're here). */
    bl_logf(LOG_ERROR, "FLASH sr=%08lx cr=%08lx",
            (unsigned long)d.sr, (unsigned long)d.cr);
    bl_logf(LOG_ERROR, "OPTR=%08lx WRP1A=%08lx WRP1B=%08lx",
            (unsigned long)d.optr,
            (unsigned long)d.wrp1ar,
            (unsigned long)d.wrp1br);
    bl_logf(LOG_ERROR, "PCROP1S=%08lx PCROP1E=%08lx ACR=%08lx",
            (unsigned long)d.pcrop1sr,
            (unsigned long)d.pcrop1er,
            (unsigned long)d.acr);
}

/* Give up on a download and show the bootloader as waiting again. */
static void abandon_download(void)
{
    g_dronecan.firmware_download_finish();
    g_handover_pending = false;
    volatile struct app_bootloader_comms *bc = bootcom();
    bc->magic    = 0;
    bc->reserved = 0;
    g_dronecan.set_node_status_override(
        UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK,
        UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE,
        0);
}

void bl_can_start_handover_download(uint8_t server_node_id, const char *path)
{
    uint16_t plen = (uint16_t)strnlen(path, sizeof(bootcom()->path) - 1);
    bl_logf(LOG_INFO, "hand-over: reading from node %u plen=%u",
            (unsigned)server_node_id, (unsigned)plen);
    reset_download_state();
    g_handover_pending    = true;
    g_handover_server     = server_node_id;
    g_handover_started_ms = millis();
    g_dronecan.clear_node_status_override();
    g_dronecan.begin_firmware_download(server_node_id, path);
    mirror_bootcom(server_node_id, path, plen);
}

/*
 * BeginFirmwareUpdate handler (server role).
 *
 * The app-side handler in dronecan.cpp writes BOOTCOM and resets. In the
 * bootloader we are *already* the responder, so we:
 *   - decode the request,
 *   - reply OK,
 *   - prime the lib's FileRead client with (server_node_id, image_path).
 *
 * Subsequent bl_can_cycle() calls drive send_firmware_read(), and chunks
 * are persisted via the firmware_write_cb registered in bl_can_start().
 */
static void handle_begin_firmware_update_server(CanardInstance *ins, CanardRxTransfer *transfer)
{
    uavcan_protocol_file_BeginFirmwareUpdateRequest req;
    if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req)) {
        bl_logf(LOG_ERROR, "BeginFirmwareUpdate: decode failed");
        return;
    }

    bl_logf(LOG_INFO, "BeginFirmwareUpdate src=%u plen=%u",
            (unsigned)transfer->source_node_id,
            (unsigned)req.image_file_remote_path.path.len);

    /* An explicit request replaces any hand-over download still waiting for
       its first reply. */
    g_handover_pending = false;

    /* Arm flash state first; if the page erase fails we surface the error in
       the response instead of silently dropping every chunk that follows. */
    bool flash_ready = bootloader_flash_begin();
    reset_download_state();

    if (flash_ready) {
        /* Drop MAINTENANCE override so the lib's SOFTWARE_UPDATE mode +
           VSSC=KB progress flows through NodeStatus during the download. */
        g_dronecan.clear_node_status_override();
        bl_logf(LOG_INFO, "flash begin OK, starting download");
    } else {
        log_flash_begin_failure();
    }

    uavcan_protocol_file_BeginFirmwareUpdateResponse reply{};
    reply.error = flash_ready
        ? UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK
        : UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_UNKNOWN;
    uint8_t buf[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
    uint16_t len = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&reply, buf);

    CanardTxTransfer reply_xfer = {
        .transfer_type       = CanardTransferTypeResponse,
        .data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
        .data_type_id        = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
        .inout_transfer_id   = &transfer->transfer_id,
        .priority            = transfer->priority,
        .payload             = buf,
        .payload_len         = len,
    };
    canardRequestOrRespondObj(ins, transfer->source_node_id, &reply_xfer);

    /* If flash erase failed, don't start the FileRead client -- the bootloader
       will stay alive on the bus so the operator can see it and retry. */
    if (!flash_ready) {
        return;
    }

    /* Per req.source_node_id semantics, the server may delegate to a third
       node; fall back to the request originator if not specified. */
    uint8_t server = req.source_node_id ? req.source_node_id : transfer->source_node_id;

    char path[201];
    uint16_t plen = req.image_file_remote_path.path.len;
    if (plen > sizeof(path) - 1) {
        plen = sizeof(path) - 1;
    }
    memcpy(path, req.image_file_remote_path.path.data, plen);
    path[plen] = 0;

    g_dronecan.begin_firmware_download(server, path);
    mirror_bootcom(server, path, plen);
}

/*
 * Custom dispatcher. Routes the small set of services the bootloader cares
 * about; everything else is ignored (lighter than pulling in the full app-side
 * dispatcher's param/restart handlers).
 */
static void bl_on_transfer_received(CanardInstance *ins, CanardRxTransfer *transfer)
{
    switch (transfer->transfer_type) {
    case CanardTransferTypeBroadcast:
        if (transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID) {
            g_dronecan.handle_DNA_Allocation(transfer);
        }
        break;

    case CanardTransferTypeRequest:
        if (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID) {
            g_dronecan.handle_GetNodeInfo(transfer);
        } else if (transfer->data_type_id == UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID) {
            handle_begin_firmware_update_server(ins, transfer);
        }
        break;

    case CanardTransferTypeResponse:
        if (transfer->data_type_id == UAVCAN_PROTOCOL_FILE_READ_ID) {
            uavcan_protocol_file_ReadResponse peek;
            bool decode_ok = (uavcan_protocol_file_ReadResponse_decode(transfer, &peek) == 0);
            const bool read_ok = decode_ok && peek.error.value == UAVCAN_PROTOCOL_FILE_ERROR_OK;

            /* First reply to a hand-over download: only now erase the old
               app, so a server that never answers costs nothing. */
            if (g_handover_pending) {
                if (!decode_ok || transfer->source_node_id != g_handover_server) {
                    break;
                }
                if (!read_ok) {
                    bl_logf(LOG_WARNING, "hand-over: server error %d, waiting for BeginFirmwareUpdate",
                            (int)peek.error.value);
                    abandon_download();
                    break;
                }
                g_handover_pending = false;
                if (!bootloader_flash_begin()) {
                    log_flash_begin_failure();
                    g_dronecan.firmware_download_finish();
                    break;
                }
                bl_logf(LOG_INFO, "hand-over: server answered, flashing");
            }

            const bool was_active = g_dronecan.firmware_download_active();
            uint32_t before = g_dronecan.firmware_download_offset();

            g_dronecan.handle_file_read_response(transfer);

            uint32_t after = g_dronecan.firmware_download_offset();

            /* The server reported an error and the client gave up: the old
               app may already be partly erased, so say so rather than sit
               in SOFTWARE_UPDATE looking busy. */
            if (decode_ok && !read_ok && was_active && !g_dronecan.firmware_download_active()) {
                bl_logf(LOG_ERROR, "download FAILED: server error %d at %u",
                        (int)peek.error.value, (unsigned)before);
                bootcom()->magic = 0;
                g_dronecan.set_node_status_override(
                    UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR,
                    UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE,
                    0);
                break;
            }

            /* progress log every 8 KB so the bus isn't drowned in chatter */
            uint32_t kb = after / 1024;
            if (kb >= g_last_progress_log_kb + 8) {
                g_last_progress_log_kb = kb;
                bl_logf(LOG_DEBUG, "rx %u KB written=%u",
                        (unsigned)kb,
                        (unsigned)bootloader_flash_bytes_written());
            }

            /* A short chunk (< 256B) signals EOF in DroneCAN FileRead, but
               only one the client accepted: a reply it ignored (a late
               answer to a retried read, or another node's) leaves the offset
               where it was, and taking that as EOF would commit a partial
               image. */
            const bool accepted = read_ok && (after - before) == peek.data.len;
            if (accepted && peek.data.len < 256) {
                bool fin_ok = bootloader_flash_finalize();
                g_last_chunk_offset = after;
                g_last_chunk_seen   = true;
                g_dronecan.firmware_download_finish();
                if (fin_ok && !bootloader_flash_failed()) {
                    bl_logf(LOG_INFO, "download done bytes=%u",
                            (unsigned)bootloader_flash_bytes_written());
                } else {
                    bl_logf(LOG_ERROR, "download FAILED err=%u after=%u",
                            (unsigned)bootloader_flash_last_error(),
                            (unsigned)after);
                }
            }
        }
        break;
    }
}

/* libcanard adapter -> our dispatcher, using user_reference for context. */
static void canard_dispatch(CanardInstance *ins, CanardRxTransfer *transfer)
{
    bl_on_transfer_received(ins, transfer);
}

void bl_can_start(uint8_t preferred_node_id, bool iwdg_reset)
{
    g_dronecan.version_major = 0;
    g_dronecan.version_minor = 4;  /* Phase 4 */

    g_dronecan.set_firmware_write_callback(bootloader_flash_write_cb);
    /* Take over the LED so main.cpp can render heartbeat / progress / error
       patterns without fighting the lib's 1 Hz cycle-toggle. */
    g_dronecan.set_cycle_led_pin(-1);

    g_dronecan.init(canard_dispatch,
                    DroneCANshouldAcceptTransfer,
                    BOOTLOADER_NODE_NAME,
                    preferred_node_id);

    /* Flag the bootloader as "not the app" in the GCS node list as soon as
       NodeStatus starts going out. Cleared when an update download begins
       so the lib's SOFTWARE_UPDATE / VSSC-progress reporting takes over. */
    g_dronecan.set_node_status_override(
        UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK,
        UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE,
        0);

    bl_logf(LOG_INFO, "bootloader up, preferred_nid=%u%s",
            (unsigned)preferred_node_id,
            iwdg_reset ? " (after IWDG reset)" : "");
}

void bl_can_cycle(void)
{
    g_dronecan.cycle();
    if (g_handover_pending && millis() - g_handover_started_ms > BL_HANDOVER_TIMEOUT_MS) {
        bl_logf(LOG_WARNING, "hand-over: no reply from node %u, waiting for BeginFirmwareUpdate",
                (unsigned)g_handover_server);
        abandon_download();
    }
    if (g_dronecan.firmware_download_active()) {
        g_dronecan.send_firmware_read();
    }

    /* Report a flash failure the first time we observe it. The failure may
       have been set deep inside the write callback (handle_file_read_response
       chain) where calling debug() would re-enter the canard tx queue --
       better to surface it here, after the response transfer is fully drained. */
    bl_flash_err_t err = bootloader_flash_last_error();
    if (err != BL_FLASH_OK && err != g_logged_err) {
        g_logged_err = err;
        bl_logf(LOG_ERROR, "flash op FAILED err=%u written=%u",
                (unsigned)err,
                (unsigned)bootloader_flash_bytes_written());

        /* Flag the failure in NodeStatus so the GCS node list shows ERROR
           health and MAINTENANCE mode instead of looking healthy while
           silently refusing to jump. VSSC carries the error code. */
        g_dronecan.set_node_status_override(
            UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR,
            UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE,
            (uint16_t)err);
    }
}

bool bl_can_update_done(void)
{
    /* "Done" only if we saw EOF *and* every flash op along the way succeeded.
       If a write was dropped, we'd otherwise jump to the still-intact old
       vector table and the old firmware would run, masking the failure. */
    return g_last_chunk_seen && !bootloader_flash_failed();
}

bool bl_can_update_failed(void)
{
    return bootloader_flash_failed();
}

bool bl_can_update_in_progress(void)
{
    return g_dronecan.firmware_download_active() || g_last_chunk_seen;
}
