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

/* uavcan.protocol.debug.LogLevel values. */
#define LOG_DEBUG   0
#define LOG_INFO    1
#define LOG_WARNING 2
#define LOG_ERROR   3

#define BL_FIRST_REPLY_TIMEOUT_MS 5000
#define BL_STALL_TIMEOUT_MS 10000

enum class DownloadPhase {
    Idle,
    AwaitingFirstReply,
    Receiving,
    EofReceived,
};

struct DownloadState {
    DownloadPhase phase = DownloadPhase::Idle;
    uint8_t server_node_id = 0;
    uint32_t started_ms = 0;
    uint32_t last_progress_ms = 0;
    uint32_t last_progress_log_kb = 0;
    bl_flash_err_t logged_flash_error = BL_FLASH_OK;
    char path[sizeof(((struct app_bootloader_comms *)0)->path)] = {};
};

static DroneCAN g_dronecan;
static DownloadState g_download;

static void bl_logf(uint8_t level, const char *fmt, ...)
{
    char msg[80];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    g_dronecan.debug(msg, level);
}

/* A reset must hold in the bootloader without automatically retrying the image. */
static void mirror_bootcom(uint8_t server, const char *path, uint16_t plen)
{
    volatile struct app_bootloader_comms *bc = bootcom();
    bc->magic          = APP_BOOTLOADER_COMMS_MAGIC;
    bc->reserved       = BOOTCOM_HANDOVER_USED;
    bc->server_node_id = server;
    bc->my_node_id     = canardGetLocalNodeID(&g_dronecan.canard);
    memcpy((void *)bc->path, path, plen);
    bc->path[plen] = 0;
    bootcom_commit();
}

static void log_flash_begin_failure(void)
{
    bl_logf(LOG_ERROR, "flash begin FAILED err=%u",
            (unsigned)bootloader_flash_last_error());
    struct bl_flash_diag d;
    bootloader_flash_get_diag(&d);
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

static void abandon_download(uint8_t health)
{
    g_dronecan.firmware_download_finish();
    g_download.phase = DownloadPhase::Idle;
    volatile struct app_bootloader_comms *bc = bootcom();
    bc->magic    = 0;
    bc->reserved = 0;
    bootcom_commit();
    g_dronecan.set_node_status_override(
        health,
        UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE,
        0);
}

static void start_download(uint8_t server_node_id, const char *path, uint16_t plen)
{
    g_download.phase = DownloadPhase::AwaitingFirstReply;
    g_download.last_progress_log_kb = 0;
    g_download.logged_flash_error = BL_FLASH_OK;
    memcpy(g_download.path, path, plen);
    g_download.path[plen] = 0;
    g_download.server_node_id = server_node_id;
    g_download.started_ms = millis();
    g_download.last_progress_ms = g_download.started_ms;
    /* Let the library report SOFTWARE_UPDATE and progress in KB. */
    g_dronecan.clear_node_status_override();
    g_dronecan.begin_firmware_download(server_node_id, g_download.path);
    mirror_bootcom(server_node_id, g_download.path, plen);
}

void bl_can_start_handover_download(uint8_t server_node_id, const char *path)
{
    uint16_t plen = (uint16_t)strnlen(path, sizeof(g_download.path) - 1);
    bl_logf(LOG_INFO, "hand-over: reading from node %u plen=%u",
            (unsigned)server_node_id, (unsigned)plen);
    start_download(server_node_id, path, plen);
}

static void handle_begin_firmware_update_server(CanardInstance *ins, CanardRxTransfer *transfer)
{
    uavcan_protocol_file_BeginFirmwareUpdateRequest req;
    if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req)) {
        bl_logf(LOG_ERROR, "BeginFirmwareUpdate: decode failed");
        return;
    }

    uint8_t server = req.source_node_id ? req.source_node_id : transfer->source_node_id;

    char path[sizeof(g_download.path)];
    uint16_t plen = req.image_file_remote_path.path.len;
    if (plen > sizeof(path) - 1) {
        plen = sizeof(path) - 1;
    }
    memcpy(path, req.image_file_remote_path.path.data, plen);
    path[plen] = 0;

    const bool repeat = g_dronecan.firmware_download_active() &&
                        server == g_download.server_node_id && strcmp(path, g_download.path) == 0;

    bl_logf(LOG_INFO, "BeginFirmwareUpdate src=%u plen=%u%s",
            (unsigned)transfer->source_node_id, (unsigned)plen,
            repeat ? " (already in progress)" : "");

    uavcan_protocol_file_BeginFirmwareUpdateResponse reply{};
    reply.error = repeat
        ? UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_IN_PROGRESS
        : UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;
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

    if (!repeat) {
        start_download(server, path, plen);
    }
}

static void handle_file_read_response(CanardRxTransfer *transfer)
{
    uavcan_protocol_file_ReadResponse response;
    const bool decode_ok = uavcan_protocol_file_ReadResponse_decode(transfer, &response) == 0;
    const bool read_ok = decode_ok && response.error.value == UAVCAN_PROTOCOL_FILE_ERROR_OK;

    /* Preserve the old app until the server's first successful reply. */
    if (g_download.phase == DownloadPhase::AwaitingFirstReply) {
        if (!decode_ok || transfer->source_node_id != g_download.server_node_id) {
            return;
        }
        if (!read_ok) {
            bl_logf(LOG_WARNING, "download: server error %d, waiting for BeginFirmwareUpdate",
                    (int)response.error.value);
            abandon_download(UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK);
            return;
        }
        g_download.phase = DownloadPhase::Receiving;
        if (!bootloader_flash_begin()) {
            log_flash_begin_failure();
            g_dronecan.firmware_download_finish();
            g_download.phase = DownloadPhase::Idle;
            return;
        }
        bl_logf(LOG_INFO, "download: server answered, flashing");
    }

    const bool was_active = g_dronecan.firmware_download_active();
    const uint32_t offset_before = g_dronecan.firmware_download_offset();

    g_dronecan.handle_file_read_response(transfer);

    const uint32_t offset_after = g_dronecan.firmware_download_offset();
    if (offset_after != offset_before) {
        g_download.last_progress_ms = millis();
    }

    if (decode_ok && !read_ok && was_active && !g_dronecan.firmware_download_active()) {
        bl_logf(LOG_ERROR, "download FAILED: server error %d at %u",
                (int)response.error.value, (unsigned)offset_before);
        abandon_download(UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR);
        return;
    }

    const uint32_t progress_kb = offset_after / 1024;
    if (progress_kb >= g_download.last_progress_log_kb + 8) {
        g_download.last_progress_log_kb = progress_kb;
        bl_logf(LOG_DEBUG, "rx %u KB written=%u",
                (unsigned)progress_kb,
                (unsigned)bootloader_flash_bytes_written());
    }

    /* Only accepted short chunks signal EOF; ignored replies must not commit an image. */
    const bool accepted = read_ok && (offset_after - offset_before) == response.data.len;
    if (accepted && response.data.len < 256) {
        const bool finalized = bootloader_flash_finalize();
        g_download.phase = DownloadPhase::EofReceived;
        g_dronecan.firmware_download_finish();
        if (finalized && !bootloader_flash_failed()) {
            bl_logf(LOG_INFO, "download done bytes=%u",
                    (unsigned)bootloader_flash_bytes_written());
        } else {
            bl_logf(LOG_ERROR, "download FAILED err=%u after=%u",
                    (unsigned)bootloader_flash_last_error(),
                    (unsigned)offset_after);
        }
    }
}

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
            handle_file_read_response(transfer);
        }
        break;
    }
}

void bl_can_start(uint8_t preferred_node_id, bool iwdg_reset)
{
    g_dronecan.version_major = 0;
    g_dronecan.version_minor = 4;

    g_dronecan.set_firmware_write_callback(bootloader_flash_write_cb);
    /* main.cpp owns the LED patterns. */
    g_dronecan.set_cycle_led_pin(-1);

    g_dronecan.init(bl_on_transfer_received,
                    DroneCANshouldAcceptTransfer,
                    BOOTLOADER_NODE_NAME,
                    preferred_node_id);

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
    if (g_download.phase == DownloadPhase::AwaitingFirstReply &&
        millis() - g_download.started_ms > BL_FIRST_REPLY_TIMEOUT_MS) {
        bl_logf(LOG_WARNING, "download: no reply from node %u, waiting for BeginFirmwareUpdate",
                (unsigned)g_download.server_node_id);
        abandon_download(UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK);
    }
    if (g_download.phase != DownloadPhase::AwaitingFirstReply &&
        g_dronecan.firmware_download_active() &&
        millis() - g_download.last_progress_ms > BL_STALL_TIMEOUT_MS) {
        bl_logf(LOG_ERROR, "download: stalled at %u, waiting for BeginFirmwareUpdate",
                (unsigned)g_dronecan.firmware_download_offset());
        abandon_download(UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR);
    }
    if (g_dronecan.firmware_download_active()) {
        g_dronecan.send_firmware_read();
    }

    /* Log outside the flash callback to avoid re-entering the canard TX queue. */
    bl_flash_err_t err = bootloader_flash_last_error();
    if (err != BL_FLASH_OK && err != g_download.logged_flash_error) {
        g_download.logged_flash_error = err;
        bl_logf(LOG_ERROR, "flash op FAILED err=%u written=%u",
                (unsigned)err,
                (unsigned)bootloader_flash_bytes_written());

        g_dronecan.set_node_status_override(
            UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR,
            UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE,
            (uint16_t)err);
    }
}

bool bl_can_update_done(void)
{
    return g_download.phase == DownloadPhase::EofReceived && !bootloader_flash_failed();
}

bool bl_can_update_failed(void)
{
    return bootloader_flash_failed();
}

bool bl_can_update_in_progress(void)
{
    return g_dronecan.firmware_download_active() || g_download.phase == DownloadPhase::EofReceived;
}
