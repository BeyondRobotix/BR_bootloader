#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* A nonzero preferred_node_id skips DNA; iwdg_reset is logged at startup. */
void bl_can_start(uint8_t preferred_node_id, bool iwdg_reset);
void bl_can_cycle(void);

/* Flash remains untouched until the server's first successful reply. */
void bl_can_start_handover_download(uint8_t server_node_id, const char *path);

/* EOF received and all flash operations succeeded. */
bool bl_can_update_done(void);

/* Download active or EOF received; keeps the fast LED pattern until the app boots. */
bool bl_can_update_in_progress(void);

/* Any flash operation failed since the last begin() attempt. */
bool bl_can_update_failed(void);

#ifdef __cplusplus
}
#endif
