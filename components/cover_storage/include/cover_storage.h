/*
 * Persistent calibration and last-stop position (NVS namespace "cover").
 *
 * Writes are asynchronous (own low priority task, latest value wins) so the
 * control loop never waits for a flash erase. Writes happen only when:
 *   - calibration changes;
 *   - a run starts (record marked "moving", only if not already marked);
 *   - a run stops (position stored, only if different from the stored one).
 * i.e. at most two small writes per movement, never periodic writes.
 *
 * A record marked "moving" at boot means power was lost (or a reset happened)
 * during a run: the position is then reported as unknown.
 *
 * Matter's own data (fabrics, Wi-Fi) lives in other namespaces; a Matter
 * factory reset does not erase this namespace unless the application asks.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cover_sm.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool moving;
    bool valid;      /* false: position was unknown at the last stop */
    uint16_t pos;
} cover_pos_record_t;

esp_err_t cover_storage_init(void);

/** @return true and fill `out` if a valid calibration is stored. */
bool cover_storage_load_calib(cover_calib_t *out);
/** @return true and fill `out` if a position record exists. */
bool cover_storage_load_pos(cover_pos_record_t *out);

/** Queue a calibration write; NULL erases the stored calibration. */
void cover_storage_save_calib(const cover_calib_t *calib);
/** Queue a position record write (skipped if identical to the stored one). */
void cover_storage_save_pos(const cover_pos_record_t *rec);

/** Synchronously erase everything in the namespace (factory reset path). */
esp_err_t cover_storage_erase_all(void);

#ifdef __cplusplus
}
#endif
