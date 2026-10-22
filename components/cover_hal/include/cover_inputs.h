/*
 * Polled digital inputs (buttons, limit switches, obstacle, driver fault).
 * Polling from the control task keeps timing deterministic and avoids ISR
 * bounce storms; debouncing is done by the caller (debounce.h).
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    COVER_IN_BTN_OPEN = 0,
    COVER_IN_BTN_CLOSE,
    COVER_IN_BTN_STOP,
    COVER_IN_BTN_FUNC,       /* commissioning window / factory reset */
    COVER_IN_LIMIT_OPEN,
    COVER_IN_LIMIT_CLOSED,
    COVER_IN_OBSTACLE,
    COVER_IN_DRIVER_FAULT,
    COVER_IN_COUNT,
} cover_input_id_t;

typedef struct {
    int gpio;                /* -1 = not present */
    bool active_high;
    bool internal_pull;      /* enable internal pull towards the INACTIVE level */
} cover_input_pin_t;

typedef struct {
    cover_input_pin_t pin[COVER_IN_COUNT];
} cover_inputs_config_t;

esp_err_t cover_inputs_init(const cover_inputs_config_t *cfg);
bool cover_inputs_present(cover_input_id_t id);
/** Raw (not debounced) active state. Absent inputs read false. */
bool cover_inputs_read(cover_input_id_t id);
const char *cover_input_name(cover_input_id_t id);

#ifdef __cplusplus
}
#endif
