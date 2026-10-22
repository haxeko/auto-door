/*
 * Refuses GPIOs that would conflict with boot, flash, USB-JTAG or the console.
 *
 * ESP32-C6 (ESP-IDF v5.5 GPIO summary):
 *  - strapping pins: GPIO4, GPIO5, GPIO8, GPIO9, GPIO15
 *  - SPI0/1 flash:   GPIO24..GPIO30
 *  - USB-JTAG:       GPIO12, GPIO13
 *  - UART0 console:  GPIO16 (TX), GPIO17 (RX) by default
 * Board specific parts (RGB LED, BOOT button ...) are NOT known here - check
 * your board schematic.
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PIN_USE_OUTPUT,      /* drives the power stage: strict */
    PIN_USE_INPUT,       /* strapping pins tolerated with a warning */
} pin_use_t;

/**
 * @param allow_strapping_input accept strapping pins for inputs (e.g. the BOOT
 *        button on GPIO9 of a dev kit). Never accepted for outputs.
 * @return ESP_OK, or ESP_ERR_INVALID_ARG if the pin must not be used.
 */
esp_err_t pin_guard_check(int gpio, pin_use_t use, const char *name, bool allow_strapping_input);

/** Detects the same GPIO configured twice. `pins` may contain -1 entries. */
esp_err_t pin_guard_check_unique(const int *pins, const char *const *names, int count);

#ifdef __cplusplus
}
#endif
