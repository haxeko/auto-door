#pragma once

#include <esp_err.h>

#include "cover_controller.h"

/**
 * Build the controller configuration from Kconfig and validate GPIOs.
 * @return ESP_ERR_INVALID_ARG if a pin is reserved / duplicated / missing.
 *         The motor must not be started in that case.
 */
esp_err_t app_config_build(cover_controller_config_t *cfg);
