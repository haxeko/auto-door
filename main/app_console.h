#pragma once

#include <esp_err.h>

/** Registers `matter esp cover ...` (requires CONFIG_ENABLE_CHIP_SHELL). */
esp_err_t app_console_register();
