#include "pin_guard.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "pin_guard";

typedef enum { RES_NONE, RES_STRAPPING, RES_FLASH, RES_USB, RES_CONSOLE } reserve_t;

static reserve_t reserved(int gpio)
{
#if CONFIG_IDF_TARGET_ESP32C6
    switch (gpio) {
    case 4: case 5: case 8: case 9: case 15:
        return RES_STRAPPING;
    case 12: case 13:
        return RES_USB;
    case 16: case 17:
        return RES_CONSOLE;
    default:
        break;
    }
    if (gpio >= 24 && gpio <= 30) {
        return RES_FLASH;
    }
    return RES_NONE;
#else
#warning "pin_guard: reserved pin table only defined for ESP32-C6"
    (void)gpio;
    return RES_NONE;
#endif
}

static const char *reserve_str(reserve_t r)
{
    switch (r) {
    case RES_STRAPPING: return "strapping pin";
    case RES_FLASH:     return "SPI flash pin";
    case RES_USB:       return "USB-JTAG pin";
    case RES_CONSOLE:   return "UART0 console pin";
    default:            return "";
    }
}

esp_err_t pin_guard_check(int gpio, pin_use_t use, const char *name, bool allow_strapping_input)
{
    if (gpio < 0) {
        return ESP_OK; /* not used */
    }
    if (use == PIN_USE_OUTPUT ? !GPIO_IS_VALID_OUTPUT_GPIO(gpio) : !GPIO_IS_VALID_GPIO(gpio)) {
        ESP_LOGE(TAG, "%s: GPIO%d is not a valid %s pin", name, gpio, use == PIN_USE_OUTPUT ? "output" : "input");
        return ESP_ERR_INVALID_ARG;
    }
    reserve_t r = reserved(gpio);
    if (r == RES_NONE) {
        return ESP_OK;
    }
    if (r == RES_STRAPPING && use == PIN_USE_INPUT && allow_strapping_input) {
        ESP_LOGW(TAG, "%s: GPIO%d is a strapping pin - it must not be held active during reset", name, gpio);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "%s: GPIO%d is a %s on ESP32-C6 and is refused", name, gpio, reserve_str(r));
    return ESP_ERR_INVALID_ARG;
}

esp_err_t pin_guard_check_unique(const int *pins, const char *const *names, int count)
{
    for (int i = 0; i < count; i++) {
        if (pins[i] < 0) {
            continue;
        }
        for (int j = i + 1; j < count; j++) {
            if (pins[i] == pins[j]) {
                ESP_LOGE(TAG, "GPIO%d used by both %s and %s", pins[i], names[i], names[j]);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    return ESP_OK;
}
