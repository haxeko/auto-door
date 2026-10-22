#include "cover_inputs.h"

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "inputs";
static cover_inputs_config_t s_cfg;

static const char *const k_names[COVER_IN_COUNT] = {
    "btn_open", "btn_close", "btn_stop", "btn_func", "limit_open", "limit_closed", "obstacle", "driver_fault",
};

const char *cover_input_name(cover_input_id_t id)
{
    return id < COVER_IN_COUNT ? k_names[id] : "?";
}

esp_err_t cover_inputs_init(const cover_inputs_config_t *cfg)
{
    s_cfg = *cfg;
    for (int i = 0; i < COVER_IN_COUNT; i++) {
        const cover_input_pin_t *p = &s_cfg.pin[i];
        if (p->gpio < 0) {
            continue;
        }
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << p->gpio,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = (p->internal_pull && !p->active_high) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pull_down_en = (p->internal_pull && p->active_high) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&io);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: GPIO%d config failed: %s", k_names[i], p->gpio, esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "%-12s GPIO%-2d active-%s%s", k_names[i], p->gpio, p->active_high ? "high" : "low",
                 p->internal_pull ? " (internal pull)" : "");
    }
    return ESP_OK;
}

bool cover_inputs_present(cover_input_id_t id)
{
    return id < COVER_IN_COUNT && s_cfg.pin[id].gpio >= 0;
}

bool cover_inputs_read(cover_input_id_t id)
{
    if (!cover_inputs_present(id)) {
        return false;
    }
    const cover_input_pin_t *p = &s_cfg.pin[id];
    return gpio_get_level(p->gpio) == (p->active_high ? 1 : 0);
}
