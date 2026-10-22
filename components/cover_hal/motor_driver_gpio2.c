/*
 * Two-output direction driver ("gpio2").
 *
 * Power stage requirements (see README):
 *  - one input per direction, both inactive = motor off and safe;
 *  - external pull resistors holding both inputs INACTIVE while the ESP32-C6 is
 *    in reset / bootloader (GPIOs are not driven then);
 *  - preferably a hardware interlock (relay changeover chain or H-bridge with
 *    shoot-through protection) - this firmware is a second line of defence only.
 */
#include "motor_driver.h"

#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "motor_gpio2";

typedef struct {
    motor_driver_t base;
    motor_gpio2_config_t cfg;
    motor_interlock_t il;
    bool fault;
} gpio2_ctx_t;

static inline int level(const motor_gpio2_config_t *cfg, bool active)
{
    return (active == cfg->active_high) ? 1 : 0;
}

void motor_driver_gpio2_all_off(const motor_gpio2_config_t *cfg)
{
    gpio_set_level(cfg->gpio_open, level(cfg, false));
    gpio_set_level(cfg->gpio_close, level(cfg, false));
}

static bool readback_ok(const gpio2_ctx_t *c, cover_dir_t dir)
{
    int lo = gpio_get_level(c->cfg.gpio_open);
    int lc = gpio_get_level(c->cfg.gpio_close);
    return lo == level(&c->cfg, dir == COVER_DIR_OPEN) && lc == level(&c->cfg, dir == COVER_DIR_CLOSE);
}

static esp_err_t gpio2_set(motor_driver_t *self, cover_dir_t dir, uint32_t now_ms)
{
    gpio2_ctx_t *c = (gpio2_ctx_t *)self->ctx;
    if (dir == c->il.current) {
        return ESP_OK;
    }
    /* Always break before make. */
    motor_driver_gpio2_all_off(&c->cfg);
    motor_interlock_commit(&c->il, COVER_DIR_NONE, now_ms);
    if (!readback_ok(c, COVER_DIR_NONE)) {
        c->fault = true;
        ESP_LOGE(TAG, "read back: outputs not inactive after switch off");
        return ESP_FAIL;
    }
    if (dir == COVER_DIR_NONE) {
        return ESP_OK;
    }
    if (c->fault) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = motor_interlock_check(&c->il, dir, now_ms);
    if (err != ESP_OK) {
        c->fault = true;
        ESP_LOGE(TAG, "interlock refused %s (dead time / reversal guard)", dir == COVER_DIR_OPEN ? "OPEN" : "CLOSE");
        return err;
    }
    gpio_set_level(dir == COVER_DIR_OPEN ? c->cfg.gpio_open : c->cfg.gpio_close, level(&c->cfg, true));
    if (!readback_ok(c, dir)) {
        motor_driver_gpio2_all_off(&c->cfg);
        c->fault = true;
        ESP_LOGE(TAG, "read back mismatch, outputs forced off");
        return ESP_FAIL;
    }
    motor_interlock_commit(&c->il, dir, now_ms);
    return ESP_OK;
}

static bool gpio2_output_fault(motor_driver_t *self)
{
    return ((gpio2_ctx_t *)self->ctx)->fault;
}

static esp_err_t setup_pin(const motor_gpio2_config_t *cfg, int gpio)
{
    /* 1) latch the inactive level in the output register, 2) enable the output. */
    esp_err_t err = gpio_set_level(gpio, level(cfg, false));
    if (err != ESP_OK) {
        return err;
    }
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT_OUTPUT, /* input enabled for read back */
        /* weak internal pull towards inactive, in addition to the external one */
        .pull_up_en = cfg->active_high ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE,
        .pull_down_en = cfg->active_high ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }
    return gpio_set_level(gpio, level(cfg, false));
}

esp_err_t motor_driver_gpio2_create(const motor_gpio2_config_t *cfg, uint32_t now_ms, motor_driver_t **out)
{
    if (!cfg || !out || cfg->gpio_open < 0 || cfg->gpio_close < 0 || cfg->gpio_open == cfg->gpio_close) {
        return ESP_ERR_INVALID_ARG;
    }
    gpio2_ctx_t *c = calloc(1, sizeof(*c));
    if (!c) {
        return ESP_ERR_NO_MEM;
    }
    c->cfg = *cfg;
    esp_err_t err = setup_pin(cfg, cfg->gpio_open);
    if (err == ESP_OK) {
        err = setup_pin(cfg, cfg->gpio_close);
    }
    if (err != ESP_OK) {
        motor_driver_gpio2_all_off(cfg);
        free(c);
        return err;
    }
    motor_interlock_init(&c->il, cfg->deadtime_ms, now_ms);
    if (!readback_ok(c, COVER_DIR_NONE)) {
        ESP_LOGE(TAG, "outputs do not read back inactive - check wiring / pin choice");
        c->fault = true;
    }
    c->base.name = "gpio2";
    c->base.set = gpio2_set;
    c->base.output_fault = gpio2_output_fault;
    c->base.ctx = c;
    *out = &c->base;
    ESP_LOGI(TAG, "outputs OPEN=GPIO%d CLOSE=GPIO%d active-%s, dead time %u ms", cfg->gpio_open, cfg->gpio_close,
             cfg->active_high ? "high" : "low", (unsigned)cfg->deadtime_ms);
    return ESP_OK;
}
