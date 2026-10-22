/*
 * Kconfig -> controller configuration, with GPIO validation.
 */
#include "app_config.h"

#include <esp_log.h>

#include "pin_guard.h"
#include "sdkconfig.h"

static const char *TAG = "app_config";

#ifndef CONFIG_COVER_GPIO_OUT_OPEN
#define CONFIG_COVER_GPIO_OUT_OPEN -1
#endif
#ifndef CONFIG_COVER_GPIO_OUT_CLOSE
#define CONFIG_COVER_GPIO_OUT_CLOSE -1
#endif
#ifndef CONFIG_COVER_GPIO_LIMIT_OPEN
#define CONFIG_COVER_GPIO_LIMIT_OPEN -1
#endif
#ifndef CONFIG_COVER_GPIO_LIMIT_CLOSED
#define CONFIG_COVER_GPIO_LIMIT_CLOSED -1
#endif
#ifndef CONFIG_COVER_SIM_OPEN_MS
#define CONFIG_COVER_SIM_OPEN_MS 15000
#endif
#ifndef CONFIG_COVER_SIM_CLOSE_MS
#define CONFIG_COVER_SIM_CLOSE_MS 16000
#endif

#ifdef CONFIG_COVER_OUT_ACTIVE_HIGH
#define OUT_ACTIVE_HIGH true
#else
#define OUT_ACTIVE_HIGH false
#endif

#define BOOLCFG(x) (x + 0 != 0)

#ifndef CONFIG_COVER_SIMULATION
#define CONFIG_COVER_SIMULATION 0
#endif
#ifndef CONFIG_COVER_SIM_LIMIT_OPEN
#define CONFIG_COVER_SIM_LIMIT_OPEN 0
#endif
#ifndef CONFIG_COVER_SIM_LIMIT_CLOSED
#define CONFIG_COVER_SIM_LIMIT_CLOSED 0
#endif
#ifndef CONFIG_COVER_BTN_ACTIVE_HIGH
#define CONFIG_COVER_BTN_ACTIVE_HIGH 0
#endif
#ifndef CONFIG_COVER_BTN_INTERNAL_PULL
#define CONFIG_COVER_BTN_INTERNAL_PULL 0
#endif
#ifndef CONFIG_COVER_LIMIT_ACTIVE_HIGH
#define CONFIG_COVER_LIMIT_ACTIVE_HIGH 0
#endif
#ifndef CONFIG_COVER_LIMIT_INTERNAL_PULL
#define CONFIG_COVER_LIMIT_INTERNAL_PULL 0
#endif
#ifndef CONFIG_COVER_OBSTACLE_ACTIVE_HIGH
#define CONFIG_COVER_OBSTACLE_ACTIVE_HIGH 0
#endif
#ifndef CONFIG_COVER_DRIVER_FAULT_ACTIVE_HIGH
#define CONFIG_COVER_DRIVER_FAULT_ACTIVE_HIGH 0
#endif
#ifndef CONFIG_COVER_SENSOR_INTERNAL_PULL
#define CONFIG_COVER_SENSOR_INTERNAL_PULL 0
#endif
#ifndef CONFIG_COVER_ALLOW_STRAPPING_INPUTS
#define CONFIG_COVER_ALLOW_STRAPPING_INPUTS 0
#endif
#ifndef CONFIG_COVER_RESTORE_POSITION
#define CONFIG_COVER_RESTORE_POSITION 0
#endif

esp_err_t app_config_build(cover_controller_config_t *c)
{
    *c = {};
    c->sm.deadtime_ms = CONFIG_COVER_DEADTIME_MS;
    c->sm.max_run_uncal_ms = CONFIG_COVER_MAX_RUN_UNCAL_MS;
    c->sm.timeout_margin_pct = CONFIG_COVER_TIMEOUT_MARGIN_PCT;
    c->sm.end_overrun_pct = CONFIG_COVER_END_OVERRUN_PCT;
    c->sm.min_travel_ms = CONFIG_COVER_MIN_TRAVEL_MS;
    c->sm.pos_report_step = CONFIG_COVER_POS_REPORT_STEP;
    c->sm.goto_tolerance = CONFIG_COVER_GOTO_TOLERANCE;
    /* has_limit_* is derived by the controller from the configured inputs */

    c->period_ms = CONFIG_COVER_CONTROL_PERIOD_MS;
    c->btn_debounce_ms = CONFIG_COVER_BTN_DEBOUNCE_MS;
    c->sensor_debounce_ms = CONFIG_COVER_SENSOR_DEBOUNCE_MS;
    c->restore_position = BOOLCFG(CONFIG_COVER_RESTORE_POSITION);
    c->default_calib.open_ms = CONFIG_COVER_DEFAULT_OPEN_MS;
    c->default_calib.close_ms = CONFIG_COVER_DEFAULT_CLOSE_MS;

    c->simulation = BOOLCFG(CONFIG_COVER_SIMULATION);
    c->sim_open_ms = CONFIG_COVER_SIM_OPEN_MS;
    c->sim_close_ms = CONFIG_COVER_SIM_CLOSE_MS;
    c->sim_limit_open = BOOLCFG(CONFIG_COVER_SIM_LIMIT_OPEN);
    c->sim_limit_closed = BOOLCFG(CONFIG_COVER_SIM_LIMIT_CLOSED);

    c->gpio2.gpio_open = CONFIG_COVER_GPIO_OUT_OPEN;
    c->gpio2.gpio_close = CONFIG_COVER_GPIO_OUT_CLOSE;
    c->gpio2.active_high = OUT_ACTIVE_HIGH;
    c->gpio2.deadtime_ms = CONFIG_COVER_DEADTIME_MS;

    const bool btn_hi = BOOLCFG(CONFIG_COVER_BTN_ACTIVE_HIGH), btn_pull = BOOLCFG(CONFIG_COVER_BTN_INTERNAL_PULL);
    const bool lim_hi = BOOLCFG(CONFIG_COVER_LIMIT_ACTIVE_HIGH), lim_pull = BOOLCFG(CONFIG_COVER_LIMIT_INTERNAL_PULL);
    const bool sen_pull = BOOLCFG(CONFIG_COVER_SENSOR_INTERNAL_PULL);
    c->inputs.pin[COVER_IN_BTN_OPEN] = {CONFIG_COVER_GPIO_BTN_OPEN, btn_hi, btn_pull};
    c->inputs.pin[COVER_IN_BTN_CLOSE] = {CONFIG_COVER_GPIO_BTN_CLOSE, btn_hi, btn_pull};
    c->inputs.pin[COVER_IN_BTN_STOP] = {CONFIG_COVER_GPIO_BTN_STOP, btn_hi, btn_pull};
    c->inputs.pin[COVER_IN_BTN_FUNC] = {CONFIG_COVER_GPIO_BTN_FUNC, btn_hi, btn_pull};
    c->inputs.pin[COVER_IN_LIMIT_OPEN] = {CONFIG_COVER_GPIO_LIMIT_OPEN, lim_hi, lim_pull};
    c->inputs.pin[COVER_IN_LIMIT_CLOSED] = {CONFIG_COVER_GPIO_LIMIT_CLOSED, lim_hi, lim_pull};
    c->inputs.pin[COVER_IN_OBSTACLE] = {CONFIG_COVER_GPIO_OBSTACLE, BOOLCFG(CONFIG_COVER_OBSTACLE_ACTIVE_HIGH),
                                        sen_pull};
    c->inputs.pin[COVER_IN_DRIVER_FAULT] = {CONFIG_COVER_GPIO_DRIVER_FAULT,
                                            BOOLCFG(CONFIG_COVER_DRIVER_FAULT_ACTIVE_HIGH), sen_pull};

    c->func_commission_hold_ms = CONFIG_COVER_FUNC_COMMISSION_HOLD_MS;
    c->func_reset_hold_ms = CONFIG_COVER_FUNC_RESET_HOLD_MS;
    c->cal_combo_hold_ms = CONFIG_COVER_CAL_COMBO_HOLD_MS;

    /* ---- validation ---- */
    if (c->func_reset_hold_ms <= c->func_commission_hold_ms) {
        ESP_LOGE(TAG, "factory reset hold time must be longer than commissioning hold time");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    const bool allow_strap = BOOLCFG(CONFIG_COVER_ALLOW_STRAPPING_INPUTS);
    int pins[COVER_IN_COUNT + 2];
    const char *names[COVER_IN_COUNT + 2];
    int n = 0;
    for (int i = 0; i < COVER_IN_COUNT; i++) {
        const char *name = cover_input_name(static_cast<cover_input_id_t>(i));
        if (pin_guard_check(c->inputs.pin[i].gpio, PIN_USE_INPUT, name, allow_strap) != ESP_OK) {
            err = ESP_ERR_INVALID_ARG;
        }
        pins[n] = c->inputs.pin[i].gpio;
        names[n++] = name;
    }
    if (!c->simulation) {
        if (c->gpio2.gpio_open < 0 || c->gpio2.gpio_close < 0) {
            ESP_LOGE(TAG, "hardware mode selected but motor output GPIOs are not set");
            err = ESP_ERR_INVALID_ARG;
        }
        if (pin_guard_check(c->gpio2.gpio_open, PIN_USE_OUTPUT, "out_open", false) != ESP_OK ||
            pin_guard_check(c->gpio2.gpio_close, PIN_USE_OUTPUT, "out_close", false) != ESP_OK) {
            err = ESP_ERR_INVALID_ARG;
        }
        pins[n] = c->gpio2.gpio_open;
        names[n++] = "out_open";
        pins[n] = c->gpio2.gpio_close;
        names[n++] = "out_close";
        if (c->inputs.pin[COVER_IN_LIMIT_OPEN].gpio < 0 || c->inputs.pin[COVER_IN_LIMIT_CLOSED].gpio < 0) {
            ESP_LOGW(TAG, "no limit switch on at least one end: that end is reached by TIME only. "
                          "Only acceptable if the motor has its own end stops.");
        }
    }
    if (pin_guard_check_unique(pins, names, n) != ESP_OK) {
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}
